#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/ktime.h> 
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/pwm.h>
#include <linux/string.h> 
#include <linux/slab.h> 

MODULE_LICENSE("GPL");
MODULE_AUTHOR("EC535 User");
MODULE_DESCRIPTION("Walker Robot Driver with Vision Bridge");

// ============================================
// Definitions and constants
// ============================================
// Define pins
#define trig_pin 47 
#define echo_pin 27
#define btn0_pin 26
#define btn1_pin 46

// PWM IDs for legacy pwm controller
// ID 1 = P9_22 (EHRPWM0A) - Steering
// ID 4 = P9_14 (EHRPWM1A) - Left Leg
// ID 5 = P9_16 (EHRPWM1B) - Right Leg
#define pwm0_id 1 
#define pwm1_id 4 
#define pwm2_id 5 

// Device name
#define DEVICE_NAME "walker"
#define major_number 61

// Define constants
static const unsigned long trig_usec = 10;     // send 10 usec pulse
static const unsigned long sound_speed = 343;  // m/s
static const unsigned long dist_goal = 20;     // cm (increased to 20cm for safer stopping)
static const unsigned long cal_trig = 0;       // calibration bias (approx)
static const int pwm_period = 20000000;        // 20ms in ns (50Hz)
static const int duty_center = 1500000;        // 1.5ms
static const int duty_span = 900000;           // +-0.9ms

// VISION CONTROL MODE: 9 = Vision System is active
#define VISION_CONTROL_MODE 9
// TIMEOUT: How long to trust vision command before stopping (in ms)
#define VISION_TIMEOUT_MS 500


// ============================================
// Global Variables
// ============================================
// GPIO
static struct gpio_desc *trig, *echo;
static ktime_t start_trig, end_trig, elapsed_trig; 
static unsigned long dist_cm = 0; // Default to 0 (STOP) for safety
static int echoed = 1; // for warning message only

// PWM
static struct pwm_device *pwm0, *pwm1, *pwm2; 
static bool pwm0_enabled = false, pwm1_enabled = false, pwm2_enabled = false;
static int pwm0_state = 0; // Steering: -5 (Left) to 5 (Right)
static int pwm1_state = -10; // Left Leg: -10 to 10
static int pwm2_state = 10; // Right Leg: -10 to 10

// Timer
static struct timer_list dist_timer; 
static bool timer_is_running = false; 

// Variable for printing/status
static int current_mode = 0; // 0=stop, 9=VISION | For device reading
static int trig_rate = 2;    // Default 2 Hz (Walking Speed)
static int count_cycle = 0; 

// VISION CONTROL VARIABLES
static unsigned long last_vision_jiffies = 0; 

// LEG AMPLITUDES (For Differential Steering)
// Default 10 (Full stride). Reduced to 0 for turning.
static int amplitude_L = 10;
static int amplitude_R = 10;


// ============================================
// Function Prototypes
// ============================================
static int dev_open(struct inode *inode, struct file *filp);
static int dev_release(struct inode *inode, struct file *filp);
static ssize_t dev_read(struct file *filp, char *buf, size_t count, loff_t *f_pos);
static ssize_t dev_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos);
static struct gpio_desc *setup_gpio(unsigned int pin);
static int gpio_direction(struct gpio_desc *pin, bool is_output, int value);
static void free_gpio_pins(void);
static void timer_callback(struct timer_list *t);
static int walker_setup(void);
static irqreturn_t echo_handler(int irq, void *dev_id);
static void blink_trig(void);
static unsigned long calc_dist_cm(unsigned long ns);
static void actuate(void);
static void move(void);

// File Operations
static struct file_operations walker_fops = {
    .open = dev_open,
    .read = dev_read,
    .write = dev_write,
    .release = dev_release,
};


// ============================================
// Function Definitions
// ============================================

// --- TIMER LOOP: The Brain ---
// perform obstacle detection (distance sensor) and actuation (servo motors) at rate of trig_rate
static void timer_callback(struct timer_list *t){
    
    // Check vision timeout
    bool vision_is_fresh = time_before(jiffies, last_vision_jiffies + msecs_to_jiffies(VISION_TIMEOUT_MS));

    // 1. SENSOR SAFETY CHECK (The "Ghost Echo" Fix)
    if (echoed == 0) {
        // If we didn't get an echo from the LAST ping, the sensor is timed out.
        // Assume obstacle is 0 (STOP) to prevent crashing.
        dist_cm = 0; 
        printk(KERN_WARNING "Warning, distance sensor not echoed!\n");
    }
    
    // 2. Fire New Ping
    echoed = 0; // Reset flag
    blink_trig(); // Measure distance

    // 3. CONTROL LOGIC
    // Obstacle Avoidance Logic
    pr_info("Dictance: %lu cm\n", dist_cm);
    if (dist_cm <= dist_goal && dist_cm > 0) { 
        // STOP if too close (and sensor is working)
        current_mode = 0;
        if (pwm0_enabled) { pwm_disable(pwm0); pwm0_enabled = false; }
        if (pwm1_enabled) { pwm_disable(pwm1); pwm1_enabled = false; }
        if (pwm2_enabled) { pwm_disable(pwm2); pwm2_enabled = false; }
    }
    else { // If no obstacle
        current_mode = VISION_CONTROL_MODE;

        if (!pwm0_enabled) { pwm_enable(pwm0); pwm0_enabled = true; }
        if (!pwm1_enabled) { pwm_enable(pwm1); pwm1_enabled = true; }
        if (!pwm2_enabled) { pwm_enable(pwm2); pwm2_enabled = true; }

        if (!vision_is_fresh) {
            // --- AUTONOMOUS MODE (Fallback) ---
            printk(KERN_INFO "walker: Vision Lost. Switching to Auto.\n");
            amplitude_L = 10; amplitude_R = 10; pwm0_state = 0; // Reset stride --> kee moving forward
        }

        move();
    }
    count_cycle++;
    mod_timer(&dist_timer, jiffies + msecs_to_jiffies(1000 / trig_rate));
}

// --- MOVE: Advance the Gait Cycle --- ( set on-time for each pwm devices and set it )
static void move(void){
    
    // if forward motion, move two legs at the same time
    if (amplitude_L == amplitude_R) {
        if (pwm1_state != -pwm2_state || (pwm1_state != 10 && pwm1_state != -10)) {
        // Reset to start position if out of sync
        pwm1_state = -10;
        pwm2_state = 10;
        }
    }

    // Flip states (Sweep the legs)
    pwm1_state = -pwm1_state;
    pwm2_state = -pwm2_state;
    
    // If turning left, lift right leg
    if (amplitude_L == 0) {
        pwm1_state = 10; // stay out of the way
    }
    // If turning right, lift left leg
    if (amplitude_R == 0) {
        pwm2_state = -10;
    } 

    // Clamp steering safety
    if (pwm0_state < -5) pwm0_state = -5;
    else if (pwm0_state > 5) pwm0_state = 5;
    
    // Apply to hardware
    actuate();
}

// --- ACTUATE: Apply PWM based on State AND Amplitude ---
static void actuate(void) {
    // 1. Steering (Hip) - Direct mapping
    // pwm0_state is -5 to 5. 
    int duty0 = duty_center + (pwm0_state * duty_span / 10);

    // 2. Legs - Apply Differential Amplitude
    // pwm1_state and [wm2_state is -10 to 10
    int duty1 = duty_center + (pwm1_state * duty_span / 10);
    int duty2 = duty_center + (pwm2_state * duty_span / 10);

    pwm_config(pwm0, duty0, pwm_period);
    pwm_config(pwm1, duty1, pwm_period);
    pwm_config(pwm2, duty2, pwm_period);
}

// --- SENSOR HELPERS ---
static void blink_trig(void) {
    // Trigger ultrasonic wave
    gpiod_set_value(trig, 1);
    udelay(trig_usec);
    gpiod_set_value(trig, 0);
    start_trig = ktime_get_ns(); // start timer
}

static irqreturn_t echo_handler(int irq, void *dev_id)
{
    end_trig = ktime_get_ns(); // end timer
    elapsed_trig = end_trig - start_trig;
    echoed = 1; // Valid echo received
    dist_cm = calc_dist_cm((unsigned long)elapsed_trig); // for obstacle avoidance
    return IRQ_HANDLED;
}

static unsigned long calc_dist_cm(unsigned long ns) {
    // distance = (time * speed) / 2
    // Speed of sound ~34300 cm/s. ns to s is 10^-9.
    // Result roughly: ns / 58000
    return ns * sound_speed / 10000000 / 2; 
}


// --- GPIO HELPERS ---
static struct gpio_desc *setup_gpio(unsigned int pin){
    struct gpio_desc *desc = gpio_to_desc(pin);
    if(IS_ERR(desc)){
        pr_err("Failed to get GPIO pin %d\n", pin);
    }
    return desc; 
}

static int gpio_direction(struct gpio_desc *pin, bool is_output, int value){
    // setup gpio pin direction (input or output)
    int ret = 0;
    if (is_output) ret = gpiod_direction_output(pin, value); 
    else ret = gpiod_direction_input(pin);
    return ret;
}

static int walker_setup(void){
    int ret;
    // Request GPIOs
    if ((ret = gpio_request(trig_pin, "trig"))) return ret;
    if ((ret = gpio_request(echo_pin, "echo"))) { gpio_free(trig_pin); return ret; }
    if ((ret = gpio_request(btn0_pin, "btn0"))) { gpio_free(trig_pin); gpio_free(echo_pin); return ret; }
    if ((ret = gpio_request(btn1_pin, "btn1"))) { gpio_free(trig_pin); gpio_free(echo_pin); gpio_free(btn0_pin); return ret; }
        
    // Convert to descriptors
    trig = setup_gpio(trig_pin); 
    echo = setup_gpio(echo_pin); 
    btn0 = setup_gpio(btn0_pin);
    btn1 = setup_gpio(btn1_pin);

    // Setup Direction
    gpio_direction(trig, true, 0);
    gpio_direction(echo, false, 0);
    gpio_direction(btn0, false, 0); 
    gpio_direction(btn1, false, 0); 

    return 0;
}

static void free_gpio_pins(void){
    if (!IS_ERR_OR_NULL(trig)) gpiod_set_value(trig, 0);
    // Important: Free using the integer defines, descriptors are invalid after this
    gpio_free(trig_pin);
    gpio_free(echo_pin);
    gpio_free(btn0_pin);
    gpio_free(btn1_pin);
}


// ============================================
// CHARACTER DEVICE FUNCTIONS
// ============================================

static ssize_t dev_read(struct file *filp, char *buf, size_t count, loff_t *f_pos){
    char msg[256];
    int len;
    const char *mode_str = (current_mode == 9) ? "VISION" : (current_mode == 0 ? "STOP" : "AUTO");
    
    len = snprintf(msg, sizeof(msg), 
            "Mode: %s | Dist: %lu cm | Rate: %d Hz | AmpL: %d | AmpR: %d\n", 
            mode_str, dist_cm, trig_rate, amplitude_L, amplitude_R);

    if (*f_pos > 0) return 0;
    if (copy_to_user(buf, msg, len)) return -EFAULT;
    *f_pos += len;
    return len;
}

static ssize_t dev_write(struct file *filp, const char __user *buffer,
                         size_t len, loff_t *offset)
{
    char command_buffer[32];
    char command_verb[16];
    int angle_value = 0;
    
    if (len > sizeof(command_buffer) - 1) len = sizeof(command_buffer) - 1;
    if (copy_from_user(command_buffer, buffer, len)) return -EFAULT;
    command_buffer[len] = '\0';
    
    // FIX: Strip trailing newline
    if (len > 0 && command_buffer[len-1] == '\n') command_buffer[len-1] = '\0';
    
    // For debugging
    // printk(KERN_INFO "walker: CMD received: '%s'\n", command_buffer);
    
    // Parse: VERB:VALUE
    if (sscanf(command_buffer, "%15[^:]:%d", command_verb, &angle_value) == 2) {
        
        last_vision_jiffies = jiffies; // Update watchdog

        if (strcmp(command_verb, "FORWARD") == 0) {
            // Map angle 45-135 to State -5 to 5
            // 90 is center (0)
            int steer = -(angle_value - 90) / 9; // switch sides
            pwm0_state = steer;
            
            // Differential Steering Logic
            if (angle_value > 100) { // Left turn
                amplitude_L = 0; 
                amplitude_R = 10;
            } else if (angle_value < 80) { // Right turn
                amplitude_L = 10;
                amplitude_R = 0;
            } else {
                amplitude_L = 10;
                amplitude_R = 10;
            }
            current_mode = VISION_CONTROL_MODE;

        } else if (strcmp(command_verb, "STOP") == 0) {
            current_mode = 0;
        }

    } else {
        // Fallback: Set Trig Rate
        int new_rate;
        if (kstrtoint(command_buffer, 10, &new_rate) == 0) {
             if (new_rate >= 1 && new_rate <= 10) trig_rate = new_rate;
        }
    }
    
    // Lazy Start Timer
    if (!timer_is_running) {
        timer_setup(&dist_timer, timer_callback, 0); 
        mod_timer(&dist_timer, jiffies + msecs_to_jiffies(1000 / trig_rate));
        timer_is_running = true;
    }
    
    return len;
}

static int dev_release(struct inode *inode, struct file *filp) {
    // Safety stop on close
    if (pwm0_enabled) { pwm_disable(pwm0); pwm0_enabled = false; }
    if (pwm1_enabled) { pwm_disable(pwm1); pwm1_enabled = false; }
    if (pwm2_enabled) { pwm_disable(pwm2); pwm2_enabled = false; }
    current_mode = 0;
    return 0;
}

static int dev_open(struct inode *inode, struct file *filp) { return 0; }


// ============================================
// INIT & EXIT
// ============================================
static int __init walker_init(void)
{
    int result;

    result = register_chrdev(major_number, DEVICE_NAME, &walker_fops);
    if (result < 0) return result;

    if (walker_setup()) {
        unregister_chrdev(major_number, DEVICE_NAME);
        return -1;
    }

    // Request interrupt pin
    irq_echo = gpiod_to_irq(echo);
    if (request_irq(irq_echo, echo_handler, IRQF_TRIGGER_FALLING, "echo_irq", NULL)) return -1;

    // Request PWMs
    pwm0 = pwm_request(pwm0_id, "pwm0");
    if (IS_ERR(pwm0)) return PTR_ERR(pwm0);
    pwm1 = pwm_request(pwm1_id, "pwm1");
    if (IS_ERR(pwm1)) return PTR_ERR(pwm1);
    pwm2 = pwm_request(pwm2_id, "pwm2");
    if (IS_ERR(pwm2)) return PTR_ERR(pwm2);

    printk(KERN_INFO "walker: Module Loaded. Waiting for command.\n");
    return 0;
}

static void __exit walker_exit(void)
{
    // Delete timer
    if (timer_pending(&dist_timer)) del_timer_sync(&dist_timer);

    // Free pwm pins (disable them first)
    if (pwm0_enabled) pwm_disable(pwm0);
    if (pwm1_enabled) pwm_disable(pwm1);
    if (pwm2_enabled) pwm_disable(pwm2);
    pwm_free(pwm0);
    pwm_free(pwm1);
    pwm_free(pwm2);

    // Free GPIO pins
    free_irq(irq_echo, NULL);
    free_gpio_pins();

    unregister_chrdev(major_number, DEVICE_NAME);
    printk(KERN_INFO "walker: Module Unloaded.\n");
}

module_init(walker_init);
module_exit(walker_exit);
