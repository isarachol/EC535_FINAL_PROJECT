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


// ============================================
// DEFINITIONS and constants
// ============================================
//Define pins
#define trig_pin 47 
#define echo_pin 27
#define btn0_pin 26
#define btn1_pin 46
#define pwm0_id 1 // back (Steering Servo at Pin 22)
#define pwm1_id 4 // left leg
#define pwm2_id 5 // right leg

//Device name
#define DEVICE_NAME "walker"
#define major_number 61
#define minor_number 0

// Define constants
static const unsigned long trig_usec = 10; // send 10 usec pulse
static const unsigned long sound_speed = 343; // m/s
static const unsigned long dist_goal = 10; // cm
static const unsigned long cal_trig = 10; // calibration bias
static const int pwm_period = 20000000; // 20ms in ns
static const int duty_center = 1500000; // 1.5ms
static const int duty_span = 900000; // +-0.9ms

// VISION CONTROL MODE: 9 = Vision System is active and controlling movement
#define VISION_CONTROL_MODE 9
// TIMEOUT: How long to trust vision command before stopping/switching (in ms)
#define VISION_TIMEOUT_MS 500


// ============================================
// Global Variable s
// ============================================
// GPIO
static struct gpio_desc *trig, *echo, *btn0, *btn1;
static ktime_t start_trig, end_trig, elapsed_trig; 
static unsigned long dist_cm = 1000000000;
static int echoed = 1;
static int irq_echo = 0; // for receiving trig signal
static int irq_btn0 = 0; //for changing mode
static int irq_btn1 = 0; //for ptrigstrain 
// PWM
static struct pwm_device *pwm0, *pwm1, *pwm2; // back, left, right
static bool pwm0_enabled, pwm1_enabled, pwm2_enabled;
static int pwm0_state, pwm1_state, pwm2_state; // state = -10, 0, 10
// Timer
static struct timer_list dist_timer; //for tracking cycle
static bool timer_is_running = false; // Flag to track if timer has started

//Variable for printing
static int current_mode = 0; // 0=stop, 1 = forward, 2=turn-left, 3=turn-right (VISION is mode 9)
static int trig_rate = 1; // default = 1 Hz
static int count_cycle = 0; //Keep track of cycle

// VISION CONTROL VARIABLES
static int vision_angle_duty = 0; 
// Removed unused vision_control_active flag
static unsigned long last_vision_jiffies = 0; 

// LEG AMPLITUDES (For Differential Steering)
// Default 10 (Full stride). Reduced to 5 or 0 for turning.
static int amplitude_L = 10; 
static int amplitude_R = 10;


// ============================================
// Function prototype
// ============================================
static int dev_open(struct inode *inode, struct file *filp);
static int dev_release(struct inode *inode, struct file *filp);
static ssize_t dev_read(struct file *filp,
        char *buf, size_t count, loff_t *f_pos);
static ssize_t dev_write(struct file *filp,
        const char *buf, size_t count, loff_t *f_pos);
static struct gpio_desc *setup_gpio(unsigned int pin);
static int gpio_direction(struct gpio_desc *pin, bool is_output, int value);
static irqreturn_t btn0_handler(int irq, void *dev_id);
static irqreturn_t btn1_handler(int irq, void *dev_id);
static void free_gpio_pins(void);
static int walker_init (void);
static void walker_exit (void);
static void timer_callback(struct timer_list *t);
static int walker_setup (void);
// Distance sensor
static irqreturn_t echo_handler(int irq, void *dev_id);
static void blink_trig(void);
static unsigned long calc_dist_cm(unsigned long ns);
// PWM
static int calc_duty(int state);
static void move(void);
static void actuate(void);


//Operation
static struct file_operations walker_fops = {
    .open = dev_open,
    .read = dev_read,
    .write = dev_write,
    .release = dev_release,
};


// ============================================
// Function Definitions
// ============================================

// PWM helper functions
// state in range [-10 , 10]
static int calc_duty(int state){
    int duty = duty_center + state * duty_span / 10;
    return duty;
}

static void actuate(void) {
    pwm_config(pwm0, calc_duty(pwm0_state), pwm_period);
    pwm_config(pwm1, calc_duty(pwm1_state), pwm_period);
    pwm_config(pwm2, calc_duty(pwm2_state), pwm_period);
}

// direction determined by pwm0_state in range [-10 , 10]
static void move(void){

    // Logic updated for Differential Steering:
    // Instead of hardcoded '10', we use 'amplitude_L' and 'amplitude_R'
    // This allows one leg to take shorter steps than the other.

    // Sync check: If states don't match the alternating pattern or max amplitude
    // (Note: Using abs() logic or simple state checks)
    
    // Toggle Logic
    // We assume pwm1 and pwm2 should be opposite phases.
    // If pwm1 is currently at positive amplitude, switch to negative.
    
    if (pwm1_state > 0) {
        pwm1_state = -amplitude_L;
        pwm2_state = amplitude_R;  // Opposite phase
    } else {
        pwm1_state = amplitude_L;
        pwm2_state = -amplitude_R; // Opposite phase
    }

    // steer 0 if forward, 5 if right, -5 if left --> need range of turning angle
    // This is handled by pwm0_state being set in dev_write
    if (pwm0_state < -5) pwm0_state = -5;
    else if (pwm0_state > 5) pwm0_state = 5;
    
    // Actuate calls calc_duty on pwm0_state, pwm1_state, pwm2_state
    actuate();
}

// Callback function 
static void timer_callback(struct timer_list *t){
    
    // Check if we are within the "valid window" of a vision command
    bool vision_is_fresh = time_before(jiffies, last_vision_jiffies + msecs_to_jiffies(VISION_TIMEOUT_MS));

    // --- 1. HANDLE VISION OVERRIDE ---
    if (vision_is_fresh) {
        
        if (current_mode == VISION_CONTROL_MODE) {
            // Ensure motors are enabled
            if (!pwm0_enabled) { pwm_enable(pwm0); pwm0_enabled = true; }
            if (!pwm1_enabled) { pwm_enable(pwm1); pwm1_enabled = true; }
            if (!pwm2_enabled) { pwm_enable(pwm2); pwm2_enabled = true; }
            
            // Advance the walking cycle!
            // 'move()' uses pwm0_state (steering) and amplitude_L/R (differential speed)
            move(); 
            
        } else {
            // Vision explicitly sent STOP
            if (pwm0_enabled) { pwm_disable(pwm0); pwm0_enabled = false; }
            if (pwm1_enabled) { pwm_disable(pwm1); pwm1_enabled = false; }
            if (pwm2_enabled) { pwm_disable(pwm2); pwm2_enabled = false; }
        }
        
    } else {
        // --- 2. AUTONOMOUS / DISTANCE LOGIC ---
        
        if (current_mode == VISION_CONTROL_MODE) {
             printk(KERN_INFO "walker: Vision signal lost (Timeout). Reverting.\n");
             current_mode = 0; 
             // Reset amplitudes to full for manual/auto mode
             amplitude_L = 10;
             amplitude_R = 10;
        }

        if (echoed == 0)
        echoed = 0;
        blink_trig();
    
        if (dist_cm <= dist_goal) {
            current_mode = 0;
            if (pwm0_enabled) { pwm_disable(pwm0); pwm0_enabled = false; }
            if (pwm1_enabled) { pwm_disable(pwm1); pwm1_enabled = false; }
            if (pwm2_enabled) { pwm_disable(pwm2); pwm2_enabled = false; }
        }
        else {
            if (!pwm0_enabled) { pwm_enable(pwm0); pwm0_enabled = true; }
            if (!pwm1_enabled) { pwm_enable(pwm1); pwm1_enabled = true; }
            if (!pwm2_enabled) { pwm_enable(pwm2); pwm2_enabled = true; }

            current_mode = 1;
            if (current_mode == 1) {
                // Autonomous mode defaults to full speed straight
                pwm0_state = 0;
                amplitude_L = 10;
                amplitude_R = 10;
                move();
            }
        }
    }

    count_cycle++;
    mod_timer(&dist_timer, jiffies + msecs_to_jiffies(1000 / trig_rate));
}


// Set up GPIO -> return pointer
static struct gpio_desc *setup_gpio(unsigned int pin){
    struct gpio_desc *desc = gpio_to_desc(pin);
    if(IS_ERR(desc)){
        pr_err("Failed to get GPIO pin %d\n", pin);
    }
    return desc; 
}

static int gpio_direction(struct gpio_desc *pin, bool is_output, int value){
    int ret = 0;
    if (is_output){
        ret= gpiod_direction_output(pin, value); 
    }else{
        ret = gpiod_direction_input(pin);
    }
    if(ret){
        pr_err("Failed to set GPIO direction\n");
        return ret; 
    }else{
        return 0;
    }
}


// Setup GPIO pin 
static int walker_setup (void){

    int ret;
    
    // Request GPIOs legacy
    ret = gpio_request(trig_pin, "trig");
    if (ret) return ret;
    ret = gpio_request(echo_pin, "echo");
    if (ret) { gpio_free(trig_pin); return ret; }
    ret = gpio_request(btn0_pin, "btn0");
    if (ret) { gpio_free(trig_pin); gpio_free(echo_pin); return ret; }
    ret = gpio_request(btn1_pin, "btn1");
    if (ret) { gpio_free(trig_pin); gpio_free(echo_pin); gpio_free(btn0_pin); return ret; }
        

    // Convert legacy to desc
    trig = setup_gpio(trig_pin); 
    if(IS_ERR(trig)) return(PTR_ERR(trig));
    echo = setup_gpio(echo_pin); 
    if(IS_ERR(echo)) return(PTR_ERR(echo));
    btn0 = setup_gpio(btn0_pin);
    if(IS_ERR(btn0)) return(PTR_ERR(btn0));
    btn1 = setup_gpio(btn1_pin);
    if(IS_ERR(btn1)) return(PTR_ERR(btn1));

    
    /*========= Setup output and input =========*/
    // out
    ret = gpio_direction(trig, true, 0);
    if (ret) return ret;
    ret = gpio_direction(echo, false, 0);
    if (ret) return ret;
    ret = gpio_direction(btn0, false, 0); 
    if (ret) return ret;
    ret = gpio_direction(btn1, false, 0); 
    if (ret) return ret;

    return 0;
}

// Clean GPIO / free pin 
static void free_gpio_pins(void){
    // Turn off all outputs
    if (!IS_ERR_OR_NULL(trig)) gpiod_set_value(trig, 0);
    
    // Release all GPIOs (CHANGED - use gpio_free)
    gpio_free(trig_pin);
    gpio_free(echo_pin);
    gpio_free(btn0_pin);
    gpio_free(btn1_pin);
}

// Echo helper function
static void blink_trig(void) {
    gpiod_set_value(trig, 1);
    udelay(trig_usec);
    gpiod_set_value(trig, 0);
    start_trig = ktime_get_ns();
}

static unsigned long calc_dist_cm(unsigned long ns) {
    return ns * sound_speed / 10000000 / 2 - cal_trig; // forth and back
}

static irqreturn_t echo_handler(int irq, void *dev_id)
{
    end_trig = ktime_get_ns();
    elapsed_trig = end_trig - start_trig;

    echoed = 1;
    dist_cm = calc_dist_cm((unsigned long)elapsed_trig);
    return IRQ_HANDLED;
}

// Press Button 0
static irqreturn_t btn0_handler(int irq, void *dev_id)
{
    // TODO
    return IRQ_HANDLED;
}

// Press Button 1
static irqreturn_t btn1_handler(int irq, void *dev_id)
{
    // TODO
    return IRQ_HANDLED;
}


// ============================================
// Character Device Functions
// ============================================

static ssize_t dev_read(struct file *filp, char *buf, size_t count, loff_t *f_pos){
    int msg_len;
    char msg[256];
    int error_count;
    const char *mode;

    switch(current_mode){ // 0=stop, 1 = forward, 2=turn-left, 3=turn-right, 9=VISION
        case 0:
            mode = "STOP";
            break;
        case 1:
            mode = "FORWARD (Auto)";
            break;
        case 2:
            mode = "TURN LEFT (Auto)";
            break;
        case 3:
            mode = "TURN RIGHT (Auto)";
            break;
        case VISION_CONTROL_MODE:
            mode = "VISION (Override)";
            break;
        default:
            mode = "Unknown";
            break;
    }

    msg_len = snprintf(msg, sizeof(msg), 
                "\nCurrent Mode: %s\n"
                "Trig Rate: %d Hz\n"
                "Distance (cm): %lu \n\n", mode, trig_rate, dist_cm); // convert to cm

    // Only read once
    if (*f_pos > 0){
        return 0;
    }

    // Copy to user space
    error_count = copy_to_user(buf, msg, msg_len);

    if (error_count == 0){
        *f_pos += msg_len;
        return msg_len;
    }else{
        pr_err("walker: Failed to send data to user\n");
        return -EFAULT;
    }
            
}

/* * VISION INTEGRATION POINT 
 * This function receives the command string from the user-space network bridge.
 * Format expected: COMMAND:ANGLE (e.g., FORWARD:90, STOP:0)
 */
static ssize_t dev_write(struct file *filp, const char __user *buffer,
                            size_t len, loff_t *offset)
{
    char command_buffer[32];
    char command_verb[16];
    int angle_value = 0;
    
    // Copy data from user space
    if (len > sizeof(command_buffer) - 1) len = sizeof(command_buffer) - 1;
    if (copy_from_user(command_buffer, buffer, len)) return -EFAULT;
    command_buffer[len] = '\0';
    
    printk(KERN_INFO "walker: RX Command: %s (Length: %zu)\n", command_buffer, len); 
    
    // --- 1. Parse the command format: VERB:VALUE ---
    if (sscanf(command_buffer, "%15[^:]:%d", command_verb, &angle_value) == 2) {
        
        // Update the timestamp of the last valid command
        last_vision_jiffies = jiffies;

        if (strcmp(command_verb, "FORWARD") == 0) {
            
            // 1. Calculate steering state (-5 to 5)
            pwm0_state = (angle_value - 90) / 9; 
            
            // 2. Set Differential Steering Amplitudes
            // If Angle < 80 (Left Turn), reduce Left Leg (pwm1)
            // If Angle > 100 (Right Turn), reduce Right Leg (pwm2)
            if (angle_value < 80) {
                amplitude_L = 5;  // Slow down left leg to pivot left
                amplitude_R = 10;
            } else if (angle_value > 100) {
                amplitude_L = 10;
                amplitude_R = 5;  // Slow down right leg to pivot right
            } else {
                amplitude_L = 10; // Full speed ahead
                amplitude_R = 10;
            }

            // 3. Store calculated PWM duty cycle for steering (for debugging mostly, move() recalculates)
            vision_angle_duty = calc_duty(pwm0_state);
            
            printk(KERN_DEBUG "walker: Vision Steer: %d, AmpL: %d, AmpR: %d\n", pwm0_state, amplitude_L, amplitude_R); 
            current_mode = VISION_CONTROL_MODE;

        } else if (strcmp(command_verb, "STOP") == 0) {
            
            current_mode = 0; // Set to manual stop mode
            // printk(KERN_INFO "walker: VISION STOP command received.\n");

        } else {
             pr_err("walker: Unknown vision command verb: %s\n", command_verb);
        }

    } else {
        // --- 2. Fallback: Check if user is trying to set trig rate (Original write purpose) ---
        int new_trig_rate;
        if (kstrtoint(command_buffer, 10, &new_trig_rate) == 0) {
             if (new_trig_rate >= 1 && new_trig_rate <= 9) {
                 trig_rate = new_trig_rate;
                 pr_info("walker: Cycle rate set to %d Hz\n", trig_rate);
             } else {
                 pr_debug("walker: Invalid cycle rate %d, ignoring\n", new_trig_rate);
             }
        } else {
             pr_err("walker: Write failed, invalid command format: %s\n", command_buffer);
        }
    }
    
    // --- START OR UPDATE TIMER ---
    if (!timer_is_running) {
        // First command received: Kickstart the loop
        timer_setup(&dist_timer, timer_callback, 0); 
        mod_timer(&dist_timer, jiffies + msecs_to_jiffies(1000 / trig_rate));
        timer_is_running = true;
        pr_info("walker: First command received. Starting control loop.\n");
    } else {
        // Loop already running: Apply new PWM immediately if needed
        // mod_timer(&dist_timer, jiffies); // Actually, let's let the natural rhythm flow
    }
    
    return len;
}

static int dev_release(struct inode *inode, struct file *filp)
{
    pr_info("walker: Device closed. Stopping motors.\n");
    
    // SAFETY: Kill motors when the bridge disconnects/dies
    if (pwm0_enabled) { pwm_disable(pwm0); pwm0_enabled = false; }
    if (pwm1_enabled) { pwm_disable(pwm1); pwm1_enabled = false; }
    if (pwm2_enabled) { pwm_disable(pwm2); pwm2_enabled = false; }
    
    current_mode = 0;
    
    // Optionally stop the timer if you want full silence until next open
    // if (timer_is_running) { del_timer_sync(&dist_timer); timer_is_running = false; }

    return 0;
}

static int dev_open(struct inode *inode, struct file *filp)
{
    pr_info("walker: Device opened\n");
    return 0;
}

static int walker_init(void)
{
    int result = 0;
    int ret = 0;

    /* Registering device */
    result = register_chrdev(major_number, DEVICE_NAME, &walker_fops);
    if (result < 0)
    {
        printk(KERN_ALERT
            "walker: cannot obtain major number %d\n", major_number);
        return result;
    }
    
    printk(KERN_INFO "walker: module loaded\n");

    //Set up GPIO pins
    ret = walker_setup();
    if (ret) {
        printk(KERN_ALERT "walker: Failed to setup GPIO pins\n");
        unregister_chrdev(major_number, "walker");
        return ret;
    }

    irq_echo = gpiod_to_irq(echo);
    irq_btn0 = gpiod_to_irq(btn0);
    irq_btn1 = gpiod_to_irq(btn1);
    if (irq_btn0 < 0 || irq_btn1 < 0 || irq_echo < 0) {
        printk(KERN_ALERT "walker: Failed to get IRQ numbers\n");
        free_gpio_pins();
        unregister_chrdev(major_number, "walker");
        return -ENODEV;
    }

    // Request IRQs 
    ret = request_irq(irq_echo, echo_handler, IRQF_TRIGGER_FALLING, "echo_irq", NULL); // falling edge for echo complete
    if (ret) {
        printk(KERN_ALERT "walker: Failed to request IRQ for echo\n");
        free_gpio_pins();
        unregister_chrdev(major_number, "walker");
        return ret;
    }

    ret = request_irq(irq_btn0, btn0_handler, IRQF_TRIGGER_RISING, "btn0_irq", NULL);
    if (ret) {
        printk(KERN_ALERT "walker: Failed to request IRQ for btn0\n");
        free_irq(irq_echo, NULL);
        free_gpio_pins();
        unregister_chrdev(major_number, "walker");
        return ret;
    }

    ret = request_irq(irq_btn1, btn1_handler, IRQF_TRIGGER_RISING, "btn1_irq", NULL);
    if (ret) {
        printk(KERN_ALERT "walker: Failed to request IRQ for btn1\n");
        free_irq(irq_echo, NULL);
        free_irq(irq_btn0, NULL);
        free_gpio_pins();
        unregister_chrdev(major_number, "walker");
        return ret;
    }

    // Initialize vision control duties to center/stop state (NOW SAFE IN FUNCTION)
    vision_angle_duty = duty_center;
    // Removed vision_motor_duty_L/R initialization as they are now dynamically set in move() via amplitude

    // Request pwm
    pwm0 = pwm_request(1, "pwm0");
    if (IS_ERR(pwm0)) {
        pr_info("pwm0 failed\n");
        // ... (cleanup code)
        return PTR_ERR(pwm0);
    }
    pwm_config(pwm0, duty_center, pwm_period);
    pwm0_enabled = false;
    pwm0_state = 0; // Changed state from -10 to 0

    pwm1 = pwm_request(4, "pwm1");
    if (IS_ERR(pwm1)) {
        pr_info("pwm1 failed\n");
        // ... (cleanup code)
        pwm_free(pwm0);
        return PTR_ERR(pwm1);
    }
    pwm_config(pwm1, duty_center, pwm_period);
    pwm1_enabled = false;
    pwm1_state = 0;

    pwm2 = pwm_request(5, "pwm2");
    if (IS_ERR(pwm2)) {
        pr_info("pwm2 failed\n");
        // ... (cleanup code)
        pwm_free(pwm0);
        pwm_free(pwm1);
        return PTR_ERR(pwm2);
    }
    pwm_config(pwm2, duty_center, pwm_period);
    pwm2_enabled = false;
    pwm2_state = 0;

    // Initialize timer setup, BUT DO NOT START IT YET (Lazy Start)
    timer_setup(&dist_timer, timer_callback, 0);
    timer_is_running = false; // Ensure flag is false
    // Removed: mod_timer(&dist_timer, jiffies + msecs_to_jiffies(1000 / trig_rate));

    return 0;
}

static void walker_exit(void)
{
    // Delete timer
    if (timer_pending(&dist_timer))
        del_timer_sync(&dist_timer);

    // Disable PWMs
    if (pwm0_enabled) pwm_disable(pwm0);
    if (pwm1_enabled) pwm_disable(pwm1);
    if (pwm2_enabled) pwm_disable(pwm2);

    // Free IRQs
    if (irq_echo > 0) free_irq(irq_echo, NULL);
    if (irq_btn0 > 0) free_irq(irq_btn0, NULL);
    if (irq_btn1 > 0) free_irq(irq_btn1, NULL);

    // Free pwm
    pwm_free(pwm0);
    pwm_free(pwm1);
    pwm_free(pwm2);

    // Free GPIO pins
    free_gpio_pins();

    /* Freeing the major number */
    unregister_chrdev(major_number, "walker");

    printk(KERN_ALERT "Removing walker module\n");
}

/* Function to init and exit functions */
module_init(walker_init);
module_exit(walker_exit);
