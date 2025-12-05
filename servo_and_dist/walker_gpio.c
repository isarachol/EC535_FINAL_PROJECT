#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/ktime.h> // allow timing in nano seconds
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>

MODULE_LICENSE("GPL");


//Define pins
#define trig_pin 47 // Don't use 117, always HIGH
#define echo_pin 27
// #define pwmb_pin 66 // back
// #define pwml_pin 69 // left
// #define pwmr_pin 45 // right
#define btn0_pin 26
#define btn1_pin 46


//Device name
#define DEVICE_NAME "walker"
#define major_number 61
#define minor_number 0


// Define constants
static const unsigned long trig_usec = 10; // send 10 usec pulse
static const unsigned long sound_speed = 343; // m/s
static const unsigned long stop_distance = 10; // cm
static const unsigned long cal_trig = 10; // calibration bias


//Global Variable 
static struct gpio_desc *trig, *echo, *btn0, *btn1; //*trig, *green, *yellow, *btn0, *btn1;
static ktime_t start_trig, end_trig, elapsed_trig; 
static unsigned long dist_um = 1000000000;
static int echoed = 1;
static int irq_echo = 0; // for receiving trig signal
static int irq_btn0 = 0; //for changing mode
static int irq_btn1 = 0; //for ptrigstrain 
static struct timer_list dist_timer; //for tracking cycle

//Variable for printing
static int current_mode = 0; // 0=stop, 1 = forward, 2=turn-left, 3=turn-right
static int trig_rate = 1; // default = 1 Hz
static int count_cycle = 0; //Keep track of cycle
// static bool pedrestrian_press = false; //For track at the start of trig
// static bool pedrestrian_cross = false; //For flashing yellow 5 cycles
// static bool lightbulb_checked = false; //For after checking lightbulb


//Function
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
static void blink_trig(void);
static unsigned long calc_dist_cm(unsigned long ns);


//Operation
static struct file_operations walker_fops = {
    .open = dev_open,
    .read = dev_read,
    .write = dev_write,
    .release = dev_release,
};


/* Function to init and exit functions */
module_init(walker_init);
module_exit(walker_exit);

// Helper function

static void blink_trig() {
	gpiod_set_value(trig, 1);
	udelay(trig_usec);
	gpiod_set_value(trig, 0);
	start_trig = ktime_get_ns();
}

static unsigned long calc_dist_cm(unsigned long ns) {
	return ns * sound_speed / 10000000 / 2 - cal_trig; // forth and back
}

//Callback function 
static void timer_callback(struct timer_list *t){
	if (echoed == 0)
		printk(KERN_ALERT "Echo not heard! Try again\n");
	echoed = 0;
	blink_trig();

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
	// ret = gpio_request(pwmb_pin, "btn1");
    // if (ret) { gpio_free(trig_pin); gpio_free(echo_pin); gpio_free(btn0_pin); gpio_free(btn1_pin); return ret; }
	// ret = gpio_request(pwml_pin, "btn1");
    // if (ret) { gpio_free(trig_pin); gpio_free(echo_pin); gpio_free(btn0_pin); gpio_free(btn1_pin); gpio_free(pwmb_pin); return ret; }
	// ret = gpio_request(pwmr_pin, "btn1");
    // if (ret) { gpio_free(trig_pin); gpio_free(echo_pin); gpio_free(btn0_pin); gpio_free(btn1_pin); gpio_free(pwmb_pin); gpio_free(pwml_pin); return ret; }
	

    // Convert legacy to desc
    trig = setup_gpio(trig_pin); 
    if(IS_ERR(trig)) return(PTR_ERR(trig));
    echo = setup_gpio(echo_pin); 
    if(IS_ERR(echo)) return(PTR_ERR(echo));
    btn0 = setup_gpio(btn0_pin);
    if(IS_ERR(btn0)) return(PTR_ERR(btn0));
    btn1 = setup_gpio(btn1_pin);
    if(IS_ERR(btn1)) return(PTR_ERR(btn1));

    // pwmb = setup_gpio(pwmb_pin);
    // if(IS_ERR(pwmb)) return(PTR_ERR(pwmb));
    // pwml = setup_gpio(pwml_pin);
    // if(IS_ERR(pwml)) return(PTR_ERR(pwml));
    // pwmr = setup_gpio(pwmr_pin);
    // if(IS_ERR(pwmr)) return(PTR_ERR(pwmr));

    /*========= Setup output and input =========*/
    // out
    ret = gpio_direction(trig, true, 0);
    if (ret) return ret;
	// ret = gpio_direction(pwmb, true, 0);
    // if (ret) return ret;
	// ret = gpio_direction(pwml, true, 0);
    // if (ret) return ret;
	// ret = gpio_direction(pwmr, true, 0);
    // if (ret) return ret;
	// in
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
	// if (!IS_ERR_OR_NULL(pwmb)) gpiod_set_value(pwmb, 0);
	// if (!IS_ERR_OR_NULL(pwml)) gpiod_set_value(pwml, 0);
	// if (!IS_ERR_OR_NULL(pwmr)) gpiod_set_value(pwmr, 0);
    
    // Release all GPIOs (CHANGED - use gpio_free)
	gpio_free(trig_pin);
	gpio_free(echo_pin);
    gpio_free(btn0_pin);
    gpio_free(btn1_pin);
    // gpio_free(pwmb_pin);
    // gpio_free(pwml_pin);
    // gpio_free(pwmr_pin);
}

// Echo heard
static irqreturn_t echo_handler(int irq, void *dev_id)
{
	end_trig = ktime_get_ns();
	elapsed_trig = end_trig - start_trig;

	echoed = 1;
	dist_um = calc_dist_cm((unsigned long)elapsed_trig);
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

static ssize_t dev_read(struct file *filp, char *buf, size_t count, loff_t *f_pos){
    int msg_len;
    char msg[256];
    int error_count;
    const char *mode;

    switch(current_mode){ // 0=stop, 1 = forward, 2=turn-left, 3=turn-right
        case 0:
            mode = "STOP";
            break;
        case 1:
            mode = "FORWARD";
            break;
        case 2:
            mode = "TURN LEFT";
            break;
        case 3:
            mode = "TURN RIGHT";
            break;
        default:
            mode = "Unknown";
            break;
    }

    msg_len = snprintf(msg, sizeof(msg), 
                "\nCurrent Mode: %s\n"
                "Trig Rate: %d Hz\n"
                "distance (cm): %lu \n\n", mode, trig_rate, dist_um); // convert to cm

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


static ssize_t dev_write(struct file *filp, const char __user *buffer,
                         size_t len, loff_t *offset)
{
    char user_input[16];
    int new_trig_rate;
    
    // Copy data from user space
    if (len > sizeof(user_input) - 1)
        len = sizeof(user_input) - 1;
    
    if (copy_from_user(user_input, buffer, len))
        return -EFAULT;
    
    user_input[len] = '\0';
    
    // The integer (1-9)
    if (kstrtoint(user_input, 10, &new_trig_rate) == 0) {
        if (new_trig_rate >= 1 && new_trig_rate <= 9) {
            trig_rate = new_trig_rate;
            pr_info("walker: Cycle rate set to %d Hz\n", trig_rate);
        } else {
            pr_debug("walker: Invalid cycle rate %d, ignoring\n", new_trig_rate);
        }
    }
    
    return len;
}

static int dev_release(struct inode *inode, struct file *filp)
{
    pr_info("walker: Device closed\n");
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
	ret = request_irq(irq_echo, echo_handler, IRQF_TRIGGER_FALLING, "echo_irq", NULL); // rising always wrong somehow
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

	// Initialize and start timer
    timer_setup(&dist_timer, timer_callback, 0);
    mod_timer(&dist_timer, jiffies + msecs_to_jiffies(1000 / trig_rate));

	return 0;
}

static void walker_exit(void)
{
    // Delete timer
    del_timer_sync(&dist_timer);

    // Free IRQs (ADDED - REQUItrig!)
	if (irq_echo > 0) free_irq(irq_echo, NULL);
    if (irq_btn0 > 0) free_irq(irq_btn0, NULL);
    if (irq_btn1 > 0) free_irq(irq_btn1, NULL);

    // Free GPIO pins
    free_gpio_pins();

    /* Freeing the major number */
    unregister_chrdev(major_number, "walker");

    printk(KERN_ALERT "Removing walker module\n");
}
