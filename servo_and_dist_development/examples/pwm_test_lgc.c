#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/pwm.h>

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Johannes 4 GNU/Linux");
MODULE_DESCRIPTION("A simple driver to access the Hardware PWM IP");

/* Variables for device and device class */
static dev_t my_device_nr;
static struct class *my_class;
static struct cdev my_device;

#define DRIVER_NAME "my_pwm_driver"
#define DRIVER_CLASS "MyModuleClass"

/* Variables for pwm  */
struct pwm_device *pwm0 = NULL;
u32 pwm_on_time = 1500000;
u32 pwm_period = 20000000;

/**
 * @brief Write data to buffer
 */
static ssize_t driver_write(struct file *File, const char *user_buffer, size_t count, loff_t *offs) {
	int to_copy, not_copied, delta;
	char value;

	/* Get amount of data to copy */
	to_copy = min(count, sizeof(value));

	/* Copy data to user */
	not_copied = copy_from_user(&value, user_buffer, to_copy);

	/* Set PWM on time */
	if(value < 'a' || value > 'j')
		printk("Invalid Value\n");
	else
		pwm_config(pwm0, 100000000 * (value - 'a'), 1000000000);

	/* Calculate data */
	delta = to_copy - not_copied;

	return delta;
}

/**
 * @brief Read data from buffer
 */
static ssize_t driver_read(struct file *filp, char *buf, size_t count, loff_t *f_pos){
    int msg_len;
    char msg[256];
    int error_count;
    const char *mode;
    const char *red_str, *yellow_str, *green_str;
    const char *ped_str;

    red_str = gpiod_get_value(red) ? "on": "off";
    yellow_str = gpiod_get_value(yellow) ? "on": "off";
    green_str = gpiod_get_value(green) ? "on": "off";

    ped_str = (pedrestrian_press || pedrestrian_cross) ? "Active" : "Not Active";

    msg_len = snprintf(msg, sizeof(msg), 
                "\nCurrent Mode: %s\n"
                "Cycle Rate: %d Hz\n"
                "Red: %s, Yellow: %s, Green: %s\n"
                "Pedestrian: %s\n\n", mode, cycle_rate, red_str, yellow_str, green_str, ped_str); 

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
        pr_err("mytraffic: Failed to send data to user\n");
        return -EFAULT;
    }
                
}

/**
 * @brief This function is called, when the device file is opened
 */
static int driver_open(struct inode *device_file, struct file *instance) {
	printk("dev_nr - open was called!\n");
	return 0;
}

/**
 * @brief This function is called, when the device file is opened
 */
static int driver_close(struct inode *device_file, struct file *instance) {
	printk("dev_nr - close was called!\n");
	return 0;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = driver_open,
	.release = driver_close,
	.write = driver_write,
	.read = driver_read
};

/**
 * @brief This function is called, when the module is loaded into the kernel
 */
static int __init ModuleInit(void) {
	printk("Hello, Kernel!\n");

	/* Allocate a device nr */
	if( alloc_chrdev_region(&my_device_nr, 0, 1, DRIVER_NAME) < 0) {
		printk("Device Nr. could not be allocated!\n");
		return -1;
	}
	printk("read_write - Device Nr. Major: %d, Minor: %d was registered!\n", my_device_nr >> 20, my_device_nr && 0xfffff);

	/* Create device class */
	if((my_class = class_create(THIS_MODULE, DRIVER_CLASS)) == NULL) {
		printk("Device class can not be created!\n");
		goto ClassError;
	}

	/* create device file */
	if(device_create(my_class, NULL, my_device_nr, NULL, DRIVER_NAME) == NULL) {
		printk("Can not create device file!\n");
		goto FileError;
	}

	/* Initialize device file */
	cdev_init(&my_device, &fops); 

	/* Regisering device to kernel */
	if(cdev_add(&my_device, my_device_nr, 1) == -1) {
		printk("Registering of device to kernel failed!\n");
		goto AddError;
	}

	pwm0 = pwm_request(0, "my-pwm");
	if(pwm0 == NULL) {
		printk("Could not get PWM0!\n");
		goto AddError;
	}

	pwm_config(pwm0, pwm_on_time, pwm_period);
	pwm_enable(pwm0);

	return 0;
AddError:
	device_destroy(my_class, my_device_nr);
FileError:
	class_destroy(my_class);
ClassError:
	unregister_chrdev_region(my_device_nr, 1);
	return -1;
}

/**
 * @brief This function is called, when the module is removed from the kernel
 */
static void __exit ModuleExit(void) {
	pwm_disable(pwm0);
	pwm_free(pwm0);
	cdev_del(&my_device);
	device_destroy(my_class, my_device_nr);
	class_destroy(my_class);
	unregister_chrdev_region(my_device_nr, 1);
	printk("Goodbye, Kernel\n");
}

module_init(ModuleInit);
module_exit(ModuleExit);
