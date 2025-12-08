#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pwm.h>
#include <linux/err.h>

#define DEVICE_NAME "my_walker"

#define min_duty 500000
#define max_duty 2500000
#define period 20000000

// static struct pwm_device *mtr
// static struct pwm_device *mtb

struct pwm_stepper {
	struct pwm_device *pwm;
    // unsigned long duty_time;
	unsigned long period;
	bool suspended;
}

static struct pwm_device *pwm;     // Pointer to a PWM device
static struct pwm_state state;     // New PWM state configuration

/* Name all the compatbile devices */
static const struct of_device_id my_walker_pwm_ids[] = {
    { .compatible = "bone-pinmux-helper" },
    {}
};

MODULE_DEVICE_TABLE(of, my_walker_pwm_ids);

/* Implement a probe and a remove function */
// static int my_walker_probe(struct platform_device *pdev)
// {
//     struct device *dev = &pdev->dev;

// 	pr_info("my_walker - Probe Function is called!\n");

    
//     pwm_get(struct device * dev, const char * con_id)
// 	return 0;
// }

static int my_walker_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwm_stepper *stepper;
	struct pwm_state state;
	int error;

	stepper = devm_kzalloc(dev, sizeof(*stepper), GFP_KERNEL);
	if (!stepper)
		return -ENOMEM;

    /* Request pwm manager? */
	stepper->pwm = devm_pwm_get(dev, NULL);
	if (IS_ERR(stepper->pwm)) {
		error = PTR_ERR(stepper->pwm);
		if (error != -EPROBE_DEFER)
			dev_err(dev, "Failed to request PWM device: %d\n",
				error);
		return error;
	}

	/* Sync up PWM state and ensure it is off. */
	pwm_init_state(stepper->pwm, &state);
	state.enabled = false;
	error = pwm_apply_state(stepper->pwm, &state);
	if (error) {
		dev_err(dev, "failed to apply initial PWM state: %d\n",
			error);
		return error;
	}

	// Set 1 kHz (1,000,000 ns period)
    state.period = 20000000;

    // 50% duty cycle
    state.duty_cycle = 800000; //state.period / 2;

    // normal polarity
    state.polarity = PWM_POLARITY_NORMAL;

    // Enable
    state.enabled = true;

    pwm_apply_state(ptdev->pwm, &state);

    dev_info(&pdev->dev, "PWM enabled\n");

	return 0;
}

static int my_walker_remove(struct platform_device *pdev)
{
    printk(KERN_INFO "TEST remove\n");
    struct pwm_stepper *ptdev = platform_get_drvdata(pdev);

    pwm_disable(ptdev->pwm);

    dev_info(&pdev->dev, "PWM disabled\n");
    return 0;
}

/* Bundle compatible devices, probe and remove in driver's struct */
static struct platform_driver my_walker_driver = {
    .driver = {
        .name = DEVICE_NAME,
        .of_match_table = my_walker_pwm_ids,
    },
    .probe = my_walker_probe,
    .remove = my_walker_remove,
    // .suspended
    // .resume
};

/* Register the driver at the OS */
#if 1
static int __init my_init(void)
{
	pr_info("my_dev - Init function is called\n");
	return platform_driver_register(&my_walker_driver);;
}

static void __exit my_exit(void)
{
	pr_info("my_dev - Exit function is called\n");
	platform_driver_unregister(&my_walker_driver);;
}

module_init(my_walker);
module_exit(my_walker);
#else
module_platform_driver(my_walker_driver);
#endif


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Simple PWM kernel module example");
MODULE_AUTHOR("You");