// pwm_test.c
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/of.h>

struct pwm_test_dev {
    struct pwm_device *pwm;
};

static int pwm_test_probe(struct platform_device *pdev)
{
    struct pwm_state state;
    struct pwm_test_dev *ptdev;
    printk(KERN_INFO "Test probe\n");

    ptdev = devm_kzalloc(&pdev->dev, sizeof(*ptdev), GFP_KERNEL);
    if (!ptdev)
        return -ENOMEM;

    // Get PWM from DT
    ptdev->pwm = devm_pwm_get(&pdev->dev, NULL);
    if (IS_ERR(ptdev->pwm)) {
        dev_err(&pdev->dev, "Failed to get PWM\n");
        return PTR_ERR(ptdev->pwm);
    }

    pwm_init_state(ptdev->pwm, &state);

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

static int pwm_test_remove(struct platform_device *pdev)
{
    printk(KERN_INFO "TEST remove\n");
    struct pwm_test_dev *ptdev = platform_get_drvdata(pdev);

    pwm_disable(ptdev->pwm);

    dev_info(&pdev->dev, "PWM disabled\n");
    return 0;
}

static const struct of_device_id pwm_test_dt_ids[] = {
    { .compatible = "example,pwm-test" },
    {}
};

MODULE_DEVICE_TABLE(of, pwm_test_dt_ids);

static struct platform_driver pwm_test_driver = {
    .driver = {
        .name = "pwm_test",
        .of_match_table = pwm_test_dt_ids,
    },
    .probe = pwm_test_probe,
    .remove = pwm_test_remove,
};

module_platform_driver(pwm_test_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("You");
MODULE_DESCRIPTION("Simple PWM test driver");