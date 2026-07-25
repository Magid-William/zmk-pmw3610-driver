#define DT_DRV_COMPAT promini_trackpoint_i2c

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(trackpoint_i2c, CONFIG_TRACKPOINT_I2C_LOG_LEVEL);

struct trackpoint_i2c_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec irq_gpio;
};

struct trackpoint_i2c_data {
    const struct device *dev;
    struct gpio_callback irq_gpio_cb;
    struct k_work_delayable poll_work;
};

static void trackpoint_i2c_poll(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct trackpoint_i2c_data *data = CONTAINER_OF(dwork, struct trackpoint_i2c_data, poll_work);
    const struct trackpoint_i2c_config *cfg = data->dev->config;

    uint8_t addr = 0x00;
    uint8_t val = 0;

    int ret = i2c_write_read_dt(&cfg->i2c, &addr, 1, &val, 1);
    if (ret == 0) {
        printk("POLL: reg[0x00]=0x%02x\n", val);
    } else {
        printk("POLL: I2C fail %d\n", ret);
    }

    k_work_schedule(&data->poll_work, K_MSEC(100));
}

static void trackpoint_i2c_gpio_callback(const struct device *gpiob,
                                          struct gpio_callback *cb, uint32_t pins) {
    (void)gpiob; (void)cb; (void)pins;
}

static int trackpoint_i2c_init(const struct device *dev) {
    struct trackpoint_i2c_data *data = dev->data;
    const struct trackpoint_i2c_config *cfg = dev->config;

    data->dev = dev;

    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    if (!device_is_ready(cfg->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
    if (ret) {
        LOG_ERR("Cannot configure IRQ GPIO: %d", ret);
        return ret;
    }

    k_work_init_delayable(&data->poll_work, trackpoint_i2c_poll);
    k_work_schedule(&data->poll_work, K_MSEC(500));

    uint8_t tst_addr = 0x00;
    uint8_t tst_val = 0;
    int tst_ret = i2c_write_read_dt(&cfg->i2c, &tst_addr, 1, &tst_val, 1);
    if (tst_ret == 0) {
        printk("I2C INIT OK: reg[0x00]=0x%02x\n", tst_val);
    } else {
        printk("I2C INIT FAIL: %d\n", tst_ret);
    }

    LOG_INF("trackpoint-i2c initialized");
    return 0;
}

#define TRACKPOINT_I2C_DEFINE(n)                                                        \
    static struct trackpoint_i2c_data data##n;                                          \
    static const struct trackpoint_i2c_config config##n = {                             \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                 \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                \
    };                                                                                  \
    DEVICE_DT_INST_DEFINE(n, trackpoint_i2c_init, NULL, &data##n, &config##n,           \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_I2C_DEFINE)
