#define DT_DRV_COMPAT promini_trackpoint_i2c

#include <zephyr/kernel.h>
#include <zephyr/device.h>
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
    struct k_work trigger_work;
};

static void trackpoint_i2c_work_callback(struct k_work *work) {
    struct trackpoint_i2c_data *data = CONTAINER_OF(work, struct trackpoint_i2c_data, trigger_work);
    const struct trackpoint_i2c_config *cfg = data->dev->config;

    uint8_t addr = 0x00;
    uint8_t val = 0;

    int ret = i2c_write_read_dt(&cfg->i2c, &addr, 1, &val, 1);
    if (ret == 0) {
        LOG_INF("I2C reg[0x00] = 0x%02x", val);
    } else {
        LOG_ERR("I2C read failed: %d", ret);
    }
}

static void trackpoint_i2c_gpio_callback(const struct device *gpiob,
                                          struct gpio_callback *cb, uint32_t pins) {
    struct trackpoint_i2c_data *data = CONTAINER_OF(cb, struct trackpoint_i2c_data, irq_gpio_cb);
    const struct trackpoint_i2c_config *cfg = data->dev->config;

    gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);
    k_work_submit(&data->trigger_work);
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

    gpio_init_callback(&data->irq_gpio_cb, trackpoint_i2c_gpio_callback,
                       BIT(cfg->irq_gpio.pin));
    ret = gpio_add_callback(cfg->irq_gpio.port, &data->irq_gpio_cb);
    if (ret) {
        LOG_ERR("Cannot add IRQ callback: %d", ret);
        return ret;
    }

    k_work_init(&data->trigger_work, trackpoint_i2c_work_callback);

    ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_LEVEL_ACTIVE);
    if (ret) {
        LOG_ERR("Cannot enable IRQ interrupt: %d", ret);
        return ret;
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
                          POST_KERNEL, INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_I2C_DEFINE)
