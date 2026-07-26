#define DT_DRV_COMPAT promini_trackpoint_i2c

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zmk/keymap.h>

LOG_MODULE_REGISTER(trackpoint_i2c, CONFIG_TRACKPOINT_I2C_LOG_LEVEL);

#define BURST_SIZE    2
#define BURST_ADDR    0x12

#define INPUT_EV_REL    0x02
#define INPUT_REL_X     0x00
#define INPUT_REL_Y     0x01

#define SWAP_XY   1
#define INVERT_X  1
#define INVERT_Y  1

struct trackpoint_i2c_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec irq_gpio;
    int8_t layer_toggle;
    int32_t layer_toggle_timeout_ms;
};

struct trackpoint_i2c_data {
    const struct device *dev;
    struct gpio_callback irq_gpio_cb;
    struct k_work_delayable poll_work;
    int64_t dx;
    int64_t dy;
    uint8_t zero_count;
#if CONFIG_TRACKPOINT_I2C_LAYER_TOGGLE
    bool layer_toggle_layer_enabled;
    int64_t layer_toggle_last_motion_time;
    struct k_work_delayable layer_toggle_deactivation_work;
#endif
};

#if CONFIG_TRACKPOINT_I2C_LAYER_TOGGLE
static void trackpoint_i2c_layer_toggle_deactivate(struct k_work *item) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(item);
    struct trackpoint_i2c_data *data =
        CONTAINER_OF(dwork, struct trackpoint_i2c_data, layer_toggle_deactivation_work);
    const struct device *dev = data->dev;
    const struct trackpoint_i2c_config *cfg = dev->config;

    LOG_INF("Deactivating layer %d (no motion for %lldms)",
            cfg->layer_toggle,
            k_uptime_get() - data->layer_toggle_last_motion_time);

    if (zmk_keymap_layer_active(cfg->layer_toggle)) {
        zmk_keymap_layer_deactivate(cfg->layer_toggle, false);
    }
    data->layer_toggle_layer_enabled = false;
}
#endif

static void trackpoint_i2c_poll(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct trackpoint_i2c_data *data = CONTAINER_OF(dwork, struct trackpoint_i2c_data, poll_work);
    const struct trackpoint_i2c_config *cfg = data->dev->config;

    uint8_t addr = BURST_ADDR;
    uint8_t buf[BURST_SIZE];

    int ret = i2c_write_read_dt(&cfg->i2c, &addr, 1, buf, BURST_SIZE);
    if (ret == 0) {
        int8_t rawx = (int8_t)buf[0];
        int8_t rawy = (int8_t)buf[1];
#if SWAP_XY
        int8_t t = rawx; rawx = rawy; rawy = t;
#endif
        int8_t x = INVERT_X ? -rawx : rawx;
        int8_t y = INVERT_Y ? -rawy : rawy;

        if (rawx == 0 && rawy == 0) {
            if (++data->zero_count >= 3) {
                data->dx = 0;
                data->dy = 0;
            }
        } else {
            data->zero_count = 0;
        }

        data->dx += x;
        data->dy += y;

        if (data->dx != 0 || data->dy != 0) {
            input_report(data->dev, INPUT_EV_REL, INPUT_REL_X, data->dx, false, K_NO_WAIT);
            input_report(data->dev, INPUT_EV_REL, INPUT_REL_Y, data->dy, true, K_NO_WAIT);
            data->dx = 0;
            data->dy = 0;

#if CONFIG_TRACKPOINT_I2C_LAYER_TOGGLE
            if (cfg->layer_toggle >= 0 && (x != 0 || y != 0)) {
                data->layer_toggle_last_motion_time = k_uptime_get();
                if (!data->layer_toggle_layer_enabled) {
                    LOG_INF("Activating layer %d on motion (x=%d y=%d)",
                            cfg->layer_toggle, x, y);
                    zmk_keymap_layer_activate(cfg->layer_toggle, false);
                    data->layer_toggle_layer_enabled = true;
                }
                k_work_reschedule(&data->layer_toggle_deactivation_work,
                                  K_MSEC(cfg->layer_toggle_timeout_ms));
            }
#endif
        }
    } else {
        LOG_WRN("I2C read failed: %d", ret);
    }

    k_work_schedule(&data->poll_work, K_MSEC(10));
}

static void trackpoint_i2c_gpio_callback(const struct device *gpiob,
                                          struct gpio_callback *cb, uint32_t pins) {
    (void)gpiob; (void)cb; (void)pins;
}

static int trackpoint_i2c_init(const struct device *dev) {
    struct trackpoint_i2c_data *data = dev->data;
    const struct trackpoint_i2c_config *cfg = dev->config;

    data->dev = dev;
    data->dx = 0;
    data->dy = 0;
    data->zero_count = 0;

#if CONFIG_TRACKPOINT_I2C_LAYER_TOGGLE
    data->layer_toggle_layer_enabled = false;
    data->layer_toggle_last_motion_time = 0;
    k_work_init_delayable(&data->layer_toggle_deactivation_work,
                          trackpoint_i2c_layer_toggle_deactivate);
#endif

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

    uint8_t tst_addr = 0x00;
    uint8_t tst_val = 0;
    int tst_ret = i2c_write_read_dt(&cfg->i2c, &tst_addr, 1, &tst_val, 1);
    if (tst_ret == 0) {
        printk("I2C INIT OK: PID=0x%02x\n", tst_val);
    } else {
        printk("I2C INIT FAIL: %d\n", tst_ret);
    }

    k_work_init_delayable(&data->poll_work, trackpoint_i2c_poll);
    k_work_schedule(&data->poll_work, K_MSEC(100));

    LOG_INF("trackpoint-i2c Exp18 raw int8 initialized");
    return 0;
}

#define TRACKPOINT_I2C_DEFINE(n)                                                        \
    static struct trackpoint_i2c_data data##n;                                          \
    static const struct trackpoint_i2c_config config##n = {                             \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                 \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                \
        .layer_toggle = DT_PROP(DT_DRV_INST(n), layer_toggle),                          \
        .layer_toggle_timeout_ms = DT_PROP(DT_DRV_INST(n), layer_toggle_timeout_ms),    \
    };                                                                                  \
    DEVICE_DT_INST_DEFINE(n, trackpoint_i2c_init, NULL, &data##n, &config##n,           \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_I2C_DEFINE)
