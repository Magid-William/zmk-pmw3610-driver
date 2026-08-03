#define DT_DRV_COMPAT promini_trackpoint_i2c

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(trackpoint_i2c, CONFIG_TRACKPOINT_I2C_LOG_LEVEL);

#define BURST_SIZE    2
#define BURST_ADDR    0x12
#define DEBUG_ADDR    0x03
#define DEBUG_SIZE    5
#define SPEED_REG     0x11

#define INPUT_EV_REL    0x02
#define INPUT_REL_X     0x00
#define INPUT_REL_Y     0x01

struct trackpoint_i2c_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec irq_gpio;
    struct gpio_dt_spec reset_gpio;
    bool swap_xy;
    bool invert_x;
    bool invert_y;
    uint8_t speed_scale;
};

struct trackpoint_i2c_data {
    const struct device *dev;
    struct gpio_callback irq_gpio_cb;
    struct k_work_delayable poll_work;
    int64_t dx;
    int64_t dy;
    uint8_t zero_count;
    uint32_t prev_poll_ms;
    uint8_t consecutive_errors;
    uint32_t poll_interval_ms;
};

static void trackpoint_i2c_poll(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct trackpoint_i2c_data *data = CONTAINER_OF(dwork, struct trackpoint_i2c_data, poll_work);
    const struct trackpoint_i2c_config *cfg = data->dev->config;

    int mot = gpio_pin_get_dt(&cfg->irq_gpio);
    if (mot < 0) {
        LOG_ERR("MOT pin read failed: %d", mot);
        k_work_schedule(&data->poll_work, K_MSEC(100));
        return;
    }
    if (mot != 0) {
        LOG_DBG("poll: Pro Mini sleeping (MOT logical HIGH / physically LOW), stopping");
        return;
    }

    uint32_t now_ms = k_uptime_get();
    uint32_t delta_ms = data->prev_poll_ms ? (now_ms - data->prev_poll_ms) : 0;
    data->prev_poll_ms = now_ms;

    uint8_t addr = BURST_ADDR;
    uint8_t buf[BURST_SIZE];

    LOG_DBG("poll start interval=%ums", delta_ms);

    /* Exp44: separate write (with STOP) + read (fresh START) transactions.
     * The nRF TWIM combined transaction (repeated START) does not deliver the
     * register byte reliably to the ATtiny85 USI slave — every read came back
     * served from the burst register. */
    int ret = i2c_write_dt(&cfg->i2c, &addr, 1);
    if (ret == 0) {
        ret = i2c_read_dt(&cfg->i2c, buf, BURST_SIZE);
    }
    if (ret == 0) {
        data->consecutive_errors = 0;
        data->poll_interval_ms = 10;

        int8_t rawx = (int8_t)buf[0];
        int8_t rawy = (int8_t)buf[1];

        LOG_INF("raw sign-extended: x=%d y=%d", (int)rawx, (int)rawy);

        if (cfg->swap_xy) {
            int8_t t = rawx; rawx = rawy; rawy = t;
            LOG_DBG("after SWAP_XY: x=%d y=%d", (int)rawx, (int)rawy);
        }
        int8_t x = cfg->invert_x ? -rawx : rawx;
        int8_t y = cfg->invert_y ? -rawy : rawy;
        LOG_DBG("after INVERT: x=%d y=%d", (int)x, (int)y);

        if (rawx == 0 && rawy == 0) {
            data->zero_count++;
            LOG_DBG("zero read ++zero_count=%u", data->zero_count);
            if (data->zero_count >= 3) {
                LOG_DBG("stale timeout: clearing dx/dy (was dx=%d dy=%d)", (int)data->dx, (int)data->dy);
                data->dx = 0;
                data->dy = 0;
            }
        } else {
            data->zero_count = 0;
        }

        data->dx += x;
        data->dy += y;
        LOG_DBG("accumulator: dx=%d dy=%d zero_count=%u", (int)data->dx, (int)data->dy, data->zero_count);

        if (data->dx != 0 || data->dy != 0) {
            LOG_INF("SEND: dev=%s ev=REL type=REL_%s val=%d",
                    data->dev->name,
                    "X", (int)data->dx);
            input_report(data->dev, INPUT_EV_REL, INPUT_REL_X, data->dx, false, K_NO_WAIT);
            LOG_INF("SEND: dev=%s ev=REL type=REL_%s val=%d sync",
                    data->dev->name,
                    "Y", (int)data->dy);
            input_report(data->dev, INPUT_EV_REL, INPUT_REL_Y, data->dy, true, K_NO_WAIT);
            data->dx = 0;
            data->dy = 0;
        } else {
            LOG_DBG("no motion to report");
        }
    } else {
        if (data->consecutive_errors < 200) {
            data->consecutive_errors++;
        }
        LOG_ERR("I2C read burst failed: %d (addr=0x%02x len=%d consecutive=%u)",
                ret, BURST_ADDR, BURST_SIZE, data->consecutive_errors);
        if (data->consecutive_errors >= 50) {
            data->poll_interval_ms = 5000;
        } else if (data->consecutive_errors >= 10) {
            data->poll_interval_ms = 1000;
        } else if (data->consecutive_errors >= 3) {
            data->poll_interval_ms = 100;
        } else {
            data->poll_interval_ms = 10;
        }
    }

    k_work_schedule(&data->poll_work, K_MSEC(data->poll_interval_ms));
}

static void trackpoint_i2c_gpio_callback(const struct device *gpiob,
                                          struct gpio_callback *cb, uint32_t pins) {
    struct trackpoint_i2c_data *data = CONTAINER_OF(cb, struct trackpoint_i2c_data, irq_gpio_cb);
    const struct trackpoint_i2c_config *cfg = data->dev->config;

    int mot = gpio_pin_get_dt(&cfg->irq_gpio);
    if (mot < 0) {
        LOG_ERR("MOT pin read failed: %d", mot);
        return;
    }

    if (mot != 0) {
        LOG_INF("sleep: MOT active (low), cancelling poll work");
        k_work_cancel_delayable(&data->poll_work);
    } else {
        LOG_INF("wake: MOT inactive (high), resuming poll work");
        k_work_schedule(&data->poll_work, K_MSEC(10));
    }
}

static int trackpoint_i2c_init(const struct device *dev) {
    struct trackpoint_i2c_data *data = dev->data;
    const struct trackpoint_i2c_config *cfg = dev->config;

    data->dev = dev;
    data->dx = 0;
    data->dy = 0;
    data->zero_count = 0;
    data->prev_poll_ms = 0;
    data->consecutive_errors = 0;
    data->poll_interval_ms = 10;

    k_work_init_delayable(&data->poll_work, trackpoint_i2c_poll);

    LOG_INF("init start");

    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }
    LOG_DBG("I2C bus ready: %s", cfg->i2c.bus->name);

    if (!device_is_ready(cfg->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }
    LOG_DBG("IRQ GPIO ready: %s pin=%d", cfg->irq_gpio.port->name, cfg->irq_gpio.pin);

    int ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
    if (ret) {
        LOG_ERR("Cannot configure IRQ GPIO: %d", ret);
        return ret;
    }

    if (!device_is_ready(cfg->reset_gpio.port)) {
        LOG_ERR("Reset GPIO device not ready");
        return -ENODEV;
    }
    LOG_DBG("Reset GPIO ready: %s pin=%d", cfg->reset_gpio.port->name, cfg->reset_gpio.pin);

    LOG_INF("asserting reset pin (100ms LOW)");
    gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
    k_msleep(100);
    LOG_INF("releasing reset pin (input+pull-up, 500ms boot wait)");
    gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_INPUT | GPIO_PULL_UP);
    k_msleep(500);

    int mot = gpio_pin_get_dt(&cfg->irq_gpio);
    if (mot < 0) {
        LOG_ERR("MOT pin read failed at init: %d", mot);
    } else {
        LOG_INF("MOT level at init: %d (active-low: 1=sleeping, 0=awake)", mot);
    }

    if (mot == 0) {
        LOG_INF("probing I2C at 0x%02x", BURST_ADDR);
        uint8_t tst_addr = 0x00;
        uint8_t tst_val = 0;
        int tst_ret = i2c_write_dt(&cfg->i2c, &tst_addr, 1);
        if (tst_ret == 0) {
            tst_ret = i2c_read_dt(&cfg->i2c, &tst_val, 1);
        }
        if (tst_ret == 0) {
            LOG_INF("I2C probe OK: reg[0x00]=0x%02x", tst_val);
        } else {
            LOG_ERR("I2C probe FAILED: %d", tst_ret);
        }

        LOG_INF("setting speed_scale=%u", cfg->speed_scale);
        uint8_t spd_wbuf[2] = { SPEED_REG, cfg->speed_scale };
        ret = i2c_write_dt(&cfg->i2c, spd_wbuf, 2);
        if (ret) {
            LOG_ERR("speed_scale write failed: %d", ret);
        }
    } else {
        LOG_INF("Pro Mini sleeping at init — skipping probe/speed, waiting for wake edge");
    }

    gpio_init_callback(&data->irq_gpio_cb, trackpoint_i2c_gpio_callback, BIT(cfg->irq_gpio.pin));
    gpio_add_callback(cfg->irq_gpio.port, &data->irq_gpio_cb);
    ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_BOTH);
    if (ret) {
        LOG_ERR("Cannot configure IRQ GPIO interrupt: %d", ret);
        return ret;
    }

    if (mot == 0) {
        LOG_INF("scheduling poll work (100ms first shot, 10ms thereafter)");
        k_work_schedule(&data->poll_work, K_MSEC(100));
    } else {
        LOG_INF("poll held — wake edge will resume it");
    }

    LOG_INF("init complete");
    return 0;
}

#define TRACKPOINT_I2C_DEFINE(n)                                                        \
    static struct trackpoint_i2c_data data##n;                                          \
    static const struct trackpoint_i2c_config config##n = {                             \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                 \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                \
        .reset_gpio = GPIO_DT_SPEC_INST_GET(n, reset_gpios),                            \
        .swap_xy = DT_INST_PROP(n, swap_xy),                                            \
        .invert_x = DT_INST_PROP(n, invert_x),                                          \
        .invert_y = DT_INST_PROP(n, invert_y),                                          \
        .speed_scale = DT_INST_PROP(n, speed_scale),                                    \
    };                                                                                  \
    DEVICE_DT_INST_DEFINE(n, trackpoint_i2c_init, NULL, &data##n, &config##n,           \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_I2C_DEFINE)
