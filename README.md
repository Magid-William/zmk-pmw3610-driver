# ZMK Trackpoint Driver

I2C trackpoint driver for ZMK. It talks to a Pro Mini (or ATtiny85) that acts
as an I2C slave and reads a PS/2 trackpoint, so ZMK gets X/Y motion without a
physical optical sensor.

Compatible node: `promini,trackpoint-i2c`

## Devicetree

```dts
trackball: trackball@42 {
    compatible = "promini,trackpoint-i2c";
    reg = <0x42>;
};
```

## Properties

| Property            | Description                                            |
|---------------------|--------------------------------------------------------|
| `irq-gpios`         | MOT data-ready pin (optional; absent = plain polling)  |
| `reset-gpios`       | Reset pin pulse on init (optional)                     |
| `swap-xy`           | Swap X/Y axes                                          |
| `invert-x` / `invert-y` | Invert an axis                                     |
| `speed-scale`       | Speed scaling factor 1-255 (255 = 100%)                |
| `curve-rate`        | PowerCurve rate, Q8.8                                  |
| `curve-exponent`    | PowerCurve exponent, Q8.8                              |
| `curve-start`       | PowerCurve output offset, Q8.8                         |

## Integrating into a ZMK shield

The driver is a standard Zephyr module: fetch it (west manifest), instantiate it
(devicetree), enable it (Kconfig), then wire its input events into ZMK pointing.

### 1. Fetch the module — `config/west.yml`

```yaml
manifest:
  remotes:
    - name: maged
      url-base: https://github.com/Magid-William
  projects:
    - name: zmk-trackpoint-driver
      remote: maged
      revision: e5234665db3ecc88c6fc1108a0b4214103b6f27e
```

Pin a revision **SHA**, not a branch, so builds are reproducible and each
experiment records exactly which driver it was tested against. Because
`zephyr/module.yml` declares the module roots (`cmake: .`, `kconfig: Kconfig`,
`dts_root: .`), the build system auto-discovers the driver, its binding, and its
`TRACKPOINT_I2C` Kconfig symbol — no CMake or Kconfig changes in the shield.

### 2. Instantiate the device — shield overlay

Add a `promini,trackpoint-i2c` node on an enabled I2C bus (the Pro Mini sits at
address `0x42`):

```dts
&i2c0 {
    compatible = "nordic,nrf-twim";
    status = "okay";
    pinctrl-0 = <&i2c0_default>;
    pinctrl-1 = <&i2c0_sleep>;
    pinctrl-names = "default", "sleep";

    trackball: trackball@42 {
        compatible = "promini,trackpoint-i2c";
        reg = <0x42>;
        // Optional tuning:
        // irq-gpios = <&gpio0 6 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;  // MOT data-ready
        // swap-xy;
        // speed-scale = <128>;   // 50% speed
        // curve-rate = <18>; curve-exponent = <256>; curve-start = <77>;
    };
};
```

- **No `irq-gpios` → plain 10 ms polling.** MOT-IRQ only makes sense while the
  slave is powered; the typical power model below leaves it unpowered in sleep,
  so polling is the right choice.
- All properties are optional; defaults live in `promini,trackpoint-i2c.yml`.

### 3. Enable driver + dependencies — shield `.conf`

```ini
CONFIG_I2C=y
CONFIG_TRACKPOINT_I2C=y
CONFIG_INPUT=y
CONFIG_ZMK_POINTING=y
# debug:
CONFIG_I2C_SHELL=y
CONFIG_SHELL=y
CONFIG_KERNEL_SHELL=y
CONFIG_LOG=y
CONFIG_LOG_PRINTK=y
```

`CONFIG_TRACKPOINT_I2C` depends on `DT_HAS_PROMINI_TRACKPOINT_I2C_ENABLED`, so
it only builds when the node exists.

### 4. Wire input into ZMK pointing

The driver only emits Zephyr input events; nothing moves the cursor until
they're consumed. Standalone (or central half), hook a `zmk,input-listener`
straight to the device:

```dts
trackball_listener: trackball_listener {
    compatible = "zmk,input-listener";
    status = "okay";
    device = <&trackball>;                    // or <&trackball_split> on a peripheral
    input-processors = <&temp_layer 2 2000>;  // optional
};
```

In a split topology the peripheral forwards events over BLE via a
`zmk,input-split` node, and the listener's `device` points at the split
instead. Optional `input-processors` (layer-toggle, report-rate-limit, ...)
can be attached to the listener.

### 5. Build

`build.yaml` entry, e.g.:

```yaml
include:
  - board: nice_nano//zmk
    shield: corne_trackpoint_right
    snippet: zmk-usb-logging
    artifact-name: corne_trackpoint_right
```

### Hardware contract

The NiceNano is the I2C **master**; the Pro Mini is the **slave** at `0x42`:

| NiceNano | Pro Mini | Note |
|----------|----------|------|
| P0.17 | D18 (A4) | SDA, 4.7kΩ pull-up to 3.3V |
| P0.20 | D19 (A5) | SCL, 4.7kΩ pull-up to 3.3V |
| GND | GND | common ground |

Power: the Pro Mini is powered from the nice_nano VCC rail (P0.13 switch). No
power-gating circuitry is needed — when ZMK enters deep sleep
(`CONFIG_ZMK_SLEEP=y`) the rail drops and the Pro Mini draws nothing (verified
with a multimeter).
