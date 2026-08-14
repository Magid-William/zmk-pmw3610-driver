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
