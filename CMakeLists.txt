zephyr_library()

zephyr_library_sources_ifdef(CONFIG_TRACKPOINT_I2C src/trackpoint-i2c.c)
