zephyr_library()

zephyr_library_sources_ifdef(CONFIG_PMW3610_ALT src/pmw3610.c)
zephyr_library_sources_ifdef(CONFIG_TRACKPOINT_I2C src/trackpoint-i2c.c)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
