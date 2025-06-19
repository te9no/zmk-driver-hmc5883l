/*
 * Copyright (c) 2023 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ZEPHYR_DRIVERS_INPUT_INPUT_HMC5883L_H_
#define ZEPHYR_DRIVERS_INPUT_INPUT_HMC5883L_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

/* HMC5883L Register Addresses */
#define HMC5883L_REG_CONFIG_A           0x00
#define HMC5883L_REG_CONFIG_B           0x01
#define HMC5883L_REG_MODE               0x02
#define HMC5883L_REG_DATA_START         0x03
#define HMC5883L_REG_CHIP_ID            0x0A

/* Configuration Values */
#define HMC5883L_MODE_CONTINUOUS        0x00
#define HMC5883L_ODR_75HZ               0x18
#define HMC5883L_GAIN_1_3GA             0x20

/* Chip ID Values */
#define HMC5883L_CHIP_ID_A              'H'
#define HMC5883L_CHIP_ID_B              '4'
#define HMC5883L_CHIP_ID_C              '3'

/* Configuration Constants */
#define AUTO_RECALIBRATION_TIMEOUT_MS   30000
#define HYSTERESIS_THRESHOLD            20
#define DEADZONE_X                      3
#define DEADZONE_Y                      3
#define DEADZONE_Z                      5

struct hmc5883l_config {
    struct i2c_dt_spec i2c;
    uint32_t polling_interval_ms;
};

struct hmc5883l_data {
    const struct device *dev;
    struct k_work_delayable work;
    
    /* Sensor readings */
    int16_t x_raw, y_raw, z_raw;
    
    /* Calibration values */
    int16_t x_offset, y_offset, z_offset;
    
    /* Previous values for movement detection */
    int16_t x_prev, y_prev, z_prev;
    
    /* Auto-calibration */
    int64_t last_movement_time;
    bool calibration_pending;
    
    /* Calibration sample collection */
    int16_t cal_samples_x[100];
    int16_t cal_samples_y[100];
    int16_t cal_samples_z[100];
    int cal_sample_count;
};

#endif /* ZEPHYR_DRIVERS_INPUT_INPUT_HMC5883L_H_ */