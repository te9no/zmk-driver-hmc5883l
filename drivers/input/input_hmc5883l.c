/*
 * Copyright (c) 2023 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_hmc5883l_input

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "input_hmc5883l.h"

LOG_MODULE_REGISTER(hmc5883l, CONFIG_INPUT_HMC5883L_LOG_LEVEL);

static int hmc5883l_read_sensor(const struct device *dev, int16_t *x, int16_t *y, int16_t *z)
{
    const struct hmc5883l_config *config = dev->config;
    uint8_t buf[6];
    int ret;

    ret = i2c_burst_read_dt(&config->i2c, HMC5883L_REG_DATA_START, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to read sensor data: %d", ret);
        return ret;
    }

    /* HMC5883L data order: X, Z, Y (note Z and Y are swapped) */
    *x = sys_be16_to_cpu((buf[0] << 8) | buf[1]);
    *z = sys_be16_to_cpu((buf[2] << 8) | buf[3]);
    *y = sys_be16_to_cpu((buf[4] << 8) | buf[5]);

    return 0;
}

static bool is_movement_detected(int16_t current, int16_t previous, int deadzone)
{
    int16_t diff = current - previous;
    return (diff < -deadzone || diff > deadzone);
}

static void hmc5883l_auto_calibrate(const struct device *dev)
{
    struct hmc5883l_data *data = dev->data;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    
    LOG_INF("Starting auto-calibration with %d samples", data->cal_sample_count);
    
    for (int i = 0; i < data->cal_sample_count; i++) {
        sum_x += data->cal_samples_x[i];
        sum_y += data->cal_samples_y[i];
        sum_z += data->cal_samples_z[i];
    }
    
    data->x_offset = sum_x / data->cal_sample_count;
    data->y_offset = sum_y / data->cal_sample_count;
    data->z_offset = sum_z / data->cal_sample_count;
    
    LOG_INF("Calibration complete: offset X=%d, Y=%d, Z=%d", 
            data->x_offset, data->y_offset, data->z_offset);
    
    data->calibration_pending = false;
    data->cal_sample_count = 0;
}

static void hmc5883l_work_handler(struct k_work *work)
{
    struct k_work_delayable *work_delayable = k_work_delayable_from_work(work);
    struct hmc5883l_data *data = CONTAINER_OF(work_delayable, struct hmc5883l_data, work);
    const struct device *dev = data->dev;
    const struct hmc5883l_config *config = dev->config;
    int16_t x_raw, y_raw, z_raw;
    int16_t x_cal, y_cal, z_cal;
    int16_t delta_x, delta_y, delta_z;
    bool movement_detected = false;
    int64_t now = k_uptime_get();
    int ret;

    ret = hmc5883l_read_sensor(dev, &x_raw, &y_raw, &z_raw);
    if (ret < 0) {
        goto reschedule;
    }

    /* Apply calibration offset */
    x_cal = x_raw - data->x_offset;
    y_cal = y_raw - data->y_offset;
    z_cal = z_raw - data->z_offset;

    /* Check for movement */
    if (is_movement_detected(x_cal, data->x_prev, DEADZONE_X) ||
        is_movement_detected(y_cal, data->y_prev, DEADZONE_Y) ||
        is_movement_detected(z_cal, data->z_prev, DEADZONE_Z)) {
        movement_detected = true;
        data->last_movement_time = now;
    }

    /* Auto-calibration logic */
    if (!movement_detected && 
        (now - data->last_movement_time) > AUTO_RECALIBRATION_TIMEOUT_MS) {
        
        if (!data->calibration_pending) {
            LOG_INF("No movement detected for %d ms, starting calibration", 
                    AUTO_RECALIBRATION_TIMEOUT_MS);
            data->calibration_pending = true;
            data->cal_sample_count = 0;
        }
        
        /* Collect calibration samples */
        if (data->cal_sample_count < ARRAY_SIZE(data->cal_samples_x)) {
            data->cal_samples_x[data->cal_sample_count] = x_raw;
            data->cal_samples_y[data->cal_sample_count] = y_raw;
            data->cal_samples_z[data->cal_sample_count] = z_raw;
            data->cal_sample_count++;
        }
        
        /* Complete calibration when enough samples collected */
        if (data->cal_sample_count >= ARRAY_SIZE(data->cal_samples_x)) {
            hmc5883l_auto_calibrate(dev);
        }
        
        goto reschedule;
    }

    /* Calculate movement deltas */
    delta_x = x_cal - data->x_prev;
    delta_y = y_cal - data->y_prev;
    delta_z = z_cal - data->z_prev;

    /* Apply hysteresis to reduce noise */
    if (abs(delta_x) < HYSTERESIS_THRESHOLD) delta_x = 0;
    if (abs(delta_y) < HYSTERESIS_THRESHOLD) delta_y = 0;
    if (abs(delta_z) < HYSTERESIS_THRESHOLD) delta_z = 0;

    /* Report input events */
    if (delta_x != 0) {
        input_report_rel(dev, INPUT_REL_X, delta_x, false, K_NO_WAIT);
        LOG_DBG("X movement: %d", delta_x);
    }
    
    if (delta_y != 0) {
        input_report_rel(dev, INPUT_REL_Y, delta_y, false, K_NO_WAIT);
        LOG_DBG("Y movement: %d", delta_y);
    }
    
    if (abs(delta_z) > CONFIG_INPUT_HMC5883L_Z_THRESHOLD) {
        /* Convert Z movement to scroll wheel */
        int8_t scroll = (delta_z > 0) ? 1 : -1;
        input_report_rel(dev, INPUT_REL_WHEEL, scroll, false, K_NO_WAIT);
        LOG_DBG("Z scroll: %d (raw: %d)", scroll, delta_z);
    }

    /* Sync input events */
    if (delta_x != 0 || delta_y != 0 || abs(delta_z) > CONFIG_INPUT_HMC5883L_Z_THRESHOLD) {
        input_report_rel(dev, INPUT_REL_X, 0, true, K_NO_WAIT);
    }

    /* Update previous values */
    data->x_prev = x_cal;
    data->y_prev = y_cal;
    data->z_prev = z_cal;

reschedule:
    k_work_reschedule(&data->work, K_MSEC(config->polling_interval_ms));
}

static int hmc5883l_init(const struct device *dev)
{
    const struct hmc5883l_config *config = dev->config;
    struct hmc5883l_data *data = dev->data;
    uint8_t chip_id[3];
    uint8_t reg_val;
    int ret;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    /* Verify chip ID */
    ret = i2c_burst_read_dt(&config->i2c, HMC5883L_REG_CHIP_ID, chip_id, sizeof(chip_id));
    if (ret < 0) {
        LOG_ERR("Failed to read chip ID: %d", ret);
        return ret;
    }

    if (chip_id[0] != HMC5883L_CHIP_ID_A || 
        chip_id[1] != HMC5883L_CHIP_ID_B || 
        chip_id[2] != HMC5883L_CHIP_ID_C) {
        LOG_ERR("Invalid chip ID: %c%c%c", chip_id[0], chip_id[1], chip_id[2]);
        return -EINVAL;
    }

    LOG_INF("HMC5883L chip ID verified: %c%c%c", chip_id[0], chip_id[1], chip_id[2]);

    /* Configure sensor - 75Hz output rate */
    reg_val = HMC5883L_ODR_75HZ;
    ret = i2c_reg_write_byte_dt(&config->i2c, HMC5883L_REG_CONFIG_A, reg_val);
    if (ret < 0) {
        LOG_ERR("Failed to configure Config A: %d", ret);
        return ret;
    }

    /* Configure gain - 1.3 Gauss */
    reg_val = HMC5883L_GAIN_1_3GA;
    ret = i2c_reg_write_byte_dt(&config->i2c, HMC5883L_REG_CONFIG_B, reg_val);
    if (ret < 0) {
        LOG_ERR("Failed to configure Config B: %d", ret);
        return ret;
    }

    /* Set continuous measurement mode */
    reg_val = HMC5883L_MODE_CONTINUOUS;
    ret = i2c_reg_write_byte_dt(&config->i2c, HMC5883L_REG_MODE, reg_val);
    if (ret < 0) {
        LOG_ERR("Failed to set continuous mode: %d", ret);
        return ret;
    }

    /* Initialize data structure */
    data->dev = dev;
    data->last_movement_time = k_uptime_get();
    data->calibration_pending = false;
    data->cal_sample_count = 0;
    
    /* Initialize calibration offsets to zero */
    data->x_offset = 0;
    data->y_offset = 0;
    data->z_offset = 0;

    /* Read initial values */
    ret = hmc5883l_read_sensor(dev, &data->x_prev, &data->y_prev, &data->z_prev);
    if (ret < 0) {
        LOG_ERR("Failed to read initial sensor values: %d", ret);
        return ret;
    }

    /* Apply initial calibration offset */
    data->x_prev -= data->x_offset;
    data->y_prev -= data->y_offset;
    data->z_prev -= data->z_offset;

    /* Initialize work queue */
    k_work_init_delayable(&data->work, hmc5883l_work_handler);

    /* Start polling */
    k_work_reschedule(&data->work, K_MSEC(config->polling_interval_ms));

    LOG_INF("HMC5883L input driver initialized");
    return 0;
}

#define HMC5883L_INIT(inst)                                                     \
    static struct hmc5883l_data hmc5883l_data_##inst;                          \
                                                                                \
    static const struct hmc5883l_config hmc5883l_config_##inst = {             \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                     \
        .polling_interval_ms = DT_INST_PROP_OR(inst, polling_interval_ms,      \
                                               CONFIG_INPUT_HMC5883L_POLLING_INTERVAL_MS), \
    };                                                                          \
                                                                                \
    DEVICE_DT_INST_DEFINE(inst, hmc5883l_init, NULL,                           \
                          &hmc5883l_data_##inst, &hmc5883l_config_##inst,      \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(HMC5883L_INIT)