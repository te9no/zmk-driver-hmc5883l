# ZMK HMC5883L Input Driver

A ZMK driver module that uses the HMC5883L 3-axis magnetometer as an input device, converting magnetic sensor readings into mouse movement and scroll events.

## Features

- **Mouse Input Conversion**: Converts HMC5883L X/Y axis readings to mouse movement (INPUT_REL_X, INPUT_REL_Y)
- **Scroll Functionality**: Converts Z-axis changes to mouse wheel scroll (INPUT_REL_WHEEL)
- **Auto-Calibration**: Automatically adjusts sensor baseline values after 30 seconds of no movement detected
- **Noise Filtering**: Reduces noise through deadzone and hysteresis functionality
- **Configurable Polling**: Sensor reading interval configurable via device tree (default: 10ms)

## Installation

### 1. Add Module

Add the module to your `config/west.yml` file:

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: your-username
      url-base: https://github.com/your-username
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml
    - name: zmk-driver-hmc5883l
      remote: your-username
      revision: main
      path: modules/zmk-driver-hmc5883l
  self:
    path: config
```

### 2. Device Tree Configuration

Define the HMC5883L sensor in your shield or board `.overlay` file:

```dts
&i2c0 {
    status = "okay";
    
    hmc5883l_input: hmc5883l@1e {
        compatible = "zmk,hmc5883l-input";
        reg = <0x1e>;
        polling-interval-ms = <10>;
    };
};

/ {
    hmc5883l_listener {
        compatible = "zmk,input-listener";
        device = <&hmc5883l_input>;
    };
};
```

### 3. Configuration Options

Adjust settings in your `.conf` file as needed:

```
# HMC5883L input driver
CONFIG_INPUT_HMC5883L=y
CONFIG_INPUT_HMC5883L_POLLING_INTERVAL_MS=10
CONFIG_INPUT_HMC5883L_Z_THRESHOLD=5

# Debug options (optional)
CONFIG_INPUT_HMC5883L_LOG_LEVEL_DBG=y
CONFIG_INPUT_HMC5883L_DEBUG_BAR_GRAPH=y

# I2C support
CONFIG_I2C=y

# Input subsystem
CONFIG_INPUT=y
```

## Configuration Parameters

### Kconfig Options

- `CONFIG_INPUT_HMC5883L`: Enable the driver (default: y)
- `CONFIG_INPUT_HMC5883L_POLLING_INTERVAL_MS`: Polling interval (default: 10ms)
- `CONFIG_INPUT_HMC5883L_Z_THRESHOLD`: Z-axis scroll detection threshold (default: 5)
- `CONFIG_INPUT_HMC5883L_DEBUG_BAR_GRAPH`: Enable bar graph debug output (default: n)
- `CONFIG_INPUT_HMC5883L_LOG_LEVEL_DBG`: Enable debug logging (default: n)

### Device Tree Properties

- `polling-interval-ms`: Sensor reading interval (default: 10ms)

## Technical Specifications

### Hardware Requirements

- HMC5883L 3-axis magnetometer sensor
- I2C communication support
- 3.3V power supply

### Sensor Configuration

- **Output Data Rate**: 75Hz
- **Measurement Range**: ±1.3 Gauss
- **Operating Mode**: Continuous measurement mode

### Processing Parameters

- **Deadzone**: X/Y axes: ±3, Z axis: ±5
- **Hysteresis Threshold**: 20
- **Auto-Calibration**: Triggered after 30 seconds of no movement
- **Calibration Samples**: 100 samples

## Usage Examples

### Basic Mouse Input

```dts
&i2c0 {
    hmc5883l_input: hmc5883l@1e {
        compatible = "zmk,hmc5883l-input";
        reg = <0x1e>;
        polling-interval-ms = <10>;
    };
};

/ {
    hmc5883l_listener {
        compatible = "zmk,input-listener";
        device = <&hmc5883l_input>;
    };
};
```

### Custom Polling Interval

```dts
hmc5883l_input: hmc5883l@1e {
    compatible = "zmk,hmc5883l-input";
    reg = <0x1e>;
    polling-interval-ms = <5>;  // Poll every 5ms
};
```

## Troubleshooting

### Build Errors

1. **I2C not enabled**:
   ```
   CONFIG_I2C=y
   ```

2. **Input subsystem disabled**:
   ```
   CONFIG_INPUT=y
   ```

3. **Device tree errors**: Verify I2C address (0x1e) is correctly configured

### Operation Verification

1. **Enable debug logging**:
   ```
   CONFIG_INPUT_HMC5883L_LOG_LEVEL_DBG=y
   ```

2. **Enable bar graph debug output**:
   ```
   CONFIG_INPUT_HMC5883L_DEBUG_BAR_GRAPH=y
   ```
   
   This will display sensor readings as ASCII bar graphs:
   ```
   Sensor readings:
   X:   120 [###########.............................]
   Y:   -45 [##########..............................]
   Z:    89 [############............................]
   ────────────────────────────────────────────────
   ```

3. **Sensor initialization**: Chip ID verification message appears at startup
4. **Calibration**: Calibration completion message appears after 30 seconds

## Hardware Wiring

### I2C Connection

| HMC5883L Pin | MCU Pin | Description |
|--------------|---------|-------------|
| VCC          | 3.3V    | Power supply |
| GND          | GND     | Ground |
| SCL          | SCL     | I2C Clock |
| SDA          | SDA     | I2C Data |

### I2C Address

The HMC5883L uses I2C address `0x1E` (7-bit addressing).

## Algorithm Details

### Movement Detection

1. **Read sensor data** every polling interval
2. **Apply calibration offset** to raw readings
3. **Calculate movement delta** from previous readings
4. **Apply deadzone filtering** to eliminate noise
5. **Apply hysteresis threshold** for movement detection
6. **Convert to input events** and report to ZMK

### Auto-Calibration Process

1. **Monitor movement** continuously
2. **Start calibration** when no movement detected for 30 seconds
3. **Collect 100 samples** during calibration period
4. **Calculate average** of collected samples
5. **Update offset values** and resume normal operation

## License

This project is released under the MIT License.

## Contributing

Bug reports and feature requests are welcome via GitHub Issues.

## Related Links

- [ZMK Firmware](https://zmk.dev/)
- [HMC5883L Datasheet](https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf)
- [Zephyr Input Subsystem](https://docs.zephyrproject.org/latest/services/input/index.html)