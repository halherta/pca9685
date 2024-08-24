# PCA9685Driver - MicroPython Driver for the I2C 12-bit 16 Channel PCA9685 PWM Controller

This is a MicroPython class for controlling the PCA9685, a 16-channel PWM controller, over I2C. The class provides methods for setting PWM frequency, controlling duty cycles, and managing servos.

Tested on the ESP32S3 microcontroller, but it should work with other microcontrollers that support the `machine.I2C` class.

### Features

- Control up to 16 PWM channels independently
- Set PWM frequency between 24 Hz and 1526 Hz
- Control PWM duty cycle via percentage, on-time, or direct rising/falling edge counts
- Servo control abstraction
- Support for switching between internal and external clock sources
- Software reset functionality

### Requirements

- MicroPython (v1.14 or later recommended)
- Tested on ESP32S3 microcontroller
- PCA9685 PWM controller

### Installation

Copy the PCA9685Driver class to your MicroPython project.

### Usage

#### Example: Basic Usage

``` python
from pca9685 import PCA9685Driver
import time

# Initialize PCA9685 with default I2C settings
pwm = PCA9685Driver()

# Print I2C address of the device (should be 0x40)
print(pwm.i2c.scan())

# Set PWM frequency to 1200Hz and duty cycle to 60% on channel 1
pwm.set_pwm_frequency(1200)
pwm.set_pwm_dc_percent(1, 60)
time.sleep(5)

```
#### Example: Servo Control

```python
# For channels 1, 14, and 15, vary the on-time from 0.5ms to 2.5ms (ideal for 180° rotation for MG996R servos)

# Set PWM frequency to 50Hz for servo control
pwm.set_pwm_frequency(50)

while True:
    for angle in range(0, 190, 10):
        pwm.servo_set_angle_custom(1, angle, 0.5, 2.5)
        pwm.servo_set_angle_custom(14, angle, 0.5, 2.5)
        pwm.servo_set_angle_custom(15, angle, 0.5, 2.5)
        print(f"angle: {angle}")
        time.sleep(0.1)
    for angle in range(180, -10, -10):
        pwm.servo_set_angle_custom(1, angle, 0.5, 2.5)
        pwm.servo_set_angle_custom(14, angle, 0.5, 2.5)
        pwm.servo_set_angle_custom(15, angle, 0.5, 2.5)
        print(f"angle: {angle}")
        time.sleep(0.1)
```

### Methods

- `set_pwm_frequency(pwm_freq)`: Set the PWM frequency. Valid range is 24 Hz to 1526 Hz.
- `set_pwm_dc_percent(chan, dc_percent)`: Set the duty cycle as a percentage (0-100%).
- `set_pwm_dc_ontime(chan, on_time_ms)`: Set the duty cycle by specifying the on-time in milliseconds.
- `set_pwm_dc(chan, falling_edge_cnt, rising_edge_cnt=0)`: Set the duty cycle by specifying rising and falling edge counts.
- `servo_set_angle(chan, angle)`: Set servo angle assuming 0° corresponds to 1ms pulse and 180° to 2ms pulse.
- `servo_set_angle_custom(chan, angle, minpulsewidth_ms, maxpulsewidth_ms)`: Set servo angle with custom pulse widths for 0° and 180°.
- `disable_clk()`: Disable the internal oscillator.
- `enable_clk()`: Enable the internal oscillator.
- `restart_clk()`: Restart the internal oscillator.
- `switch_to_ext_clk()`: Switch to using an external clock.
- `sw_reset()`: Perform a software reset.

### Notes

- The default I2C address for PCA9685 is 0x40.
- The class assumes a default PWM frequency of 50 Hz, which is typical for servo motors.
- When instantiating the PCA9685Driver object, ensure the I2C pins (SCL, SDA) are correctly set according to your hardware configuration. The defaults are `scl_pin=48` and `sda_pin=47`

### License

This project is open-source and available under the MIT License.

Feel free to contribute, open issues, or suggest improvements.

Enjoy controlling your servos and LEDs with the PCA9685 using MicroPython!
