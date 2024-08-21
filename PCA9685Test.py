from PCA9685Driverv2 import PCA9685Driver
import time
p = PCA9685Driver()
#print address of device (0x40 == 64)
print(p.i2c.scan())
#set frequency to 1200Hz and DC = 60%
p.set_pwm_frequency(1200)
p.set_pwm_dc_percent(1, 60)
time.sleep_ms(5000)

# for channels 1,14 and 15, vary the on time from 0.5 to 2.5ms
# Ideal for 180 deg rotation for MG996R servos
# frequency should be 50Hz
while True:
    for angle in range(0,190,10):
        p.servo_set_angle_custom(1,angle,0.5,2.5)
        p.servo_set_angle_custom(14,angle,0.5,2.5)
        p.servo_set_angle_custom(15,angle,0.5,2.5)
        print("angle: {}".format(angle))
        time.sleep_ms(100)
    for angle in range(180,-10,-10):
        p.servo_set_angle_custom(1,angle,0.5,2.5)
        p.servo_set_angle_custom(14,angle,0.5,2.5)
        p.servo_set_angle_custom(15,angle,0.5,2.5)
        print("angle: {}".format(angle))
        time.sleep_ms(100)