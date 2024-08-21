from machine import Pin, I2C
import time
import struct

class PCA9685Driver:
    """PCA9685 driver class"""
    OSCILLATION_FREQ = 25000000
    PWM_RESOLUTION = 4096
    def __init__(self, i2c_periph_addr = 0x40, i2c_channel=0, scl_pin=48, sda_pin=47, i2c_freq = 100000):
        self.periph_addr = i2c_periph_addr # set I2C Peripheral address
        self.pwm_frequency = 50 #initialize PWM frequency to 50Hz. This can be changed via setPWMFrequency function
        self.prescaler = round(PCA9685Driver.OSCILLATION_FREQ/(PCA9685Driver.PWM_RESOLUTION*self.pwm_frequency))-1 # calculate the prescaler
        self.i2c = I2C(i2c_channel, scl=Pin(scl_pin), sda=Pin(sda_pin), freq=i2c_freq) # initialize I2C bus & pins
        
        self._write_reg(PCA9685Registers.PRESCALE, self.prescaler) # write prescaler value to PRESCALE register
                                                                 # needs to be done before turning on the internal oscillator
        
        #Turn on internal oscillator
        mode1_val = self._read_reg(PCA9685Registers.MODE1)
        #print(mode1_val)
        mode1_val = self._clear_bit(mode1_val, MODE1RegBits.SLEEP)
        self._write_reg(PCA9685Registers.MODE1, mode1_val)
        #print(mode1_val)
        time.sleep_us(500)
    
        
    def _read_reg(self, register):
        """Helper function to read register contents. Converts byte data to an integer value """
        return int.from_bytes(self.i2c.readfrom_mem(self.periph_addr, register, 1), 'big')
    
    def _write_reg(self, register, value):
        """Helper function to write bytes into a register. Converts 'value' from int type to byte type"""
        self.i2c.writeto_mem(self.periph_addr, register, struct.pack('B', value))
    
    def _clear_bit(self, value, bit):
        return value & ~(1 << bit)

    def _set_bit(self, value, bit):
        return value | (1 << bit)
    
    
    def set_pwm_frequency(self,pwm_freq):
        """Sets the PWM Frequency. Need to make sure internal oscillator is of before adjusting prescaler value"""
        if( pwm_freq >= 24 and pwm_freq <= 1526): # PCA9685 can generate frequencies in this range (more or less)
            mode1_val = self._read_reg(PCA9685Registers.MODE1)
            if(((mode1_val >> MODE1RegBits.SLEEP) & 0x01) == 0 ): # if internal oscillator is already ON... 
                mode1_val = self._set_bit(mode1_val, MODE1RegBits.SLEEP)
                self._write_reg(PCA9685Registers.MODE1, mode1_val) # turn it OFF before adjusting prescaler value
                time.sleep_us(500)
                
            self.pwm_frequency = pwm_freq  
            self.prescaler = round(PCA9685Driver.OSCILLATION_FREQ/(PCA9685Driver.PWM_RESOLUTION*self.pwm_frequency))-1 # Now calculate the new prescaler value for new frequency
            self._write_reg(PCA9685Registers.PRESCALE, self.prescaler) # write it to PRESCALER register
            
            mode1_val = self._read_reg(PCA9685Registers.MODE1) #RMW op
            if(((mode1_val >> MODE1RegBits.SLEEP) & 0x01) != 0 ):
                mode1_val = self._clear_bit(mode1_val, MODE1RegBits.SLEEP)
                self._write_reg(PCA9685Registers.MODE1, mode1_val) # Turn internal oscillator back ON again
                time.sleep_us(500)
        else:
            print("PWM Frequency needs to be between 24 and 1526 Hz")
    
    
    def set_pwm_dc_percent(self,chan, dc_percent):
        """Sets duty cycle as a percentage between 0 & 100%. Can use decimal percentage values i.e. 88.5"""
        if( chan >= 0 and chan <= 15): # Validate channel number
            if(dc_percent >= 0 and dc_percent <= 100):  # validate duty cycle value
                off_val = round(dc_percent*4095/100)
                self._write_reg(chan*4+6, 0) # write start ON and OFF times into the 4 registers associated with channel
                self._write_reg(chan*4+7, 0)
                self._write_reg(chan*4+8, (off_val & 0xFF))
                self._write_reg(chan*4+9, ((off_val >> 8) & 0x0F))
            else:
                print("Duty cycle value needs to be between 0 and 100")
        else:
            print("Channel value needs to be between 0 and 15")
    
           
    def set_pwm_dc_ontime(self,chan, on_time_ms):
        """Sets duty cycle by ON time. Rising edge is assumed to start at 0 """
        if( chan >= 0 and chan <= 15):# Validate channel number
            period_ms = (1.0 / self.pwm_frequency)*1000 #calculate period
            if(on_time_ms >= 0.0 and on_time_ms <= period_ms): # ON time has to be less than the period
                off_val = round(on_time_ms*4095/period_ms) #Calculate ON time duration
                self._write_reg(chan*4+6, 0) #write start ON and OFF times into the 4 registers associated with channel
                self._write_reg(chan*4+7, 0)
                self._write_reg(chan*4+8, (off_val & 0xFF))
                self._write_reg(chan*4+9, ((off_val >> 8) & 0x0F))
            else:
                print("ON time value needs to be between 0.0 msec and {} msec".format(period_ms))
        else:
            print("Channel value needs to be between 0 and 15")    
    
      
    def set_pwm_dc(self, chan, falling_edge_cnt ,rising_edge_cnt=0):
        """Most flexible. Sets duty cycle by ON time via two parameters; rising edge start time and falling edge start time"""
        if( chan >= 0 and chan <= 15): # Validate channel number
            if(rising_edge_cnt >= 0 and rising_edge_cnt <= 4095): #Rising edge time/cnt must be between 0 and 4095
                if(falling_edge_cnt > rising_edge_cnt and falling_edge_cnt <= 4095):# Whereas falling edge cnt must be larger than rising edge cnt
                    self._write_reg(chan*4+6, rising_edge_cnt & 0xFF) #write start ON and OFF times into the 4 registers associated with channel
                    self._write_reg(chan*4+7, (rising_edge_cnt >> 8) & 0x0F)
                    self._write_reg(chan*4+8, falling_edge_cnt & 0xFF)
                    self._write_reg(chan*4+9, (falling_edge_cnt >> 8) & 0x0F)
                else:
                    print("falling_edge_cnt  value needs to be between rising_edge_cnt: {} and 4095".format(rising_edge_cnt))
            else:
                print("rising_edge_cnt  value needs to be between 0 and 4095")
        else:
            print("Channel value needs to be between 0 and 15")
    
        
    def servo_set_angle(self, chan, angle):
        """Abstracted Servo function. Assumes angle 0 => 1ms ON pulse and  angle 180 => 2ms ON pulse"""
        if self.pwm_frequency != 50:
            self.set_pwm_frequency(50)
        
        period_ms = (1.0/ self.pwm_frequency) * 1000
        
        if angle >= 0.0 and angle <= 180.0 :
            on_time_ms = 1.0 + (angle/180.0)*1.0
            self.set_pwm_dc_ontime(chan, on_time_ms)
        else:
             print("Angle value needs to be between 0 and 180")
             
               
    def servo_set_angle_custom(self, chan, angle, minpulsewidth_ms, maxpulsewidth_ms):
        """Abstracted Servo function. lets user set  ON pulse for angle 0 => 'minpulsewidth_ms' angle 180 => 'maxpulsewidth_ms'"""
        if self.pwm_frequency != 50:
            self.set_pwm_frequency(50)
        
        period_ms = (1.0/ self.pwm_frequency) * 1000
        
        if angle >= 0.0 and angle <= 180.0 :
            if ( maxpulsewidth_ms >= minpulsewidth_ms):
                on_time_ms = minpulsewidth_ms + (angle/180.0)*(maxpulsewidth_ms - minpulsewidth_ms) 
                self.set_pwm_dc_ontime(chan, on_time_ms)
            else:
                print("maxpulsewidth_ms needs to be larger than or equal to minpulsewidth_ms")
        else:
             print("Angle value needs to be between 0 and 180")
   
   
    def disable_clk(self):
        """Disable internal oscillator"""
        mode1_val = self._read_reg(PCA9685Registers.MODE1)
        if(((mode1_val >> MODE1RegBits.SLEEP) & 0x01) != 1 ):
            mode1_val = self._set_bit(mode1_val,MODE1RegBits.SLEEP)
            self._write_reg(PCA9685Registers.MODE1, mode1_val)
            time.sleep_us(500)
            
           
    def enable_clk(self):
        """Enable internal oscillator"""
        mode1_val = self._read_reg(PCA9685Registers.MODE1)
        if(((mode1_val >> MODE1RegBits.SLEEP) & 0x01) != 0 ):
           mode1_val = self._clear_bit(mode1_val,MODE1RegBits.SLEEPT)
           self._write_reg(PCA9685Registers.MODE1, mode1_val)
           time.sleep_us(500)
            
            
    def restart_clk(self):
        """Restart internal oscillator from where it left of. See section 7.3.1.1 in datasheet"""
        mode1_val = self._read_reg(PCA9685Registers.MODE1)
        if(((mode1_val >> MODE1RegBits.RESTART) & 0x01) == 1 ):
            mode1_val = self._clear_bit(mode1_val,MODE1RegBits.SLEEP)
            self._write_reg(PCA9685Registers.MODE1, mode1_val)
            time.sleep_us(500)
            mode1_val = self._read_reg(PCA9685Registers.MODE1)
            mode1_val = self._set_bit(mode1_val, MODE1RegBits.RESTART)
            self._write_reg(PCA9685Registers.MODE1, mode1_val)
            time.sleep_us(500)
            
             
    def switch_to_ext_clk(self):
        """Switch to using external clock. See section 7.3.1 p14 in datasheet"""
        mode1_val = self._read_reg(PCA9685Registers.MODE1)
        mode1_val = self._set_bit(mode1_val, MODE1RegBits.SLEEP)
        self._write_reg(PCA9685Registers.MODE1, mode1_val)
        time.sleep_us(500)
        mode1_val = self._read_reg(PCA9685Registers.MODE1)
        mode1_val |= ((1 << MODE1RegBits.SLEEP) | (1 << MODE1RegBits.EXTCLK)) 
        self._write_reg(PCA9685Registers.MODE1, mode1_val)
    
        
    def sw_reset(self):
        """Software reset see section 7.6 in Datasheet"""
        self.i2c.writeto(0x00, struct.pack('B', 0x06))

class MODE1RegBits:
    """MODE1 register bits"""
    SLEEP = 4
    AI = 5
    EXTCLK = 6
    RESTART = 7
    
class PCA9685Registers:
    """List of PCA9685 internal registers"""
    MODE1 = 0x00
    MODE2 = 0x01
    SUBADR1 = 0x02
    SUDADR2 = 0x03
    SUBADR3 = 0x04
    ALLCALLADR = 0x05
    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09
    LED1_ON_L = 0x0A
    LED1_ON_H = 0x0B
    LED1_OFF_L = 0x0C
    LED1_OFF_H = 0x0D
    LED2_ON_L = 0x0E
    LED2_ON_H = 0x0F
    LED2_OFF_L = 0x10
    LED2_OFF_H = 0x11
    LED3_ON_L = 0x12
    LED3_ON_H = 0x13
    LED3_OFF_L = 0x14
    LED3_OFF_H = 0x15
    LED4_ON_L = 0x16
    LED4_ON_H = 0x17
    LED4_OFF_L = 0x18
    LED4_OFF_H = 0x19
    LED5_ON_L = 0x1A
    LED5_ON_H = 0x1B
    LED5_OFF_L = 0x1C
    LED5_OFF_H = 0x1D
    LED6_ON_L = 0x1E
    LED6_ON_H = 0x1F
    LED6_OFF_L = 0x20
    LED6_OFF_H = 0x21
    LED7_ON_L = 0x22
    LED7_ON_H = 0x23
    LED7_OFF_L = 0x24
    LED7_OFF_H = 0x25
    LED8_ON_L = 0x26
    LED8_ON_H = 0x27
    LED8_OFF_L = 0x28
    LED8_OFF_H = 0x29
    LED9_ON_L = 0x2A
    LED9_ON_H = 0x2B
    LED9_OFF_L = 0x2C
    LED9_OFF_H = 0x2D
    LED10_ON_L = 0x2E
    LED10_ON_H = 0x2F
    LED10_OFF_L = 0x30
    LED10_OFF_H = 0x31
    LED11_ON_L = 0x32
    LED11_ON_H = 0x33
    LED11_OFF_L = 0x34
    LED11_OFF_H = 0x35
    LED12_ON_L = 0x36
    LED12_ON_H = 0x37
    LED12_OFF_L = 0x38
    LED12_OFF_H = 0x39
    LED13_ON_L = 0x3A
    LED13_ON_H = 0x3B
    LED13_OFF_L = 0x3C
    LED13_OFF_H = 0x3D
    LED14_ON_L = 0x3E
    LED14_ON_H = 0x3F
    LED14_OFF_L = 0x40
    LED14_OFF_H = 0x41
    LED15_ON_L = 0x42
    LED15_ON_H = 0x43
    LED15_OFF_L = 0x44
    LED15_OFF_H = 0x45
    ALL_LED_ON_L = 0xFA
    ALL_LED_ON_H = 0xFB
    ALL_LED_OFF_L = 0xFC
    ALL_LED_OFF_H = 0xFD
    PRESCALE = 0xFE
    TEST_MODE = 0xFF
    
    



