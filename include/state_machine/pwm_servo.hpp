#ifndef PWM_SERVO_H
#define PWM_SERVO_H

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define I2C_DEVICE "/dev/i2c-2"
#define PCA9685_SLAVE_ADDR 0x40
#define MODE1 0x00 // Mode  register  1
#define MODE2 0x01 // Mode  register  2
#define SUBADR1 0x02 // I2C-bus subaddress 1
#define SUBADR2 0x03 // I2C-bus subaddress 2
#define SUBADR3 0x04 // I2C-bus subaddress 3
#define ALLCALLADR 0x05 // channel All Call I2C-bus address
#define channel0 0x6 // channel0 start register
#define channel0_ON_L 0x6 // channel0 output and brightness control byte 0
#define channel0_ON_H 0x7 // channel0 output and brightness control byte 1
#define channel0_OFF_L 0x8 // channel0 output and brightness control byte 2
#define channel0_OFF_H 0x9 // channel0 output and brightness control byte 3
#define channel_MULTIPLIER 4 // For the other 15 channels
#define ALLchannel_ON_L 0xFA // load all the channeln_ON registers, byte 0 (turn 0-7 channels on)
#define ALLchannel_ON_H 0xFB // load all the channeln_ON registers, byte 1 (turn 8-15 channels on)
#define ALLchannel_OFF_L 0xFC // load all the channeln_OFF registers, byte 0 (turn 0-7 channels off)
#define ALLchannel_OFF_H                                                                           \
    0xFD // load all the channeln_OFF registers, byte 1 (turn 8-15 channels off)
#define PRE_SCALE 0xFE // prescaler for output frequency
#define CLOCK_FREQ 25000000.0 // 25MHz default osc clock
#define ANGLE_RANGE 180
#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500

extern int i2c_fd;

void PCA9685_init();
void write_byte(uint8_t reg, uint8_t val);
void set_pwm_freq(int freq);
void set_pwm_duty(uint8_t channel, int value);
void set_pwm(uint8_t channel, int on_value, int off_value);
void set_pwm_angle(uint8_t channel, int angle);

int get_pwm(uint8_t channel);

uint8_t read_byte(uint8_t reg);

#endif /*PWM_SERVO_H*/