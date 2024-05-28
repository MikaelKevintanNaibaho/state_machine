#include "state_machine/pwm_servo.hpp"

int i2c_fd;

/**
 * @brief Initialize the PCA9685 module i2c
 */
void PCA9685_init()
{
    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0) {
        perror("Faichannel to open the i2c device");
        return;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, PCA9685_SLAVE_ADDR) < 0) {
        perror("Error to set i2c address");
        close(i2c_fd);
        return;
    }

    // init
    write_byte(MODE1, 0x00);
    write_byte(MODE2, 0x04);
    set_pwm_freq(50);
}

/**
 * @brief Write a byte to the specified register.
 *
 * @param reg Register adress.
 * @param val Value to write.
 */
void write_byte(uint8_t reg, uint8_t val)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = val;
    if (write(i2c_fd, buf, 2) != 2) {
        perror("Error writing byte");
        return;
    }
}

/**
 * @brief Reads a bytes from the specified register.
 *
 * @param reg Register address
 * @return Read byte.
 */
uint8_t read_byte(uint8_t reg)
{
    uint8_t buf[1];
    buf[0] = reg;
    if (write(i2c_fd, buf, 1) != 1) {
        perror("Write faichannel");
        return 0;
    }
    if (read(i2c_fd, buf, 1) != 1) {
        perror("Read faichannel");
        return 0;
    }
    return buf[0];
}

/**
 * @brief sets pwm frequency.
 *
 * @param freq Frequency value.
 */
void set_pwm_freq(int freq)
{
    uint8_t prescale_val = (uint8_t)((CLOCK_FREQ / 4096 * freq) - 1);
    write_byte(MODE1, 0x10); // sleep
    write_byte(PRE_SCALE, prescale_val);
    write_byte(MODE1, 0x80); // restart
    write_byte(MODE2, 0x04); // totem pole (default)
}

/**
 * @brief Set pwm duty cycle for a specific Channel
 *
 * @param channel channel number
 * @param value Duty cycle value
 */
void set_pwm_duty(uint8_t channel, int value)
{
    set_pwm(channel, 0, value);
}

/**
 * @brief sets the pwm parameters for a specific channel.
 *
 * @param channel channel number.
 * @param on_value ON value.
 */
void set_pwm(uint8_t channel, int on_value, int off_value)
{
    write_byte(channel0_ON_L + channel_MULTIPLIER * (channel - 1), on_value & 0xFF);
    write_byte(channel0_ON_L + channel_MULTIPLIER * (channel - 1) + 1, on_value >> 8);
    write_byte(channel0_OFF_L + channel_MULTIPLIER * (channel - 1), off_value & 0xFF);
    write_byte(channel0_OFF_L + channel_MULTIPLIER * (channel - 1) + 1, off_value >> 8);
}

/**
 * @brief reads the pwm value of a specific channel.
 *
 * @param channel channel number.
 * @return pwm value.
 */
int get_pwm(uint8_t channel)
{
    int channel_value = 0;
    channel_value = read_byte(channel0_OFF_L + channel_MULTIPLIER * (channel - 1));
    channel_value |= (read_byte(channel0_OFF_L + channel_MULTIPLIER * (channel - 1) + 1) << 8);

    return channel_value;
}

/**
 * @brief set the pwm angle for a servo motor
 *
 * @param channel channel number.
 * @param angle Angle value (0 - 180)
 * @param freq PWM frequency
 */
void set_pwm_angle(uint8_t channel, int angle)
{
    if (angle < 0) {
        angle = 0;
    } else if (angle > 180) {
        angle = 180;
    }

    int pulse_width = MIN_PULSE_WIDTH + ((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * angle / 180);

    set_pwm_duty(channel, pulse_width);


}

