#include "state_machine/capit.hpp"


void set_angle_mg(int angle)
{
    set_pwm_angle(CAPIT_BASE, angle, FREQ);
}

void set_angle_sg(int angle)
{
    set_pwm_angle(CAPIT_UJUNG, angle, FREQ);
}

void buka_capit(void)
{
    set_angle_sg(180);
}
void tutup_capit(void)
{
    set_angle_sg(80);
}
void turun_capit(void)
{
    set_angle_mg(180);
}
void naik_capit(void)
{
    set_angle_mg(90);
}

void capit(void)
{
    turun_capit();
    usleep(500);
    buka_capit();

}
void letak(void)
{
    tutup_capit();
    usleep(500);
    naik_capit();
}