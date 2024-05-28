#include <wiringPi.h>
#include "state_machine/interrupt.hpp"

int is_program_running = 0;

void switch_interrupt(void)
{
    if (digitalRead(SWITCH_PIN) == LOW) {
        if (!is_program_running) {
            start_program();
        }
    } else {
        if (is_program_running) {
            stop_program();
        }
    }
}

void init_interrupt(void)
{
    wiringPiSetupGpio();
    pinMode(SWITCH_PIN, INPUT);
    pullUpDnControl(SWITCH_PIN, PUD_UP);
    wiringPiISR(SWITCH_PIN, INT_EDGE_FALLING, &switch_interrupt);
}

void start_program(void)
{
    printf("starting program ...\n");
    is_program_running = 1;
}

void stop_program(void)
{
    printf("stopping program ...\n");
    is_program_running = 0;
}