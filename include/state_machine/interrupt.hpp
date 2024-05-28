#ifndef INTERRUPT_HPP
#define INTERRUPT_HPP

#include <stdio.h>

#define SWITCH_PIN 17

// int is_program_running = 0;

extern int is_program_running;

void switch_interrupt(void);
void init_interrupt(void);
void start_program(void);
void stop_program(void);

#endif /* INTERRUPT_HPP*/
