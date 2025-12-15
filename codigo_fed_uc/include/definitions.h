#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <stdint.h>

// - - - - - - - - - - -  PINS DEFINITIONS - - - - - - - - - -
#define PWM_PIN PB1

// - - - - - - - - - - -  USEFUL MACROS - - - - - - - - - -
#define PIN_B_ENABLE(PIN) (DDRB |= (1 << PIN))
#define PIN_B_ON(PIN) (PORTB |= (1 << PIN))
#define PIN_B_OFF(PIN) (PORTB &= ~(1 << PIN))

// - - - - - - - - - - -  FUNCTIONS - - - - - - - - - -
void init_pwm();
void init_adc();
void init_timer_0();

#endif // DEFINITIONS_H