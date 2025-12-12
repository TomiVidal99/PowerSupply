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
static inline uint16_t saturate_u32_to_u16(uint32_t v, uint16_t max);
static uint16_t compute_controller(uint16_t error);

#endif // DEFINITIONS_H