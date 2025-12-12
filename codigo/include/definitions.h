#include <stdint.h>

// - - - - - - - - - - -  USEFUL MACROS - - - - - - - - - -
#define PIN_B_ENABLE(PIN) (DDRB |= (1<<PIN))
#define PIN_B_ON(PIN) (PORTB |= (1<<PIN))
#define PIN_B_OFF(PIN) (PORTB &= ~(1<<PIN))

// - - - - - - - - - - -  FUNCTIONS - - - - - - - - - -
// void init_voltage_reg_pwm();
void init_adc();
void init_timer0();
void init_button_interrupt();