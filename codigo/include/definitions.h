#include <stdint.h>

// - - - - - - - - - - -  FUNCIONES - - - - - - - - - - 
void init_adc5(); // se inicializa el ADC
void init_timer1(); // se configura para que dispare una interrupción cada 1ms
void set_pwm_duty_cycle(uint8_t);
void init_button_interrupt(); // dispara una interrupcion cuando se presiona el boton, para cambiar la referencia