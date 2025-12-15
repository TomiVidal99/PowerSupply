/*
 * main.c
 * Author: Tomás Vidal
 * Last revision: 11/12/2025
 *
 * TLDR: PID to feedback voltage and current,
 * so you can stablish those values at the output
 * The measurements are read with a voltage dividir of 2.
 * The feedback it's done through a PWM of frequency 10KHz.
 * So we can easily filter the average and feed it through the
 * base of TIP122.
 *
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "definitions.h"
#include "voltage_controller.h" // PID Parameters

#define SET_PWM(duty_percent)                             \
  do                                                      \
  {                                                       \
    uint8_t __p = (duty_percent);                         \
    if (__p > 100)                                        \
      __p = 100;                                          \
    OCR1A = (uint16_t)((uint32_t)__p * (ICR1 + 1) / 100); \
  } while (0)

/**
 * If this is defined then it no longer behaves as
 * a PID controller, it's just a step response identification
 */
// #define STEP_RESPONSE
#define REFERENCE_OFFSET_MV (200U) 

// PID variables
static volatile int16_t reference = 1200U;
static volatile int32_t error_prev = 0U;
static volatile int32_t control_action_prev = 0U;

int main(void)
{
#ifdef STEP_RESPONSE
  PIN_B_ENABLE(PB4);
  init_pwm();
  init_adc();
  sei();
  _delay_ms(1);

#define AVOID_ALINEAL_GAP_PERCENT 40

  SET_PWM(AVOID_ALINEAL_GAP_PERCENT);
  _delay_ms(2000);

  PIN_B_ON(PB4);
  SET_PWM(100 - AVOID_ALINEAL_GAP_PERCENT);
  _delay_ms(5000);

  PIN_B_OFF(PB4);
  SET_PWM(AVOID_ALINEAL_GAP_PERCENT);
  _delay_ms(5000);
#else
  PIN_B_ENABLE(PB4);
  init_pwm();
  init_adc();
  init_timer_0();
  sei();

  SET_PWM(30);

#endif
  while (1)
  {
  }
}

#ifndef STEP_RESPONSE
/**
 * This is triggered by timer0
 * (init_timer_0)
 */
ISR(TIMER0_OVF_vect)
{
  TCNT0 = 6; // Reload preload to keep exactly 1 ms period

  uint32_t v_out_ms;

  // Read voltage (ADC0)
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
    ;
  v_out_ms = (ADC * (5000UL)) / (1023UL);

  // - - - - - - - - - -  PID
  int32_t error = (int32_t)reference - (int32_t)2*(REFERENCE_OFFSET_MV + v_out_ms);
  int32_t control_action = (PID_GAIN_E_0 * (error) +
                            PID_GAIN_E_1 * (error_prev) +
                            PID_GAIN_U * (control_action_prev)) /
                           PID_GAIN_SCALE;

  if (control_action < 0)
    control_action = 0;
  else if (control_action > PWM_TOP)
    control_action = (uint16_t)PWM_TOP;

  OCR1A = (uint16_t)control_action;

  error_prev = error;
  control_action_prev = OCR1A;
  // - - - - - - - - - -

  PIN_B_ON(PB4);
  _delay_us(50);
  PIN_B_OFF(PB4);
}
#endif // STEP_RESPONSE

void init_adc()
{
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRA |= (1 << ADEN);
}

/**
 * This timer triggers every 100us an interrupt
 * (ISR(TIMER1_COMPA_vect))
 *
 * To vary the duty cycle just change: OCR1A
 * ICR1 = 199 in our case for a 100us PWM
 * Duty cycle = OCR1A / (ICR1 + 1)
 * Check:
 * https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
 * (page 91) for info!
 */
void init_pwm()
{
  PIN_B_ENABLE(PWM_PIN);
  TCCR1A = (1 << WGM11);
  TCCR1B = (1 << WGM12) | (1 << WGM13);
  ICR1 = 199;
  TCCR1A |= (1 << COM1A1);
  TCCR1B |= (1 << CS11);
}

/**
 * This triggers an interrupt every 1ms
 */
void init_timer_0()
{
  TCCR0A = 0;
  TCCR0B = (1 << CS01) | (1 << CS00);
  TCNT0 = 6;
  TIMSK0 |= (1 << TOIE0);
}