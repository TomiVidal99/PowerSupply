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
#define PID_BIAS_OFFSET (100U - 82U) // this is an offset due to aliniarities

// PID variables
volatile uint16_t adc_sample;
static int32_t e_k_1;
static int32_t e_k_2;
static int32_t u_k_1;

uint16_t setpoint = 200U;

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

  // Read voltage (ADC0)
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
    ;
  adc_sample = ADC;

  // PID
  uint16_t error;
  uint16_t actual_reference = setpoint + PID_BIAS_OFFSET;
  if (adc_sample > actual_reference)
    error = adc_sample - actual_reference;
  else
    error = actual_reference - adc_sample;
  uint16_t control = compute_controller(error);
  OCR1A = control;

  // OCR1A = 0;

  PIN_B_ON(PB4);
  _delay_us(50);
  PIN_B_OFF(PB4);
}
#endif // STEP_RESPONSE

static uint16_t compute_controller(uint16_t error)
{
  int32_t scaled_e0 = (int32_t)error;
  int32_t term_b0 = (PID_B0 * scaled_e0);
  int32_t term_b1 = (PID_B1 * e_k_1);
  int32_t sum_b = term_b0 + term_b1;
  int32_t term_a1 = (PID_A1 * u_k_1);
  int32_t num = sum_b - term_a1;
  int32_t denom = (int32_t)PID_COEF_SCALE;
  int32_t u_scaled = num / denom;
  if (u_scaled < 0)
    u_scaled = 0;
  uint32_t u_out = (uint32_t)u_scaled;
  if (u_out > PWM_TOP)
    u_out = PWM_TOP;
  e_k_2 = e_k_1;
  e_k_1 = scaled_e0;
  u_k_1 = (int32_t)u_out;
  return (uint16_t)u_out;
}

static inline uint16_t saturate_u32_to_u16(uint32_t v, uint16_t max)
{
  if (v > (uint32_t)max)
    return max;
  return (uint16_t)v;
}

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