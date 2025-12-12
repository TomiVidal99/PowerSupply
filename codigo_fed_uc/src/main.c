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

int main(void)
{
  PIN_B_ENABLE(PB4);
  init_pwm();
  sei();

  SET_PWM(30);

  while (1)
  {
  }
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