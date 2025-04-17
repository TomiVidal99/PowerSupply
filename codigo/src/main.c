/*
 * main.c
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <stdio.h>

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "definitions.h"
#include "voltage_controller.h"

#include "uart.h"

#define OUTPUT_NET_GAIN (2.0f) // Gain that occurs from the PWM filtered to the output

// Controller for the voltage
volatile float voltage_output_volts_prev = 0.0f;
volatile float voltage_control_action_prev = 0.0f;
volatile float voltage_reference_volts = 3.59f;

// static volatile uint16_t timer0_counter;
// volatile uint8_t modes_counter = 0;

char *debug_output = "test\n\r";

volatile uint8_t debounce_counter = 0;

int main(void)
{
  DDRB |= (1 << PB4);
  init_voltage_reg_pwm();
  // init_timer0();
  USART_init();
  init_button_interrupt();
  init_adc();
  sei();

  while (1)
  {
  }
  return 0;
}

void init_timer0()
{
  DDRB |= (1 << PB4);
  TCCR0A = (1 << WGM01); // CTC mode
  TCCR0B = (1 << CS01) | (1 << CS00);
  OCR0A = 249;
  TIMSK0 = (1 << OCIE0A);
}

void init_voltage_reg_pwm()
{
  DDRB |= (1 << PB1);
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
  ICR1 = 2499;

  TIMSK1 = (1 << TOIE1);
  OCR1A = ICR1 * 0;
  // OCR1A = ICR1 * 0.4; // 40% -> 2 Volt de pwm -> 8 Volts de salida
}

void init_adc()
{
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRA |= (1 << ADEN);
}

// Timer 1 Compare A Match Interrupt Service Routine (ISR)
// Interruption that's triggered to control the voltage regulator
ISR(TIMER1_OVF_vect)
{
  float voltage_output_volts = 0;
  float control_action = 0;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // read the voltage regulator voltage node so we can make a closed loop system
  // ADMUX = (ADMUX & 0xF0) | (0 & 0x0F);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
    ;
  voltage_output_volts = ((ADC * 5.0f) / 1024.0f) * OUTPUT_NET_GAIN;

  // MAKE A controller for the voltage feedback
  control_action = VOLTAGE_E_GAIN_1 * (voltage_reference_volts - voltage_output_volts) + VOLTAGE_E_GAIN_2 * (voltage_reference_volts - voltage_output_volts_prev) + VOLTAGE_U_GAIN * voltage_control_action_prev;

  // sprintf(debug_output, "PWM Duty cycle: %d, temp1: %d\r\n", (uint16_t)((control_action) * 1000.0f), (uint16_t)(voltage_output_volts * 1000.0f));
  // sprintf(debug_output, "(%d)e[n] + (%d)e[n-1] + (%d)u[n-1]\n\r", (uint16_t)(() * 1000.0f), (uint16_t)(voltage_output_volts * 1000.0f));
  // USART_putstring(debug_output);

  // Update the PWM
  if (control_action > 5.0f)
  {
    OCR1A = ICR1;
  }
  else if (control_action < 0.0f)
  {
    OCR1A = 0;
  }
  else
  {
    OCR1A = ICR1 * (control_action * 20.0f); // 20.0f because 100% / 5V
  }

  // Update the previos states/samples
  voltage_control_action_prev = control_action;
  voltage_output_volts_prev = voltage_output_volts;

  PORTB ^= (1 << PB4);

  if (debounce_counter > 0)
  {
    debounce_counter--;
  }
}

// ISR(TIMER0_COMPA_vect)
// {
//   timer0_counter++;
//   if (timer0_counter < 1000)
//   {
//     PORTB ^= (1 << PB1);
//     return;
//   }
//   timer0_counter = 0;
//   modes_counter++;

//   if (modes_counter == 1)
//   {
//     // set PWM to 20%
//     OCR1A = ICR1 * 0.2;
//     PORTB ^= (1 << PB4);
//   }
//   else if (modes_counter == 3)
//   {
//     // set PWM to 80%
//     OCR1A = ICR1 * 0.8;
//     PORTB ^= (1 << PB4);
//   }
//   else if (modes_counter == 7)
//   {
//     // set PWM to 20%
//     OCR1A = ICR1 * 0.2;
//     PORTB ^= (1 << PB4);
//   }
// }

// habilita la interrupción de PD1 (boton en la placa)
void init_button_interrupt()
{
  DDRD &= ~(1 << PD3);
  PORTD |= (1 << PD3);
  EICRA |= (1 << ISC11);
  EICRA &= ~(1 << ISC10);
  EIMSK |= (1 << INT1);
}

// Cuando se presiona el botton
ISR(INT1_vect)
{
  if (debounce_counter > 0)
  {
    return;
  }
  if (!(PIND & (1 << PD3)))
  {
    if (voltage_reference_volts == 1.0f)
    {
      voltage_reference_volts = 1.5f;
    }
    else if (voltage_reference_volts == 1.5f) 
    {
      voltage_reference_volts = 2.0f;
    }
    else if (voltage_reference_volts == 2.0f)
    {
      voltage_reference_volts = 2.5f;
    }
    else if (voltage_reference_volts == 2.5f)
    {
      voltage_reference_volts = 3.0f;
    }
    else if (voltage_reference_volts == 3.0f)
    {
      voltage_reference_volts = 1.0f;
    }

    sprintf(debug_output, "new reference: %d\r\n", (uint16_t)((voltage_reference_volts) * 1000.0f));
    USART_putstring(debug_output);
    debounce_counter = 100;
  }
}