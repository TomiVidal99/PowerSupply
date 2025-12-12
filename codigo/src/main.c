/*
 * main.c
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <stdio.h>
//
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "string.h"

#include "definitions.h"
#include "voltage_controller.h"

#include "DEBUG.h"

#include "uart.h"
#include "LCD.h"
#include "i2c.h"

// - - - - - - - DEBUG FLAGS -- - - - - - - - -
// debug flag, when active a pin will toggle to debug the ADC
#define DEBUG_ADC
#define DEBUG_UI

#define PROGRAM_VERSION "v1"
#define INIT_MESSAGE_NAME "PowerSupply " PROGRAM_VERSION ""

#define UPDATE_UI_TIMOUT_MS (200)
#define ADC_READ_COMPENSATION_OFFSET_MV (320)
#define ADC_READ_COMPENSATION_OFFSET_MA (29)

// When taking a sample of the ADC, actually we take this OVERSAMPLING_SAMPLES amount
#define OVERSAMPLING_SAMPLES 10

#define CURRENT_SENSE_GAIN (1.0f)
#define CURRENT_OFFSET (0.0f)
#define LCD_COLS (16)

volatile uint16_t voltage_measurements_volts[UPDATE_UI_TIMOUT_MS] = {};
volatile uint16_t current_measurements_amps[UPDATE_UI_TIMOUT_MS] = {};
volatile uint8_t voltage_index = 0;
volatile uint8_t current_index = 0;

static volatile uint16_t timer0_counter;

char *debug_output = "test\n\r";

// volatile uint8_t debounce_counter = 0;

int main(void)
{

  i2c_init();
  lcd_init();
  _delay_ms(10);

  // Welcome message
  // --------------------------------------
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_write_string(INIT_MESSAGE_NAME);
  lcd_set_cursor(1, 0);
  lcd_write_string("Por TOMAS VIDAL");

  _delay_ms(1000);
  lcd_clear();
  // --------------------------------------

  // ----- DEBUG ------
#ifdef DEBUG_ADC
  PIN_B_ENABLE(DEBUG_ADC_PIN);
#endif
#ifdef DEBUG_UI
  PIN_B_ENABLE(DEBUG_UI_PIN);
#endif
  // -------------

  init_timer0();
  // USART_init();
  // init_button_interrupt();
  init_adc();
  _delay_ms(10);
  sei();

  while (1)
  {
  }
  return 0;
}

// I Use this timer to handle the update of the screen
void init_timer0()
{
  DDRB |= (1 << PB4);
  TCCR0A = (1 << WGM01); // CTC mode
  TCCR0B = (1 << CS01) | (1 << CS00);
  OCR0A = 249;
  TIMSK0 = (1 << OCIE0A);
}

void init_adc()
{
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRA |= (1 << ADEN);
}

// Timer callback that handles the UI
// Triggers every 1ms, but the UI only updates every UPDATE_UI_TIMOUT_MS
ISR(TIMER0_COMPA_vect)
{
  int i;
  uint32_t oversampling_acc;

#ifdef DEBUG_ADC
  PIN_B_ON(DEBUG_ADC_PIN);
#endif
  // Get measurements of voltage and current
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Current
  oversampling_acc = 0;
  for (i = 0; i < OVERSAMPLING_SAMPLES; i++)
  {
    ADMUX = (ADMUX & 0xF0) | 1; // Channel 1
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
      ; // Wait
    oversampling_acc += (uint32_t)ADC;
  }
  current_measurements_amps[current_index++] = oversampling_acc / OVERSAMPLING_SAMPLES;
#ifdef DEBUG_ADC
  PIN_B_OFF(DEBUG_ADC_PIN);
#endif

#ifdef DEBUG_ADC
  PIN_B_ON(DEBUG_ADC_PIN);
#endif
  // Voltage
  oversampling_acc = 0;
  for (i = 0; i < OVERSAMPLING_SAMPLES; i++)
  {
    ADMUX = (ADMUX & 0xF0) | 0; // Channel 0
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
      ;
    oversampling_acc += (uint32_t)ADC;
  }
  voltage_measurements_volts[voltage_index++] = oversampling_acc / OVERSAMPLING_SAMPLES;
#ifdef DEBUG_ADC
  PIN_B_OFF(DEBUG_ADC_PIN);
#endif

  timer0_counter++;
  if (timer0_counter < UPDATE_UI_TIMOUT_MS)
  {
    return;
  }
  timer0_counter = 0;
  voltage_index = 0;
  current_index = 0;

  // Get the average of the measurements
  uint32_t voltage = 0;
  uint32_t current = 0;
  uint32_t power; // TODO: maybe make an integral of the power? (avg over perior of 1 second or some like that)

  for (i = 0; i < UPDATE_UI_TIMOUT_MS; i++)
  {
    voltage += (uint32_t)voltage_measurements_volts[i];
    current += (uint32_t)current_measurements_amps[i];
  }

  voltage = voltage / UPDATE_UI_TIMOUT_MS;
  current = current / UPDATE_UI_TIMOUT_MS - voltage;
  // TODO: maybe check for a derivative of the current, and just make it smoother??
  power = voltage * current;

  voltage = voltage * 5000UL / 1023UL - ADC_READ_COMPENSATION_OFFSET_MV;
  if (voltage > ((uint32_t)0 - (uint32_t)(2 * ADC_READ_COMPENSATION_OFFSET_MV)) ||
      voltage < ((uint32_t)(2 * ADC_READ_COMPENSATION_OFFSET_MV)))
  {
    voltage = 0;
  }

  current = current * 5000UL / 1023UL - ADC_READ_COMPENSATION_OFFSET_MV;
  if (current > ((uint32_t)0 - (uint32_t)(2 * ADC_READ_COMPENSATION_OFFSET_MV)) ||
      current < ((uint32_t)(2 * ADC_READ_COMPENSATION_OFFSET_MV)))
  {
    current = 0;
  }

  // - - - - -  UPDATE UI  - - - - - - -
#ifdef PB2
  PIN_B_ON(DEBUG_UI_PIN);
#endif
  // Update UI
  char lcd_message[LCD_COLS];

  // update the power consumption
  for (i = 0; i < LCD_COLS; i++)
    lcd_message[i] = ' ';
  snprintf(lcd_message, sizeof(lcd_message), "POW: %d mW", (uint16_t)power);
  lcd_set_cursor(1, 0);
  lcd_write_string(lcd_message);

  // update the voltage and current display
  for (i = 0; i < LCD_COLS; i++)
    lcd_message[i] = ' ';
  sprintf(lcd_message, "%dmV | %dmA", (uint16_t)voltage, (uint16_t)current);
  lcd_set_cursor(0, 0);
  lcd_write_string(lcd_message);

#ifdef DEBUG_UI
  PIN_B_OFF(DEBUG_UI_PIN);
#endif
  // - - - - - - - - - - - - - - -
}

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
// ISR(INT1_vect)
// {
//   if (debounce_counter > 0)
//   {
//     return;
//   }
//   if (!(PIND & (1 << PD3)))
//   {
//     if (voltage_reference_volts == 1.0f)
//     {
//       voltage_reference_volts = 1.5f;
//     }
//     else if (voltage_reference_volts == 1.5f)
//     {
//       voltage_reference_volts = 2.0f;
//     }
//     else if (voltage_reference_volts == 2.0f)
//     {
//       voltage_reference_volts = 2.5f;
//     }
//     else if (voltage_reference_volts == 2.5f)
//     {
//       voltage_reference_volts = 3.0f;
//     }
//     else if (voltage_reference_volts == 3.0f)
//     {
//       voltage_reference_volts = 1.0f;
//     }

//     sprintf(debug_output, "new reference: %d\r\n", (uint16_t)((voltage_reference_volts) * 1000.0f));
//     USART_putstring(debug_output);
//     debounce_counter = 100;
//   }
// }

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// OLD FEEDBACK CODE
//
//
//    /*
//     * main.c
//     */
//
//    #ifndef F_CPU
//    #define F_CPU 16000000UL
//    #endif
//
//    #include <stdio.h>
//
//    #include <avr/interrupt.h>
//    #include <avr/sleep.h>
//    #include <util/delay.h>
//
//    #include "definitions.h"
//    #include "voltage_controller.h"
//
//    #include "uart.h"
//    #include "LCD.h"
//    #include "i2c.h"
//
//    #define PROGRAM_VERSION "v1"
//    #define INIT_MESSAGE_NAME "PowerSupply " PROGRAM_VERSION ""
//
//    #define OUTPUT_NET_GAIN (2.0f) // Gain that occurs from the PWM filtered to the output
//    #define UPDATE_UI_TIMOUT_MS (200)
//    #define ADC_READ_COMPENSATION_OFFSET (0.9f)
//
//    #define CURRENT_SENSE_GAIN (1.0f)
//    #define CURRENT_OFFSET (0.0f)
//
//    // Controller for the voltage
//    volatile float voltage_output_volts_prev = 0.0f;
//    volatile float voltage_control_action_prev = 0.0f;
//    volatile float voltage_reference_volts = 5.0f;
//
//    float current_output_amps = 0.0f;
//
//    static volatile uint16_t timer0_counter;
//
//    char *debug_output = "test\n\r";
//
//    // volatile uint8_t debounce_counter = 0;
//
//    // Get the response of the voltage close loop to determine the PID parameters
//    #define STEP_RESPONSE
//
//    int main(void)
//    {
//
//    #ifdef STEP_RESPONSE
//      DDRB |= (1 << PB4);
//      DDRB |= (1 << PB1);
//      init_voltage_reg_pwm();
//
//      OCR1A = ICR1 * 0.1;
//      _delay_ms(5000);
//
//      PORTB |= (1 << PB4);
//      OCR1A = ICR1 * 0.9;
//      _delay_ms(10000);
//
//      PORTB ^= ~(1 << PB4);
//      OCR1A = ICR1 * 0.1;
//      _delay_ms(5000);
//
//      OCR1A = 0;
//    #else
//
//      i2c_init();
//      lcd_init();
//      _delay_ms(10);
//
//      // Welcome message
//      // --------------------------------------
//      lcd_clear();
//      lcd_set_cursor(0, 0);
//      lcd_write_string(INIT_MESSAGE_NAME);
//      lcd_set_cursor(1, 0);
//      lcd_write_string("Por TOMAS VIDAL");
//
//      _delay_ms(1000);
//      lcd_clear();
//      // --------------------------------------
//
//      DDRB |= (1 << PB4);
//      DDRB |= (1 << PB1);
//      init_voltage_reg_pwm();
//      init_timer0();
//      USART_init();
//      // init_button_interrupt();
//      init_adc();
//      _delay_ms(10);
//      sei();
//
//    #endif
//
//      while (1)
//      {
//      }
//      return 0;
//    }
//
//    // I Use this timer to handle the update of the screen
//    void init_timer0()
//    {
//      DDRB |= (1 << PB4);
//      TCCR0A = (1 << WGM01); // CTC mode
//      TCCR0B = (1 << CS01) | (1 << CS00);
//      OCR0A = 249;
//      TIMSK0 = (1 << OCIE0A);
//    }
//
//    void init_voltage_reg_pwm()
//    {
//      // DDRB |= (1 << PB1);
//      TCCR1A = (1 << COM1A1) | (1 << WGM11);
//      TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
//      ICR1 = 2499;
//
//      TIMSK1 = (1 << TOIE1);
//      OCR1A = ICR1 * 0;
//      // OCR1A = ICR1 * 0.4; // 40% -> 2 Volt de pwm -> 8 Volts de salida
//    }
//
//    void init_adc()
//    {
//      ADMUX |= (1 << REFS0);
//      ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
//      ADCSRA |= (1 << ADEN);
//    }
//
//    // Timer 1 Compare A Match Interrupt Service Routine (ISR)
//    // Interruption that's triggered to control the voltage regulator
//    ISR(TIMER1_OVF_vect)
//    {
//    #ifndef STEP_RESPONSE
//
//      float voltage_output_volts = 0;
//      float control_action = 0;
//
//      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//      // Read the current with the ADC1
//      ADMUX = (ADMUX & 0xF0) | 1; // Channel 1
//      ADCSRA |= (1 << ADSC);
//      while (ADCSRA & (1 << ADSC))
//        ; // Wait
//      current_output_amps = ((ADC * 5.0f) / 1024.0f) * CURRENT_SENSE_GAIN - CURRENT_OFFSET;
//
//      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//      // read the voltage regulator voltage node so we can make a closed loop system
//      ADMUX = (ADMUX & 0xF0) | 0; // Channel 0
//      ADCSRA |= (1 << ADSC);
//      while (ADCSRA & (1 << ADSC))
//        ;
//      voltage_output_volts = ((ADC * 5.0f) / 1024.0f) * OUTPUT_NET_GAIN - ADC_READ_COMPENSATION_OFFSET;
//
//      // Make a controller for the voltage feedback
//      control_action = VOLTAGE_E_GAIN_1 * (voltage_reference_volts - voltage_output_volts) + VOLTAGE_E_GAIN_2 * (voltage_reference_volts - voltage_output_volts_prev) + VOLTAGE_U_GAIN * voltage_control_action_prev;
//
//      // sprintf(debug_output, "PWM Duty cycle: %d, temp1: %d\r\n", (uint16_t)((control_action) * 1000.0f), (uint16_t)(voltage_output_volts * 1000.0f));
//      // sprintf(debug_output, "(%d)e[n] + (%d)e[n-1] + (%d)u[n-1]\n\r", (uint16_t)(() * 1000.0f), (uint16_t)(voltage_output_volts * 1000.0f));
//      // USART_putstring(debug_output);
//
//      // Update the PWM
//      if (control_action > 5.0f)
//      {
//        OCR1A = ICR1;
//      }
//      else if (control_action < 0.0f)
//      {
//        OCR1A = 0;
//      }
//      else
//      {
//        OCR1A = ICR1 * (control_action * 20.0f); // 20.0f because 100% / 5V
//      }
//
//      // Update the previos states/samples
//      voltage_control_action_prev = control_action;
//      voltage_output_volts_prev = voltage_output_volts;
//
//      PORTB ^= (1 << PB4);
//
//      // if (debounce_counter > 0)
//      // {
//      //   debounce_counter--;
//      // }
//
//    #endif
//    }
//
//    // Timer callback that handles the UI
//    // Triggers every 1ms, but the UI only updates every UPDATE_UI_TIMOUT_MS
//    ISR(TIMER0_COMPA_vect)
//    {
//      timer0_counter++;
//      if (timer0_counter < UPDATE_UI_TIMOUT_MS)
//      {
//        PORTB ^= (1 << PB1);
//        return;
//      }
//      timer0_counter = 0;
//
//      // Update UI
//      char lcd_message[16] = "";
//
//      uint16_t voltage_int = (uint16_t)(voltage_output_volts_prev * 100);
//      sprintf(lcd_message, "V: %d.%02d", voltage_int / 100, voltage_int % 100);
//      lcd_set_cursor(0, 0);
//      lcd_write_string(lcd_message);
//
//      uint16_t current_int = (uint16_t)(current_output_amps * 100);
//      sprintf(lcd_message, "I: %d.%02d", current_int / 100, current_int % 100);
//      lcd_set_cursor(1, 0);
//      lcd_write_string(lcd_message);
//    }
//
//    // habilita la interrupción de PD1 (boton en la placa)
//    void init_button_interrupt()
//    {
//      DDRD &= ~(1 << PD3);
//      PORTD |= (1 << PD3);
//      EICRA |= (1 << ISC11);
//      EICRA &= ~(1 << ISC10);
//      EIMSK |= (1 << INT1);
//    }
//
//    // Cuando se presiona el botton
//    // ISR(INT1_vect)
//    // {
//    //   if (debounce_counter > 0)
//    //   {
//    //     return;
//    //   }
//    //   if (!(PIND & (1 << PD3)))
//    //   {
//    //     if (voltage_reference_volts == 1.0f)
//    //     {
//    //       voltage_reference_volts = 1.5f;
//    //     }
//    //     else if (voltage_reference_volts == 1.5f)
//    //     {
//    //       voltage_reference_volts = 2.0f;
//    //     }
//    //     else if (voltage_reference_volts == 2.0f)
//    //     {
//    //       voltage_reference_volts = 2.5f;
//    //     }
//    //     else if (voltage_reference_volts == 2.5f)
//    //     {
//    //       voltage_reference_volts = 3.0f;
//    //     }
//    //     else if (voltage_reference_volts == 3.0f)
//    //     {
//    //       voltage_reference_volts = 1.0f;
//    //     }
//
//    //     sprintf(debug_output, "new reference: %d\r\n", (uint16_t)((voltage_reference_volts) * 1000.0f));
//    //     USART_putstring(debug_output);
//    //     debounce_counter = 100;
//    //   }
//    // }