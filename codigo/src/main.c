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
// #include "uart.h"
#include "PID.h"

char *debug_output = "test\n\r";

int main(void)
{
  while (1)
  {
  }
  return 0;
}