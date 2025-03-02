#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL) + 1)) - 1)

#include <avr/io.h>

// Declaración de funciones
void USART_init(void);
unsigned char USART_receive(void);
void USART_send(unsigned char data);
void USART_putstring(char *StringPtr);