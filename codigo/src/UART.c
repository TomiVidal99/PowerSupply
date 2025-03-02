#include <avr/io.h>

#include "UART.h"

void USART_init(void)
{
	UBRR0H = (uint8_t)(BAUD_PRESCALLER >> 8); // preescalador del baudrate parte alta
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);	  // preescalador del baudrate parte baja
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);	  // se habilita transmisi�n y recepci�n
	UCSR0C = ((1 << UCSZ00) | (1 << UCSZ01)); // configura 8 bits de datos
}
void USART_send(unsigned char data)
{
	while (!(UCSR0A & (1 << UDRE0)))
		;		 // espera mientras el registro est� lleno
	UDR0 = data; // se asigna el valor a transmitir al registro de datos
}
unsigned char USART_receive(void)
{
	while (!(UCSR0A & (1 << RXC0)))
		;		 // espera mientras el flag RX sea cero
	return UDR0; // si el flag es uno devuelve el contenido del registro de datos
}
void USART_putstring(char *StringPtr)
{
	while (*StringPtr != 0x00)
	{							// Se chequea si no hay m�s caracteres para enviar
		USART_send(*StringPtr); // usando esta simple funci�n se env�a de a un caracter
		StringPtr++;			// se incrementa el puntero para leer el pr�ximo car�cter
	}
}