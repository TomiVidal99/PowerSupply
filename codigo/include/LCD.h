#ifndef  __LCD_
#define __LCD_

#include <avr/io.h>
#include <util/delay.h>

// I2C address of the PCF8574 module (may be 0x27 or 0x3F usually)
#define LCD_I2C_ADDR 0x3F

// Functions
void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_write_char(char data);
void lcd_write_string(const char* str);

#endif