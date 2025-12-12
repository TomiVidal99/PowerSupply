#ifndef I2C_H
#define I2C_H

#include <avr/io.h>

void i2c_init(void);
void i2c_start(uint8_t address);
void i2c_write(uint8_t data);
void i2c_stop(void);

/**
 * HOW TO USE:
 * #include <i2c.h>
 * #include <LCD.h>
 *
 * int main(void)
 * {
 *
 *   i2c_init();
 *   lcd_init();
 *
 *   lcd_set_cursor(0, 0);
 *   lcd_write_string("Hello, world!");
 *
 *   lcd_set_cursor(1, 0);
 *   lcd_write_string("I2C LCD 1602A");
 *
 *   while (1)
 *   {
 *   }
 *   return 0;
 * }
 */

#endif