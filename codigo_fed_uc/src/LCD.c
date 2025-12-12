#include "LCD.h"
#include "i2c.h"  // <- you need a basic i2c library too (I'll explain below)

// Control bits
#define LCD_BACKLIGHT 0x08
#define ENABLE_BIT    0x04

static void lcd_send_nibble(uint8_t nibble, uint8_t mode);
static void lcd_send_byte(uint8_t byte, uint8_t mode);

void lcd_init(void) {
    _delay_ms(50); // Wait for LCD to power up

    lcd_send_nibble(0x03, 0);
    _delay_ms(5);
    lcd_send_nibble(0x03, 0);
    _delay_us(150);
    lcd_send_nibble(0x03, 0);
    _delay_us(150);
    lcd_send_nibble(0x02, 0); // Set 4-bit mode
    _delay_us(150);

    lcd_send_byte(0x28, 0); // Function set: 4-bit, 2 lines, 5x8 dots
    lcd_send_byte(0x08, 0); // Display off
    lcd_send_byte(0x01, 0); // Clear display
    _delay_ms(2);
    lcd_send_byte(0x06, 0); // Entry mode set: Increment cursor
    lcd_send_byte(0x0C, 0); // Display on, cursor off, blink off
}

void lcd_clear(void) {
    lcd_send_byte(0x01, 0);
    _delay_ms(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t address[] = {0x00, 0x40}; // Line addresses
    lcd_send_byte(0x80 | (address[row] + col), 0);
}

void lcd_write_char(char data) {
    lcd_send_byte(data, 1);
}

void lcd_write_string(const char* str) {
    while (*str) {
        lcd_write_char(*str++);
    }
}

// --- Internal functions ---

static void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble << 4) | LCD_BACKLIGHT | mode;
    i2c_start(LCD_I2C_ADDR << 1);
    i2c_write(data | ENABLE_BIT);
    _delay_us(1);
    i2c_write(data & ~ENABLE_BIT);
    i2c_stop();
}

static void lcd_send_byte(uint8_t byte, uint8_t mode) {
    lcd_send_nibble(byte >> 4, mode);
    lcd_send_nibble(byte & 0x0F, mode);
}
