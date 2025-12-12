#include "i2c.h"

void i2c_init(void) {
    TWSR = 0x00;
    TWBR = 0x48; // 100kHz @ 16MHz CPU
}

void i2c_start(uint8_t address) {
    TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
    TWDR = address;
    TWCR = (1<<TWEN)|(1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWEN)|(1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

void i2c_stop(void) {
    TWCR = (1<<TWSTO)|(1<<TWEN)|(1<<TWINT);
}