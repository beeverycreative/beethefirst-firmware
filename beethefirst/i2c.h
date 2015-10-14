#include <stdint.h>
#include <stdlib.h>

#include "lpc17xx.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_pinsel.h"

void i2c_init(void);
void i2c_setup_Pot(void);
uint32_t i2c_send(uint8_t byte1, uint8_t byte2, uint8_t byte3);

uint32_t i2c_send_Start (LPC_I2C_TypeDef *I2Cx);
void i2c_send_stop (LPC_I2C_TypeDef *I2Cx);
uint32_t i2c_send_byte (LPC_I2C_TypeDef *I2Cx, uint8_t databyte);
