

#ifndef SPI_H_
#define SPI_H_

#include "lpc_types.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_ssp.h"

/* SPI clock rate setting.
SSP0_CLK = SystemCoreClock / divider,
The divider must be a even value between 2 and 254!
In SPI mode, max clock speed is 20MHz for MMC and 25MHz for SD */
//#define SPI_CLOCKRATE_LOW   (uint32_t) (250)   /* 100MHz / 250 = 400kHz */
//#define SPI_CLOCKRATE_HIGH  (uint32_t) (4)     /* 100MHz / 4 = 25MHz */
#define SPI_CLOCKRATE_LOW   (uint32_t) 25000
#define SPI_CLOCKRATE_HIGH  (uint32_t) 4000000
/* SPI Init/DeInit functions ---------*/
void 	spi_init();
void    SPI_ConfigClockRate (uint32_t SPI_CLOCKRATE);
void    SPI_CS_Low (void);
void    SPI_CS_High (void);
uint8_t SPI_SendByte (uint8_t data);
uint8_t SPI_RecvByte (void);

void SPI_MAX_init(void);
void SPI_MAX_CS_Low (void);
void SPI_MAX_CS_High (void);
uint16_t SPI_MAX_SendByte (uint16_t data);
uint16_t SPI_MAX_RecvByte (void);

#endif /* SPI_H_ */
