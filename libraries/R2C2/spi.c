#include "spi.h"
#include "lpc17xx_pinsel.h"
#include "ios.h"

/********************************************************************//**
 * @brief		Initializes the SPIx peripheral according to the specified
 *               parameters in the UART_ConfigStruct.
 * @param[in]	SPIx	SPI peripheral selected, should be LPC_SPI
 * @param[in]	SPI_ConfigStruct Pointer to a SPI_CFG_Type structure
 *                    that contains the configuration information for the
 *                    specified SPI peripheral.
 * @return 		None
 *********************************************************************/
void spi_init (void)
{
#ifndef BTF_SMOOTHIE
  /* Enable SSPI0 block */
  LPC_SC->PCONP |= (1 << 21);

  /* Set SSEL0 as GPIO, output high */
  LPC_PINCON->PINSEL1 &= ~(3 << 0);          /* Configure P0.16(SSEL) as GPIO */
  LPC_GPIO0->FIODIR |= (1 << 16);            /* set P0.16 as output */

  /* Configure other SSP pins: SCK, MISO, MOSI */
  LPC_PINCON->PINSEL0 &= ~(3UL << 30);
  LPC_PINCON->PINSEL0 |=  (2UL << 30);          /* P0.15: SCK0 */
  LPC_PINCON->PINSEL1 &= ~((3<<2) | (3<<4));
  LPC_PINCON->PINSEL1 |=  ((2<<2) | (2<<4));  /* P0.17: MISO0, P0.18: MOSI0 */

  /* Configure SSP0_PCLK to CCLK(100MHz), default value is CCLK/4 */
  LPC_SC->PCLKSEL1 &= ~(3 << 10);
  LPC_SC->PCLKSEL1 |=  (1 << 10);  /* SSP0_PCLK=CCLK */

  /* 8bit, SPI frame format, CPOL=0, CPHA=0, SCR=0 */
  LPC_SSP0->CR0 = (0x07 << 0) |     /* data width: 8bit*/
      (0x00 << 4) |     /* frame format: SPI */
      (0x00 << 6) |     /* CPOL: low level */
      (0x00 << 7) |     /* CPHA: first edge */
      (0x00 << 8);      /* SCR = 0 */

  /* Enable SSP0 as a master */
  LPC_SSP0->CR1 = (0x00 << 0) |   /* Normal mode */
      (0x01 << 1) |   /* Enable SSP0 */
      (0x00 << 2) |   /* Master */
      (0x00 << 3);    /* slave output disabled */

  /* Configure SSP0 clock rate to 400kHz (100MHz/250) */
  SPI_ConfigClockRate (SPI_CLOCKRATE_LOW);
  //setSSPclock(LPC_SSP0, 400000);

  /* Set SSEL to high */
  SPI_CS_High ();
#endif
#ifdef BTF_SMOOTHIE
  /* Enable SSPI1 block */
  LPC_SC->PCONP |= (1 << 10);

  PINSEL_CFG_Type PinCfg;

  //SSEL1 CONFIG
  PinCfg.Funcnum = PINSEL_FUNC_3;
  PinCfg.Portnum = 0;
  PinCfg.Pinnum = 6;
  PINSEL_ConfigPin(&PinCfg);
  LPC_GPIO0->FIODIR |= (1 << 6);            /* set P0.6 as output */

  //MOSI1 CONFIG
  PinCfg.Funcnum = PINSEL_FUNC_3;
  PinCfg.Portnum = 0;
  PinCfg.Pinnum = 9;
  PINSEL_ConfigPin(&PinCfg);
  LPC_GPIO0->FIODIR |= (1 << 9);            /* set P0.9 as output */
  //MISO1 CONFIG
  PinCfg.Funcnum = PINSEL_FUNC_3;
  PinCfg.Portnum = 0;
  PinCfg.Pinnum = 8;
  PINSEL_ConfigPin(&PinCfg);
  LPC_GPIO0->FIODIR &= ~(1 << 8);            /* set P0.8 as INPUT */
  //SCK1 CONFIG
  PinCfg.Funcnum = PINSEL_FUNC_3;
  PinCfg.Portnum = 0;
  PinCfg.Pinnum = 7;
  PINSEL_ConfigPin(&PinCfg);
  LPC_GPIO0->FIODIR |= (1 << 7);            /* set P0.7 as output */

  /* Configure SSP1_PCLK to CCLK(100MHz), default value is CCLK/4 */
  LPC_SC->PCLKSEL0 &= ~(3 << 20);
  LPC_SC->PCLKSEL0 |=  (1 << 20);  /* SSP1_PCLK=CCLK */

  /* 8bit, SPI frame format, CPOL=0, CPHA=0, SCR=0 */
  LPC_SSP1->CR0 = (0x07 << 0) |     /* data width: 8bit*/
      (0x00 << 4) |     /* frame format: SPI */
      (0x00 << 6) |     /* CPOL: low level */
      (0x00 << 7) |     /* CPHA: first edge */
      (0x00 << 8);      /* SCR = 0 */

  /* Enable SSP0 as a master */
  LPC_SSP1->CR1 = (0x00 << 0) |   /* Normal mode */
      (0x01 << 1) |   /* Enable SSP0 */
      (0x00 << 2) |   /* Master */
      (0x00 << 3);    /* slave output disabled */

  /* Configure SSP0 clock rate to 400kHz (100MHz/250) */
  SPI_ConfigClockRate (SPI_CLOCKRATE_LOW);
  //setSSPclock(LPC_SSP0, 400000);

  /* Set SSEL to high */
  SPI_CS_High ();



#endif
}
/**
 * @brief  Configure SSP0 clock rate.
 *
 * @param  SPI_CLOCKRATE: Specifies the SPI clock rate.
 *         The value should be SPI_CLOCKRATE_LOW or SPI_CLOCKRATE_HIGH.
 * @retval None
 *
 * SSP0_CLK = CCLK / SPI_CLOCKRATE
 */
void SPI_ConfigClockRate (uint32_t SPI_CLOCKRATE)
{
  /* CPSR must be an even value between 2 and 254 */
  //LPC_SSP0->CPSR = (SPI_CLOCKRATE & 0xFE);
#ifndef BTF_SMOOTHIE
  LPC_SSP0->CPSR = (SPI_CLOCKRATE & 0xFF);
#endif
#ifdef BTF_SMOOTHIE
  LPC_SSP1->CPSR = (SPI_CLOCKRATE & 0xFF);
#endif
}

/**
 * @brief  Set SSEL to low: select spi slave.
 *
 * @param  None.
 * @retval None
 */
void SPI_CS_Low (void)
{
  /* SSEL is GPIO, set to high.  */
#ifndef BTF_SMOOTHIE
  LPC_GPIO0->FIOPIN &= ~(1 << 16);
#endif
#ifdef BTF_SMOOTHIE
  LPC_GPIO0->FIOPIN &= ~(1 << 6);
#endif
}

/**
 * @brief  Set SSEL to high: de-select spi slave.
 *
 * @param  None.
 * @retval None
 */
void SPI_CS_High (void)
{
  /* SSEL is GPIO, set to high.  */
#ifndef BTF_SMOOTHIE
  LPC_GPIO0->FIOPIN |= (1 << 16);
#endif
#ifdef BTF_SMOOTHIE
  LPC_GPIO0->FIOPIN |= (1 << 6);
#endif
}

/**
 * @brief  Send one byte via MOSI and simutaniously receive one byte via MISO.
 *
 * @param  data: Specifies the byte to be sent out.
 * @retval Returned byte.
 *
 * Note: Each time send out one byte at MOSI, Rx FIFO will receive one byte.
 */
uint8_t SPI_SendByte (uint8_t data)
{
#ifndef BTF_SMOOTHIE
  /* Put the data on the FIFO */
  LPC_SSP0->DR = data;
  /* Wait for sending to complete */
  while (LPC_SSP0->SR & SSP_SR_BSY);
  /* Return the received value */
  return (LPC_SSP0->DR);
#endif
#ifdef BTF_SMOOTHIE
  /* Put the data on the FIFO */
  LPC_SSP1->DR = data;
  /* Wait for sending to complete */
  while (LPC_SSP1->SR & SSP_SR_BSY);
  /* Return the received value */
  return (LPC_SSP1->DR);
#endif
}

/**
 * @brief  Receive one byte via MISO.
 *
 * @param  None.
 * @retval Returned received byte.
 */
uint8_t SPI_RecvByte (void)
{
  /* Send 0xFF to provide clock for MISO to receive one byte */
  return SPI_SendByte (0xFF);
}
