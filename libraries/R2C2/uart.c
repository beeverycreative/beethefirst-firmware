/* Copyright (c) 2011-2013 BEEVC - Electronic Systems        */
/*
 * This file is part of BEESOFT software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version. BEESOFT is
 * distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
 * License for more details. You should have received a copy of the
 * GNU General Public License along with BEESOFT. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

#include "uart.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"

//Debug UART can be 0 or 3
#define DBG_UART_NUM  3
#define DBG_UART      LPC_UART3

void uart_init(void)
{
	// UART Configuration structure variable
	UART_CFG_Type UARTConfigStruct;
	// UART FIFO configuration Struct variable
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	// Pin configuration for UART
	PINSEL_CFG_Type PinCfg;

	/*
	* Initialize UART3 pin connect: P4.28 -> TXD3; P4.29 -> RXD3
	* or P0.2 -> TXD0, P0.3 -> RXD0
	*/
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
#if DBG_UART_NUM == 3	
	PinCfg.Funcnum = PINSEL_FUNC_3;
	PinCfg.Portnum = 4;
	PinCfg.Pinnum = 28;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 29;
	PINSEL_ConfigPin(&PinCfg);
#else
	PinCfg.Funcnum = PINSEL_FUNC_1;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 3;
	PINSEL_ConfigPin(&PinCfg);
#endif	

	/* Initialize UART Configuration parameter structure to default state:
		* Baudrate = as below
		* 8 data bit
		* 1 Stop bit
		* None parity
		*/
	UART_ConfigStructInit(&UARTConfigStruct);
	UARTConfigStruct.Baud_rate = 57600;

	// Initialize UART peripheral with given to corresponding parameter
	UART_Init(DBG_UART, &UARTConfigStruct);

	/* Initialize FIFOConfigStruct to default state:
	*                              - FIFO_DMAMode = DISABLE
	*                              - FIFO_Level = UART_FIFO_TRGLEV0
	*                              - FIFO_ResetRxBuf = ENABLE
	*                              - FIFO_ResetTxBuf = ENABLE
	*                              - FIFO_State = ENABLE
	*/
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

	// Initialize FIFO for UART peripheral
	UART_FIFOConfig(DBG_UART, &UARTFIFOConfigStruct);

	// Enable UART Transmit
	UART_TxCmd(DBG_UART, ENABLE);
}

char uart_data_available(void)
{
	return (DBG_UART->LSR & UART_LSR_RDR);
}

char uart_receive(void)
{
	return (UART_ReceiveByte(DBG_UART));
}

void uart_send(char byte)
{
  //LSR - Line Status Register. Contains flags for transmit and
  //receive status, including line errors.

	while ( (DBG_UART->LSR & UART_LSR_THRE) == 0) ;
	UART_SendByte(DBG_UART, byte);
}

void uart_sendHex(unsigned long data)
{
  uart_writestr("0x");

  unsigned char buff[8] = {'1','1','1','1','1','1','1','1'};

  for(int i = 8;i>0;i--){

      unsigned char b = data & 0x0F;

      unsigned char bW = b + '0';
      if(bW > '9'){
          bW = b +'0' + 7;
      }

      //uart_send(bW);
      buff[i-1] = bW;
      data = data >> 4;
  }

  uart_writestr(buff);

}

void uart_writestr(char *data)
{
	uint8_t i = 0;
	char r;

 	while ((r = data[i++]))
		uart_send(r);
}
