/*
        LPCUSB, an USB device driver for LPC microcontrollers
        Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)
        Copyright (c) 2011-2013 BEEVC - Electronic Systems

        Redistribution and use in source and binary forms, with or without
        modification, are permitted provided that the following conditions are met:

        1. Redistributions of source code must retain the above copyright
           notice, this list of conditions and the following disclaimer.
        2. Redistributions in binary form must reproduce the above copyright
           notice, this list of conditions and the following disclaimer in the
           documentation and/or other materials provided with the distribution.
        3. The name of the author may not be used to endorse or promote products
           derived from this software without specific prior written permission.

        THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
        IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
        OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
        IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
        INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
        NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
        DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
        THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
        (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
        THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "LPC17xx.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include <stdio.h>
#include <string.h>         // memcpy
#include "usbapi.h"
#include "usbdebug.h"
#include "serial_fifo.h"
#include "sbl_config.h"

#define BULK_OUT_EP     0x05
#define BULK_IN_EP      0x82

#define MAX_PACKET_SIZE 64

#define LE_WORD(x)      ((x)&0xFF),((x)>>8)

fifo_t txfifo;
fifo_t rxfifo;

static unsigned char txbuf[SERIAL_FIFO_SIZE];
static unsigned char rxbuf[SERIAL_FIFO_SIZE];

static U8 abBulkBuf[64];
static U8 abClassReqData[8];

// forward declaration of interrupt handler
void USBIntHandler(void);

static U8 abDescriptors[] = {

    // device descriptor
      0x12,
      DESC_DEVICE,
      LE_WORD(0x0110),            // bcdUSB
      0xFF,                       // bDeviceClass
      0x00,                       // bDeviceSubClass
      0x00,                       // bDeviceProtocol
      MAX_PACKET_SIZE0,           // bMaxPacketSize
      LE_WORD(0x29C9),            // idVendor
      LE_WORD(0x001),               // idProduct
      LE_WORD(0x0100),            // bcdDevice
      0x01,                       // iManufacturer
      0x02,                       // iProduct
      0x03,                       // iSerialNumber
      0x01,                       // bNumConfigurations

    // configuration descriptor
      0x09,
      DESC_CONFIGURATION,
      LE_WORD(32),                // wTotalLength
      0x01,                       // bNumInterfaces
      0x01,                       // bConfigurationValue
      0x00,                       // iConfiguration
      0xC0,                       // bmAttributes
      0x32,                       // bMaxPower -- 0X32 ==> 100mA

    // generic class interface descriptor
      0x09,
      DESC_INTERFACE,
      0x00,                       // bInterfaceNumber
      0x00,                       // bAlternateSetting
      0x02,                       // bNumEndPoints
      0xFF,                       // bInterfaceClass = generic
      0x00,                       // bInterfaceSubClass
      0x00,                       // bInterfaceProtocol
      0x00,                       // iInterface
    // data EP OUT
      0x07,
      DESC_ENDPOINT,
      BULK_OUT_EP,                // bEndpointAddress
      0x02,                       // bmAttributes = bulk
      LE_WORD(MAX_PACKET_SIZE),   // wMaxPacketSize
      0x00,                       // bInterval
    // data EP in
      0x07,
      DESC_ENDPOINT,
      BULK_IN_EP,                 // bEndpointAddress
      0x02,                       // bmAttributes = bulk
      LE_WORD(MAX_PACKET_SIZE),   // wMaxPacketSize
      0x00,                       // bInterval

    // string descriptors122
      0x04,
      DESC_STRING,
      LE_WORD(0x0409),

      32,
      DESC_STRING,
      'B', 0, 'E', 0, 'E', 0, 'V', 0, 'E', 0, 'R', 0, 'Y', 0, 'C', 0, 'R', 0, 'E', 0, 'A', 0, 'T', 0, 'I', 0, 'V', 0, 'E', 0,

      24,
      DESC_STRING,
      'B', 0, 'E', 0, 'E', 0, 'T', 0, 'H', 0, 'E', 0, 'F', 0, 'I', 0, 'R', 0, 'S', 0, 'T', 0,

      24,
      DESC_STRING,
      '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0,/* bootloader version 3.x.x */

    // terminating zero
      0
};


/**
    Local function to handle incoming bulk data

    @param [in] bEP
    @param [in] bEPStatus
 */
static void BulkOut(U8 bEP, U8 bEPStatus)
{
  int i, iLen;
  int result = 0;

  result = _fifo_free(&rxfifo);

  if (result < MAX_PACKET_SIZE)
  {
    return;
  }

  // get data from USB into intermediate buffer
  iLen = USBHwEPRead(bEP, abBulkBuf, sizeof(abBulkBuf));

  for (i = 0; i < iLen; i++)
  {
    // put into FIFO
    if (!_fifo_put(&rxfifo, abBulkBuf[i]))
    {
      // overflow... :(
      ASSERT(FALSE);
      break;
    }
  }
}


/**
    Local function to handle outgoing bulk data

    @param [in] bEP
    @param [in] bEPStatus
 */
static void BulkIn(U8 bEP, U8 bEPStatus)
{
  int i, iLen;

  // Verifica se não há data para enviar ao PC
  if (_fifo_avail(&txfifo) == 0)
  {
    // no more data, disable further NAK interrupts until next USB frame
    USBHwNakIntEnable(0); // não é gerada nenhuma interrupção sempre que o host tenta ler/escrever nos EPs mas este estão cheios/vazios.
    return;
  }

  // get bytes from transmit FIFO into intermediate buffer
  for (i = 0; i < MAX_PACKET_SIZE; i++)
  {
    if (!_fifo_get(&txfifo, &abBulkBuf[i]))
    {
      break; // sai do ciclo quando o fifo fica vazio, terminando a transferência de informação do &txfifo para &abBulkBuf[].
    }
  }
  iLen = i;

  // send over USB
  if (iLen > 0)
  {
    USBHwEPWrite(bEP, abBulkBuf, iLen);
  }
}

static void USBFrameHandler(U16 wFrame)
{
  if (_fifo_avail(&txfifo) > 0) // Se há data no fifo de tx...
  {
    // data available, enable NAK interrupt on bulk in
    //
    // Caso esta interrupção não seja ligada, não há transmissão de dados, uma vez que a interrupão sobre o EP IN só
    // acontece após transferência com sucesso. É necessário activar esta interrupção para que seja chamada a BulkIn()
    // e acontecer a transferência.
    // Caso haja um RTOS, uma task pode ver regularmente quando o EP IN está vazio e fazer a transferência, em vez de
    // este USBFrame Interrupt.
    USBHwNakIntEnable(INACK_BI);
  }
}

/**
    Interrupt handler

    Simply calls the USB ISR
 */
//void USBIntHandler(void)
void USB_IRQHandler(void)
{
  USBHwISR();
}

void enable_USB_interrupts(void);

void USBSerial_Init(void)
{
    char serialnumber[10] = {0};
    char *pmem117;

    pmem117 = SECTOR_14_START;
    int serialnumber_present = 0;
    for (int i = 0; i < 10; i++) {
        serialnumber[9 - i] = *pmem117;
        pmem117++;
        if(*pmem117 != 0xFF)
            serialnumber_present = 1;
    }
    if(serialnumber_present == 1){
            abDescriptors[138] = serialnumber[0];
            abDescriptors[140] = serialnumber[1];
            abDescriptors[142] = serialnumber[2];
            abDescriptors[144] = serialnumber[3];
            abDescriptors[146] = serialnumber[4];
            abDescriptors[148] = serialnumber[5];
            abDescriptors[150] = serialnumber[6];
            abDescriptors[152] = serialnumber[7];
            abDescriptors[154] = serialnumber[8];
            abDescriptors[156] = serialnumber[9];
    }

    // initialise stack
    USBInit();

    // register descriptors
    USBRegisterDescriptors(abDescriptors);

    // register endpoint handlers
    USBHwRegisterEPIntHandler(BULK_IN_EP, BulkIn);
    USBHwRegisterEPIntHandler(BULK_OUT_EP, BulkOut);

    // register frame handler
    USBHwRegisterFrameHandler(USBFrameHandler);

    // enable bulk-in interrupts on NAKs
    USBHwNakIntEnable(INACK_BI); // é gerada uma interrupção sempre que o host tenta ler do EP IN mas este está vazio.

    // initialise VCOM
    fifo_init(&rxfifo, rxbuf);
    fifo_init(&txfifo, txbuf);

    NVIC_EnableIRQ(USB_IRQn);

    // connect to bus
    USBHwConnect(TRUE);
}
