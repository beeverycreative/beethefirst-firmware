/* Copyright (c) 2013 bitBOX */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

//#include "core_cm3.h"
#include "lpc17xx_nvic.h"
#include "spi.h"
#include "ff.h"
#include "debug.h"

/**
 * @brief  Initiate a system reset request.
 *
 * Initiate a system reset request to reset the MCU
 */
static __INLINE void reset(void)
{
  SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)      |
                 (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                 SCB_AIRCR_SYSRESETREQ_Msk);                   /* Keep priority group unchanged */
  __DSB();                                                     /* Ensure completion of memory access */
  while(1);                                                    /* wait until reset */
}

/* Activa modo bootloader. É criado um ficheiro com nome "boot" no uSDCard. O bootloader irá verificar a existência deste
 * ficheiro para entrar em bootloader. */
unsigned int go_to_bootloader (void)
{
  /* initialize SPI for SDCard */
  spi_init();

  /* necessário para a FAT */
  FATFS fs;       /* Work area (file system object) for logical drive */
  FIL file_boot;       /* file object */
  FRESULT res;    /* FatFs function common result code */

  /* Register a work area for logical drive 0 */
  res = f_mount(0, &fs);
  if(res){
      debug("Err mount fs\n");
      return 1;
  }//no need for else

  /* Abre o ficheiro "boot". Caso não exista, é criado. */
  res = f_open(&file_boot, "boot", FA_WRITE|FA_OPEN_ALWAYS);
  if (res){
      debug("Error opening/creating boot\n");

      f_mount(0, NULL); // desmonta
      return 2;
  }//no need for else

  res = f_close(&file_boot);
  if (res){
      debug("Error closing boot\n");

      f_mount(0, NULL); // desmonta
      return 3;
  }//no need for else

  f_mount(0, NULL); // desmonta

  reset(); // reset ao sistema

  return 0; // tudo bem, sem erro
}
unsigned int go_to_reset (void)
{
   reset(); // reset ao sistema

  return 0; // tudo bem, sem erro
}
