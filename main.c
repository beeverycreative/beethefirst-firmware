/* Copyright (c) 2011-2013 BEEVC - Electronic Systems        */


#include "r2c2.h"

#define USER_FLASH_START 0x10000 /* For USB bootloader */
//#define USER_FLASH_START 0x0 /* No USB bootloader */

extern int app_main (void);

void startup_delay(void)
{
  for (volatile unsigned long i = 0; i < 500000; i++) { ; }
}

/*********************************************************************//**
 * @brief	Main sub-routine
 **********************************************************************/
int main(void)
{
  // DeInit NVIC and SCBNVIC
  NVIC_DeInit();
  NVIC_SCBDeInit();

  /* Configure the NVIC Preemption Priority Bits:
   * two (2) bits of preemption priority, six (6) bits of sub-priority.
   * Since the Number of Bits used for Priority Levels is five (5), so the
   * actual bit number of sub-priority is three (3)
   */
  NVIC_SetPriorityGrouping(0x05);

  /* Change the Vector Table to the USER_FLASH_START
  in case the user application uses interrupts */
  SCB->VTOR = (USER_FLASH_START & 0x1FFFFF80);

  SysTickTimer_Init(); // Initialize the timer for millis()

  delay_ms(1000);

  // Initialize USB<->Serial
  serial_init();
  
  app_main ();

  /* should never get here */
  while(1) ;
}

