/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "integer.h"
#include "diskio.h"
#include "sdcard.h"

#define MMC             1

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
    BYTE pdrv				/* Physical drive nmuber (0..) */
){
  DSTATUS stat;
  int result;

  if (pdrv != MMC) {

      result = MMC_disk_initialize();

      // translate the reslut code here

      return stat;
  }else{

      return STA_NOINIT;
  }
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
    BYTE pdrv		/* Physical drive nmuber (0..) */
){
  DSTATUS stat;
  int result;

  if(pdrv != MMC){
      result = MMC_disk_status();
      return stat;
  }else{
      return STA_NOINIT;
  }
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
    BYTE pdrv,		/* Physical drive nmuber (0..) */
    BYTE *buff,		/* Data buffer to store read data */
    DWORD sector,	/* Sector address (LBA) */
    UINT count		/* Number of sectors to read (1..128) */
){
  DRESULT res;
  int result;

  if(pdrv != MMC){
      result = MMC_disk_read(buff, sector, count);      return res;

  }else{
      return RES_PARERR;
  }
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
    BYTE pdrv,			/* Physical drive nmuber (0..) */
    const BYTE *buff,	/* Data to be written */
    DWORD sector,		/* Sector address (LBA) */
    UINT count			/* Number of sectors to write (1..128) */
){
  DRESULT res;
  int result;

  if(pdrv != MMC){
      result = MMC_disk_write(buff, sector, count);      return res;

  }else{
      return RES_PARERR;
  }
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
    BYTE pdrv,		/* Physical drive nmuber (0..) */
    BYTE cmd,		/* Control code */
    void *buff		/* Buffer to send/receive control data */
){
  DRESULT res;
  int result;

  if(pdrv != MMC){
      result = MMC_disk_ioctl(cmd, buff);      return res;

  }else{
      return RES_PARERR;
  }
}
#endif
