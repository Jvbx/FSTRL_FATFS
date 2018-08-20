/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/* 
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future. 
 * Kept to ensure backward compatibility with previous CubeMx versions when 
 * migrating projects. 
 * User code previously added there should be copied in the new user sections before 
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "at45db.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
at45db at45db_dataflash;
/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);  
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read, 
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */  
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
    Stat = STA_NOINIT;
    if (at45db_init(&at45db_dataflash) ==  AT45DB_OK) return RES_OK;
    return Stat;
  /* USER CODE END INIT */
}
 
/**
  * @brief  Gets Disk Status 
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
    Stat = STA_NOINIT;
   AT45DB_RESULT at45db_resp = at45db_isrdy(&at45db_dataflash);
   switch  (at45db_resp)
   {
     case             AT45DB_READY: {return RES_OK; break;} 
     case              AT45DB_BUSY: {return RES_OK; break;}
     case             AT45DB_ERROR: {return RES_ERROR; break;}
     case  AT45DB_INVALID_ARGUMENT: {return RES_PARERR; break;}
                           default: return Stat;  
   }
   
    return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s) 
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
  /* USER CODE HERE */
  AT45DB_RESULT res;
    while(count > 0) 
      {
        res = at45db_read_page(&at45db_dataflash, buff, sector);
        if (res != AT45DB_OK) return RES_ERROR;
        buff += _MIN_SS;
        sector++;
        count--;
      } 
   return RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)  
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{ 
  /* USER CODE BEGIN WRITE */
  /* USER CODE HERE */
   AT45DB_RESULT res;
    while(count > 0) 
      {
        res = at45db_w_pagethroughbuf1(&at45db_dataflash, buff, sector, 0);
        if (res != AT45DB_OK) return RES_ERROR;
        sector++;
        buff += _MIN_SS;
        count--;
      }
  // if (at45db_wait_cplt(&at45db_dataflash) == AT45DB_OK) return RES_OK;     
   return RES_OK;

  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation  
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
  DRESULT res = RES_ERROR;
  switch (cmd)
  {
  /* Generic command (Used by FatFs) */
    case CTRL_SYNC:{       return RES_OK;         break;}        /* Complete pending write process (needed at _FS_READONLY == 0) */
    case GET_SECTOR_COUNT:{*(uint32_t*)buff = 4096;  return RES_OK;      break;}        /* Get media size (needed at _USE_MKFS == 1) */
    case GET_SECTOR_SIZE:{ *(uint16_t*)buff = (uint8_t)512;            return RES_OK;      break;}        /* Get sector size (needed at _MAX_SS != _MIN_SS) */  //<- not our case anyway
    case GET_BLOCK_SIZE:{ *(uint32_t*)buff = 1;      return RES_OK;      break;}        /* Get erase block size (needed at _USE_MKFS == 1) */
    case CTRL_TRIM:{return RES_OK; break;}       /* Inform device that the data on the block of sectors is no longer used (needed at _USE_TRIM == 1) */

/* Generic command (Not used by FatFs) */
    case CTRL_POWER:  {    return  RES_PARERR;    break;}       /* Get/Set power status */
    case CTRL_LOCK:   {    return  RES_PARERR;    break;}       /* Lock/Unlock media removal */
    case CTRL_EJECT:  {    return  RES_PARERR;    break;}       /* Eject media */
    case CTRL_FORMAT: {    return  RES_PARERR;    break;}       /* Create physical format on the media */

/* MMC/SDC specific ioctl command */
    case MMC_GET_TYPE:{    return  RES_PARERR;    break;}       /* Get card type */
    case MMC_GET_CSD: {    return  RES_PARERR;    break;}       /* Get CSD */
    case MMC_GET_CID: {    return  RES_PARERR;    break;}       /* Get CID */
    case MMC_GET_OCR: {    return  RES_PARERR;    break;}       /* Get OCR */
    case MMC_GET_SDSTAT:{  return  RES_PARERR;    break;}       /* Get SD status */

/* ATA/CF specific ioctl command */
    case ATA_GET_REV:{     return  RES_PARERR;    break;}       /* Get F/W revision */
    case ATA_GET_MODEL:{   return  RES_PARERR;    break;}       /* Get model name */
    case ATA_GET_SN	:{     return  RES_PARERR;    break;}       /* Get serial number */
    default:  return res;
  }
  
  
  
  
  
   
    return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
