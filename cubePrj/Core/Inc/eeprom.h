/***************************************************************************
*
*    eeprom.h  -  description
*    ----------------------------------------------------
*    begin                : 25 dec 2014
*    copyright            : (C) 2014 Nik
*    email                : neekeetos@yahoo.com
*
****************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef EEPROM_H_
#define EEPROM_H_

/*
After reset, write into the FLASH control register (FLASH_CR) is not allowed so as to
protect the Flash memory against possible unwanted operations due, for example, to
electric disturbances. The following sequence unlocks these registers:
1. Write KEY1 = 0x4567 0123 in the FLASH key register (FLASH_KEYR)
2. Write KEY2 = 0xCDEF 89AB in the FLASH key register (FLASH_KEYR).
*/
#define FLASH_EEPROM_MAGIC 0xCAFE0620
#define FLASH_EEPROM_EMPTY 0xFFFFFFFF


/*
#ifdef STM32F0XX //page is 2kBt
    #define FLASH_PAGE_SIZE 0x800
#else // 1 kBt
    #define FLASH_PAGE_SIZE 0x400
#endif
*/
void eepromInit(void);
int  eepromSaveParam(int type, void * src,int slen);
int  eepromLoadParam(int type, void * dst);
int  eepromVoidParam(int type);
void eepromErasePage(uint32_t address);
uint32_t eepromGetFreeAddress(uint32_t len);
void eepromCopyPageData(void);

//extern uint32_t eeprom;

#endif /* EEPROM_H_ */
