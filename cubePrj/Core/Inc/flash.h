/***************************************************************************
*
*    flash.h  -  description
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
#ifndef FLASH_H_
#define FLASH_H_

#include  <stm32f1xx.h>

//#define FLASH_PAGE				 ((uint8)0x7F)

uint8_t flash_ready(void) ;
void flash_erase_page(uint32_t address) ;
void flash_unlock(void) ;
void flash_lock() ;
void flash_write_word(uint32_t address,uint32_t data) ;
void flash_read(uint32_t address,void * dest,int lenBytes);
void flash_write(uint32_t flashAddr, void * data,int lenBytes);

#endif /* FLASH_H_ */
