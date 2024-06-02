/***************************************************************************
*
*    flash.c  -  description
*    ----------------------------------------------------
*    begin                : 25 dec 2014
*    copyright            : (C) 2014 Nik
*    email                : neekeetos@yahoo.com
*
*    modified             : 2016
*    copyright            : (C) 2016 by Andrey Sobol
*    email                : andrey.sobol.nn@gmail.com
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

#ifdef STM32F1XX
   #include "stm32f1xx_conf.h"
#endif
#ifdef STM32F0XX
    #include "stm32f0xx_conf.h"
#endif

#include "main.h"
#include "flash.h"

//----------------------------------------------------------------------------
uint8_t flash_ready(void) {	return !(FLASH->SR & FLASH_SR_BSY);}
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
uint8_t flash_opCompleted(void) {	return (FLASH->SR & FLASH_SR_EOP);}
//----------------------------------------------------------------------------
/*
To erase a page (2Kbytes), follow the procedure below:
1.Check that no Flash memory operation is ongoing by checking the BSY1 bit of the
FLASH status register (FLASH_SR).
2. Check and clear all error programming flags due to a previous programming. If not,
PGSERR is set.
3. Set the PER bit and select the page to erase (PNB) in the FLASH control register (FLASH_CR).
4. Set the STRT bit of the FLASH control register (FLASH_CR).
5. Wait until the BSY1 bit of the FLASH status register (FLASH_SR) is cleared.
*/
void flash_erase_page(uint32_t address) {

	 while(!flash_ready());
    FLASH->CR|= FLASH_CR_PER;
    FLASH->AR =address;
    FLASH->CR|= FLASH_CR_STRT;
    while(!flash_ready());
    FLASH->CR&= ~FLASH_CR_PER;
}
//----------------------------------------------------------------------------
//1. Write KEY1 = 0x4567 0123 in the FLASH key register (FLASH_KEYR)
//2. Write KEY2 = 0xCDEF 89AB in the FLASH key register (FLASH_KEYR).
void flash_unlock(void) {  FLASH->KEYR = 0x45670123;  FLASH->KEYR = 0xCDEF89AB;}
//----------------------------------------------------------------------------
void flash_lock() {	FLASH->CR |= FLASH_CR_LOCK;}
//----------------------------------------------------------------------------
void flash_write_word(uint32_t address,uint32_t data) {

    FLASH->CR |= FLASH_CR_PG;

    while(!flash_ready())		;

    *(__IO uint16_t*)address = (uint16_t)(data&0xFFFF);

    while(!flash_ready())		;

    FLASH->CR &= ~(FLASH_CR_PG);

}

//----------------------------------------------------------------------------
void flash_read(uint32_t address,void * dest,int lenBytes) {
	uint16_t * dst = (uint16_t * )dest;
	volatile uint16_t * src = (uint16_t * )address;
	for(int i =0;i<(lenBytes>>1);i++) dst[i] = src[i];
}

//----------------------------------------------------------------------------
void flash_write(uint32_t flashAddr, void * data,int lenBytes)
{
	uint16_t * buf = (uint16_t * )data;

    flash_unlock();

    for(int i =0;i<lenBytes;i+=2) {
    flash_write_word(flashAddr + i,buf[i>>1]);
    }
    flash_lock();

}
//----------------------------------------------------------------------------
