/***************************************************************************
*
*    eeprom.c  -  description
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

#include "stdint.h"

#include "main.h"
#include "flash.h"
#include "eeprom.h"

uint32_t eeprom;

#ifdef STM32G030xx //page is 2kBt
    const uint16_t __attribute__ (( aligned(2048) )) __attribute__ ((section (".eeprom"))) eep1[1024]  = { [0 ... 1023] = 0xffff };
    //const uint16_t __attribute__ (( aligned(2048) )) __attribute__ ((section (".eeprom"))) eep2[1024]  = { [0 ... 1023] = 0xffff };
#else
    const uint16_t __attribute__ (( aligned(1024) )) __attribute__ ((section (".eeprom"))) eep1[512]  = { [0 ... 511] = 0xffff };
 //   const uint16_t __attribute__ (( aligned(1024) )) __attribute__ ((section (".eeprom"))) eep2[512]  = { [0 ... 511] = 0xffff };
#endif


typedef struct
{
    uint16_t type;
    uint16_t len;
} eeprom_record_hdr;

//----------------------------------------------------------------------------
void eepromErasePage(uint32_t address)
{
    flash_unlock();
    flash_erase_page((uint32_t)address);
    flash_lock();
}
//----------------------------------------------------------------------------
uint32_t eepromGetFreeAddress(uint32_t slen)
{
    uint32_t addr = (eeprom + 4);
    eeprom_record_hdr hdr;

    do {
        flash_read(addr,&hdr,sizeof(hdr));
        if(hdr.len != 0xFFFF) addr += hdr.len;
    }while( (hdr.len != 0xFFFF) && ( addr < (eeprom + FLASH_PAGE_SIZE) ) );

    if( addr >= (eeprom + FLASH_PAGE_SIZE - slen)) return (0);

    return (addr);
}
//----------------------------------------------------------------------------
int eepromSaveParam(int type, void * src,int slen) // 0 - ok
{
    char data[64];
    uint32_t addr = (eeprom + 4);
    eeprom_record_hdr hdr;
    uint32_t flag = 0;


    if( eepromLoadParam( type, data ) > 0 )
    {
        for(int i =0;i<slen;i++)
            if( ((char * )src)[i] != data[i] ) flag++;

        if(flag == 0) return (2);
    }

    eepromVoidParam(type);

    addr = eepromGetFreeAddress( slen + sizeof(hdr) );

    hdr.len = slen + sizeof(hdr);
    hdr.type = type;

    flash_write(addr,&hdr,sizeof(hdr));
    flash_write(addr + sizeof(hdr),src,slen);

    return (1);
}
//----------------------------------------------------------------------------
int eepromVoidParam(int type) // 0 - ok
{
    uint32_t addr = eeprom + 4;
    eeprom_record_hdr hdr;

    do {
        flash_read(addr, &hdr, sizeof(hdr) );

        if(hdr.type == type )	{
            hdr.type = 0;
            flash_write(addr, &hdr, sizeof(hdr) ); // zero type
        }
        if(hdr.len != 0xFFFF) addr += hdr.len;

    }while( (hdr.len != 0xFFFF) && ( addr < ((uint32_t)eeprom) + FLASH_PAGE_SIZE ) );

    return (0);
}
//----------------------------------------------------------------------------
int eepromLoadParam(int type, void * dst)
{
    int n = 0;
    uint32_t addr = eeprom + 4;
    eeprom_record_hdr hdr;

    do {
        flash_read( addr, &hdr, sizeof(hdr) );

        if( hdr.type == type ){ flash_read( addr +  sizeof(hdr), dst, hdr.len - sizeof(hdr) );	n++;}

        addr += hdr.len;

    }while( (hdr.len != 0xFFFF) && ( addr < ((uint32_t)eeprom) + FLASH_PAGE_SIZE ) );

    return (n);
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
void eepromInit(void)
{
    uint32_t tmp1;

    eeprom = (uint32_t)eep1;

    flash_read((uint32_t)eep1,&tmp1,sizeof(tmp1));

    if(tmp1 == FLASH_EEPROM_MAGIC ) {
    	eeprom = (uint32_t)eep1; return;
    }
    //if(tmp2 == FLASH_EEPROM_MAGIC && tmp1 == (uint32_t)FLASH_EEPROM_EMPTY) {eeprom = (uint32_t)eep2; return;}

#ifndef EEPROM_OFF
    eepromErasePage((uint32_t)eep1);
    //eepromErasePage((uint32_t)eep2);
#endif

    eeprom = (uint32_t)eep1;
#ifndef EEPROM_OFF
    uint32_t magic = (FLASH_EEPROM_MAGIC);
    flash_write(eeprom,&magic,sizeof(magic));
#endif
}
//----------------------------------------------------------------------------
