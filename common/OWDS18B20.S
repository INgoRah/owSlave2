
// Copyright (c) 2017, Tobias Mueller tm(at)tm3d.de
// All rights reserved. 
// 
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are 
// met: 
// 
//  * Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer. 
//  * Redistributions in binary form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the 
//    distribution. 
//  * All advertising materials mentioning features or use of this 
//    software must display the following acknowledgement: This product 
//    includes software developed by tm3d.de and its contributors. 
//  * Neither the name of tm3d.de nor the names of its contributors may 
//    be used to endorse or promote products derived from this software 
//    without specific prior written permission. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 

#define _CHANGEABLE_ID_
#define _ZERO_POLLING_

#include "../common/OWConfig.s"
#include "../common/OWCRC8.s"

.extern pack,8

//.extern  am2302_temp,2



.macro CHIP_INIT 	
Init_EEPROM_read:
	sbic _SFR_IO_ADDR(EECR), EEPE
	rjmp Init_EEPROM_read
	ldi r_temp,0
	out _SFR_IO_ADDR(EEARH), r_temp
	ldi r_temp,2
	out _SFR_IO_ADDR(EEARL), r_temp
	sbi _SFR_IO_ADDR(EECR), EERE
	in r_temp,_SFR_IO_ADDR(EEDR)
	sbrs r_temp,7
	rcall hrc_recall_eeprom_func
.endm

.macro COMMAND_TABLE
		rjmp h_readscratchpad
		rjmp h_writescratchpad
		rjmp h_convert_run
.endm

#include "../common/OWRomFunctions.s"
#include "../common/OWTimerInterrupt.s"



; Ab hier Geraeteabhaenging
#define OW_READ_SCRATCHPAD OW_FIRST_COMMAND+0
#define OW_WRITE_SCRATCHPAD OW_FIRST_COMMAND+1
#define OW_CONVERT_RUN  OW_FIRST_COMMAND+2


;---------------------------------------------------
;	READ COMMAND and start operation
;---------------------------------------------------


h_readcommand:
	clr r_bytep
#ifndef _DIS_FLASH_
	FLASH_COMMANDS ; muss zu erst sein....
#endif
	cjmp 0xBE,hrc_set_readscratchpad
	cjmp 0x4E,hrc_set_writescratchpad
	cjmp 0x44,hrc_set_convertT
	cjmp 0x48,hrc_copy_scratchpad
	cjmp 0xB8,hrc_recall_eeprom
	FW_CONFIG_INFO
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	//ldi r_mode,OW_SLEEP
	rjmp handle_end_sleep

hrc_set_readscratchpad:
	ldi r_mode,OW_READ_SCRATCHPAD
	ldi r_sendflag,1
	CRCInit2
	rjmp h_readscratchpad

hrc_set_writescratchpad:
	ldi r_mode,OW_WRITE_SCRATCHPAD
	ldi r_bytep,2 ;start to write in 2
	rjmp handle_end

hrc_recall_eeprom:
	rcall hrc_recall_eeprom_func
	rjmp handle_end



hrc_set_convertT:
	ldi r_temp,1
	sts gcontrol,r_temp
	ldi r_mode,OW_CONVERT_RUN
	ldi r_sendflag,3 ;set bit 0 and 1 for no zero polling
h_convert_run:
	ldi r_bcount,0
	ldi r_rwbyte,0
	rjmp handle_end_no_bcount	

hrc_copy_scratchpad:
	ldi r_bytep,2
	configZ pack,r_bytep
	clr r_bytep
hrc_copy_scratchpad_EEPROM_write:
	sbic _SFR_IO_ADDR(EECR), EEPE	
	rjmp hrc_copy_scratchpad_EEPROM_write
	ldi r_temp, (0<<EEPM1)|(0<<EEPM0)
	out _SFR_IO_ADDR(EECR), r_temp
	ldi r_temp,0
	out _SFR_IO_ADDR(EEARH),r_temp
	out _SFR_IO_ADDR(EEARL), r_bytep
	ld  r_rwbyte,Z+
	out _SFR_IO_ADDR(EEDR), r_rwbyte
	sbi _SFR_IO_ADDR(EECR), EEMPE
	sbi _SFR_IO_ADDR(EECR), EEPE
	inc r_bytep
	cpi r_bytep,3
	brne hrc_copy_scratchpad_EEPROM_write
	rjmp handle_end


hrc_recall_eeprom_func:
	ldi r_bytep,2
	configZ pack,r_bytep
	clr r_bytep
	clr r_temp
hrc_recall_eeprom_EEPROM_read:
	sbic _SFR_IO_ADDR(EECR), EEPE
	rjmp hrc_recall_eeprom_EEPROM_read
	out _SFR_IO_ADDR(EEARH), r_temp
	out _SFR_IO_ADDR(EEARL), r_bytep
	sbi _SFR_IO_ADDR(EECR), EERE
	in r_rwbyte,_SFR_IO_ADDR(EEDR)
	st Z+,r_rwbyte
	inc r_bytep
	cpi r_bytep,3
	brne hrc_recall_eeprom_EEPROM_read
	ret






;---------------------------------------------------
;   READ SCRATCHPAD
;---------------------------------------------------

h_readscratchpad:
	cpi  r_bytep,8
	breq h_readscratchpad_crc
	cpi  r_bytep,9
	breq h_readscratchpad_all
	configZ pack,r_bytep
	ld   r_rwbyte,Z
	rjmp h_readscratchpad_endc
h_readscratchpad_crc:
	lds  r_rwbyte,crc
h_readscratchpad_endc:
	inc  r_bytep
	ldi  r_bcount,1 
	rjmp handle_end
h_readscratchpad_all:
	rjmp handle_end_sleep





;---------------------------------------------------
;   WRITE SCRATCHPAD
;---------------------------------------------------

h_writescratchpad:
	configZ pack,r_bytep
	inc  r_bytep
	cpi  r_bytep,5
	breq h_writescratchpad_all
	st   Z,r_rwbyte
	rjmp handle_end
h_writescratchpad_all:
	ori r_rwbyte,0x1F ; Alle unteren Bits sind immer 1
	st   Z,r_rwbyte
	rjmp handle_end_sleep





#include "../common/OWPinInterrupt.s"
.end