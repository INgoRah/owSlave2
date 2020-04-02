
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
#define _HANDLE_CC_COMMAND_
//#define  __4MHZ__
//#define _DB_

#include "../common/OWConfig.s"
#include "../common/OWCRC8.s"

.extern pack1,8

.extern pack2,8



.comm block,1 ; Block der augegeben, geschrieben wird (Parameter von READ/WRITE Scratchpad)
.comm cpsp,1 ; Copy Scratchpad marker


.macro CHIP_INIT 	;r_temp is pushed other Registers should be saved
	ldi r_temp,0
	sts cpsp,r_temp
Init_EEPROM_read:
	push r_bytep
	push r_rwbyte
	sbic _SFR_IO_ADDR(EECR), EEPE
	rjmp Init_EEPROM_read
	ldi r_temp,0
	out _SFR_IO_ADDR(EEARH), r_temp
	ldi r_temp,2
	out _SFR_IO_ADDR(EEARL), r_temp
	sbi _SFR_IO_ADDR(EECR), EERE
	in r_temp,_SFR_IO_ADDR(EEDR)
	sbrs r_temp,7
	rcall hrc_recall_eeprom_func1
	pop r_rwbyte
	pop r_bytep
.endm

.macro COMMAND_TABLE
		rjmp h_readscratchpad1
		rjmp h_writescratchpad1
		rjmp h_convert_run1
		rjmp h_readscratchpad2
		rjmp h_writescratchpad2
		rjmp h_convert_run2
.endm

#include "../common/OWRomFunctionsDual.s"
#include "../common/OWTimerInterrupt.s"



; Ab hier Geraeteabhaenging
#define OW_READ_SCRATCHPAD1 OW_FIRST_COMMAND+0
#define OW_WRITE_SCRATCHPAD1 OW_FIRST_COMMAND+1
#define OW_CONVERT_RUN1  OW_FIRST_COMMAND+2
#define OW_READ_SCRATCHPAD2 OW_FIRST_COMMAND+3
#define OW_WRITE_SCRATCHPAD2 OW_FIRST_COMMAND+4
#define OW_CONVERT_RUN2  OW_FIRST_COMMAND+5


;---------------------------------------------------
;	READ COMMAND and start operation
;---------------------------------------------------

#ifdef _HANDLE_CC_COMMAND_
h_readcommand12:
	clr r_bytep
	cjmp 0x44,hrc_set_convertT12
	ldi r_mode,OW_SLEEP
	rjmp handle_end
#endif


h_readcommand1:
	clr r_bytep
#ifndef _DIS_FLASH_
	FLASH_COMMANDS ; muss zu erst sein....
#endif
	cjmp 0xBE,hrc_set_readscratchpad1
	cjmp 0x4E,hrc_set_writescratchpad1
	cjmp 0x44,hrc_set_convertT1
	cjmp 0x48,hrc_copy_scratchpad1
	cjmp 0xB8,hrc_recall_eeprom1
	FW_CONFIG_INFO1
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	ldi r_mode,OW_SLEEP
	rjmp handle_end

hrc_set_readscratchpad1:
	ldi r_mode,OW_READ_SCRATCHPAD1
	ldi r_sendflag,1
	CRCInit2
	rjmp h_readscratchpad1

hrc_set_writescratchpad1:
	ldi r_mode,OW_WRITE_SCRATCHPAD1
	ldi r_bytep,2 ;start to write in 2
	rjmp handle_end

hrc_recall_eeprom1:
	rcall hrc_recall_eeprom_func1
	rjmp handle_end

#ifdef _HANDLE_CC_COMMAND_
hrc_set_convertT12:
	ldi r_temp,3
	sts gcontrol,r_temp
	rjmp hrc_set_convertT12b  //egal ob hrc_set_convertT12b oder hrc_set_convertT22b
#endif

hrc_set_convertT1:
	ldi r_temp,1
	sts gcontrol,r_temp
hrc_set_convertT12b:
	ldi r_mode,OW_CONVERT_RUN1
	ldi r_sendflag,3 ;set bit 0 and 1 for no zero polling
h_convert_run1:
	ldi r_bcount,0
	ldi r_rwbyte,0
	rjmp handle_end_no_bcount	



hrc_copy_scratchpad1:
	ldi r_bytep,2
	configZ pack1,r_bytep
	clr r_bytep
hrc_copy_scratchpad_EEPROM_write1:
	sbic _SFR_IO_ADDR(EECR), EEPE	
	rjmp hrc_copy_scratchpad_EEPROM_write1
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
	brne hrc_copy_scratchpad_EEPROM_write1
	rjmp handle_end


hrc_recall_eeprom_func1:
	ldi r_bytep,2
	configZ pack1,r_bytep
	clr r_bytep
	clr r_temp
hrc_recall_eeprom_EEPROM_read1:
	sbic _SFR_IO_ADDR(EECR), EEPE
	rjmp hrc_recall_eeprom_EEPROM_read1
	out _SFR_IO_ADDR(EEARH), r_temp
	out _SFR_IO_ADDR(EEARL), r_bytep
	sbi _SFR_IO_ADDR(EECR), EERE
	in r_rwbyte,_SFR_IO_ADDR(EEDR)
	st Z+,r_rwbyte
	inc r_bytep
	cpi r_bytep,3
	brne hrc_recall_eeprom_EEPROM_read1
	ret






;---------------------------------------------------
;   READ SCRATCHPAD
;---------------------------------------------------

h_readscratchpad1:
	cpi  r_bytep,8
	breq h_readscratchpad_crc1
	cpi  r_bytep,9
	breq h_readscratchpad_all1
	configZ pack1,r_bytep
	ld   r_rwbyte,Z
	rjmp h_readscratchpad_endc1
h_readscratchpad_crc1:
	lds  r_rwbyte,crc
h_readscratchpad_endc1:
	inc  r_bytep
	ldi  r_bcount,1 
	rjmp handle_end
h_readscratchpad_all1:
	rjmp handle_end_sleep





;---------------------------------------------------
;   WRITE SCRATCHPAD
;---------------------------------------------------

h_writescratchpad1:
	configZ pack1,r_bytep
	inc  r_bytep
	cpi  r_bytep,5
	breq h_writescratchpad_all1
	st   Z,r_rwbyte
	rjmp handle_end
h_writescratchpad_all1:
	;ori r_rwbyte,0x1F ; Alle unteren Bits sind immer 1 -->VOC use different
	st   Z,r_rwbyte
	rjmp handle_end_sleep

;*****************************************************************************************************************************************************************************************
;*****************************************************************************************************************************************************************************************
;*****************************************************************************************************************************************************************************************
;*****************************************************************************************************************************************************************************************
;*****************************************************************************************************************************************************************************************



h_readcommand2:
	clr r_bytep
#ifndef _DIS_FLASH_
	FLASH_COMMANDS ; muss zu erst sein....
#endif
	cjmp 0xBE,hrc_set_readscratchpad2
	cjmp 0x4E,hrc_set_writescratchpad2
	cjmp 0x44,hrc_set_convertT2
	cjmp 0x48,hrc_copy_scratchpad2
	cjmp 0xB8,hrc_recall_eeprom2
	FW_CONFIG_INFO2
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	ldi r_mode,OW_SLEEP
	rjmp handle_end

hrc_set_readscratchpad2:
	ldi r_mode,OW_READ_SCRATCHPAD2
	ldi r_sendflag,1
	CRCInit2
	rjmp h_readscratchpad2

hrc_set_writescratchpad2:
	ldi r_mode,OW_WRITE_SCRATCHPAD2
	ldi r_bytep,2 ;start to write in 2
	rjmp handle_end

hrc_recall_eeprom2:
	rcall hrc_recall_eeprom_func2
	rjmp handle_end

hrc_set_convertT2:
	ldi r_temp,1
	sts gcontrol,r_temp
hrc_set_convertT22b:
	ldi r_mode,OW_CONVERT_RUN2
	ldi r_sendflag,3 ;set bit 0 and 1 for no zero polling
h_convert_run2:
	ldi r_bcount,0
	ldi r_rwbyte,0
	rjmp handle_end_no_bcount	



hrc_copy_scratchpad2:
	ldi r_bytep,2
	configZ pack2,r_bytep
	clr r_bytep
hrc_copy_scratchpad_EEPROM_write2:
	sbic _SFR_IO_ADDR(EECR), EEPE	
	rjmp hrc_copy_scratchpad_EEPROM_write2
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
	brne hrc_copy_scratchpad_EEPROM_write2
	rjmp handle_end


hrc_recall_eeprom_func2:
	ldi r_bytep,2
	configZ pack2,r_bytep
	//clr r_bytep
	ldi r_bytep,8
	clr r_temp
hrc_recall_eeprom_EEPROM_read2:
	sbic _SFR_IO_ADDR(EECR), EEPE
	rjmp hrc_recall_eeprom_EEPROM_read2
	out _SFR_IO_ADDR(EEARH), r_temp
	out _SFR_IO_ADDR(EEARL), r_bytep
	sbi _SFR_IO_ADDR(EECR), EERE
	in r_rwbyte,_SFR_IO_ADDR(EEDR)
	st Z+,r_rwbyte
	inc r_bytep
	cpi r_bytep,3
	brne hrc_recall_eeprom_EEPROM_read2
	ret






;---------------------------------------------------
;   READ SCRATCHPAD
;---------------------------------------------------

h_readscratchpad2:
	cpi  r_bytep,8
	breq h_readscratchpad_crc2
	cpi  r_bytep,9
	breq h_readscratchpad_all2
	configZ pack2,r_bytep
	ld   r_rwbyte,Z
	rjmp h_readscratchpad_endc2
h_readscratchpad_crc2:
	lds  r_rwbyte,crc
h_readscratchpad_endc2:
	inc  r_bytep
	ldi  r_bcount,1 
	rjmp handle_end
h_readscratchpad_all2:
	rjmp handle_end_sleep





;---------------------------------------------------
;   WRITE SCRATCHPAD
;---------------------------------------------------

h_writescratchpad2:
	configZ pack2,r_bytep
	inc  r_bytep
	cpi  r_bytep,5
	breq h_writescratchpad_all2
	st   Z,r_rwbyte
	rjmp handle_end
h_writescratchpad_all2:
	;ori r_rwbyte,0x1F ; Alle unteren Bits sind immer 1 -->VOC use different
	st   Z,r_rwbyte
	rjmp handle_end_sleep





#include "../common/OWPinInterrupt.s"
.end
