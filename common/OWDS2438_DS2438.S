
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

#ifndef _NOCHANGEABLE_ID_
#define _CHANGEABLE_ID_
#endif
#define _ZERO_POLLING_
#define _HANDLE_CC_COMMAND_
#define _NO_BOOTLOADER_PIN_	
//#define __4MHZ__

//#define _DB_

#include "../common/OWConfig.s"
#include "../common/OWCRC8.s"


#if  defined(__AVR_ATtiny25__)
.extern pack2,16
.extern pack1,16
#else
.extern pack2,64
.extern pack1,64
#endif
.extern  DS2438_1_TEMP,2
.extern  DS2438_1_VAD,2
.extern  DS2438_2_TEMP,2
.extern  DS2438_2_VAD,2
.comm block,1 ; Block der augegeben, geschrieben wird (Parameter von READ/WRITE Scratchpad)
.comm cpsp2,1 ; Copy Scratchpad marker

.macro CHIP_INIT 	;r_temp is pushed other Registers should be saved
	ldi r_temp,0
	sts cpsp2,r_temp
.endm

.macro COMMAND_TABLE
		rjmp h_readscratchpad_adr1
		rjmp h_readscratchpad1
		rjmp h_writescratchpad_adr1
		rjmp h_writescratchpad1
		rjmp h_readscratchpad_adr2
		rjmp h_readscratchpad2
		rjmp h_writescratchpad_adr2
		rjmp h_writescratchpad2
		rjmp h_copyscratchpad_adr2
.endm

#include "../common/OWRomFunctionsDual.s"
#include "../common/OWTimerInterrupt.s"



; Ab hier Geraeteabhaenging
#define OW_READ_SCRATCHPAD_ADR1 OW_FIRST_COMMAND+0
#define OW_READ_SCRATCHPAD1 OW_FIRST_COMMAND+1
#define OW_WRITE_SCRATCHPAD_ADR1 OW_FIRST_COMMAND+2
#define OW_WRITE_SCRATCHPAD1 OW_FIRST_COMMAND+3
#define OW_READ_SCRATCHPAD_ADR2 OW_FIRST_COMMAND+4
#define OW_READ_SCRATCHPAD2 OW_FIRST_COMMAND+5
#define OW_WRITE_SCRATCHPAD_ADR2 OW_FIRST_COMMAND+6
#define OW_WRITE_SCRATCHPAD2 OW_FIRST_COMMAND+7
#define OW_COPY_SCRATCHPAD_ADR2 OW_FIRST_COMMAND+8


;---------------------------------------------------
;	READ COMMAND and start operation
;---------------------------------------------------

#ifdef _HANDLE_CC_COMMAND_
h_readcommand12:
	clr r_bytep
	cljmp 0x44,hrc_set_convertT12
	cljmp 0xB4,hrc_set_convertV12
	ldi r_mode,OW_SLEEP
	rjmp handle_end
#endif


h_readcommand1:
	clr r_bytep
#ifndef _DIS_FLASH_
	FLASH_COMMANDS ; muss zu erst sein....
#endif
	cset 0xBE,OW_READ_SCRATCHPAD_ADR1
	cset 0x4E,OW_WRITE_SCRATCHPAD_ADR1
	cjmp 0x44,hrc_set_convertT1
	cjmp 0xB4,hrc_set_convertV1
	FW_CONFIG_INFO1
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	rjmp handle_end_sleep


hrc_set_convertT1:
	rcall chrc_set_convertT1
	rjmp handle_end_sleep

chrc_set_convertT1:
	ldi r_temp,1
	sts gcontrol,r_temp
	lds r_temp,DS2438_1_TEMP
	sts pack1+1,r_temp
	lds r_temp,DS2438_1_TEMP+1
	sts pack1+2,r_temp
	ret

hrc_set_convertV1:
	rcall chrc_set_convertV1
	rjmp handle_end_sleep

chrc_set_convertV1:
	lds r_temp,pack1
	sbrs r_temp,3
	rjmp hrc_set_convertVV1
	lds r_temp,DS2438_1_VDD  
	sts pack1+3,r_temp
	lds r_temp,DS2438_1_VDD+1
	sts pack1+4,r_temp
	rjmp hrc_set_convertend1
hrc_set_convertVV1:
	ldi r_temp,2
	sts gcontrol,r_temp
	lds r_temp,DS2438_1_VAD
	sts pack1+3,r_temp
	lds r_temp,DS2438_1_VAD+1
	sts pack1+4,r_temp
hrc_set_convertend1:
	ret



;---------------------------------------------------
;   READ SCRATCHPAD
;---------------------------------------------------

h_readscratchpad_adr1:
	lsl r_rwbyte
	lsl r_rwbyte
	lsl r_rwbyte
#if  defined(__AVR_ATtiny25__)
	andi r_rwbyte,0x01 ;nur Page 0 und 1 und das immer wiederholen
#endif
	sts block,r_rwbyte
	ldi r_sendflag,1
	ldi r_mode,OW_READ_SCRATCHPAD1
	CRCInit2
h_readscratchpad1:
	cpi  r_bytep,8
	breq h_readscratchpad_crc1
	cpi  r_bytep,9
	breq h_readscratchpad_all1
	lds  r_temp,block
	add  r_temp,r_bytep
	configZ pack1,r_temp
	ld   r_rwbyte,Z
	rjmp handle_end_inc
h_readscratchpad_crc1:
	lds  r_rwbyte,crc
	rjmp handle_end_inc
h_readscratchpad_all1:
	rjmp handle_end_sleep




;---------------------------------------------------
;   WRITE SCRATCHPAD
;---------------------------------------------------
h_writescratchpad_adr1:
	lsl r_rwbyte
	lsl r_rwbyte
	lsl r_rwbyte
#if  defined(__AVR_ATtiny25__)
	andi r_rwbyte,0x01 ;nur Page 0 und 1 und das immer wiederholen
#endif
	sts block,r_rwbyte
	ldi r_mode,OW_WRITE_SCRATCHPAD1
	ldi  r_bcount,1 
	rjmp handle_end	
h_writescratchpad1:
	cpi  r_bytep,8
	breq h_writescratchpad_all1
	lds  r_temp,block
	add  r_temp,r_bytep
	configZ pack1,r_temp
	st   Z,r_rwbyte
	rjmp handle_end_inc
h_writescratchpad_all1:
	rjmp handle_end_sleep








#ifdef _HANDLE_CC_COMMAND_
hrc_set_convertT12:
	rcall chrc_set_convertT1
	rcall chrc_set_convertT2
	rjmp handle_end_sleep

hrc_set_convertV12:
	rcall chrc_set_convertV1
	rcall chrc_set_convertV2
	rjmp handle_end_sleep
#endif


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
	cset 0xBE,OW_READ_SCRATCHPAD_ADR2
	cset 0x4E,OW_WRITE_SCRATCHPAD_ADR2
	cjmp 0x44,hrc_set_convertT2
	cjmp 0xB4,hrc_set_convertV2
	cset 0x48,OW_COPY_SCRATCHPAD_ADR2
	FW_CONFIG_INFO2
	//cljmp 0x85,hrc_fw_configinfo2
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	rjmp handle_end_sleep

hrc_set_convertT2:
	rcall chrc_set_convertT2
	rjmp handle_end_sleep

hrc_set_convertV2:
	rcall chrc_set_convertV2
	rjmp handle_end_sleep


chrc_set_convertT2:
	ldi r_temp,3
	sts gcontrol,r_temp
	lds r_temp,DS2438_2_TEMP
	sts pack2+1,r_temp
	lds r_temp,DS2438_2_TEMP+1
	sts pack2+2,r_temp
	ret

chrc_set_convertV2:
	lds r_temp,pack2
	sbrs r_temp,3
	rjmp hrc_set_convertVV2
	lds r_temp,DS2438_2_VDD  
	sts pack2+3,r_temp
	lds r_temp,DS2438_2_VDD+1
	sts pack2+4,r_temp
	rjmp hrc_set_convertend2
hrc_set_convertVV2:
	ldi r_temp,4
	sts gcontrol,r_temp
	lds r_temp,DS2438_2_VAD
	sts pack2+3,r_temp
	lds r_temp,DS2438_2_VAD+1
	sts pack2+4,r_temp
hrc_set_convertend2:
	ret



;---------------------------------------------------
;   READ SCRATCHPAD
;---------------------------------------------------

h_readscratchpad_adr2:
	lsl r_rwbyte
	lsl r_rwbyte
	lsl r_rwbyte
#if  defined(__AVR_ATtiny25__)
	andi r_rwbyte,0x01 ;nur Page 0 und 1 und das immer wiederholen
#endif
	sts block,r_rwbyte
	ldi r_sendflag,1
	ldi r_mode,OW_READ_SCRATCHPAD2
	CRCInit2
h_readscratchpad2:
	cpi  r_bytep,8
	breq h_readscratchpad_crc2
	cpi  r_bytep,9
	breq h_readscratchpad_all2
	lds  r_temp,block
	add  r_temp,r_bytep
	configZ pack2,r_temp
	ld   r_rwbyte,Z
	rjmp handle_end_inc
h_readscratchpad_crc2:
	lds  r_rwbyte,crc
	rjmp handle_end_inc
h_readscratchpad_all2:
	rjmp handle_end_sleep




;---------------------------------------------------
;   WRITE SCRATCHPAD
;---------------------------------------------------
h_writescratchpad_adr2:
	lsl r_rwbyte
	lsl r_rwbyte
	lsl r_rwbyte
#if  defined(__AVR_ATtiny25__)
	andi r_rwbyte,0x01 ;nur Page 0 und 1 und das immer wiederholen
#endif
	sts block,r_rwbyte
	ldi r_mode,OW_WRITE_SCRATCHPAD2
	ldi  r_bcount,1 
	rjmp handle_end	
h_writescratchpad2:
	cpi  r_bytep,8
	breq h_writescratchpad_all2
	lds  r_temp,block
	add  r_temp,r_bytep
	configZ pack2,r_temp
	st   Z,r_rwbyte
	rjmp handle_end_inc
h_writescratchpad_all2:
	rjmp handle_end_sleep



h_copyscratchpad_adr2:
	ori r_rwbyte,0x80 ;hoechstes bit ist 1 zum Testen
	sts cpsp2,r_rwbyte
	rjmp handle_end_sleep





#include "../common/OWPinInterrupt.s"
.end
