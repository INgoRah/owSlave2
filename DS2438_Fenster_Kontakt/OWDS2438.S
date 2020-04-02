// Copyright (c) 2015, Tobias Mueller tm(at)tm3d.de
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
//#define _DIS_FLASH_
#define _ZERO_POLLING_
//#define _DB_
//#define __4MHZ__

#include "../common/OWConfig.s"
#include "../common/OWCRC8.s"

#if  defined(__AVR_ATtiny25__)
.extern pack,16
#else
.extern pack,64
#endif
.extern  ds_temp,2
.extern  ds_vdd,2
.extern  ds_vad,2

.comm block,1 ; Block der augegeben, geschrieben wird (Parameter von READ/WRITE Scratchpad)


.macro CHIP_INIT
.endm

; Ab hier Geraeteabhaenging
#define OW_READ_SCRATCHPAD_ADR OW_FIRST_COMMAND+0
#define OW_READ_SCRATCHPAD OW_FIRST_COMMAND+1
#define OW_WRITE_SCRATCHPAD_ADR OW_FIRST_COMMAND+2
#define OW_WRITE_SCRATCHPAD OW_FIRST_COMMAND+3



.macro COMMAND_TABLE
		rjmp h_readscratchpad_adr
		rjmp h_readscratchpad
		rjmp h_writescratchpad_adr
		rjmp h_writescratchpad
.endm

#include "../common/OWRomFunctions.s"
#include "../common/OWTimerInterrupt.s"


;---------------------------------------------------
;	READ COMMAND and start operation
;---------------------------------------------------


h_readcommand:
	clr r_bytep
#ifndef _DIS_FLASH_
	FLASH_COMMANDS ; muss zu erst sein....
#endif
	cset 0xBE,OW_READ_SCRATCHPAD_ADR
	cset 0x4E,OW_WRITE_SCRATCHPAD_ADR
	cjmp 0x44,hrc_set_convertT
	cjmp 0xB4,hrc_set_convertV
	FW_CONFIG_INFO
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	rjmp handle_end_sleep


hrc_set_convertT:
	ldi r_temp,1
	sts gcontrol,r_temp
	lds r_temp,ds_temp
	sts pack+1,r_temp
	lds r_temp,ds_temp+1
	sts pack+2,r_temp
	rjmp handle_end_sleep
hrc_set_convertV:
	lds r_temp,pack
	sbrs r_temp,3
	rjmp hrc_set_convertVV
	ldi r_temp,2
	sts gcontrol,r_temp
	//ldi r_temp,0xF4#
	lds r_temp,ds_vdd
	sts pack+3,r_temp
	//ldi r_temp,0x01
	lds r_temp,ds_vdd+1
	sts pack+4,r_temp
	rjmp hrc_set_convertend
hrc_set_convertVV:
	ldi r_temp,4
	sts gcontrol,r_temp
	lds r_temp,ds_vad
	sts pack+3,r_temp
	lds r_temp,ds_vad+1
	sts pack+4,r_temp
hrc_set_convertend:
	rjmp handle_end_sleep



;---------------------------------------------------
;   READ SCRATCHPAD
;---------------------------------------------------

h_readscratchpad_adr:
	lsl r_rwbyte
	lsl r_rwbyte
	lsl r_rwbyte
#if  defined(__AVR_ATtiny25__)
	andi r_rwbyte,0x01 ;nur Page 0 und 1 und das immer wiederholen
#endif
	sts block,r_rwbyte
	ldi r_sendflag,1
	ldi r_mode,OW_READ_SCRATCHPAD
	ldi r_temp,8
	sts gcontrol,r_temp
	CRCInit2
h_readscratchpad:
	cpi  r_bytep,8
	breq h_readscratchpad_crc
	cpi  r_bytep,9
	breq h_readscratchpad_all
	lds  r_temp,block
	add  r_temp,r_bytep
	configZ pack,r_temp
	ld   r_rwbyte,Z
	rjmp handle_end_inc
h_readscratchpad_crc:
	lds  r_rwbyte,crc
	rjmp handle_end_inc
h_readscratchpad_all:
	rjmp handle_end_sleep




;---------------------------------------------------
;   WRITE SCRATCHPAD
;---------------------------------------------------
h_writescratchpad_adr:
	lsl r_rwbyte
	lsl r_rwbyte
	lsl r_rwbyte
#if  defined(__AVR_ATtiny25__)
	andi r_rwbyte,0x01 ;nur Page 0 und 1 und das immer wiederholen
#endif
	sts block,r_rwbyte
	ldi r_mode,OW_WRITE_SCRATCHPAD
	ldi  r_bcount,1 
	rjmp handle_end	
h_writescratchpad:
	cpi  r_bytep,8
	breq h_writescratchpad_all
	lds  r_temp,block
	add  r_temp,r_bytep
	configZ pack,r_temp
	st   Z,r_rwbyte
	rjmp handle_end_inc
h_writescratchpad_all:
	rjmp handle_end_sleep





#include "../common/OWPinInterrupt.s"
.end