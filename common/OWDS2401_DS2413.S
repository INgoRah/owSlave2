
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
//#define _HANDLE_CC_COMMAND_
//#define _DB_

#include "../common/OWConfig.s"
#include "../common/OWCRC8.s"

.extern pin_state,1
.extern pin_set,1
.comm resv1,1


.macro CHIP_INIT 	;r_temp is pushed other Registers should be saved
.endm

.macro COMMAND_TABLE
		rjmp h_accessread
		rjmp h_accesswrite
		rjmp h_accesswrite_read
.endm

#include "../common/OWRomFunctionsDual.s"
#include "../common/OWTimerInterrupt.s"



; Ab hier Geraeteabhaenging
#define OW_ACCESSREAD OW_FIRST_COMMAND+0
#define OW_ACCESSWRITE OW_FIRST_COMMAND+1
#define OW_ACCESSWRITE_READ OW_FIRST_COMMAND+2

;---------------------------------------------------
;	READ COMMAND and start operation
;---------------------------------------------------

#ifdef _HANDLE_CC_COMMAND_
/*h_readcommand12:
	clr r_bytep
	cset 0x44,hrc_set_convertT12
	ldi r_mode,OW_SLEEP
	rjmp handle_end*/
#endif


h_readcommand1:
	clr r_bytep
#ifndef _DIS_FLASH_
	FLASH_COMMANDS ; muss zu erst sein....
#endif

/*	cset 0xBE,OW_READ_SCRATCHPAD_ADR1
	cset 0x4E,OW_WRITE_SCRATCHPAD_ADR1
	cjmp 0x44,hrc_set_convertT1
	cjmp 0xB4,hrc_set_convertV1*/
	FW_CONFIG_INFO1
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
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
	cjmp 0xF5,hrc_accessread
	cset 0x5A,OW_ACCESSWRITE
/*	cset 0xBE,OW_READ_SCRATCHPAD_ADR2
	cset 0x4E,OW_WRITE_SCRATCHPAD_ADR2
	cjmp 0x44,hrc_set_convertT2
	cjmp 0xB4,hrc_set_convertV2*/
	FW_CONFIG_INFO2
	//cljmp 0x85,hrc_fw_configinfo2
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	rjmp handle_end_sleep


hrc_accessread:
	ldi r_sendflag,1
	ldi r_mode,OW_ACCESSREAD
h_accessread:
	lds r_temp,pin_state
	andi r_temp,0x0F
	mov r_rwbyte,r_temp
	com r_rwbyte
	swap r_rwbyte
	andi r_rwbyte,0xF0
	or r_rwbyte,r_temp
	rjmp handle_end




h_accesswrite_read:
	rjmp handle_end_sleep
	


h_accesswrite:
	cpi  r_bytep,1
	breq h_accesswrite_compl
	sts resv1,r_rwbyte
	rjmp handle_end_inc
h_accesswrite_compl:
	com r_rwbyte
	lds r_temp,resv1
	cp r_temp,r_rwbyte
	brne h_accesswrite_error
	sts  pin_set,r_rwbyte
	ldi r_mode,OW_ACCESSWRITE_READ
	ldi r_rwbyte,0xAA
	ldi r_sendflag,1
	rjmp handle_end_inc
h_accesswrite_error:
	rjmp handle_end_sleep
/*
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






	*/


#include "../common/OWPinInterrupt.s"
.end
