
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
//#define __4MHZ__
        

#include "../common/OWConfig.s"
#include "../common/OWCRC16.s"

.extern pin_state1,1
.extern pin_state1,1
.extern pin_state2,1
.extern pin_set2,1
.comm resv1,1
.comm resv2,1
.macro CHIP_INIT 	;r_temp is pushed other Registers should be saved
.endm

.macro COMMAND_TABLE

		rjmp h_accessread1
		rjmp h_accesswrite1
		rjmp h_accesswrite_read1

.endm

#include "../common/OWRomFunctions.s"
#include "../common/OWTimerInterrupt.s"


; Ab hier Geraeteabhaenging
#define OW_ACCESSREAD1 OW_FIRST_COMMAND+0
#define OW_ACCESSWRITE1 OW_FIRST_COMMAND+1
#define OW_ACCESSWRITE_READ1 OW_FIRST_COMMAND+2


;---------------------------------------------------
;	READ COMMAND and start operation
;---------------------------------------------------

h_readcommand:
	clr r_bytep
#ifndef _DIS_FLASH_
	FLASH_COMMANDS ; muss zu erst sein....
#endif
	cjmp 0xF5,hrc_accessread1
	cset 0x5A,OW_ACCESSWRITE1
/*	cset 0xBE,OW_READ_SCRATCHPAD_ADR2
	cset 0x4E,OW_WRITE_SCRATCHPAD_ADR2
	cjmp 0x44,hrc_set_convertT2
	cjmp 0xB4,hrc_set_convertV2*/
	FW_CONFIG_INFO
	//cljmp 0x85,hrc_fw_configinfo2
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	rjmp handle_end_sleep


hrc_accessread1:
	ldi r_sendflag,1
	ldi r_mode,OW_ACCESSREAD1
h_accessread1:
	lds r_temp,pin_state1
	andi r_temp,0x0F
	mov r_rwbyte,r_temp
	com r_rwbyte
	swap r_rwbyte
	andi r_rwbyte,0xF0
	or r_rwbyte,r_temp
	rjmp handle_end




h_accesswrite_read1:
	//lds r_rwbyte,pin_state1
	rjmp handle_end_sleep
	


h_accesswrite1:
	cpi  r_bytep,1
	breq h_accesswrite_compl1
	sts resv1,r_rwbyte
	rjmp handle_end_inc
h_accesswrite_compl1:
	com r_rwbyte
	lds r_temp,resv1
	cp r_temp,r_rwbyte
	brne h_accesswrite_error1
	sts  pin_set1,r_rwbyte
	ldi r_mode,OW_ACCESSWRITE_READ1
	ldi r_rwbyte,0xAA
	ldi r_sendflag,1
	rjmp handle_end_inc
h_accesswrite_error1:
	rjmp handle_end_sleep



#include "../common/OWPinInterrupt.s"
.end
