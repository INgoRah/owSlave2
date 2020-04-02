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

//#define __4MHZ__
#define _CHANGEABLE_ID_
#define _ZERO_POLLING_
//#define _DB_
#include "../common/OWConfig.s"
#include "../common/OWCRC16.s"

.extern pack,45
.extern counters,16

.macro CHIP_INIT 	
.endm

.macro COMMAND_TABLE
		rjmp h_writescratchpad
		rjmp h_writescratchpad_crc
		rjmp h_readscratchpad
		rjmp h_copyscratchpad
		rjmp h_readmemory_addr
		rjmp h_readmemory
		rjmp h_readmemorycounter_addr
		rjmp h_readmemorycounter
		rjmp h_readmemorycounter_ex
.endm

#include "../common/OWRomFunctions.s"
#include "../common/OWTimerInterrupt.s"



; Ab hier Geraeteabhaenging
#define OW_WRITE_SCRATCHPAD OW_FIRST_COMMAND+0
#define OW_WRITE_SCRATCHPAD_CRC OW_FIRST_COMMAND+1
#define OW_READ_SCRATCHPAD OW_FIRST_COMMAND+2
#define OW_COPY_SCRATCHPAD OW_FIRST_COMMAND+3
#define OW_READ_MEMORY_ADDR OW_FIRST_COMMAND+4
#define OW_READ_MEMORY OW_FIRST_COMMAND+5
#define OW_READ_MEMORYCOUNTER_ADDR OW_FIRST_COMMAND+6
#define OW_READ_MEMORYCOUNTER OW_FIRST_COMMAND+7
#define OW_READ_MEMORYCOUNTER_EX OW_FIRST_COMMAND+8

;---------------------------------------------------
;	READ COMMAND and start operation
;---------------------------------------------------


h_readcommand:
	clr r_bytep
#ifndef _DIS_FLASH_
	FLASH_COMMANDS ; muss zu erst sein....
#endif
	cset 0x0F,OW_WRITE_SCRATCHPAD
	cjmp 0xAA,hrc_set_readscratchpad
	cset 0x5A,OW_COPY_SCRATCHPAD
	cset 0xF0,OW_READ_MEMORY_ADDR
	cset 0xA5,OW_READ_MEMORYCOUNTER_ADDR
	FW_CONFIG_INFO
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	ldi r_mode,OW_SLEEP
	rjmp handle_end

hrc_set_readscratchpad:
	ldi r_mode,OW_READ_SCRATCHPAD
	ldi r_sendflag,1
	rjmp h_readscratchpad

h_writescratchpad:
	configZ pack,r_bytep
	inc  r_bytep
	st   Z,r_rwbyte
	cpi  r_bytep,2
	breq h_writescratchpad_block
	brsh h_writescratchpad_set_eoffset ;;33
	rjmp handle_end  ;handle_end zu weit entfernt fuer br...
h_writescratchpad_set_eoffset:
	cpi  r_bytep,35
	breq h_writescratchpad_setcrc
	mov r_temp,r_bytep
	subi r_temp,4
	sts pack+2,r_temp ;AA und PF cleared
	rjmp handle_end
;Start writeing to 32 Byte Block ; skip status byte	
h_writescratchpad_block:
	lds r_temp,pack ; Adresse low byte
	andi r_temp,0x1F ;32 byte
	add r_bytep,r_temp ;Zur angegebenen Startadresse springen
	;ori r_temp,0x20 ; Set PF flag
	sts pack+2,r_temp  ;E4:E0 vorher setzen
	; Byte 3 ueberspringen
	rjmp handle_end_inc

h_writescratchpad_setcrc:
	;copy crc to pack
	lds r_temp,crc
	com r_temp
	sts pack+43,r_temp
	lds r_temp,crc+1
	com r_temp ; invertieren , komischer name.....
	sts pack+44,r_temp
	ldi  r_mode,OW_WRITE_SCRATCHPAD_CRC
	ldi r_sendflag,1
	ldi r_bytep,43
h_writescratchpad_crc:
	cpi r_bytep,45
	breq h_writescratchpad_crc_end
	configZ pack,r_bytep
	ld r_rwbyte,Z
	rjmp handle_end_inc
h_writescratchpad_crc_end:
	rjmp handle_end_sleep
	
	
h_readscratchpad:
	cpi r_bytep,35
	breq h_readscratchpad_end
	cpi r_bytep,3
	brne h_readscratchpad_read_byte
h_readscratchpad_set_offset:
	lds r_temp,pack
	andi r_temp,0x1F
	ldi r_temp2,3
	add r_temp,r_temp2
	mov r_bytep,r_temp
h_readscratchpad_read_byte:
	configZ pack,r_bytep
	ld r_rwbyte,Z
	rjmp handle_end_inc
h_readscratchpad_end:
	rjmp handle_end_sleep



h_copyscratchpad:
	cpi  r_bytep,3
	brsh h_copyscratchpad_ok
	configZ pack,r_bytep
	inc  r_bytep
	ld   r_temp,Z
	cp r_temp,r_rwbyte
	brne h_copyscratchpad_nok
	cpi  r_bytep,3
	breq h_copyscratchpad_ok
	ldi  r_bcount,1 
	rjmp handle_end
h_copyscratchpad_ok:
	ldi r_rwbyte,0xAA
	ldi r_sendflag,1
	rjmp handle_end
h_copyscratchpad_nok:
	lds r_temp,pack+3
	andi r_temp,~0x80
	sts pack+3,r_temp
	rjmp handle_end_sleep


h_readmemory_addr:
	cpi r_bytep,0
	brne h_readmrmory_addr_byte1
	sts pack,r_rwbyte
	rjmp handle_end_inc
h_readmrmory_addr_byte1:
	sts pack+1,r_rwbyte
	ldi r_mode,OW_READ_MEMORY
	ldi r_sendflag,1
	clr r_bytep
	rjmp h_readmemory2
h_readmemory:
	lds r_bytep,pack
	lds r_temp2,pack+1
	inc r_bytep
	clr r_temp
	adc r_temp2,r_temp
	sbrc r_temp2,1
	rjmp h_readmemory_end
	sts pack+1,r_temp
	sts pack,r_bytep
h_readmemory2:
	lds r_bytep,pack
	andi r_bytep,0x1F
	configZ pack+3,r_bytep
	ld   r_rwbyte,Z
	rjmp handle_end
h_readmemory_end:
	rjmp handle_end_sleep








h_readmemorycounter_addr:
	cpi r_bytep,0
	brne h_readmrmorycounter_addr_byte1
	sts pack,r_rwbyte
	inc r_bytep
	;ldi r_bcount,1
	rjmp handle_end
h_readmrmorycounter_addr_byte1:
	sts pack+1,r_rwbyte
	ldi r_mode,OW_READ_MEMORYCOUNTER
	;ldi r_bcount,1 
	ldi r_sendflag,1
	clr r_bytep
	rjmp h_readmemorycounter2
h_readmemorycounter:
	lds r_bytep,pack
	lds r_temp2,pack+1
	ldi r_temp,1  ;inc leider kein c flag
	add r_bytep,r_temp
	clr r_temp
	adc r_temp2,r_temp
	mov r_temp,r_bytep
	andi r_temp,0x1F
	breq h_readmemorycounter_next
	sts pack+1,r_temp2
	sts pack,r_bytep
h_readmemorycounter2:  ;Lesen von dem worauf die erstenzwei bytes zeigen
	lds r_bytep,pack
	andi r_bytep,0x1F
	configZ pack+3,r_bytep
	ld   r_rwbyte,Z
	;ldi r_bcount,1
	rjmp handle_end
//h_readmemorycounter_end:
//	ldi  r_mode,OW_SLEEP
//	clr r_sendflag
//	rjmp handle_end
h_readmemorycounter_next:  ; rest lesen
	ldi  r_mode,OW_READ_MEMORYCOUNTER_EX
	ldi r_bytep,34
	lds r_temp2,pack
	lds r_temp,pack+1
	//lsr r_temp
	//ror r_temp2

	lsl r_temp2
	rol r_temp
	cpi r_temp,3
	brne h_readmemorycounter_cFF
	andi r_temp2,0xC0
	swap r_temp2

	;cpi r_temp,0xE0

	configZ counters,r_temp2
	ld r_temp,Z+
	sts pack+35,r_temp	
	ld r_temp,Z+
	sts pack+36,r_temp	
	ld r_temp,Z+
	sts pack+37,r_temp	
	ld r_temp,Z+
	sts pack+38,r_temp	
	rjmp h_readmemorycounter_ex
h_readmemorycounter_cFF:
	ldi r_temp,0xFF
	sts pack+35,r_temp	
	sts pack+36,r_temp	
	sts pack+37,r_temp	
	sts pack+38,r_temp	

h_readmemorycounter_ex:
	inc r_bytep
	cpi r_bytep,45
	breq h_readmemorycounter_ex_end
	cpi r_bytep,43
	brne h_readmemorycounter_ex2
	lds r_temp,crc
	com r_temp
	sts pack+43,r_temp
	lds r_temp,crc+1
	com r_temp
	sts pack+44,r_temp
h_readmemorycounter_ex2:
	;ldi r_bcount,1
	configZ pack,r_bytep
	ld   r_rwbyte,Z
	rjmp handle_end
h_readmemorycounter_ex_end:
	lds r_bytep,pack
	lds r_temp2,pack+1
	ldi r_temp,1  ;inc leider kein c flag
	add r_bytep,r_temp
	clr r_temp
	adc r_temp2,r_temp
	sbrc r_temp2,1 ;am ene von allem 
	rjmp h_readmemorycounter_ex_sleep
	CRCInit1
	ldi  r_mode,OW_READ_MEMORYCOUNTER
	sts pack+1,r_temp2
	sts pack,r_bytep
	rjmp h_readmemorycounter2
h_readmemorycounter_ex_sleep:
	ldi r_mode,OW_SLEEP
	clr r_sendflag
	rjmp handle_end



	

#include "../common/OWPinInterrupt.s"
.end