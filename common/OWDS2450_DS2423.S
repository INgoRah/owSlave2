
// Copyright (c) 2018, Tobias Mueller tm(at)tm3d.de
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
#include "../common/OWCRC16.s"

.extern pack1,20
.extern pack2,45
.extern counters2,16
//Bleiben gleich denn es werden nicht beide gleichzeitig abgefragt
.comm addr,1 ;zweites Adressbyte ist unnoetig (Warum auch immer fuer 32 Byte 16 Bit Adressen verwendet werden....)
.comm crcsave,1 ; zwischenspeicherspeicher fuer crc nur zweites byte....
//.extern  am2302_temp,2

.comm gcontrol1,1
.comm gcontrol2,1

.macro CHIP_INIT 	
.endm

.macro COMMAND_TABLE
		rjmp h_readmemoryaddr1
		rjmp h_readmemory1
		rjmp h_readmemorycrc11
		rjmp h_readmemorycrc21
		rjmp h_writememoryaddr1
		rjmp h_writememory1
		rjmp h_writememorycrc11
		rjmp h_writememorycrc21
		rjmp h_writememoryreadback1
		rjmp h_convert1
		rjmp h_convertcrc11
		rjmp h_convertcrc21
		rjmp h_convert_conv1

		rjmp h_writescratchpad2
		rjmp h_writescratchpad_crc2
		rjmp h_readscratchpad2
		rjmp h_copyscratchpad2
		rjmp h_readmemory_addr2
		rjmp h_readmemory2
		rjmp h_readmemorycounter_addr2
		rjmp h_readmemorycounter2
		rjmp h_readmemorycounter_ex2
.endm

#include "../common/OWRomFunctionsDual.s"
#include "../common/OWTimerInterrupt.s"



; Ab hier Geraeteabhaenging
#define OW_READ_MEMORY_ADDR1 OW_FIRST_COMMAND+0
#define OW_READ_MEMORY1 OW_FIRST_COMMAND+1
#define OW_READ_MEMORY_CRC11 OW_FIRST_COMMAND+2
#define OW_READ_MEMORY_CRC21 OW_FIRST_COMMAND+3
#define OW_WRITE_MEMORY_ADDR1 OW_FIRST_COMMAND+4
#define OW_WRITE_MEMORY1 OW_FIRST_COMMAND+5
#define OW_WRITE_MEMORY_CRC11 OW_FIRST_COMMAND+6
#define OW_WRITE_MEMORY_CRC21 OW_FIRST_COMMAND+7
#define OW_WRITE_MEMORY_READBACK1 OW_FIRST_COMMAND+8
#define OW_CONVERT1 OW_FIRST_COMMAND+9
#define OW_CONVERT_CRC11 OW_FIRST_COMMAND+10
#define OW_CONVERT_CRC21 OW_FIRST_COMMAND+11
#define OW_CONVERT_CONV1 OW_FIRST_COMMAND+12

#define OW_WRITE_SCRATCHPAD2 OW_FIRST_COMMAND+13
#define OW_WRITE_SCRATCHPAD_CRC2 OW_FIRST_COMMAND+14
#define OW_READ_SCRATCHPAD2 OW_FIRST_COMMAND+15
#define OW_COPY_SCRATCHPAD2 OW_FIRST_COMMAND+16
#define OW_READ_MEMORY_ADDR2 OW_FIRST_COMMAND+17
#define OW_READ_MEMORY2 OW_FIRST_COMMAND+18
#define OW_READ_MEMORYCOUNTER_ADDR2 OW_FIRST_COMMAND+19
#define OW_READ_MEMORYCOUNTER2 OW_FIRST_COMMAND+20
#define OW_READ_MEMORYCOUNTER_EX2 OW_FIRST_COMMAND+21



;---------------------------------------------------
;	READ COMMAND and start operation
;---------------------------------------------------


h_readcommand1:
	clr r_bytep
#ifndef _DIS_FLASH_
	FLASH_COMMANDS ; muss zu erst sein....
#endif
	cset 0xAA,OW_READ_MEMORY_ADDR1
	cset 0x55,OW_WRITE_MEMORY_ADDR1
	cset 0x3C,OW_CONVERT1
	FW_CONFIG_INFO1
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	ldi r_mode,OW_SLEEP
	rjmp handle_end


h_readmemoryaddr1:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_readmemory_addr_byte11 ;nein dann weiter
	andi r_rwbyte,0x1F  ; nur Adressen zwischen 0 und 0x1F zulassen
	sts addr,r_rwbyte  ;speichern des ersten bytes
	rjmp handle_end_inc
h_readmemory_addr_byte11:  ;zweiters Addressbyte wird nicht gespeichert!
	ldi r_mode,OW_READ_MEMORY1 ;weiter zu read Memory
	;;ldi r_bcount,1 ;ist unten
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	clr r_bytep
	rjmp h_readmemory21
h_readmemory1:
	lds r_bytep,addr
	inc r_bytep
	sts addr,r_bytep
	andi r_bytep,0x07
	breq h_readmemory_init_crc1
h_readmemory21:
	lds r_bytep,addr
	;andi r_bytep,0x1F ist oben
	configZ pack1,r_bytep
	ld   r_rwbyte,Z
	;ldi r_bcount,1
	rjmp handle_end ;sendet das Byte und geht zu h_readmemory
h_readmemory_init_crc1:; init erstes CRC byte
	lds r_rwbyte,crc
	com r_rwbyte
	lds r_temp,crc+1
	com r_temp
	sts crcsave,r_temp
	ldi r_mode,OW_READ_MEMORY_CRC11
	;ldi r_bcount,1
	rjmp handle_end
h_readmemory_end1:
	ldi  r_mode,OW_SLEEP
	clr r_sendflag
	rjmp handle_end
h_readmemorycrc11:;init zweites CRC Byte
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_READ_MEMORY_CRC21
	rjmp handle_end
h_readmemorycrc21:;weiteres senden..... nach zweitem Byte
	lds r_temp,addr
	andi r_temp,0xE0
	brne h_readmemory_end1; ende des speichers
	ldi r_mode,OW_READ_MEMORY1
	CRCInit1 ;Start with new CRC
	rjmp h_readmemory21

h_writememoryaddr1:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_writememory_addr_byte11 ;nein dann weiter
	andi r_rwbyte,0x1F  ; nur Adressen zwischen 0 und 0x1F zulassen
	sts addr,r_rwbyte  ;speichern des ersten bytes
	inc r_bytep
	;ldi r_bcount,1
	rjmp handle_end
h_writememory_addr_byte11:  ;zweiters Addressbyte wird nicht gespeichert!
	ldi r_mode,OW_WRITE_MEMORY1 ;weiter zu read Memory
	;ldi r_bcount,1 ;; _________________________________________________in handle_end integrieren.....
	lds r_bytep,addr
	rjmp handle_end ;read Memory Byte
h_writememory1:
	lds r_bytep,addr
	configZ pack1,r_bytep
	st Z,r_rwbyte
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY_CRC11
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	lds r_rwbyte,crc
	com r_rwbyte
	lds r_temp,crc+1
	com r_temp
	sts crcsave,r_temp
	rjmp handle_end
h_writememorycrc11:
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY_CRC21
	rjmp handle_end
h_writememorycrc21:
	lds r_temp,addr
	configZ pack1,r_temp
	ld r_rwbyte,Z
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY_READBACK1
	rjmp handle_end
h_writememoryreadback1:
	ldi r_temp,0x00
	sts crc+1,r_temp
	lds r_temp,addr
	inc r_temp
	sts addr,r_temp
	sts crc,r_temp
	ldi r_sendflag,0
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY1
	rjmp handle_end

h_convert1:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_convert_byte11 ;nein dann weiter
	inc r_bytep
	sts pack1+0x20,r_rwbyte
	;ldi r_bcount,1
	rjmp handle_end
h_convert_byte11: ;zweies byte glesen go crc#
	sts pack1+0x21,r_rwbyte
	lds r_rwbyte,crc
	com r_rwbyte
	lds r_temp,crc+1
	com r_temp
	sts crcsave,r_temp
	ldi r_mode,OW_CONVERT_CRC11
	;ldi r_bcount,1
	ldi r_sendflag,1
	rjmp handle_end	
h_convertcrc11:
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_CONVERT_CRC21
	rjmp handle_end
h_convertcrc21:
	ldi r_temp,1
	sts gcontrol1,r_temp
	;ldi r_bcount,1
	ldi r_mode,OW_CONVERT_CONV1
	;clr r_sendflag
	ldi r_sendflag,3 ;set bit 0 and 1 for no zero polling
h_convert_conv1:
	ldi r_bcount,0
	ldi r_rwbyte,0
	rjmp handle_end_no_bcount

/////////////////////////////////////////////////////////////////////



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
	cset 0x0F,OW_WRITE_SCRATCHPAD2
	cjmp 0xAA,hrc_set_readscratchpad2
	cset 0x5A,OW_COPY_SCRATCHPAD2
	cset 0xF0,OW_READ_MEMORY_ADDR2
	cset 0xA5,OW_READ_MEMORYCOUNTER_ADDR2
	FW_CONFIG_INFO2
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	ldi r_mode,OW_SLEEP
	rjmp handle_end

hrc_set_readscratchpad2:
	ldi r_mode,OW_READ_SCRATCHPAD2
	ldi r_sendflag,1
	rjmp h_readscratchpad2

h_writescratchpad2:
	configZ pack2,r_bytep
	inc  r_bytep
	st   Z,r_rwbyte
	cpi  r_bytep,2
	breq h_writescratchpad_block2
	brsh h_writescratchpad_set_eoffset2 ;;33
	rjmp handle_end  ;handle_end zu weit entfernt fuer br...
h_writescratchpad_set_eoffset2:
	cpi  r_bytep,35
	breq h_writescratchpad_setcrc2
	mov r_temp,r_bytep
	subi r_temp,4
	sts pack2+2,r_temp ;AA und PF cleared
	rjmp handle_end
;Start writeing to 32 Byte Block ; skip status byte	
h_writescratchpad_block2:
	lds r_temp,pack2 ; Adresse low byte
	andi r_temp,0x1F ;32 byte
	add r_bytep,r_temp ;Zur angegebenen Startadresse springen
	;ori r_temp,0x20 ; Set PF flag
	sts pack2+2,r_temp  ;E4:E0 vorher setzen
	; Byte 3 ueberspringen
	rjmp handle_end_inc

h_writescratchpad_setcrc2:
	;copy crc to pack2
	lds r_temp,crc
	com r_temp
	sts pack2+43,r_temp
	lds r_temp,crc+1
	com r_temp ; invertieren , komischer name.....
	sts pack2+44,r_temp
	ldi  r_mode,OW_WRITE_SCRATCHPAD_CRC2
	ldi r_sendflag,1
	ldi r_bytep,43
h_writescratchpad_crc2:
	cpi r_bytep,45
	breq h_writescratchpad_crc_end2
	configZ pack2,r_bytep
	ld r_rwbyte,Z
	rjmp handle_end_inc
h_writescratchpad_crc_end2:
	rjmp handle_end_sleep
	
	
h_readscratchpad2:
	cpi r_bytep,35
	breq h_readscratchpad_end2
	cpi r_bytep,3
	brne h_readscratchpad_read_byte2
h_readscratchpad_set_offset2:
	lds r_temp,pack2
	andi r_temp,0x1F
	ldi r_temp2,3
	add r_temp,r_temp2
	mov r_bytep,r_temp
h_readscratchpad_read_byte2:
	configZ pack2,r_bytep
	ld r_rwbyte,Z
	rjmp handle_end_inc
h_readscratchpad_end2:
	rjmp handle_end_sleep



h_copyscratchpad2:
	cpi  r_bytep,3
	brsh h_copyscratchpad_ok2
	configZ pack2,r_bytep
	inc  r_bytep
	ld   r_temp,Z
	cp r_temp,r_rwbyte
	brne h_copyscratchpad_nok2
	cpi  r_bytep,3
	breq h_copyscratchpad_ok2
	ldi  r_bcount,1 
	rjmp handle_end
h_copyscratchpad_ok2:
	ldi r_rwbyte,0xAA
	ldi r_sendflag,1
	ldi r_temp,1
	sts gcontrol2,r_temp
	rjmp handle_end
h_copyscratchpad_nok2:
	lds r_temp,pack2+3
	andi r_temp,~0x80
	sts pack2+3,r_temp
	rjmp handle_end_sleep


h_readmemory_addr2:
	cpi r_bytep,0
	brne h_readmrmory_addr_byte12
	sts pack2,r_rwbyte
	rjmp handle_end_inc
h_readmrmory_addr_byte12:
	sts pack2+1,r_rwbyte
	ldi r_mode,OW_READ_MEMORY2
	ldi r_sendflag,1
	clr r_bytep
	rjmp h_readmemory22
h_readmemory2:
	lds r_bytep,pack2
	lds r_temp2,pack2+1
	inc r_bytep
	clr r_temp
	adc r_temp2,r_temp
	sbrc r_temp2,1
	rjmp h_readmemory_end2
	sts pack2+1,r_temp2
	sts pack2,r_bytep
h_readmemory22:
	ldi  zl,lo8(pack2+45)       
    ldi  zh,hi8(pack2+45)
	lds r_bytep,pack2
	add  zl,r_bytep
	lds r_bytep,pack2+1
	adc zh,r_bytep
	//configZ pack2+3,r_bytep
	ld   r_rwbyte,Z
	rjmp handle_end
h_readmemory_end2:
	rjmp handle_end_sleep








h_readmemorycounter_addr2:
	cpi r_bytep,0
	brne h_readmrmorycounter_addr_byte12
	sts pack2,r_rwbyte
	inc r_bytep
	;ldi r_bcount,1
	rjmp handle_end
h_readmrmorycounter_addr_byte12:
	sts pack2+1,r_rwbyte
	ldi r_mode,OW_READ_MEMORYCOUNTER2
	;ldi r_bcount,1 
	ldi r_sendflag,1
	clr r_bytep
	rjmp h_readmemorycounter22
h_readmemorycounter2:
	lds r_bytep,pack2
	lds r_temp2,pack2+1
	ldi r_temp,1  ;inc leider kein c flag
	add r_bytep,r_temp
	clr r_temp
	adc r_temp2,r_temp
	mov r_temp,r_bytep
	andi r_temp,0x1F
	breq h_readmemorycounter_next2
	sts pack2+1,r_temp2
	sts pack2,r_bytep
h_readmemorycounter22:  ;Lesen von dem worauf die erstenzwei bytes zeigen
	lds r_bytep,pack2
	andi r_bytep,0x1F
	configZ pack2+3,r_bytep
	ld   r_rwbyte,Z
	;ldi r_bcount,1
	rjmp handle_end
//h_readmemorycounter_end:
//	ldi  r_mode,OW_SLEEP
//	clr r_sendflag
//	rjmp handle_end
h_readmemorycounter_next2:  ; rest lesen
	ldi  r_mode,OW_READ_MEMORYCOUNTER_EX2
	ldi r_bytep,34
	lds r_temp2,pack2
	lds r_temp,pack2+1
	//lsr r_temp
	//ror r_temp2

	lsl r_temp2
	rol r_temp
	cpi r_temp,3
	brne h_readmemorycounter_cFF2
	andi r_temp2,0xC0
	swap r_temp2

	;cpi r_temp,0xE0

	configZ counters2,r_temp2
	ld r_temp,Z+
	sts pack2+35,r_temp	
	ld r_temp,Z+
	sts pack2+36,r_temp	
	ld r_temp,Z+
	sts pack2+37,r_temp	
	ld r_temp,Z+
	sts pack2+38,r_temp	
	rjmp h_readmemorycounter_ex2
h_readmemorycounter_cFF2:
	ldi r_temp,0xFF
	sts pack2+35,r_temp	
	sts pack2+36,r_temp	
	sts pack2+37,r_temp	
	sts pack2+38,r_temp	

h_readmemorycounter_ex2:
	inc r_bytep
	cpi r_bytep,45
	breq h_readmemorycounter_ex_end2
	cpi r_bytep,43
	brne h_readmemorycounter_ex22
	lds r_temp,crc
	com r_temp
	sts pack2+43,r_temp
	lds r_temp,crc+1
	com r_temp
	sts pack2+44,r_temp
h_readmemorycounter_ex22:
	;ldi r_bcount,1
	configZ pack2,r_bytep
	ld   r_rwbyte,Z
	rjmp handle_end
h_readmemorycounter_ex_end2:
	lds r_bytep,pack2
	lds r_temp2,pack2+1
	ldi r_temp,1  ;inc leider kein c flag
	add r_bytep,r_temp
	clr r_temp
	adc r_temp2,r_temp
	sbrc r_temp2,1 ;am ene von allem 
	rjmp h_readmemorycounter_ex_sleep2
	CRCInit1
	ldi  r_mode,OW_READ_MEMORYCOUNTER2
	sts pack2+1,r_temp2
	sts pack2,r_bytep
	rjmp h_readmemorycounter22
h_readmemorycounter_ex_sleep2:
	ldi r_mode,OW_SLEEP
	clr r_sendflag
	rjmp handle_end





#include "../common/OWPinInterrupt.s"
.end