
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
.extern pack2,20

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

		rjmp h_readmemoryaddr2
		rjmp h_readmemory2
		rjmp h_readmemorycrc12
		rjmp h_readmemorycrc22
		rjmp h_writememoryaddr2
		rjmp h_writememory2
		rjmp h_writememorycrc12
		rjmp h_writememorycrc22
		rjmp h_writememoryreadback2
		rjmp h_convert2
		rjmp h_convertcrc12
		rjmp h_convertcrc22
		rjmp h_convert_conv2
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

#define OW_READ_MEMORY_ADDR2 OW_FIRST_COMMAND+13
#define OW_READ_MEMORY2 OW_FIRST_COMMAND+14
#define OW_READ_MEMORY_CRC12 OW_FIRST_COMMAND+15
#define OW_READ_MEMORY_CRC22 OW_FIRST_COMMAND+16
#define OW_WRITE_MEMORY_ADDR2 OW_FIRST_COMMAND+17
#define OW_WRITE_MEMORY2 OW_FIRST_COMMAND+18
#define OW_WRITE_MEMORY_CRC12 OW_FIRST_COMMAND+19
#define OW_WRITE_MEMORY_CRC22 OW_FIRST_COMMAND+20
#define OW_WRITE_MEMORY_READBACK2 OW_FIRST_COMMAND+21
#define OW_CONVERT2 OW_FIRST_COMMAND+22
#define OW_CONVERT_CRC12 OW_FIRST_COMMAND+23
#define OW_CONVERT_CRC22 OW_FIRST_COMMAND+24
#define OW_CONVERT_CONV2 OW_FIRST_COMMAND+25



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


;---------------------------------------------------
;	READ COMMAND and start operation
;---------------------------------------------------


h_readcommand2:
	clr r_bytep
#ifndef _DIS_FLASH_
	FLASH_COMMANDS ; muss zu erst sein....
#endif
	cset 0xAA,OW_READ_MEMORY_ADDR2
	cset 0x55,OW_WRITE_MEMORY_ADDR2
	cset 0x3C,OW_CONVERT2
	FW_CONFIG_INFO2
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	ldi r_mode,OW_SLEEP
	rjmp handle_end


h_readmemoryaddr2:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_readmemory_addr_byte12 ;nein dann weiter
	andi r_rwbyte,0x1F  ; nur Adressen zwischen 0 und 0x1F zulassen
	sts addr,r_rwbyte  ;speichern des ersten bytes
	rjmp handle_end_inc
h_readmemory_addr_byte12:  ;zweiters Addressbyte wird nicht gespeichert!
	ldi r_mode,OW_READ_MEMORY2 ;weiter zu read Memory
	;;ldi r_bcount,1 ;ist unten
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	clr r_bytep
	rjmp h_readmemory22
h_readmemory2:
	lds r_bytep,addr
	inc r_bytep
	sts addr,r_bytep
	andi r_bytep,0x07
	breq h_readmemory_init_crc2
h_readmemory22:
	lds r_bytep,addr
	;andi r_bytep,0x1F ist oben
	configZ pack2,r_bytep
	ld   r_rwbyte,Z
	;ldi r_bcount,1
	rjmp handle_end ;sendet das Byte und geht zu h_readmemory
h_readmemory_init_crc2:; init erstes CRC byte
	lds r_rwbyte,crc
	com r_rwbyte
	lds r_temp,crc+1
	com r_temp
	sts crcsave,r_temp
	ldi r_mode,OW_READ_MEMORY_CRC12
	;ldi r_bcount,1
	rjmp handle_end
h_readmemory_end2:
	ldi  r_mode,OW_SLEEP
	clr r_sendflag
	rjmp handle_end
h_readmemorycrc12:;init zweites CRC Byte
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_READ_MEMORY_CRC22
	rjmp handle_end
h_readmemorycrc22:;weiteres senden..... nach zweitem Byte
	lds r_temp,addr
	andi r_temp,0xE0
	brne h_readmemory_end2; ende des speichers
	ldi r_mode,OW_READ_MEMORY2
	CRCInit1 ;Start with new CRC
	rjmp h_readmemory22

h_writememoryaddr2:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_writememory_addr_byte12 ;nein dann weiter
	andi r_rwbyte,0x1F  ; nur Adressen zwischen 0 und 0x1F zulassen
	sts addr,r_rwbyte  ;speichern des ersten bytes
	inc r_bytep
	;ldi r_bcount,1
	rjmp handle_end
h_writememory_addr_byte12:  ;zweiters Addressbyte wird nicht gespeichert!
	ldi r_mode,OW_WRITE_MEMORY2 ;weiter zu read Memory
	;ldi r_bcount,1 ;; _________________________________________________in handle_end integrieren.....
	lds r_bytep,addr
	rjmp handle_end ;read Memory Byte
h_writememory2:
	lds r_bytep,addr
	configZ pack2,r_bytep
	st Z,r_rwbyte
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY_CRC12
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	lds r_rwbyte,crc
	com r_rwbyte
	lds r_temp,crc+1
	com r_temp
	sts crcsave,r_temp
	rjmp handle_end
h_writememorycrc12:
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY_CRC22
	rjmp handle_end
h_writememorycrc22:
	lds r_temp,addr
	configZ pack2,r_temp
	ld r_rwbyte,Z
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY_READBACK2
	rjmp handle_end
h_writememoryreadback2:
	ldi r_temp,0x00
	sts crc+1,r_temp
	lds r_temp,addr
	inc r_temp
	sts addr,r_temp
	sts crc,r_temp
	ldi r_sendflag,0
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY2
	rjmp handle_end

h_convert2:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_convert_byte12 ;nein dann weiter
	inc r_bytep
	sts pack2+0x20,r_rwbyte
	;ldi r_bcount,1
	rjmp handle_end
h_convert_byte12: ;zweies byte glesen go crc#
	sts pack2+0x21,r_rwbyte
	lds r_rwbyte,crc
	com r_rwbyte
	lds r_temp,crc+1
	com r_temp
	sts crcsave,r_temp
	ldi r_mode,OW_CONVERT_CRC12
	;ldi r_bcount,1
	ldi r_sendflag,1
	rjmp handle_end	
h_convertcrc12:
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_CONVERT_CRC22
	rjmp handle_end
h_convertcrc22:
	ldi r_temp,1
	sts gcontrol2,r_temp
	;ldi r_bcount,1
	ldi r_mode,OW_CONVERT_CONV2
	;clr r_sendflag
	ldi r_sendflag,3 ;set bit 0 and 1 for no zero polling
h_convert_conv2:
	ldi r_bcount,0
	ldi r_rwbyte,0
	rjmp handle_end_no_bcount



#include "../common/OWPinInterrupt.s"
.end