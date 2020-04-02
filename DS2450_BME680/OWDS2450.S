
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
#define  __4MHZ__

#include "../common/OWConfig.s"
#include "../common/OWCRC16.s"

.extern pack,8
.comm addr,1 ;zweites Adressbyte ist unnoetig (Warum auch immer fuer 32 Byte 16 Bit Adressen verwendet werden....)
.comm crcsave,1 ; zwischenspeicherspeicher fuer crc nur zweites byte....
//.extern  am2302_temp,2



.macro CHIP_INIT 	
.endm

.macro COMMAND_TABLE
		rjmp h_readmemoryaddr
		rjmp h_readmemory
		rjmp h_readmemorycrc1
		rjmp h_readmemorycrc2
		rjmp h_writememoryaddr
		rjmp h_writememory
		rjmp h_writememorycrc1
		rjmp h_writememorycrc2
		rjmp h_writememoryreadback
		rjmp h_convert
		rjmp h_convertcrc1
		rjmp h_convertcrc2
		rjmp h_convert_conv
.endm

#include "../common/OWRomFunctions.s"
#include "../common/OWTimerInterrupt.s"



; Ab hier Geraeteabhaenging
#define OW_READ_MEMORY_ADDR OW_FIRST_COMMAND+0
#define OW_READ_MEMORY OW_FIRST_COMMAND+1
#define OW_READ_MEMORY_CRC1 OW_FIRST_COMMAND+2
#define OW_READ_MEMORY_CRC2 OW_FIRST_COMMAND+3
#define OW_WRITE_MEMORY_ADDR OW_FIRST_COMMAND+4
#define OW_WRITE_MEMORY OW_FIRST_COMMAND+5
#define OW_WRITE_MEMORY_CRC1 OW_FIRST_COMMAND+6
#define OW_WRITE_MEMORY_CRC2 OW_FIRST_COMMAND+7
#define OW_WRITE_MEMORY_READBACK OW_FIRST_COMMAND+8
#define OW_CONVERT OW_FIRST_COMMAND+9
#define OW_CONVERT_CRC1 OW_FIRST_COMMAND+10
#define OW_CONVERT_CRC2 OW_FIRST_COMMAND+11
#define OW_CONVERT_CONV OW_FIRST_COMMAND+12

;---------------------------------------------------
;	READ COMMAND and start operation
;---------------------------------------------------


h_readcommand:
	clr r_bytep
#ifndef _DIS_FLASH_
	FLASH_COMMANDS ; muss zu erst sein....
#endif
	cset 0xAA,OW_READ_MEMORY_ADDR
	cset 0x55,OW_WRITE_MEMORY_ADDR
	cset 0x3C,OW_CONVERT
	FW_CONFIG_INFO
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	ldi r_mode,OW_SLEEP
	rjmp handle_end


h_readmemoryaddr:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_readmemory_addr_byte1 ;nein dann weiter
	andi r_rwbyte,0x1F  ; nur Adressen zwischen 0 und 0x1F zulassen
	sts addr,r_rwbyte  ;speichern des ersten bytes
	rjmp handle_end_inc
h_readmemory_addr_byte1:  ;zweiters Addressbyte wird nicht gespeichert!
	ldi r_mode,OW_READ_MEMORY ;weiter zu read Memory
	;;ldi r_bcount,1 ;ist unten
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	clr r_bytep
	rjmp h_readmemory2
h_readmemory:
	lds r_bytep,addr
	inc r_bytep
	sts addr,r_bytep
	andi r_bytep,0x07
	breq h_readmemory_init_crc
h_readmemory2:
	lds r_bytep,addr
	;andi r_bytep,0x1F ist oben
	configZ pack,r_bytep
	ld   r_rwbyte,Z
	;ldi r_bcount,1
	rjmp handle_end ;sendet das Byte und geht zu h_readmemory
h_readmemory_init_crc:; init erstes CRC byte
	lds r_rwbyte,crc
	com r_rwbyte
	lds r_temp,crc+1
	com r_temp
	sts crcsave,r_temp
	ldi r_mode,OW_READ_MEMORY_CRC1
	;ldi r_bcount,1
	rjmp handle_end
h_readmemory_end:
	ldi  r_mode,OW_SLEEP
	clr r_sendflag
	rjmp handle_end
h_readmemorycrc1:;init zweites CRC Byte
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_READ_MEMORY_CRC2
	rjmp handle_end
h_readmemorycrc2:;weiteres senden..... nach zweitem Byte
	lds r_temp,addr
	andi r_temp,0xE0
	brne h_readmemory_end; ende des speichers
	ldi r_mode,OW_READ_MEMORY
	CRCInit1 ;Start with new CRC
	rjmp h_readmemory2

h_writememoryaddr:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_writememory_addr_byte1 ;nein dann weiter
	andi r_rwbyte,0x1F  ; nur Adressen zwischen 0 und 0x1F zulassen
	sts addr,r_rwbyte  ;speichern des ersten bytes
	inc r_bytep
	;ldi r_bcount,1
	rjmp handle_end
h_writememory_addr_byte1:  ;zweiters Addressbyte wird nicht gespeichert!
	ldi r_mode,OW_WRITE_MEMORY ;weiter zu read Memory
	;ldi r_bcount,1 ;; _________________________________________________in handle_end integrieren.....
	lds r_bytep,addr
	rjmp handle_end ;read Memory Byte
h_writememory:
	lds r_bytep,addr
	configZ pack,r_bytep
	st Z,r_rwbyte
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY_CRC1
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	lds r_rwbyte,crc
	com r_rwbyte
	lds r_temp,crc+1
	com r_temp
	sts crcsave,r_temp
	rjmp handle_end
h_writememorycrc1:
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY_CRC2
	rjmp handle_end
h_writememorycrc2:
	lds r_temp,addr
	configZ pack,r_temp
	ld r_rwbyte,Z
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY_READBACK
	rjmp handle_end
h_writememoryreadback:
	ldi r_temp,0x00
	sts crc+1,r_temp
	lds r_temp,addr
	inc r_temp
	sts addr,r_temp
	sts crc,r_temp
	ldi r_sendflag,0
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY
	rjmp handle_end

h_convert:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_convert_byte1 ;nein dann weiter
	inc r_bytep
	sts pack+0x20,r_rwbyte
	;ldi r_bcount,1
	rjmp handle_end
h_convert_byte1: ;zweies byte glesen go crc#
	sts pack+0x21,r_rwbyte
	lds r_rwbyte,crc
	com r_rwbyte
	lds r_temp,crc+1
	com r_temp
	sts crcsave,r_temp
	ldi r_mode,OW_CONVERT_CRC1
	;ldi r_bcount,1
	ldi r_sendflag,1
	rjmp handle_end	
h_convertcrc1:
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_CONVERT_CRC2
	rjmp handle_end
h_convertcrc2:
	ldi r_temp,1
	sts gcontrol,r_temp
	;ldi r_bcount,1
	ldi r_mode,OW_CONVERT_CONV
	;clr r_sendflag
	ldi r_sendflag,3 ;set bit 0 and 1 for no zero polling
h_convert_conv:
	ldi r_bcount,0
	ldi r_rwbyte,0
	rjmp handle_end_no_bcount	




#include "../common/OWPinInterrupt.s"
.end