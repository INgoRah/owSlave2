
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
#define _HANDLE_CC_COMMAND_
#include "../common/OWConfig.s"
#include "../common/OWCRC8_16.s"

.extern pack1,8

.extern pack2,8
.comm addr,1 ;zweites Adressbyte ist unnoetig (Warum auch immer fuer 32 Byte 16 Bit Adressen verwendet werden....)
.comm crcsave,1 ; zwischenspeicherspeicher fuer crc nur zweites byte....
//.extern  am2302_temp,2
.comm stat_to_sample,1

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
		rjmp h_readpioregaddr2
		rjmp h_readpioreg2
		rjmp h_readpioregcrc12
		rjmp h_readpioregcrc22
		rjmp h_readchanel2
		rjmp h_readchanel_crc2
		rjmp h_writechanel2
		rjmp h_writecomchanel2
		rjmp h_writesendaa2
		rjmp h_writesendchanel2
		rjmp h_resetactivity2
		rjmp h_writeregaddr2
		rjmp h_writereg2
.endm

#include "../common/OWRomFunctionsDual.s"
#include "../common/OWTimerInterrupt.s"



; Ab hier Geraeteabhaenging

#define OW_READ_SCRATCHPAD1 OW_FIRST_COMMAND+0
#define OW_WRITE_SCRATCHPAD1 OW_FIRST_COMMAND+1
#define OW_CONVERT_RUN1  OW_FIRST_COMMAND+2

#define OW_READ_PIO_REG_ADDR2 OW_FIRST_COMMAND+3
#define OW_READ_PIO_REG2 OW_FIRST_COMMAND+4
#define OW_READ_PIO_REG_CRC12 OW_FIRST_COMMAND+5
#define OW_READ_PIO_REG_CRC22 OW_FIRST_COMMAND+6
#define OW_READ_CHANEL2 OW_FIRST_COMMAND+7
#define OW_READ_CHANEL_CRC2 OW_FIRST_COMMAND+8
#define OW_WRITE_CHANEL2 OW_FIRST_COMMAND+9
#define OW_WRITE_COMCHANEL2 OW_FIRST_COMMAND+10
#define OW_WRITE_SENDAA2 OW_FIRST_COMMAND+11
#define OW_WRITE_SEND_CHANEL2 OW_FIRST_COMMAND+12
#define OW_RESET_ACTIVITY2 OW_FIRST_COMMAND+13
#define OW_WRITE_REG_ADDR2 OW_FIRST_COMMAND+14
#define OW_WRITE_REG2 OW_FIRST_COMMAND+15

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
	rjmp hrc_set_convertT1
#endif

hrc_set_convertT1:
	ldi r_temp,16
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
	lds  r_rwbyte,crc8
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
	cset 0xF0,OW_READ_PIO_REG_ADDR2
	cljmp 0xF5,hrc_readchanel2
	cset 0x5A,OW_WRITE_CHANEL2
	cljmp 0xC3,hrc_reset_activity2
	cset 0xCC,OW_WRITE_REG_ADDR2
	FW_CONFIG_INFO2
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	ldi r_mode,OW_SLEEP
	rjmp handle_end


h_readpioregaddr2:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_readpioreg_addr_byte12 ;nein dann weiter
	//andi r_rwbyte,0x1F  ; nur Adressen zwischen 0 und 0x1F zulassen
	subi r_rwbyte,0x89  ;beim lesen von 0x88 --> 0xFF inc addr -> 0x00
	sts addr,r_rwbyte  ;speichern des ersten bytes
	rjmp handle_end_inc
h_readpioreg_addr_byte12:  ;zweiters Addressbyte wird nicht gespeichert!
	ldi r_mode,OW_READ_PIO_REG2 ;weiter zu read Memory
	;;ldi r_bcount,1 ;ist unten
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	clr r_bytep
h_readpioreg2:
	lds r_bytep,addr
	inc r_bytep
	sts addr,r_bytep
	cpi r_bytep,0x08
	breq h_readpioreg_init_crc2
	brge h_readpioreg_end2 ; groeser dann nix senden
	configZ pack2,r_bytep
	ld   r_rwbyte,Z
	;ldi r_bcount,1
	rjmp handle_end ;sendet das Byte und geht zu h_readmemory
h_readpioreg_init_crc2:; init erstes CRC byte
	lds r_rwbyte,crc16
	com r_rwbyte
	lds r_temp,crc16+1
	com r_temp
	sts crcsave,r_temp
	ldi r_mode,OW_READ_PIO_REG_CRC12
	;ldi r_bcount,1
	rjmp handle_end
h_readpioreg_end2:
	ldi  r_mode,OW_SLEEP
	clr r_sendflag
	rjmp handle_end
h_readpioregcrc12:;init zweites CRC Byte
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_READ_PIO_REG_CRC22
	rjmp handle_end
h_readpioregcrc22: ; 2. CRC Byte gesendet
	rjmp h_readpioreg_end2


hrc_readchanel2:
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	ldi r_mode,OW_READ_CHANEL2
	ldi r_temp,8
	sts gcontrol,r_temp
	rjmp h_readchanel12
h_readchanel2:
	ldi r_temp,4
	sts gcontrol,r_temp
h_readchanel12:
	cpi r_bytep,31
	brge h_readchanelcrc12
	lds r_rwbyte,stat_to_sample
	sts pack2,r_rwbyte //sample 
	rjmp handle_end_inc
h_readchanelcrc12:
	lds r_rwbyte,crc16
	com r_rwbyte
	lds r_temp,crc16+1
	com r_temp
	sts crcsave,r_temp
	ldi r_mode,OW_READ_CHANEL_CRC2
	;ldi r_bcount,1
	rjmp handle_end
h_readchanel_crc2:
	clr r_bytep
	ldi r_mode,OW_READ_CHANEL2
	lds r_rwbyte,crcsave
	rjmp handle_end
	
h_writechanel2:
	sts crcsave,r_rwbyte
	ldi r_mode,OW_WRITE_COMCHANEL2
	rjmp handle_end
h_writecomchanel2:
	com r_rwbyte
	lds r_temp,crcsave
	cp r_rwbyte,r_temp
	breq h_writeok2
	rjmp handle_end_sleep
h_writeok2:
	sts pack2+1,r_rwbyte
	ldi r_temp2,1
	sts gcontrol,r_temp2
	clr r_sendflag
	ldi r_rwbyte,0xAA
	ldi r_mode,OW_WRITE_SENDAA2
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	rjmp handle_end
h_writesendaa2:
	lds r_rwbyte,pack2
	ldi r_mode,OW_WRITE_SEND_CHANEL2
	rjmp handle_end
h_writesendchanel2:
	rjmp handle_end_sleep



hrc_reset_activity2:
	ldi r_temp,2
	sts gcontrol,r_temp
	ldi r_rwbyte,0xAA
	ldi r_mode,OW_RESET_ACTIVITY2
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	rjmp handle_end
h_resetactivity2:
	rjmp handle_end_sleep


h_writeregaddr2:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_writeregddr_byte12 ;nein dann weiter
	//andi r_rwbyte,0x1F  ; nur Adressen zwischen 0 und 0x1F zulassen
	subi r_rwbyte,0x8B  
	brmi h_writereg_end2
	sts addr,r_rwbyte  ;speichern des ersten bytes
	rjmp handle_end_inc
h_writeregddr_byte12:  ;zweiters Addressbyte wird nicht gespeichert!
	ldi r_mode,OW_WRITE_REG2 ;weiter zu write Memory
	;;ldi r_bcount,1 ;ist unten
	clr r_bytep
	rjmp handle_end
h_writereg2:
	lds r_temp,addr
	configZ pack2+3,r_temp
	st Z,r_rwbyte
	cpi r_temp,5
	brge h_writereg_end2
	inc r_temp
	sts addr,r_temp
	rjmp handle_end_sleep
		
h_writereg_end2:
	rjmp handle_end_sleep





#include "../common/OWPinInterrupt.s"
.end