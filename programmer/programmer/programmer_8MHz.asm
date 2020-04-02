.def srbyte = r12  ;byty for search rom algorithm
.def writelow = r13;  marker for send low
.def zero =r14;  always zero 
.def smode=r15; if 1 then send 
.def temp = r16 ; 
.def temp2 = r17; 
.def mode = r18 ; 
.def bitp = r19 ; bit counter ... shift...
.def rwbyte = r21;
.def param = r22;
.def bytep = r23  ;byte counter
#define spmcrval  param


.equ OWM_READ_ROM_COMMAND=0 ; 0 wegen schnellen test ist dieser wert 0! Daturch wird die Sprungdabelle nicht verwendet
.equ OWM_SLEEP=1 ; Warten auf Reset
.equ OWM_MATCH_ROM=2
.equ OWM_SEARCH_ROM_S=3  ;send bit
.equ OWM_SEARCH_ROM_R=4  ;resive master
.equ OWM_READ_COMMAND=5
.equ OWM_WRITE_SCRATCHPAD=6
.equ OWM_READ_SCRATCHPAD=7
.equ OWM_PROGRAMM_PAGE=8
.equ OWM_RECALL_FLASH=9


.equ OW_DDR = DDRB 
.equ OW_PIN = PORTB2
.equ OW_PORT = PORTB 
.equ OW_PINN = PINB 

;.equ SRAM_START = 0x60

.macro set_clock
		ldi temp,0x80;
		out CLKPR,temp
		ldi temp,@0
		out CLKPR,temp
.endmacro 

.macro owwl
		sbic OW_PINN,OW_PIN
		rjmp pc-1
.endmacro 

.macro owwh
		sbis OW_PINN,OW_PIN
		rjmp pc-1
.endmacro 




 ;---------------------------------------------------
; START of PROG 
;---------------------------------------------------


.CSEG 
.ORG 0x000
jreset:
		rjmp start ; Reset-Vector 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 
		reti ; 


.ORG 0x0EC0

start: 
		cli
		ldi temp,0
		mov zero,temp
		set_clock 0x00 ;8mhz
		ldi mode,OWM_SLEEP
		ldi temp,(1<<CS01) //1us
		out TCCR0B,temp
		ldi temp, HIGH(RAMEND) ; HIGH-Byte der obersten RAM-Adresse 
		out SPH, temp 
		ldi temp, LOW(RAMEND) ; LOW-Byte der obersten RAM-Adresse 
		out SPL, temp 
		;ldi temp,1
		;out DDRB,temp
		clr writelow
		clr bitp
	
		;sbi PORTB,0
		ldi ZL,low(pro_owid*2) 
		ldi ZH,high(pro_owid*2) 
		ldi XL,low(sowid)
		ldi XH,high(sowid)
		;ldi temp2,8
pro_copy_loop: ;copy ID on SRAM for better handling
		lpm temp,Z+
		st X+,temp
		cpi XL,SRAM_START+8
		brlo pro_copy_loop
pro_loop:
		;sbi PORTB,0
		owwl				;wait for line goes low (polling)
		sbrs writelow,0		;test of zero send
		rjmp pro_loop1		;no ? goes next
		sbi OW_DDR,OW_PIN	;yes pull line to low
		ldi param,50		;wait for 50 us
		rcall wait_time
		clr writelow		;reset write low indecator
		cbi OW_DDR,OW_PIN   ;release line
		owwh				;wait for line is high (it can takes some time cause of the capacity of line)
pro_loop1:
		tst smode			;smode=1 for slave sends to master
		breq pro_loop_resv
pro_loop_send:
		tst bitp
		brne pro_loop_send1
		rcall pro_hb
		tst smode
		breq pro_loop_end ; now reading ... do nothing 
pro_loop_send1:  ;prebare next bit
		sbrs rwbyte ,0; if bit 0 set in rwbyte then skip next command
		inc writelow
		lsl bitp
		ror rwbyte
		rjmp pro_loop_end

pro_loop_resv:
		ldi param,15  ;wait 15us
		rcall wait_time
		lsr rwbyte
		;cbi PORTB,0
		sbic OW_PINN,OW_PIN  ;test line
		ori rwbyte,0x80
		lsl bitp
		brne pro_loop_end ;no handle need
		rcall pro_hb
		tst smode
		brne pro_loop_send ; Nach dem Gelesen byte koennte gesendet werden muessen....
pro_loop_end:
		//owwh
		out TCNT0,zero
pro_loop_end_test_reset:
		sbic OW_PINN,OW_PIN  //leitung wieder high
		rjmp pro_loop
		in temp,TCNT0
		cpi temp,130
		brlo pro_loop_end_test_reset
		rcall pro_sleep_s2
		rjmp pro_loop




pro_sleep:
		out TCNT0,zero
pro_sleep_s1:
		sbic OW_PINN,OW_PIN  //leitung wieder high
		ret
		in temp,TCNT0
		cpi temp,200
		brlo pro_sleep_s1
		//leitung wieder high
pro_sleep_s2:
		owwh 
		ldi param,40
		rcall wait_time
		//Presents Impuls
		sbi OW_DDR,OW_PIN
		ldi param,130
		rcall wait_time
		cbi OW_DDR,OW_PIN
		//init read byte
		ldi bitp,0x01
		ldi rwbyte,0
		clr smode
		ldi mode,OWM_READ_ROM_COMMAND
		//Wait for all other devices presents impuls finished
		ldi param,40
		rcall wait_time
		ret



pro_hb:
		ldi ZL,low(pro_stable) 
		ldi ZH,high(pro_stable) 
		add ZL,mode 
		adc ZH,zero
		icall
		ret

pro_stable: 
		rjmp pro_read_rom_command
		rjmp pro_sleep 
		rjmp pro_match_rom
		rjmp pro_search_rom_s
		rjmp pro_search_rom_r
		rjmp pro_read_command
		rjmp pro_write_scratchpad
		rjmp pro_read_scratchpad
		rjmp pro_programm_page
		rjmp pro_recall_flash

pro_read_rom_command:
		ldi mode,OWM_SLEEP
		cpi rwbyte,0xCC
		brne pro_rcc_1
		ldi mode,OWM_READ_COMMAND
		rjmp pro_out_bitp1
pro_rcc_1:
		cpi rwbyte,0xF0 ;Searchrom
		brne pro_rcc_2
		ldi XL,low(sowid)  ;init sram pointer
		ldi XH,high(sowid)
		ld srbyte,X+
		ldi bytep,0
		rjmp pro_serchrom_next_bit
pro_rcc_2:
		cpi rwbyte,0x55 ;Matchrom
		brne pro_rcc_3
//		rcall pro_owidinit
		ldi XL,low(sowid)  ;init sram pointer
		ldi XH,high(sowid)
		ldi mode,OWM_MATCH_ROM
		rjmp pro_out_bytep0

pro_rcc_3:
		ret

pro_match_rom:		
		ld temp,X+
		cp temp,rwbyte
		breq pro_match_rom_next
		ldi mode,OWM_SLEEP
		ret
pro_match_rom_next:					
		cpi XL,SRAM_START+8
		breq pro_match_rom_found
		rjmp pro_out_bitp1
pro_match_rom_found:
	    ldi mode,OWM_READ_COMMAND
		rjmp pro_out_bitp1

pro_read_command:
		ldi mode,OWM_SLEEP
		cpi rwbyte,0x0F ;; Write to Scratchpad
		brne pro_rc_1
		ldi mode,OWM_WRITE_SCRATCHPAD
		ldi XL,low(scratchpad)  ;init sram pointer
		ldi XH,high(scratchpad)
		rjmp pro_out_bytep0
pro_rc_1:
		cpi rwbyte,0xAA
		brne pro_rc_2
		ldi mode,OWM_READ_SCRATCHPAD  ;;Read from Scratchpad
		ldi XL,low(scratchpad)  ;init sram pointer
		ldi XH,high(scratchpad)
		inc smode
		ld rwbyte,X+
		rjmp pro_out_bytep0

pro_rc_2:
		cpi rwbyte,0xB8
		brne pro_rc_3
		ldi mode,OWM_RECALL_FLASH  ;; copy Flash page in Scratchpad
		ldi XL,low(scratchpad)  ;init sram pointer
		ldi XH,high(scratchpad)
		rjmp pro_out_bytep0
pro_rc_3:
		cpi rwbyte,0x55 ; copy Scratchpad to Flash
		brne pro_rc_4
		ldi mode,OWM_SLEEP
		rjmp pro_programm_page
		
pro_rc_4:
		cpi rwbyte,0x89 ; Reset Device /Boot (new) Firmware
		brne pro_rc_5
		rjmp jreset		
pro_rc_5:
		cpi rwbyte,0x8B ; Clear the OWID saved in EEPROM / one ID1
		brne pro_rc_6	
		ldi temp,7
pro_rc_5a:
		ldi XL,low(E2END)
		ldi XH,high(E2END)
		sub XL,temp
		out EEARH,XH
		out EEARL,XL
		ldi temp, (0<<EEPM1)|(0<<EEPM0)
		out EECR, temp
		ldi temp,0xFF
		out EEDR, temp
		sbi EECR, EEMPE
		sbi EECR, EEPE
		ret


pro_rc_6:
		cpi rwbyte,0x8C ; Clear the OWID saved in EEPROM / one ID2
		brne pro_rc_7
		ldi temp,7+8
		rjmp pro_rc_5a

pro_rc_7:
		ret

pro_write_scratchpad:
		st X+,rwbyte
		cpi XL,SRAM_START+8+66
		brlo pro_write_scratchpad_next
		ldi mode,OWM_SLEEP
		ret		
pro_write_scratchpad_next:
		ldi bitp,1
		ret

pro_read_scratchpad:
		cpi XL,SRAM_START+8+66
		brlo pro_read_scratchpad_next
		ldi mode,OWM_SLEEP
		clr smode
		ret
pro_read_scratchpad_next:
		ld rwbyte,X+
		rjmp pro_out_bitp1


pro_programm_page:
.equ PAGESIZEB = PAGESIZE*2;PAGESIZEB is page size in BYTES, not words
// .org SMALLBOOTSTART
write_page:
		;transfer data from RAM to Flash page buffer
		ldi bytep, PAGESIZEB ;init loop variable
		ldi YL,low(scratchpad)  ;init sram pointer
		ldi YH,high(scratchpad)
		ld ZL,Y+
		ld ZH,Y+
		;page erase
		ldi spmcrval, (1<<PGERS) + (1<<SPMEN)
		rcall do_spm
wrloop:
		ld r0, Y+
		ld r1, Y+
		ldi spmcrval, (1<<SPMEN)
		rcall do_spm
		adiw ZH:ZL, 2
		subi bytep, 2;use subi for PAGESIZEB<=256
		brne wrloop
		;execute page write
		subi ZL, low(PAGESIZEB) ;restore pointer
		sbci ZH, high(PAGESIZEB) ;not required for PAGESIZEB<=256
		ldi spmcrval, (1<<PGWRT) + (1<<SPMEN)
		rcall do_spm
		;read back and check, optional
		ldi bytep, PAGESIZEB
		subi YL, low(PAGESIZEB) ;restore pointer
		sbci YH, high(PAGESIZEB)
rdloop:
		lpm r0, Z+
		ld r1, Y+
		cpse r0, r1
		rjmp error
		subi bytep, 2;use subi for PAGESIZEB<=256
		brne rdloop
		;return
		ret
do_spm:
		 ;input: spmcrval determines SPM action
		;disable interrupts if enabled, store status
		in temp2, SREG
		cli
		;check for previous SPM complete
wait:
		in temp, SPMCSR
		sbrc temp, SPMEN
		rjmp wait
		;SPM timed sequence
		out SPMCSR, spmcrval
		spm
		;restore SREG (to enable interrupts if originally enabled)
		out SREG, temp2
		ret

error:


		ret

pro_recall_flash:
		st X+,rwbyte
		;inc bytep
		cpi XL,SRAM_START+8+2
		brlo pro_out_bitp1;pro_recall_flash_next
		lds ZL,scratchpad
		lds ZH,scratchpad+1
pro_recall_flash_cl:
		lpm temp,Z+
		st X+,temp
		cpi XL,SRAM_START+8+66
		brne pro_recall_flash_cl
		ldi mode,OWM_SLEEP
		ret

pro_out_read_command:
		ldi mode,OWM_READ_COMMAND
pro_out_bytep0:
		ldi bytep,0
pro_out_bitp1:
		ldi bitp,1
		ret		



pro_serchrom_next_bit:
		mov rwbyte,srbyte
		mov temp2,rwbyte
		com rwbyte
		ror temp2  ;first bit in C
		rol rwbyte ;C in first bit 
		inc smode
		ldi bitp,0x40
		ldi mode,OWM_SEARCH_ROM_R  ;next mod Resive
		ret


pro_search_rom_s:
		clr temp2
		lsr srbyte ;shift in C lowest bit
		ror temp2  ; shift in temp2 as highest bit
		andi rwbyte,0x80  ;  clear other bits
		eor temp2,rwbyte
		breq pro_search_rom_s_goon
		ldi mode,OWM_SLEEP
		ret
pro_search_rom_s_goon:
		inc bytep
		mov temp2,bytep
		andi temp2,0x07
		brne pro_serchrom_next_bit ;prepare next bit
		mov temp2,bytep
		andi temp2,0x40 ;;end
		brne pro_search_rom_found
		;read next byte
		ld srbyte,X+
		rjmp pro_serchrom_next_bit

pro_search_rom_found:
		ldi mode,OWM_READ_COMMAND
		rjmp pro_out_bytep0

pro_search_rom_r:
		clr smode
		ldi mode,OWM_SEARCH_ROM_S
		ldi bitp,0  ;go to searchrom_s after bit get
		ret

pro_owid: .DB  0xA3, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xFA





wait_time:
		out TCNT0,zero
wait_time1:
		in temp,TCNT0
		cp temp,param
		brlo wait_time1
		ret


.DSEG
sowid: .BYTE 8
scratchpad: .BYTE 66