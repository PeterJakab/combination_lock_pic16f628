;**********************************************************************
;                                                                     *
;    Filename:	    cl2.asm                                           *
;    Date:                                                            *
;    File Version:  Combination lock rewritten                        *
;                                                                     *
;    Author:        Peter Jakab <el@jap.hu>                           *
;                   http://jap.hu/electronic/                         *
;**********************************************************************
;NOTES
;
; the rewritten combination lock has the following changes:
;
; - no multiplexer is needed (the old version used a generic devboard)
; - no row diodes needed (rows are never driven HIGH)
; - low power consumption due to keyboard wake-up
; - stores the code in the internal EEPROM
; - user defined codelength and pulse output
; - user adjustable running frequency
; - improved code changing function with a "change" indicator LED
;
; PIC ports used:
;
; PA1 output: code change indicator LED
; PA2 output: output pulse to control a relay
; PA3 output: piezo beeper output
; PB0-PB3 outputs: keypad row select pulldown outputs (weak-pullup or driven LOW)
; PB4-PB7 inputs:  keypad column inputs with internal pullup
;
;**********************************************************************
;HISTORY
;
; 020-20010929 rewrite started
; 021-20011022 udelay calibrated to 100 usec (4, 10 MHz)
; 022-20011022 scan, input and compare functions work
; 023-20011022 code change function works
; 024-20041003 adapt source to the new pic16F628
; 025-20041016 add an LCD (on popular request)
;
;**********************************************************************
	list	p=16f628
	__CONFIG   _CP_OFF & _WDT_OFF & _PWRTE_ON & _INTRC_OSC_NOCLKOUT & _LVP_OFF & _BODEN_ON & _MCLRE_OFF

#include <p16F628.inc>
#include "lcdlib.inc"

#define HIDDEN_CODE '*'
mhz		EQU D'4'  ; processor frequency in MHz
pulsewidth	EQU D'150'; delay in 20ms steps (150=3 sec)
clen		EQU 4     ; length of code

msg_line	EQU 0x80  ; LCD position for messages
code_line	EQU 0xc0  ; LCD position for code

; EEPROM contents
eeprom		CODE 0x2100
		de "1234"   ; default code (clen chars are used)
			    ; which is stored in EEPROM

		UDATA
; RAM registers
dcnt0		RES 1 ; delay counter 0
dcnt1		RES 1 ; delay counter 1
dcnt2		RES 1 ; delay counter 2
beepcnt		RES 1 ; beep cycle counter
keycode		RES 1
rowcnt		RES 1
colcnt		RES 1
colstatus	RES 1


cod		RES clen ; actual code
cod_end		
readlen		RES 1
readbuf		RES clen
readbuf_end	
tmptr		RES 1 ; pointer for comparing and copying readbuf
tmbyte		RES 1 ; temp storage for comparing and copying

startup		CODE 0
  		goto main
		nop
		nop
		nop
		retfie

prog		CODE


messages	clrf PCLATH
		movwf PCL

msg_code	dt "enter code:     ", 0
msg_change	dt "change code to: ", 0
msg_repeat	dt "repeat new code:", 0
msg_ok		dt "code accepted   ", 0


keytable	;determine pressed key's real code from scancode
		movf keycode, W

		clrf PCLATH
		addwf PCL, F
		dt 0x60
		dt "123a"
		dt "456b"
		dt "789c"
		dt "*0#d"

eep_read	; read EEPROM contents to RAM from cod to cod_end-1
		movlw cod
		movwf FSR
		bsf STATUS, RP0
		bcf STATUS, IRP
		clrf EEADR

eep_0		;bcf INTCON, GIE
		bsf STATUS, RP0
		bsf EECON1, RD
		;bsf INTCON, GIE
		movf EEDATA, W
		movwf INDF

		incf FSR, F
		incf EEADR, F

		bcf STATUS, RP0
		movlw cod_end
		subwf FSR, W
		bnz eep_0
		return

eep_write	; save RAM contents to EEPROM from cod to cod_end-1
		movlw cod
		movwf FSR
		bsf STATUS, RP0
		bcf STATUS, IRP
		clrf EEADR

eep_1		bsf STATUS, RP0
		movf INDF, W
		movwf EEDATA
		;bcf INTCON, GIE
		bsf EECON1, WREN
		movlw 0x55
		movwf EECON2
		movlw 0xaa
		movwf EECON2
		bsf EECON1, WR

		; wait for write completition
eep_2		;bsf INTCON, GIE
		nop
		nop
		;bcf INTCON, GIE
		btfsc EECON1, WR
		goto eep_2
		;bsf INTCON, GIE

		incf EEADR, F
		bcf STATUS, RP0
		incf FSR, F

		movlw cod_end
		subwf FSR, W
		bnz eep_1
		return

udelay		; delay W * 100 usec
		movwf dcnt0

udelay0		movlw 8 * mhz
		movwf dcnt1

udelay1		decfsz dcnt1, F
		goto udelay1

		decfsz dcnt0, F
		goto udelay0

		return

beep		movwf beepcnt
beep0		bsf PORTA, 3 ; beepctl bit
		movlw 3
		call udelay
		bcf PORTA, 3 ; beepctl bit
		movlw 3
		call udelay
		decfsz beepcnt, F
		goto beep0
		return

keyscan		; scan the keyboard
		clrf keycode
		movlw 4
		movwf rowcnt

		movlw 0xfe
		tris PORTB ; select row 0

rowscan		movlw 0xa0
		call udelay
		swapf PORTB, W
		movwf colstatus

		movlw 4
		movwf colcnt

colscan		incf keycode, F
		rrf colstatus, F
		btfss STATUS, C
		goto keytable ; a key was found

		decfsz colcnt, F
		goto colscan

		bsf STATUS, C

		bsf STATUS, RP0
		rlf TRISB, F ; select next row
		bcf STATUS, RP0

		decfsz rowcnt, F
		goto rowscan
		retlw 0 ; no key was found

main		; program starts here
		movlw 7
		movwf CMCON

		clrf PORTA
		clrw
		tris PORTA ; porta all output
		clrf PORTB
		movlw 0xf0 ; pb4-7 inputs
		tris PORTB
		bsf STATUS, RP0 ; bank 1
		bcf OPTION_REG, NOT_RBPU ;internal pullups on port B enabled
		bcf STATUS, RP0 ;bank 0

		call lcd_init

warm		movlw 0xf0
		call beep
		call eep_read ; read code from eeprom to ram at cod

loop		clrf PORTA ; clear output

		movlw msg_line
		call lcd_cmdout
		movlw msg_code
		call lcd_strout
		
		call read ; read code from keyboard into readbuf
		movlw cod
		call compbuf ; compare code in readbuf with code at cod
		bnz loop ; the code is different

		; the code matches, check which enter (#*) was pressed

		movlw '*'
		subwf keycode, W ; * changes code
		bz codechange

pulseout	; # operates output
		movlw 0x04 ; RA2 is output
		movwf PORTA

		movlw msg_line
		call lcd_cmdout
		movlw msg_ok
		call lcd_strout

		movlw pulsewidth
		movwf dcnt2

out0		movlw d'200'
		call udelay
		decfsz dcnt2, F
		goto out0

		goto loop

codechange	movlw 2     ; * changes code
		movwf PORTA ; indicate changing the code

		movlw msg_line
		call lcd_cmdout
		movlw msg_change
		call lcd_strout

		call read   ; read new code into readbuf

		movlw cod
		call copybuf ; copy new code into cod

		movlw msg_line
		call lcd_cmdout
		movlw msg_repeat
		call lcd_strout

		call read ; read new code twice
		movlw cod ; and check if the new code is confirmed
		call compbuf ; wrong code entry, restart with the original code
		bnz warm

		; new code is comfirmed twice, store into eeprom
		call eep_write
		goto loop

read		clrf readlen

readloop	movlw code_line
		call lcd_cmdout

		clrf colcnt
readpr0		movf colcnt, W
		subwf readlen, W
		bz readpr1

#ifdef HIDDEN_CODE
		movlw HIDDEN_CODE
#else
		movlw readbuf
		addwf colcnt, W
		movwf FSR
		movf INDF, W
#endif
		call lcd_chrout
		incf colcnt, F
		goto readpr0

readpr1		movf colcnt, W
		sublw clen
		bz readpr2
		movlw '-'
		call lcd_chrout
		incf colcnt, F
		goto readpr1

readpr2		movlw code_line
		addwf readlen, W
		call lcd_cmdout
		movlw 0x0e
		call lcd_cmdout

readloop2	; wait until no key is pressed
		movlw 0xf0
		tris PORTB ; rows all LOW
		movf PORTB, W
		andlw 0xf0 ; keymask
		xorlw 0xf0
		btfss STATUS, Z
		goto readloop2

		movlw  0xf0  ; wait 24 ms
		call udelay  ; (debounce)

		; no key pressed, go to sleep
		movf PORTB, W
		movlw 1<<RBIE ; enable RB port change wake-up
		movwf INTCON
		sleep

key_pressed
		call keyscan
		andlw 0xff
		movwf keycode
		bz readloop2

		movlw 0xf0   ; wait 24 ms
		call udelay  ; (debounce)

		; check if the buffer is full
		movlw clen
		subwf readlen, W
		bnz read_notfull

		; buffer is full, can return if an enter key (*#) is pressed

		; check for ENTER
		call read_chkenter
		bnz read_notenter

		; enter is pressed, return
		movlw 0x40
		call beep
		movlw code_line
		call lcd_cmdout
		movlw clen+1
		movwf colcnt
readpr3		movlw ' '
		call lcd_chrout
		decfsz colcnt, F
		goto readpr3

		movf keycode, W
		return

read_notenter
		; buffer is full, but more characters entered
		; shift the buffer
		movlw readbuf+1
		movwf FSR

read_shift	movf INDF, W
		decf FSR, F
		movwf INDF
		incf FSR, F
		incf FSR, F
		movlw readbuf_end
		subwf FSR, W
		bnz read_shift

		decf readlen, F

read_notfull	call read_chkenter ; if the buffer is not full and an
		bz read ; enter key (*#) is pressed, clear buffer

		movlw 0x40
		call beep

		movlw readbuf
		addwf readlen, W
		movwf FSR
		movf keycode, W
		movwf INDF
		incf readlen, F
		goto readloop

read_chkenter	; check if a * or # is pressed which indicates
		; the end of entry
		movlw '#'
		subwf keycode, W
		btfsc STATUS, Z
		return ; Z=1, enter
		movlw '*'
		subwf keycode, W
		return

compbuf		; compare read buffer to a code in RAM at W

		movwf tmptr    ; compare pointer
		clrf readlen   ; compare index starts from 0

comp0		movlw readbuf
		addwf readlen, W
		movwf FSR
		movf INDF, W
		movwf tmbyte ; the read byte which is compared

		movf tmptr, W
		addwf readlen, W
		movwf FSR
		movf INDF, W ; the byte readbuf is compared to
		subwf tmbyte, W
		btfss STATUS, Z
		return ; Z=0: the code is different

		incf readlen, F
		movlw clen
		subwf readlen, W
		bnz comp0 ; compare next character
		; Z=1: the code is the same

		return

copybuf		; copy readbuf to RAM at W
		movwf tmptr    ; copy pointer
		clrf readlen   ; copy index starts from 0

copy0		movlw readbuf
		addwf readlen, W
		movwf FSR
		movf INDF, W
		movwf tmbyte ; the read byte which is copied

		movf tmptr, W
		addwf readlen, W
		movwf FSR
		movf tmbyte, W ; the byte from readbuf
		movwf INDF

		incf readlen, F
		movlw clen
		subwf readlen, W
		bnz copy0 ; copy next character

		return

		end

