
; OSTC Mk.2, 2N and 2C - diving computer code
; Copyright (C) 2015 HeinrichsWeikamp GbR

;    This program is free software: you can redistribute it and/or modify
;    it under the terms of the GNU General Public License as published by
;    the Free Software Foundation, either version 3 of the License, or
;    (at your option) any later version.

;    This program is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;    GNU General Public License for more details.

;    You should have received a copy of the GNU General Public License
;    along with this program.  If not, see <http://www.gnu.org/licenses/>.


; Interface, Downloader, MD2 Hash send routine
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 10/30/05
; last updated: 12/27/07
; known bugs:
; ToDo: 

uart_store_tissues:
	bcf		uart_store_tissue_data			; Clear flag
	bcf		PIE1,RCIE						; No Interrupt for UART
	bsf		LED_blue
	call	simulator_save_tissue_data		; store and set flag for automatic restore
	movlw		'k'							; send echo
	movwf		TXREG						; When done
	call		rs232_wait_tx				; wait for UART
	bcf			LED_blue
	bsf			PIE1,RCIE				; Interrupt for RS232
	goto		surfloop_loop			; return to surface loop

; reset Decodata
reset_decodata:
	bcf			uart_reset_decodata				; clear flag
	bcf			PIE1,RCIE						; No Interrupt for UART
	bsf			LED_blue

	call		deco_clear_tissue			; Reset Decodata
	movlb		b'00000001'						; select ram bank 1
	call		deco_calc_desaturation_time	; calculate desaturation time
	movlb		b'00000001'						; select ram bank 1
	call		deco_clear_CNS_fraction			; clear CNS
	movlb		b'00000001'						; select ram bank 1

	clrf		nofly_time+0				; Clear nofly time
	clrf		nofly_time+1				; Clear nofly time

	movlw		'h'							; send echo
	movwf		TXREG						; When done
	call		rs232_wait_tx				; wait for UART

	bsf			oneminupdate				; set flag, so display will be updated at once
	bcf			LED_blue
	bsf			PIE1,RCIE					; Interrupt for RS232
	goto		surfloop_loop				; return to surface loop

; send internal EEPROM BANK 0 via the UART
send_int_eeprom_b0:
	bcf			uart_send_int_eeprom	; clear flag
	movlw		.0						; Point to Bank0
	rcall		send_internal_eeprom1	; sends complete 1st. page of internal EEPROM
	goto		surfloop_loop			; return to surface loop

; send internal EEPROM BANK 1 via the UART
send_int_eeprom_b1:
	bcf			uart_send_int_eeprom2	; clear flag
	movlw		d'1'
	movwf		EEADRH					; Point to Bank1
	rcall		send_internal_eeprom1	; sends complete 2nd page of internal EEPROM
	goto		surfloop_loop			; return to surface loop

; send internal EEPROM BANK 2 via the UART
send_int_eeprom_b2:
	bcf			uart_send_int_eeprom3	; clear flag
	movlw		d'2'
	movwf		EEADRH					; Point to Bank2
	rcall		send_internal_eeprom1	; sends complete 2nd page of internal EEPROM
	goto		surfloop_loop			; return to surface loop


; Send firmware version and 16bytes MD2 hash via the UART
send_md2_hash:
	bcf			uart_send_hash			; clear flag
	bcf			PIE1,RCIE				; No Interrupt for UART
	bsf			LED_blue

	call		rs232_wait_tx					; wait for UART
	movlw		softwareversion_x				; Softwareversion
	movwf		TXREG
	call		rs232_wait_tx					; wait for UART
	movlw		softwareversion_y				; Softwareversion
	movwf		TXREG

	lfsr		FSR2, char_O_hash
	movlw		d'16'
	movwf		temp1
send_md2_hash2:
	call		rs232_wait_tx					; wait for UART
	movff		POSTINC2,TXREG					; copy hash byte to TXREG
	decfsz		temp1,F
	bra		send_md2_hash2					; loop 16 times

	bcf			LED_blue
	bsf			PIE1,RCIE				; Interrupt for RS232
	goto		surfloop_loop			; return to surface loop


; Sends first 256Byte from internal and first 32KB from external EEPROM using the UART module
menu_interface:
	bcf		dump_external_eeprom			; clear flag
	bcf		PIE1,RCIE					; No Interrupt for UART
	bsf		LED_blue
	call	DISP_ClearScreen
    call	DISP_divemask_color
	DISPLAYTEXT	.15						; "Interface"
    call    DISP_standard_color
	movlw		d'5'
	movwf		uart1_temp
menu_interface1:
	movlw		0xAA						; Startbytes
	movwf		TXREG
	call		rs232_wait_tx				; wait for UART
	decfsz	uart1_temp
	bra		menu_interface1
	movlw		0x55						; finish preamble
	movwf		TXREG

	DISPLAYTEXT	.16						; "Start"

	call		get_free_EEPROM_location		; 

	movlw		d'1'						; increase
	addwf		eeprom_address+0,F
	movlw		d'0'
	addwfc		eeprom_address+1,F

;For debug only
;clrf	eeprom_address+0,F
;clrf	eeprom_address+1,F

	DISPLAYTEXT	.17						; "Data"

	movlw		.0							; Point to Bank0
	rcall		send_internal_eeprom1		; sends complete 1st. page of internal EEPROM
	bsf			LED_blue

	call		rs232_wait_tx				; wait for UART
	movff		batt_voltage+0,TXREG			; Battery

	call		rs232_wait_tx				; wait for UART
	movff		batt_voltage+1,TXREG			; Battery 

	call		rs232_wait_tx				; wait for UART
	movlw		softwareversion_x				; Softwareversion
	movwf		TXREG

	call		rs232_wait_tx				; wait for UART
	movlw		softwareversion_y				; Softwareversion
	movwf		TXREG

	DISPLAYTEXT .18						; "Header"

	setf		uart1_temp					; low address counter
	setf		uart2_temp					; high address counter

menu_interface3:
	bsf		SSPCON2,SEN					; Start condition
	call		WaitMSSP

	movlw		b'10101110'			; Bit0=0: WRITE, Bit0=1: READ, BLOCK2
#IFDEF SEAWOOCH
	btfsc		seawooch_EE512
	bra			ee512_100
#ENDIF
	btfss		eeprom_address+1,7	; Access Block2?
	movlw		b'10100110'			; No, -> Bit0=0: WRITE, Bit0=1: READ, BLOCK1
#IFDEF SEAWOOCH
ee512_100:
#ENDIF
	movwf		SSPBUF					; control byte
	call		WaitMSSP	
	btfsc		SSPCON2,ACKSTAT
	bra		menu_interface3				; No Ack, try again!
	
	movff		eeprom_address+1,SSPBUF			; High Address byte
	call		WaitMSSP	
	call		I2C_WaitforACK
	movff		eeprom_address+0,SSPBUF			; Low Address byte
	call		WaitMSSP	
	call		I2C_WaitforACK
	bsf		SSPCON2,RSEN				; Start condition
	call		WaitMSSP

	movlw		b'10101111'			; Bit0=0: WRITE, Bit0=1: READ, BLOCK2
#IFDEF SEAWOOCH
	btfsc		seawooch_EE512
	bra			ee512_110
#ENDIF
	btfss		eeprom_address+1,7	; Access Block2?
	movlw		b'10100111'			; No, -> Bit0=0: WRITE, Bit0=1: READ, BLOCK1
#IFDEF SEAWOOCH
ee512_110:
#ENDIF

	movwf		SSPBUF					; control byte
	call		WaitMSSP	
	call		I2C_WaitforACK

	DISPLAYTEXT	.19						; "Profile"

menu_interface2:
	call		rs232_wait_tx				; wait for UART

	movlw		d'1'
	addwf		uart1_temp,F
	movlw		d'0'
	addwfc		uart2_temp,F

; Slow but safe...
	call		I2CREAD2					; same as I2CREAD but with automatic address increase 
	movff		SSPBUF, TXREG

	movlw		0xFF
	cpfseq		uart2_temp					;=0xFFFF?
	bra			menu_interface2				; No, continue
	cpfseq		uart1_temp					;=0xFFFF?
	bra			menu_interface2				; No, continue

	DISPLAYTEXT	.20						; Done.

	WAITMS	d'250'
	bcf			LED_blue
	bsf			PIE1,RCIE					; Interrupt for RS232
	goto		surfloop_no_display_init	; back to surfacemode

send_internal_eeprom1:
	movwf		EEADRH						; Point to Bank "WREG"
	bcf			PIE1,RCIE					; No Interrupt for UART
	bsf			LED_blue
	clrf		uart1_temp					; Send the total of 256bytes
	clrf		EEADR						; Send bytes 0-255 from internal EEPROM
send_internal_eeprom2:
	call		read_eeprom					; read byte
	movff		EEDATA,TXREG				; send byte
	incf		EEADR,F						; increase pointer
	call		rs232_wait_tx				; wait for UART
	decfsz		uart1_temp,F				; until limit reached
	bra			send_internal_eeprom2
	clrf		EEADRH						; Point to Bank0
	bcf			LED_blue
	bsf			PIE1,RCIE				; Interrupt for RS232
	return