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


; internal EEPROM and RS232 UART interface
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 02/01/06
; last updated: 090109
; known bugs:
; ToDo: 

write_int_eeprom	macro	eeprom_address
	movlw	eeprom_address
	call	write_int_eeprom_1
	endm

write_int_eeprom_1:
	movwf	EEADR
	bra		write_eeprom			; writes and "returns" after write

read_int_eeprom	macro	eeprom_address
	movlw	eeprom_address
	call	read_int_eeprom_1
	endm

read_int_eeprom_1:
	movwf	EEADR
	bra		read_eeprom					; reads and "returns" after write

internal_eeprom_access_b2:				; accesses internal EEPROM BANK 2 via the UART
	bcf		internal_eeprom_write3		; clear flag!
	movlw	d'2'
	movwf	EEADRH						;BANK2
	movlw	"n"
	bra		internal_eeprom_access1		; Continue with common routines

internal_eeprom_access_b1:				; accesses internal EEPROM BANK 1 via the UART
	bcf		internal_eeprom_write2		; clear flag!
	movlw	d'1'
	movwf	EEADRH						;BANK1
	movlw	"i"
	bra		internal_eeprom_access1		; Continue with common routines

internal_eeprom_access_b0:				; accesses internal EEPROM BANK 0 via the UART
	bcf		internal_eeprom_write		; clear flag!
	clrf	EEADRH						; Bank0
	movlw	"d"
;	bra		internal_eeprom_access1		; Continue with common routines
internal_eeprom_access1:
	movwf	TXREG						; Send command echo ("i", "d" or "n")
	bsf		no_sensor_int				; No Sensor Interrupt
	movlw	d'4'
	movwf	EEADR
	bcf		PIE1,RCIE					; no interrupt for UART
	bcf		PIR1,RCIF					; clear flag
	bsf		LED_blue					; LEDusb ON

internal_eeprom_access2:
	rcall	rs232_get_byte				; Get byte to write...
	movff	RCREG,EEDATA				; copy to write register
	bsf		LED_red						; show activity

	btfsc	rs232_recieve_overflow		; overflow recieved?
	bra		internal_eeprom_access3		; Yes, abort!

	rcall	write_eeprom				; No, write one byte
	bcf		LED_red
	movff	EEDATA,TXREG				; Send echo!
	rcall	rs232_wait_tx				; Wait for UART
	incfsz 	EEADR,F						; Do until EEADR rolls to zero
	bra		internal_eeprom_access2
internal_eeprom_access2a:
	bcf		LED_blue					; LEDusb OFF
	bcf		PIR1,RCIF					; clear flag
	bsf		PIE1,RCIE					; re-enable interrupt for UART
	clrf	EEADRH						; Point to Bank0 again
	bcf		rs232_recieve_overflow		; Clear Flag
	bcf		no_sensor_int				; Renable Sensor Interrupt
	goto	restart

internal_eeprom_access3:				; Overflow! Abort writing
	movlw	0xFF
	movwf	TXREG						; Error Byte
	bra		internal_eeprom_access2a	; Quit

read_eeprom: 							; reads from internal eeprom
	bcf		EECON1,EEPGD
	bcf		EECON1,CFGS
	bsf		EECON1,RD
	return

write_eeprom:							; writes into internal eeprom
	bcf		EECON1,EEPGD
	bcf		EECON1,CFGS
	bsf		EECON1,WREN

	bcf		INTCON,GIE					; even the RTC will be delayed for the next 5 instructions...
	movlw	0x55		
	movwf	EECON2
	movlw	0xAA
	movwf	EECON2
	bsf		EECON1,WR
	bsf		INTCON,GIE					; ...but the flag for the ISR routines were still set, so they will interrupt now!

write_eep2:
	btfsc	EECON1,WR		
	bra 	write_eep2					; wait about 4ms...
	bcf		EECON1,WREN
	return

enable_rs232:				;IO Ports must be input in order to activate the module
	bsf		TRISC,6			; TX Pin
	bsf		TRISC,7			; RX Pin

	movlw	b'00100100'			; BRGH=1
	movwf	TXSTA
	movlw	b'10010000'
	movwf	RCSTA
	movlw	b'00001000'
	movwf	BAUDCON
	clrf	SPBRGH
	movlw	SPBRG_VALUE			; Take care of the baud rate when changing Fosc!
	movwf	SPBRG
	clrf	RCREG
	clrf	PIR1
	bsf		PIE1,RCIE			; enable interrupt for RS232
	return

disable_rs232:
	clrf	TXSTA
	clrf	RCSTA
	bcf		PIE1,RCIE			; disable interrupt for RS232
	bcf		TRISC,6			; TX Pin
	bcf		TRISC,7			; RX Pin
	bcf		PORTC,6			; TX Pin
	bcf		PORTC,7			; RX Pin
	return

rs232_wait_tx:
	btfss	RCSTA,SPEN			; Transmitter active?
	return						; No, return!
	nop
	btfss	TXSTA,TRMT			; RS232 Busy?
	bra		rs232_wait_tx		; yes, wait...
	return						; Done.


rs232_get_byte:
	bcf		PIR1,RCIF		; clear flag
	bcf		rs232_recieve_overflow		; clear flag
	clrf 	uart1_temp
rs232_get_byte2:
	clrf 	uart2_temp
rs232_get_byte3:
	btfsc 	PIR1,RCIF		; data arrived?
	return					; data received

	nop						; Wait 1us * 255 * 255 = 65ms+x Timeout/Byte
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	btfsc 	PIR1,RCIF		; data arrived?
	return	
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	btfsc 	PIR1,RCIF		; data arrived?
	return	
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	btfsc 	PIR1,RCIF		; data arrived?
	return	
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	btfsc 	PIR1,RCIF		; data arrived?
	return	

	decfsz 	uart2_temp,F
	bra 	rs232_get_byte3
	decfsz 	uart1_temp,F
	bra		rs232_get_byte2
						; timeout occoured (about 20ms)
	bcf		RCSTA,CREN		; Clear receiver status
	bsf		RCSTA,CREN
	bsf		rs232_recieve_overflow		; set flag
	return				; and return anyway

#IFNDEF CCR_CTRL        ; Disable bootloader in CCR compilation
uart_115k_bootloader:
	bcf		PIE1,RCIE				; disable interrupt for RS232
	call	DISP_ClearScreen		; Clear screen
	movlw	color_red
    call	DISP_set_color			; Set to Red
	DISPLAYTEXTH	d'302'			; Bootloader
	bcf		RCSTA,CREN				; Clear receiver status
	bsf		RCSTA,CREN
	bcf		PIR1,RCIF				; clear flag
	movlw	d'200'					; one second
	movwf	uart1_temp
uart_115k_bootloader0:
	btfsc	PIR1,RCIF				; New byte in UART?
	bra		uart_115k_bootloader1	; Yes, Check if 0xC1
	WAITMS	d'5'
	decfsz	uart1_temp,F
	bra		uart_115k_bootloader0
uart_115k_bootloader2:
	DISPLAYTEXTH	d'304'			; Aborted!
	movlw	d'8'					; Two seconds
	movwf	uart1_temp
uart_115k_bootloader3:
	WAITMS	d'250'
	decfsz	uart1_temp,F
	bra		uart_115k_bootloader3
	goto	restart
	
uart_115k_bootloader1:
	movlw	0xC1
	cpfseq	RCREG					; 115200Baud Bootloader request?
	bra		uart_115k_bootloader2	; No, Abort

; Vault date and time during update
; EEPROM Bank1
; Byte 5: =0xAA -> reload time and date in "restart:"
; Byte 6-11: YYMMDDHHMMSS
    movlw   .1
    movwf   EEADRH
    movff   year,EEDATA
    write_int_eeprom	d'6'
    movff   month,EEDATA
    write_int_eeprom	d'7'
    movff   day,EEDATA
    write_int_eeprom	d'8'
    movff   hours,EEDATA
    write_int_eeprom	d'9'
    movff   mins,EEDATA
    write_int_eeprom	d'10'
    movff   secs,EEDATA
    write_int_eeprom	d'11'
    movlw   0xAA
    movwf   EEDATA
    write_int_eeprom	d'5'        ; Set flag
    clrf    EEADRH
	DISPLAYTEXTH	d'303'			; Yes, "Please wait!"
	clrf	INTCON					; Interrupts disabled
	bcf		PIR1,RCIF				; clear flag
	goto	0x17F56					; Enter straight into bootloader. Good luck!
#ENDIF

;#IFDEF CCR_CTRL
; RS232 Routines:
;send_19k2	macro	char			; Sends char in standard pin mode
;	movlw	char
;	movwf	uart1_temp
;	call	send_19k2_1
;	endm

;send_19k2_1:
;;send_19k2_IO_1:
;;	call	enable_rs232_19k2_tx	; enable UART TX
;	call	rs232_wait_tx           ;Wait in case previous transfer still in progress.
;	movff	uart1_temp,TXREG
;;	call	disable_rs232_19k2_tx	; disable UART TX for higher current I/O
;	return
;;	call	enable_rs232_19k2_tx	; enable UART TX
;;	movff	uart1_temp,TXREG
;;	call	rs232_wait_tx
;;	call	disable_rs232_19k2_tx	; disable UART TX for higher current I/O
;;	return

;enable_rs232_19k2_IO:
;	bsf		TRISC,7				; RX Pin
;	bcf		TRISC,6				; TX PIN
;
;	movlw	b'00000100'			; Only receiver mode, BRGH=1
;	movwf	TXSTA
;	movlw	b'10000000'
;	movwf	RCSTA
;	movlw	b'00001000'			; BRG16=1
;	movwf	BAUDCON
;	bsf		RCSTA,CREN				; Enable
;	movlw	d'1'
;	movwf	SPBRGH
;	movlw	d'159'				; Take care of the baud rate when changing Fosc! (32MHz/19200/4)-1
;	movwf	SPBRG
;	clrf	RCREG
;	bcf		PIE1,RCIE			; disable interrupt for RS232
;	clrf	PIR1
;	bsf		PORTC,6				; TX PIN
;	return
;#ENDIF

#IFDEF CCR_CTRL
    ; Initialize UART for 115200 Interrupt mode
enable_rs232_115k_ISR:
	bsf		TRISC,6             ; TX Pin
	bsf		TRISC,7             ; RX Pin

	movlw	b'00100100'			; TXEN = 1, BRGH=1
	movwf	TXSTA
	movlw	b'10000000'         ;SPEN = 1
	movwf	RCSTA
	movlw	b'00001000'
	movwf	BAUDCON
	bsf		RCSTA,CREN			; Enable UART

	clrf	SPBRGH
	movlw	d'69'               ; Error -.079% with Fosc = 32MHz
	movwf	SPBRG

	clrf	RCREG
	bsf		PIE1,RCIE			; enable interrupt for RS232
	clrf	PIR1
	return

send_message:
    ; Initialize and send message to eCTRL
    ; TODO: check if last transfer still is in process.
    movff   eCTRL_F, WREG           ; Check "send disable" flag
;    btfss   eCTRL_F_send_disable_W
;    bra     send_message_next_0
;	return                          ; Send is disabled
;send_message_next_0:
    btfsc   eCTRL_F_send_disable_W
    bra     send_message_next_end
    movlb   eCTRL_PAGE              ; Select eCTRL Data Page
    ; Filling out buffer with actual data to send
    clrf    eCTRL_out_buff_sum      ; Reset Send Control summ register
    movlw	":"                     ; store "Start" symbol
    movwf   eCTRL_out_buff+0x00
    ; TODO Cyclic command/info transmit
    clrf    WREG
    cpfsgt  eCTRL_CMD               ; New Immediate Command is pending
    bra     send_message_next_cyc
    ; Immediate message ready to send
    movlw   eCTRL_CMD_ACT           ; is SP changed?
    cpfseq  eCTRL_CMD
    bra     send_message_next_1
    bra     send_message_SP
send_message_next_1:                ; Calibration request
    movlw   eCTRL_CMD_CAL
    cpfseq  eCTRL_CMD
    bra     send_message_next_2
    bra     send_message_CAL
send_message_next_2:                ; Sleep request
    movlw   eCTRL_CMD_SLEEP
    cpfseq  eCTRL_CMD
    bra     send_message_next_3
    bra     send_message_SLEEP
send_message_next_3:                ; BT activation request
    movlw   eCTRL_CMD_BT_EN
    cpfseq  eCTRL_CMD
    bra     send_message_next_cyc
    bra     send_message_BT_EN

    ; Unrecognized Immediate command pending - continue with cycke message then
send_message_next_cyc:
    ; Cycling message
    movlw   0x03
    cpfslt  eCTRL_CMD_CNT           ; W > Number_of_Cyclic commands?
    clrf    eCTRL_CMD_CNT           ; Reset counter
    incf    eCTRL_CMD_CNT,F         ; 1,2,3
    movff   eCTRL_CMD_CNT, eCTRL_CMD_TMP

	dcfsnz	eCTRL_CMD_TMP,F
    bra     send_message_SP         ; 1
	dcfsnz	eCTRL_CMD_TMP,F
    bra     send_message_P          ; 2
	dcfsnz	eCTRL_CMD_TMP,F
    bra     send_message_HUD        ; 3
;Prepare CMD & DATA to send
send_message_NOP:
;    movlw   0x00
    clrf    WREG
    movwf   eCTRL_DATA              ; Load DATA
    movlw   eCTRL_CMD_NOP
    movwf   eCTRL_CMD               ; Load CMD
    bra     send_message_next_5
send_message_SLEEP:
;    movlw   0x00
    clrf    WREG
    movwf   eCTRL_DATA              ; Load DATA
    movlw   eCTRL_CMD_SLEEP
    movwf   eCTRL_CMD               ; Load CMD
    bra     send_message_next_5
send_message_BT_EN:
    movlw   eCTRL_BT_EN_TIMEOUT
    movwf   eCTRL_DATA              ; Load DATA
    movlw   eCTRL_CMD_BT_EN
    movwf   eCTRL_CMD               ; Load CMD
    bra     send_message_next_5
send_message_P:
    movff   eCTRL_P,eCTRL_DATA      ; Load DATA
    movlw   eCTRL_CMD_P_SET
    movwf   eCTRL_CMD               ; Load CMD
    bra     send_message_next_5
send_message_SP:
    movff   eCTRL_SP_DC,eCTRL_DATA  ; Load DATA
    ; send 0 if Manually controlled SENSOR OFF
    movff   flag_s2, WREG
    btfsc   CTRL_sol_off_mode_W
    clrf    eCTRL_DATA              ; send data = 0 for make solenoid OFF
    movlw   eCTRL_CMD_ACT
    movwf   eCTRL_CMD               ; Load CMD
    bra     send_message_next_5
send_message_CAL:
    movff   eCTRL_CAL_ppO2,eCTRL_DATA   ; Load DATA
    movlw   eCTRL_CMD_CAL
    movwf   eCTRL_CMD               ; Load CMD
    bra     send_message_next_5
send_message_HUD:
    movff   eCTRL_HUD,eCTRL_DATA    ; Load DATA
    movlw   eCTRL_CMD_HUD
    movwf   eCTRL_CMD               ; Load CMD

send_message_next_5:
    movff   flag2, eCTRL_CMD_TMP
    btfsc   eCTRL_CMD_TMP,1         ; divemode copy from flag2
    iorlw   0x80    ;Divemode flag set

    ; Byte 0 (Symbol 1-2)
    addwf   eCTRL_out_buff_sum,F    ; Add tranceiving value to Control Summ register
    rcall   conv2ascii
    movff   eCTRL_TMP_Hi, eCTRL_out_buff+0x01
    movwf   eCTRL_out_buff+0x02
    ; Byte 1 (Symbol 3-4)
;    movlw   0x00                    ; Command Lo
    clrf    WREG
    addwf   eCTRL_out_buff_sum,F    ; Add tranceiving value to Control Summ register
    rcall   conv2ascii
    movff   eCTRL_TMP_Hi, eCTRL_out_buff+0x03
    movwf   eCTRL_out_buff+0x04
    ; Select appropriated DATA
    movff   eCTRL_DATA, WREG        ; Load DATA
    ; Byte 2 (Symbol 5-6)
    addwf   eCTRL_out_buff_sum,F    ; Add tranceiving value to Control Summ register
    rcall   conv2ascii
    movff   eCTRL_TMP_Hi, eCTRL_out_buff+0x05
    movwf   eCTRL_out_buff+0x06
    ; Byte 3 (Symbol 7-8)
    movf    eCTRL_DEPTH,W           ; Depth !!!!!!!!!!!!!!!!!!!!!
    tstfsz  eCTRL_ALG
    clrf    WREG                    ; Send Depth = 0 when disabled
    addwf   eCTRL_out_buff_sum,F    ; Add tranceiving value to Control Summ register
    rcall   conv2ascii
    movff   eCTRL_TMP_Hi, eCTRL_out_buff+0x07
    movwf   eCTRL_out_buff+0x08
    ; Byte 4 (Symbol 9-10)
    comf    eCTRL_out_buff_sum, F   ; Control Sum register Two's compliment
    incf    eCTRL_out_buff_sum, W   ;
    rcall   conv2ascii
    movff   eCTRL_TMP_Hi, eCTRL_out_buff+0x09
    movwf   eCTRL_out_buff+0x0A
    ; Symbol 11
    movlw   0x0D                    ; End of message <CR>
    movwf   eCTRL_out_buff+0x0B

    ; Initialize Tx with message len
    clrf    eCTRL_out_buff_cnt      ; Reset symbol index
    movlw   d'11'                   ; Number of symbol to send
   	movwf	eCTRL_out_mess_len      ; Set message len

    movlb   0x01                    ; Restore original Data Page

	call	rs232_wait_tx           ; Wait in case previous transfer still in progress.
;    movlw	0x00                    ; send "Wake UP" symbol
    clrf    WREG                    ; send "Wake UP" symbol
    movwf	TXREG
	bsf		PIE1,TXIE               ; enable TX interrupt for RS232
send_message_next_end:
	return

conv2ascii:
    ; Input is byte in WREG
    movwf    eCTRL_TMP              ; store the byte
    ; Convert the high nybble of the next byte to an ASCII character.
    swapf    WREG, W                ; process the high nybble first
    rcall    hex2char
    movwf    eCTRL_TMP_Hi           ; store it in TMP Hi
    ; Convert the low nybble of the same byte.
    movf     eCTRL_TMP, W           ; re-read the byte
    rcall    hex2char
	return

hex2char:
   andlw    0x0f                    ; clamp the value to one nybble
   addlw    0xf6                    ; shift a "letter" nybble down to 0
   btfss    STATUS, N               ; was result negative?
   addlw    0x7                     ; no, convert to character, less common constant
   addlw    0x3a                    ; yes, add constant to adjust
   return

#ENDIF