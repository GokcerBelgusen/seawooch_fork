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


; routines for AD converter, Realtime clock initialisation
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 10/30/05
; last updated: 05/15/08
; known bugs:
; ToDo: 

get_battery_voltage:			; starts ADC and waits until fnished
  ; In MPLAB Sim mode (hardware emulation), use a DMCI slider to
  ; directly set a 16 bit value in the range 0..1023
  ; In normal mode, jut wait for the value to be ready:

  ifndef TESTING
	bsf		ADCON0,0			; power on ADC
	nop
	bsf		ADCON0,1			; start ADC
	
get_battery_voltage2:
	btfsc	ADCON0,1			; Wait...
	bra		get_battery_voltage2
  endif

; 3.3V/1024=3,2227mV Input/Bit=9,6680mV Battery/Bit. 
; Example: 434*9,6680mV=4195,9mV Battery. 

	movff	ADRESH,xA+1
	movff	ADRESL,xA+0
	movlw	LOW		d'966'				
	movwf	xB+0
	movlw	HIGH	d'966'				
	movwf	xB+1
	call	mult16x16			; AD_Result*966
	movlw	d'100'
	movwf	xB+0
	clrf	xB+1
	call	div32x16		  ;xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder
	movff	xC+0,batt_voltage+0	; store value
	movff	xC+1,batt_voltage+1
	bcf		ADCON0,0			; power off ADC

	; Check if we should enter deep-sleep mode
	
	movff	batt_voltage+0,sub_b+0	
	movff	batt_voltage+1,sub_b+1
	movlw	LOW		d'2600'			; must be greater then 2600mV...
	movwf	sub_a+0
	movlw	HIGH	d'2600'
	movwf	sub_a+1
	call	subU16					;  sub_c = sub_a - sub_b
	bcf		enter_error_sleep		; Clear flag
	btfsc	neg_flag				; neg_flag=1 if eeprom40:41 < 2000
	bra		get_battery_voltage3	; Battery in OK range
	
	movlw	d'2'
	movwf	fatal_error_code		; Battery very low!
	bsf		enter_error_sleep		; enter error routine

get_battery_voltage3:
    SAFE_2BYTE_COPY amb_pressure, sub_b

#IFDEF SEAWOOCH
	movlw	LOW		d'25500'			; must be lower then 25500mbar
	movwf	sub_a+0
	movlw	HIGH	d'25500'
	movwf	sub_a+1
#ELSE
	movlw	LOW		d'15001'			; must be lower then 15001mbar
	movwf	sub_a+0
	movlw	HIGH	d'15001'
	movwf	sub_a+1
#ENDIF
	call	subU16					;  sub_c = sub_a - sub_b
	bcf		enter_error_sleep		; Clear flag
	btfss	neg_flag				; 
	bra		get_battery_voltage4	; Pressure in OK range
	
	movlw	d'3'
	movwf	fatal_error_code		; too deep
	bsf		enter_error_sleep		; enter error routine
	; Continue with rest of routine

get_battery_voltage4:
	; check if the battery control memory needs to be initialised!
	bcf		initialize_battery1		; clear check-flag
	clrf    EEADRH
	read_int_eeprom d'40'			; get lowest battery voltage seen in mV
	movff	EEDATA,sub_b+0
	read_int_eeprom d'41'
	movff	EEDATA,sub_b+1
	
	movlw	LOW		d'2000'			; must be greater then 2000mV...
	movwf	sub_a+0
	movlw	HIGH	d'2000'
	movwf	sub_a+1
	call	subU16					; sub_c = sub_a - sub_b
	btfss	neg_flag				; neg_flag=1 if eeprom40:41 < 2000
	bsf		initialize_battery1		; battery need to be initialised

	movlw	LOW		d'4500'			; must be lower then 4500mV...
	movwf	sub_a+0
	movlw	HIGH	d'4500'
	movwf	sub_a+1
	call	subU16					; sub_c = sub_a - sub_b
	btfsc	neg_flag				; neg_flag=1 if eeprom40:41 < 4500
	bsf		initialize_battery1		; battery need to be initialised
	
	btfss	initialize_battery1		; battery need to be initialised?
	bra		get_battery_no_init		; No, we have already valid values, just check for new extremas

get_battery_voltage_reset:
	; Init EEPROM for battery control
	; Reset lowest battery seen
	movlw	LOW			d'4200'		; reset to 4.2V
	movwf	EEDATA
	write_int_eeprom	d'40'
	movlw	HIGH		d'4200'		; reset to 4.2V
	movwf	EEDATA
	write_int_eeprom	d'41'
	movff	month,EEDATA
	write_int_eeprom	d'42'
	movff	day,EEDATA
	write_int_eeprom	d'43'
	movff	year,EEDATA
	write_int_eeprom	d'44'
    SAFE_2BYTE_COPY temperature,lo
	movff	lo,EEDATA
	write_int_eeprom	d'45'
	movff	hi,EEDATA
	write_int_eeprom	d'46'
	; Reset charge statistics
	clrf	EEDATA
	write_int_eeprom	d'47'		; last complete charge
	write_int_eeprom	d'48'		; last complete charge
	write_int_eeprom	d'49'		; last complete charge
	write_int_eeprom	d'50'		; total cycles
	write_int_eeprom	d'51'		; total cycles
	write_int_eeprom	d'52'		; total complete cycles
	write_int_eeprom	d'53'		; total complete cycles
	; Reset temperature extremas
    SAFE_2BYTE_COPY temperature,lo
	movff	lo,EEDATA	; Reset mimimum extrema
	write_int_eeprom	d'54'
	movff	hi,EEDATA
	write_int_eeprom	d'55'
	movff	month,EEDATA
	write_int_eeprom	d'56'
	movff	day,EEDATA
	write_int_eeprom	d'57'
	movff	year,EEDATA
	write_int_eeprom	d'58'
	movff	lo,EEDATA	; Reset maximum extrema
	write_int_eeprom	d'59'
	movff   hi,EEDATA
	write_int_eeprom	d'60'
	movff	month,EEDATA
	write_int_eeprom	d'61'
	movff	day,EEDATA
	write_int_eeprom	d'62'
	movff	year,EEDATA
	write_int_eeprom	d'63'
get_battery_no_init:	
	read_int_eeprom d'40'			; get lowest battery voltage seen in mV
	movff	EEDATA,sub_b+0
	read_int_eeprom d'41'
	movff	EEDATA,sub_b+1
	movff	batt_voltage+0,sub_a+0
	movff	batt_voltage+1,sub_a+1
	call	subU16					; sub_c = sub_a - sub_b
	btfss	neg_flag				; new lowest battery voltage?
	return							; no, quit routine
	; Yes, store new value together with the date and temperature values
	movff	batt_voltage+0,EEDATA
	write_int_eeprom	d'40'
	movff	batt_voltage+1,EEDATA
	write_int_eeprom	d'41'
	movff	month,EEDATA
	write_int_eeprom	d'42'
	movff	day,EEDATA
	write_int_eeprom	d'43'
	movff	year,EEDATA
	write_int_eeprom	d'44'
    SAFE_2BYTE_COPY temperature,lo
	movff	lo,EEDATA
	write_int_eeprom	d'45'
	movff	hi,EEDATA
	write_int_eeprom	d'46'
	return

RTCinit:						; resets RTC 
	movlw	0x80
	movwf	TMR1H
	nop							; See errata DS80284E-page 2
	clrf	TMR1L
; Reset RTC if any part of the time/date is out of range
	movlw	d'60'				; Limit
	cpfslt	secs				; Check part
	bra		RTCinit2			; Reset time...
	movlw	d'60'				; Limit
	cpfslt	mins				; Check part
	bra		RTCinit2			; Reset time...
	movlw	d'24'				; Limit
	cpfslt	hours				; Check part
	bra		RTCinit2			; Reset time...
	movlw	d'32'				; Limit
	cpfslt	day					; Check part
	bra		RTCinit2			; Reset time...
	movlw	d'12'				; Limit
	cpfslt	month				; Check part
	bra		RTCinit2			; Reset time...
	movlw	d'100'				; Limit
	cpfslt	year				; Check part
	bra		RTCinit2			; Reset time...
; Make sure day and month <> zero
	movlw	.0
	cpfsgt	day					; =0?
	incf	day,F				; Yes, +1
	cpfsgt	month				; =0?
	incf	month,F				; Yes, +1
	bsf		PIE1, TMR1IE		; Enable clock int
	return

RTCinit2:
	movlw	.00
	movwf	secs
	movlw	.00
	movwf	mins
	movlw	.12
	movwf	hours
	movlw	.1
	movwf	day
	movlw	.5
	movwf	month
	movlw	.13
	movwf	year
	bsf		PIE1, TMR1IE		; Enable clock int
	return

reset_battery_stats:
	bcf		uart_reset_battery_stats	; Clear flag
    clrf    EEADRH
	bcf		PIE1,RCIE					; no interrupt for UART
	call	rs232_get_byte				; Get Byte
	bcf		PIR1,RCIF					; clear flag

	btfsc	rs232_recieve_overflow		; Byte received?
	bra		reset_battery_stats_exit	; No, exit
	movlw	'f'	
	cpfseq	RCREG						; Really reset statistics?
	bra		reset_battery_stats_exit	; No, exit
	call	rs232_get_byte					; Get byte
	bcf		PIR1,RCIF					; clear flag

	btfsc	rs232_recieve_overflow		; Byte received?
	bra		reset_battery_stats_exit	; No, exit
	movlw	'f'
	cpfseq	RCREG						; Really reset statistics?
	bra		reset_battery_stats_exit	; No, exit
; Yes, Reset now.
	rcall	get_battery_voltage_reset	; Reset Statistics
	movlw	'f'
	movwf	TXREG
	call	rs232_wait_tx				; Wait for uart

reset_battery_stats_exit:
	bsf		PIE1,RCIE					; re-enable interrupt for UART
	goto	surfloop_loop				; return to surface loop