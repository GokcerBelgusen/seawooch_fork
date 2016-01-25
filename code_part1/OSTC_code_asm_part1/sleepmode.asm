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
; Routines for sleepmode
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 050520
; last updated: 090507
; known bugs:
; ToDo: 
sleeploop:							; enter sleepmode!
; first check if the 16 hash char are=0
	lfsr	FSR2, char_O_hash
	movlw	d'16'
	movwf	temp1
sleeploop1:
	tstfsz	POSTINC2		; Test hash values
	bra		sleeploop2		; At least one char is not zero -> Do not build hash
	decfsz	temp1,F
	bra		sleeploop1
 	; build hash  (about 90sek @ 16MHz)
	call	DISP_ClearScreen		; clear
	DISPLAYTEXT	.1					; "Building MD2 hash"
	DISPLAYTEXT	.2					; "Please wait..."
	call	deco_hash			    ; calculate MD2 hash
	movlb	b'00000001'				; Back to Bank1
sleeploop2:
	call	DISP_DisplayOff			; display off
#IFDEF CCR_CTRL
   	btfss	FLAG_const_ppO2_mode    ; Don't disable UART if compilled for eCTRL
#ENDIF
	call	disable_rs232			; disable UART module

; Save surface mode custom view
	movff	menupos3,EEDATA			; Copy to EEDATA
	write_int_eeprom	d'93'		; Write last selected customview surface mode into EEPROM

	clrf	divemins+0
	clrf	divemins+1
	bcf		TRISB,6
	bcf		TRISB,7
	bcf		PORTB,6
	bcf		PORTB,7					; Disable UART
sleeploop_loop:
	btfsc	oneminupdate			; one minute in sleep?
	rcall	onemin_sleep			; do oneminute tasks, e.g. calculate desaturation

	btfsc	onesecupdate			; one second in sleep?
	rcall	onesec_sleep			; check switches, check pressure sensor, etc.

	btfss	sleepmode				; wake up? (This bit will be set in other routines)
	goto	restart					; yes
	nop
	sleep							; Sleep until Timer1 will wake up the device
	nop	
	bra		sleeploop_loop			; do loop until someting happens


onemin_sleep:
	call	get_battery_voltage		; get battery voltage
	btfsc	enter_error_sleep		; Enter Fatal Error Routine?
	goto	fatal_error_sleep		; Yes (In Sleepmode_vxx.asm!)
	
    ;---- adjust airpressure compensation any 15 minutes
	incf	divemins+1,F			; counts to 14...
	movlw	d'14'
	cpfsgt	divemins+1
	bra		onemin_sleep2			; 15 minutes not done!
	clrf	divemins+1				; reset counter

	rcall	pressuretest_sleep_fast	; Gets pressure without averaging (faster!)

    SAFE_2BYTE_COPY amb_pressure_avg, amb_pressure	; copy for compatibility

	call	check_temp_extrema		; Check for temperature extremas

	call	deco_calc_CNS_decrease_15min		; compute CNS decay in sleep only
	movlb	b'00000001'

	movff	last_surfpressure_15min+0,last_surfpressure_30min+0	; save older airpressure
	movff	last_surfpressure_15min+1,last_surfpressure_30min+1	; save older airpressure	
    SAFE_2BYTE_COPY amb_pressure, last_surfpressure_15min		; save new airpressure

	GETCUSTOM15	d'7'				; loads max_sufpressure into lo, hi
	movff	lo,sub_a+0				; max. "allowed" airpressure in mbar
	movff	hi,sub_a+1				
	movff	last_surfpressure_15min+0,sub_b+0
	movff	last_surfpressure_15min+1,sub_b+1
	call	sub16					; sub_c = sub_a - sub_b
	btfss	neg_flag                ; Is 1080mbar < amb_pressure ?
	bra		onemin_sleep2			; NO: current airpressure is lower then "allowed" airpressure, ok!

    ; not ok! Overwrite with max. "allowed" airpressure
	GETCUSTOM15	d'7'				; loads max_sufpressure into lo, hi
	movff	lo,last_surfpressure_15min+0	; max. "allowed" airpressure in mbar
	movff	hi,last_surfpressure_15min+1	; max. "allowed" airpressure in mbar

onemin_sleep2:
    SAFE_2BYTE_COPY amb_pressure, int_I_pres_respiration ; LOW copy pressure to deco routine
	GETCUSTOM8	d'11'				; Saturation multiplier %
	movff	WREG,char_I_saturation_multiplier
	GETCUSTOM8	d'12'				; Desaturation multiplier %
	movff	WREG,char_I_desaturation_multiplier
	call	deco_calc_wo_deco_step_1_min	; "calc_tissue_sleep"
	movlb	b'00000001'									; RAM Bank1 selected

	bcf		oneminupdate			; all done
	return

onesec_sleep:
	call	test_charger			; charger on?
	
	btfss	nofly_active
	bra		onesec_sleep_nonofly
	
	bsf		LED_blue				; Set nofly LED
	
	nop
	sleep
	nop

onesec_sleep_nonofly:
	bcf		LED_blue				; Clear	nofly LED
	incf	divemins+0,F 			; counts to #test_pressure_in_sleep (5)
	movlw	d'5'
	cpfsgt	divemins+0				; here: temp variable
	bra		onesec_sleep1			; #test_pressure_in_sleep not done yet
	clrf	divemins+0

	rcall	pressuretest_sleep_fast	; Gets pressure without averaging (faster!)
    SAFE_2BYTE_COPY amb_pressure_avg, amb_pressure	; copy for compatibility

        ; compare current ambient pressure with threshold CF6==1160mbar.
	GETCUSTOM15	d'6'				; loads pressure threshold into lo,hi
	movff	lo,sub_a+0				; power on if ambient pressure is greater threshold
	movff	hi,sub_a+1	
        SAFE_2BYTE_COPY amb_pressure_avg, sub_b
	call	sub16					; Is (1160mbar - averaged(amb_pressure)) < 0 ?
	bsf		sleepmode
	btfsc	neg_flag				; Wake up from Sleep?
	bcf		sleepmode				; amb_pressure>pressure_offset_divemode: wake up!

onesec_sleep1:
	bcf		onesecupdate			; all done.
	btfsc	switch_left
	bra		onesec_sleep1a
	btfsc	switch_right
	bra		onesec_sleep1a
#IFDEF CCR_CTRL
onesec_sleep1a_no:
#ENDIF
; No button pressed
	bcf		INTCON,INT0IF				; Clear flag
	bcf		INTCON3,INT1IF				; Clear flag
	bcf		switch_right
	bcf		switch_left
	bcf		T0CON,TMR0ON				; Stop Timer 0
	return
onesec_sleep1a:	; At least one button pressed....
#IFDEF CCR_CTRL
    ; Wake up only by two keys pressed
	movf	PORTB,W						; move portb into latch register
    andlw   b'00000011'                 ; select only buttons
    bnz     onesec_sleep1a_no
#ENDIF
	bcf		INTCON,INT0IF				; Clear flag
	bcf		INTCON3,INT1IF				; Clear flag
	bcf		switch_right
	bcf		switch_left
	bcf		T0CON,TMR0ON				; Stop Timer 0
	bcf		sleepmode                   ; wake up!

    ; Restart altimeter averaging, so next averaging starts right over...
	call    altimeter_restart

	return
	
pressuretest_sleep_fast:				; Get pressure without averaging (Faster to save some power in sleep mode)
	call		get_temperature_start		; and start temperature integration (73,5us)
	sleep
	nop
	sleep
	nop
	sleep								; Wait at least 35ms (every 16.5ms Timer1 wakeup)
	call		get_temperature_value		; State 1: Get temperature	
	call		get_pressure_start	 	; Start pressure integration.
	sleep
	nop
	sleep
	nop
	sleep								; Wait at least 35ms (every 16.5ms Timer1 wakeup)
	call		get_pressure_value		; State2: Get pressure (51us)
	clrf		temperature_avg+0
	clrf		temperature_avg+1
	clrf		amb_pressure_avg+0
	clrf		amb_pressure_avg+1			; clear for sleep routine
	call		calculate_compensation		; calculate temperature compensated pressure (233us)
	return

fatal_error_sleep:
	call	DISP_DisplayOff			; display off
	call	disable_rs232			; disable UART module

	bcf		TRISB,6
	bcf		TRISB,7
	bcf		PORTB,6
	bcf		PORTB,7					; Disable UART
fatal_error_sleep_loop:
	movff	fatal_error_code,temp4
	movlw	d'15'
	movwf	temp1
fatal_error_sleep_loop1:
	sleep
	nop
	decfsz	temp1,F
	bra		fatal_error_sleep_loop1
fatal_error_sleep_loop2:
	call	get_battery_voltage			; get battery voltage
	btfss	enter_error_sleep			; REALLY enter Fatal Error Routine?
	goto    restart                     ; No

	btfss	CHRG_IN						; If CHRG_IN=0 -> CC active
    goto    restart                     ; wake up

	bsf		LED_red
	clrwdt
	WAIT10US	d'5'
	bcf		LED_red
	sleep
	nop
	decfsz	temp4,F
	bra		fatal_error_sleep_loop2
	bra		fatal_error_sleep_loop