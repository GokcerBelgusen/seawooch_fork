
; OSTC - diving computer code
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


; menu "ppO2 Sensors"
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 080902
; last updated: 080902
; updated by BANDiver, SEAWOOCH, 2015
; known bugs:
; ToDo: 

menu_ppO2_sensor:
    btfss   ext_ppO2_enable         ; Check for eCTRL mode
	bra menu_ppO2_sensor_disabled					; exit...

	movlw	d'3'                    ;Set cursor to string #3
	movwf	menupos
	call	enable_rs232_115k_ISR		; enables UART but with TX Pin set to 1 as IO for higher current operation

menu_ppO2_sensor_return:
menu_ppO2_sensor2:
	call	DISP_ClearScreen
	call	DISP_standard_color
    ; Check if we are going to Turn OFF
    movff   eCTRL_F, WREG
    btfss   eCTRL_F_sleep_req_W
    bra     menu_ppO2_sensor3
    DISPLAYTEXT	.002                    ; "Please Wait..."
    bra     menu_ppO2_sensor4

menu_ppO2_sensor3:
	call	DISP_ppo2_sensor_mask
	call	refresh_cursor
menu_ppO2_sensor4:
    call    menu_pre_loop_common

menu_ppO2_sensor_loop:
	call	check_switches_menu

	movlw	d'3'        ;items 1 - ppO2, and 2 - mV, nothing to select - jump to item 3
	cpfslt	menupos
	bra     menu_ppO2_sensor_loop2
	movlw	d'3'
	movwf	menupos
	call	DISP_menu_cursor

menu_ppO2_sensor_loop2:
	btfss	menubit
	goto	restart						; exit menu, restart

	btfss	onesecupdate                ; Check for 1 sec. tasks
	bra		menu_ppO2_sensor_loop3

	call	menu_check_dive_and_timeout	; "Goto restart" or sets sleepmode flag

    ; Check if we are going to Turn OFF
    movff   eCTRL_F, WREG
    btfsc   eCTRL_F_sleep_req_W
	bra		menu_ppO2_sensor_loop2a

    rcall   menu_o2_sensor_show_ppO2
    rcall   menu_o2_sensor_show_mV

menu_ppO2_sensor_loop2a:
	bcf		onesecupdate				; End of one second tasks
	btfsc	sleepmode
	goto	restart

menu_ppO2_sensor_loop3:
	btfsc	menubit2
	goto	menu_ppO2_sensor_modules		; call submenu

	bra		menu_ppO2_sensor_loop

menu_ppO2_sensor_modules:						; calls submenu
	dcfsnz	menupos,F
	bra		menu_o2_sensor_eCTRL_OFF	; Turn eCTRL OFF
	dcfsnz	menupos,F
	bra		menu_o2_sensor_eCTRL_OFF	; Turn eCTRL OFF
	dcfsnz	menupos,F
	bra		menu_o2_sensor_change_setpoint	; Manual Setpoint Change (only for Pre-dive Check)
	dcfsnz	menupos,F
	bra		menu_o2_sensor_calibrate_gas; Calibrate with %O2 of the "Startgas"
	dcfsnz	menupos,F
	bra		menu_o2_sensor_eCTRL_OFF	; Turn eCTRL OFF
menu_ppO2_sensor_disabled:
	movlw	d'1'
	movwf	menupos
	goto	menu_const_ppO2_return					; exit...

menu_o2_sensor_calibrate_gas:
    GETCUSTOM8  eCTRL_CF_CALO2             ; persentage of the O2 for calibration  eCTRL_CF
    movff   WREG, uart1_temp

    GETCUSTOM8  eCTRL_CF_CALAMB             ; Absolute Calibration?  eCTRL_CF
    movff   WREG,lo
    clrf    WREG
    cpfsgt  lo
    bra     menu_o2_sensor_calibrate_gas_a

    movff   uart1_temp,xA+0 ; Get O2 percentage for Calibrate with
	clrf	xA+1
	movff	amb_pressure+0,xB+0
	movff	amb_pressure+1,xB+1
	call 	mult16x16			;xA*xB=xC
	movlw	LOW		d'1000'
	movwf	xB+0
	movlw	HIGH	d'1000'
	movwf	xB+1
	call	div32x16  ; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder
	movff	xC+0,uart1_temp		; Result for UART send

menu_o2_sensor_calibrate_gas_a:
    movff   uart1_temp, eCTRL_CAL_ppO2    ; Save ppO2 of the Gas used during calibration
    movlw   eCTRL_CMD_CAL
    movff   WREG, eCTRL_CMD               ; Send Calibration request with next message

	movlw	d'4'
	movwf	menupos
	bra		menu_ppO2_sensor_return	; Return to ppO2 Module menu

menu_o2_sensor_eCTRL_OFF:
    movlw   eCTRL_CMD_SLEEP
    movff   WREG, eCTRL_CMD               ; Send OFF requestn with next message
    movlb   eCTRL_PAGE                      ;Select eCTRL Data Page
    bsf     eCTRL_F_sleep_req     ;Request to go to sleep after message sent
    movlb   0x01                  ;Select original page
;TODO: Inform about going to sleep 
	movlw	d'6'
	movwf	menupos
	bra		menu_ppO2_sensor_return	; Return to ppO2 Module menu

menu_o2_sensor_change_setpoint:
	movff   eCTRL_SP_manual, lo
    incf	lo              ;
	movlw	d'3'						; check overflow
    cpfslt  lo
    clrf    lo
	movff   lo, eCTRL_SP_manual
    call    DISP_SP_manual
	movff	lo, char_I_const_ppO2	; Use SetPoint
	movff	lo, ppO2_setpoint_store	; Store also in this byte...
	movff	lo, eCTRL_SP_DC           ; Store also in this byte...
    movlw   eCTRL_CMD_ACT
    movff   WREG, eCTRL_CMD               ; Send OFF requestn with next message

	bsf		setpoint_changed
	bsf		event_occured				; set global event flag

	movlw	d'3'
	movwf	menupos
	bra		menu_ppO2_sensor3		; Return to ppO2 Module menu	
	
menu_o2_sensor_show_ppO2:
	WIN_FONT	FT_SMALL
	WIN_LEFT	.20
	WIN_TOP     .35
	bsf		leftbind

    ;Copy Sensor Status to tmp for color processing
    movff   eCTRL_StatLo,eCTRL_color_tmp

	lfsr	FSR2,letter
	movff	eCTRL_ppo2_s1,lo		; copy measured value
	call	DISP_ppO2_sensor_dive

	WIN_LEFT	.55
	lfsr	FSR2,letter
	movff	eCTRL_ppo2_s2,lo		; copy measured value
	call	DISP_ppO2_sensor_dive

	WIN_LEFT	.90
	lfsr	FSR2,letter
	movff	eCTRL_ppo2_s3,lo		; copy measured value
	call	DISP_ppO2_sensor_dive

	WIN_LEFT	.125
	lfsr	FSR2,letter
	movff	eCTRL_ppo2_av,lo		; copy current Averaged of all sensors
	call    DISP_ppO2_sensor_dive_no_col

    return

menu_o2_sensor_show_mV:
	WIN_TOP		.65
	WIN_LEFT	.13
    bcf     leftbind
	lfsr	FSR2,letter

    btfsc	ext_ppO2_valid
    bra     menu_o2_sensor_show_mV_do  ;show ppO2 & mV

	movlw	d'21'
	movwf	temp1
	call	DISP_display_clear_common_y1
	return

menu_o2_sensor_show_mV_do:
    clrf    hi
    movff	eCTRL_mv_s1,lo              ; copy measured value
	rcall   show_mv_value

    movff	eCTRL_mv_s2,lo              ; copy measured value
	rcall   show_mv_value

    movff	eCTRL_mv_s3,lo              ; copy measured value
	rcall   show_mv_value

	call	word_processor

; Show eCTRL Ext Battery value
	WIN_TOP		.65
	WIN_LEFT	.122

	bcf		leftbind

	lfsr	FSR2,letter
    rcall   show_bat_value
	return

show_mv_value:
	output_8 				; Show value
	STRCAT  "mV"
    return

show_bat_value:
	call    DISP_standard_color
    btfsc	ext_ppO2_valid
    bra     show_bat_value_do       ;show BAT voltage

	STRCAT  "...."
    bra     show_bat_value_show
show_bat_value_do:
    movff	eCTRL_battery,lo        ; copy battery voltage
    movlw   eCTRL_BAT_LOW           ; Battery threshold value
    cpfsgt  lo
    call    DISP_warnings_color     ; BAT is LOW - show in RED.

    clrf    hi
    movlw   d'200'
    addwf   lo,f
    clrf    WREG
    addwfc  hi,f
	bsf		leftbind
	output_16dp	3
	STRCAT  "V"
show_bat_value_show:
	call	word_processor
	bcf		leftbind
	call    DISP_standard_color
    return

eCTRL_mode_update:
    bsf     ext_ppO2_enable
    GETCUSTOM8  eCTRL_CF_ENABLE             ; eCTRL enabled eCTRL_CF
	movwf	lo
	movlw	d'1'
	cpfseq	lo					; =1?
    bcf     ext_ppO2_enable
    return
