
; OSTC - diving computer code
; Copyright (C) 2008 HeinrichsWeikamp GbR

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
; updated by BANDiver, SEAWOOCH, 2014
; known bugs:
; ToDo:

menu_ppO2_setup:
    btfss   ext_ppO2_enable         ; Check for eCTRL mode
	bra     menu_ppO2_setup_disabled					; exit...

	movlw	d'1'                    ;Set cursor to string #3
	movwf	menupos

menu_ppO2_setup_return:
menu_ppO2_setup2:

	call	DISP_ClearScreen
	call	DISP_standard_color
menu_ppO2_setup3:
	call	DISP_ppo2_setup_mask
	call	refresh_cursor
    call    menu_pre_loop_common

menu_ppO2_setup_loop:
	call	check_switches_menu

	movlw	d'3'    ; empty menu item - jump to Exit
	cpfseq	menupos
	bra		menu_ppO2_setup_loop2
	movlw	d'6'
    movwf   menupos
    call    DISP_menu_cursor

menu_ppO2_setup_loop2:
	btfss	menubit
	goto	restart						; exit menu, restart

	btfss	onesecupdate                ; Check for 1 sec. tasks
	bra		menu_ppO2_setup_loop3

	call	menu_check_dive_and_timeout	; "Goto restart" or sets sleepmode flag

	bcf		onesecupdate				; End of one second tasks
	btfsc	sleepmode
	goto	restart

menu_ppO2_setup_loop3:
	btfsc	menubit2
	goto	menu_ppO2_setup_modules		; call submenu

	bra		menu_ppO2_setup_loop

menu_ppO2_setup_modules:						; calls submenu
	dcfsnz	menupos,F
	bra		menu_ppO2_setup_eCTRL_BT         ;Start BT enable timer
	dcfsnz	menupos,F
	bra		menu_ppO2_setup_reset_scr_time      ;Reset Scrubber life time
	dcfsnz	menupos,F
	bra		menu_ppO2_setup_nothing
	dcfsnz	menupos,F
	bra		menu_ppO2_setup_nothing
	dcfsnz	menupos,F
	bra		menu_ppO2_setup_nothing
menu_ppO2_setup_disabled:
	movlw	d'1'
	movwf	menupos
	goto	menu_const_ppO2_return					; exit...

menu_ppO2_setup_eCTRL_BT:
    movff   eCTRL_BT_timer, WREG    ;Load current timer value
    tstfsz  WREG
    bra     menu_ppO2_setup_eCTRL_BT_dis
    ; Timer is 0 - start it
    movlw   eCTRL_CMD_BT_EN
    movff   WREG, eCTRL_CMD               ; Send BT Enable requestn with next message
    movlw   eCTRL_BT_EN_TIMEOUT
menu_ppO2_setup_eCTRL_BT_exit:
    movff   WREG, eCTRL_BT_timer        ;Load Timer.
    goto	restart
menu_ppO2_setup_eCTRL_BT_dis:
    ; Timer is not 0 - stop it
    clrf    WREG
    bra     menu_ppO2_setup_eCTRL_BT_exit

menu_ppO2_setup_reset_scr_time:
	call	DISP_confirmbox				; Returns WREG=0 for Cancel (Or Timeout) and WREG=1 for OK!
	movwf	menupos						; Used as temp
	tstfsz	menupos
    bra     menu_ppO2_setup_res_scr_time_do
	bra		menu_ppO2_setup_res_scr_t_exit			; Cancel!
menu_ppO2_setup_res_scr_time_do:
	clrf    EEADRH                      ; Make sure to select eeprom bank 0
    clrf    EEDATA
	write_int_eeprom	d'120'; Reset Scruber life Counter Lo
	write_int_eeprom	d'121'; Reset Scruber life Counter Hi
menu_ppO2_setup_res_scr_t_exit:
	movlw	d'1'
	movwf	menupos
	bra		menu_ppO2_setup_return	; Return to ppO2 Module menu

menu_ppO2_setup_nothing:
	movlw	d'1'
	movwf	menupos
	bra		menu_ppO2_setup_return	; Return to ppO2 Module menu


