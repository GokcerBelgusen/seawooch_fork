
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


; Constant ppO2 Setup menu
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 08/04/04
; last updated: 120920
; known bugs:
; ToDo: 
;
; New in 2.52 Diluents stored seperately in EEPROM Bank0
;Byte96-97:
;Diluent 1 Default (%O2,%He)
;Byte98-99:
;Diluent 1 Current (%O2,%He)
;Byte100-101:
;Diluent 2 Default (%O2,%He)
;Byte102-103:
;Diluent 2 Current (%O2,%He)
;Byte104-105:
;Diluent 3 Default (%O2,%He)
;Byte106-107:
;Diluent 3 Current (%O2,%He)
;Byte108-109:
;Diluent 4 Default (%O2,%He)
;Byte110-111:
;Diluent 4 Current (%O2,%He)
;Byte112-113:
;Diluent 5 Default (%O2,%He)
;Byte114-115:
;Diluent 5 Current (%O2,%He)
;Byte116:
;First Diluent (1-5)

; EEPROM BANK0 Byte96-97:
; Diluent 1 (%O2,%He)
; EEPROM BANK0 Byte98-99:
; Diluent 2 (%O2,%He)
; EEPROM BANK0 Byte100-101:
; Diluent 3 (%O2,%He)
; EEPROM BANK0 Byte102-103:
; Diluent 4 (%O2,%He)
; EEPROM BANK0 Byte104-105:
; Diluent 5 (%O2,%He)
; EEPROM BANK0 Byte106:
; First Diluent (1-5)


menu_const_ppO2:
	movlw	d'1'
	movwf	menupos
menu_const_ppO2_return:
    call	DISP_ClearScreen
menu_const_ppO2_return2:
    call    DISP_ccr_setup_menu_mask
	call	refresh_cursor
    call    menu_pre_loop_common
    call    show_sp_mode

menu_const_ppO2_preloop:
	call	check_switches_menu
#IFNDEF CCR_CTRL
    ; Old Menu was not full - was need to check and skip lines
	movlw	d'4'
	cpfseq	menupos
	bra		menu_const_ppO2_preloop2	; Returns
	movlw	d'6'
    movwf   menupos
    call    DISP_menu_cursor
#ENDIF


menu_const_ppO2_preloop2:
	btfsc	menubit2
	bra		do_ccr_pre_menu             ; call submenu
	btfss	menubit
	goto	restart						; exit menu, restart
	btfsc	onesecupdate
	call	menu_check_dive_and_timeout	; "Goto restart" or sets sleepmode flag
	bcf		onesecupdate				; End of one second tasks
	btfsc	sleepmode
	goto	restart
	bra		menu_const_ppO2_preloop

do_ccr_pre_menu:
#IFDEF CCR_CTRL
    ; Reorder Menu structure for CCR
    dcfsnz	menupos,F
	goto	menu_ppO2_sensor     		; CCR Controller Control Menu
	dcfsnz	menupos,F
	goto	menu_diluentsetup
	dcfsnz	menupos,F
	goto	menu_const_ppO2_setpoints
	dcfsnz	menupos,F
	goto	menu_const_ppO2_toggle_mode
	dcfsnz	menupos,F
	goto	menu_ppO2_setup             ; CCR Controller Setup Menu
#ELSE
    dcfsnz	menupos,F
	goto	menu_diluentsetup
	dcfsnz	menupos,F
	goto	menu_const_ppO2_setpoints
	dcfsnz	menupos,F
	goto	menu_const_ppO2_toggle_mode
	dcfsnz	menupos,F
	goto	exit_menu_const_ppO2			; exit...
#ENDIF
	dcfsnz	menupos,F
	goto	exit_menu_const_ppO2			; exit...
exit_menu_const_ppO2:			; exit...
#IFDEF CCR_CTRL
    ; Reorder Menu structure for CCR
	movlw	d'1'                ; Return to different Menu
	movwf	menupos
	goto	menu2
#ELSE
	movlw	d'2'
	movwf	menupos
	goto	more_menu2
#ENDIF
menu_const_ppO2_toggle_mode:
    read_int_eeprom .116        ; SP Mode
    movlw   .1                  ; Preload 1
    tstfsz  EEDATA              ; =0?
    movlw   .0                  ; No.
    movwf   EEDATA              ; Set new value
    write_int_eeprom .116       ; write back result
#IFDEF CCR_CTRL
    ; Reorder Menu structure for CCR
	movlw	d'4'                ; Different return position
#ELSE
	movlw	d'3'
#ENDIF

	movwf	menupos
    goto    menu_const_ppO2_return2  ; Done.

show_sp_mode:
    read_int_eeprom .116        ; SP Mode
    tstfsz  EEDATA              ; =0?
	bra		show_sp_mode2       ; No
	DISPLAYTEXTH	.363		; =0: Manual
	return
show_sp_mode2:
	DISPLAYTEXTH	.364		; =1: Auto
	return


menu_diluentsetup:
	movlw	d'1'
	movwf	menupos

menu_diluentsetup_prelist:
	call	DISP_ClearScreen
	call	menu_pre_loop_common		; Clear some menu flags, timeout and switches
    call	DISP_divemask_color
	DISPLAYTEXT	.231			; Dil. Setup - Gaslist
    call	DISP_standard_color
	clrf	decodata+0				; Here: # of gas 0-4
	clrf	divemins+0				; Here: # of Gas * 4
	movlw	d'5'
	movwf	waitms_temp		; here: stores row for gas list
	movlw	d'93'
	movwf	wait_temp			; here: stores eeprom address for gas list

menu_diluentsetup_list:
	WIN_LEFT	.20
	movlw	d'4'
	addwf	wait_temp,F			; Increase eeprom address for diluent list
	movlw	d'30'
	addwf	waitms_temp,F		; Increase row
	movf	waitms_temp,W		; Load row into WREG
	movff	WREG,win_top
	lfsr	FSR2,letter
	PUTC	TXT_DIL_C
	movff	decodata+0,lo
	incf	lo,F
	bsf		leftbind
	output_99
	PUTC	':'
	call	word_processor

	WIN_LEFT	.40
	movf	waitms_temp,W		; Load row into WREG
	movff	WREG,win_top
	lfsr	FSR2,letter

	movlw	d'115'
	movwf	EEADR
	call	read_eeprom			; Get current startgas 1-5 # into EEDATA
	decf	EEDATA,W			; 0-4
	cpfseq	decodata+0			; =current displayed gas #?
	bra		menu_diluentsetup_Tx; no, do not display *
	PUTC	'*'					; display *
    call	word_processor

menu_diluentsetup_Tx:
	WIN_LEFT	.48
	movf	waitms_temp,W		; Load row into WREG
	movff	WREG,win_top
	lfsr	FSR2,letter

	movff	wait_temp, EEADR	; Gas %He - Set address in internal EEPROM
	incf	EEADR,F				; Gas %He - Set address in internal EEPROM
	call	read_eeprom			; Read He value from EEPROM
	movff	EEDATA,lo			; Move EEDATA -> lo
	movf	lo,f				; Move lo -> f
	movlw	d'0'				; Move 0 -> WREG
	cpfsgt	lo					; He > 0?
	bra 	menu_diluentsetup_Nx	; NO check o2

	; YES Write TX 15/55
   STRCAT  TXT_TX3
	movff	wait_temp, EEADR	; Gas %O2 - Set address in internal EEPROM
	call	read_eeprom			; O2 value
	movff	EEDATA,lo
	output_8
	PUTC	'/'
	incf	EEADR,F				; Gas #hi: %He - Set address in internal EEPROM
	call	read_eeprom			; He value
	movff	EEDATA,lo
	output_8
	bra 	menu_diluentsetup_list0

; New v1.44se
menu_diluentsetup_Nx:
	movff	wait_temp, EEADR	; Gas %O2 - Set address in internal EEPROM
	call	read_eeprom			; Read O2 value from EEPROM
	movff	EEDATA,lo			; Move EEDATA -> lo
	movf	lo,f				; Move lo -> f
	movlw	d'21'				; Move 21 -> WREG
	cpfsgt	lo					; o2 > 21%
	bra 	menu_diluentsetup_Air	; NO AIR
	movlw	d'100'				; Move 100 -> WREG
	cpfslt	lo					; o2 < 100%
	bra		menu_diluentsetup_O2	; NO write O2

	; YES Write NX 32
    STRCAT  TXT_NX3
	output_8
	bra 	menu_diluentsetup_list0

menu_diluentsetup_O2:
    STRCAT  TXT_O2_3
	output_8
	bra 	menu_diluentsetup_list0

menu_diluentsetup_Air:
	cpfseq	lo					; o2 = 21%
	bra     menu_diluentsetup_Err

    STRCAT  TXT_AIR4
	output_8
	bra 	menu_diluentsetup_list0

menu_diluentsetup_Err:
    STRCAT  TXT_ERR4
	output_8

menu_diluentsetup_list0:
	call	word_processor

	incf	decodata+0,F
	movlw	d'5'
	cpfseq	decodata+0
	goto	menu_diluentsetup_list

	DISPLAYTEXT	.11                     ; Exit
	call	wait_switches               ; Waits until switches are released, resets flag if button stays pressed!
	call	DISP_menu_cursor

menu_diluentsetup_loop:
	call	check_switches_logbook

	btfsc	menubit3
	bra		menu_diluentsetup_list2              ; move cursor

	btfsc	menubit2
	bra		do_diluentsetup_list        ; call gas-specific submenu

	btfsc	onesecupdate
	call	menu_check_dive_and_timeout	; "Goto restart" or sets sleepmode flag

	bcf		onesecupdate                ; 1 sec. functions done

	btfsc	sleepmode
#IFDEF CCR_CTRL
    ; Reorder Menu structure for CCR
	bra		do_diluentsetup_list_exit
#ELSE
	bra		menu_const_ppO2
#ENDIF

	bra		menu_diluentsetup_loop

menu_diluentsetup_list2:
	incf	menupos,F
	movlw	d'7'
	cpfseq	menupos			; =7?
	bra		menu_diluentsetup_list3	; No
	movlw	d'1'
	movwf	menupos

menu_diluentsetup_list3:
	clrf	timeout_counter2
	call	DISP_menu_cursor

	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!

	bcf		menubit3		; clear flag
	bra		menu_diluentsetup_loop


do_diluentsetup_list:
	dcfsnz	menupos,F
	bra		diluent_list_edit_gas1
	dcfsnz	menupos,F
	bra		diluent_list_edit_gas2
	dcfsnz	menupos,F
	bra		diluent_list_edit_gas3
	dcfsnz	menupos,F
	bra		diluent_list_edit_gas4
	dcfsnz	menupos,F
	bra		diluent_list_edit_gas5
#IFDEF CCR_CTRL
do_diluentsetup_list_exit:
    ; Reorder Menu structure for CCR
	movlw	d'2'                        ; Different return position
	movwf	menupos
	bra		menu_const_ppO2_return      ; Exit List
#ELSE
	bra		menu_const_ppO2             ; Exit List
#ENDIF

diluent_list_edit_gas1:
	movlw	d'0'                        ; Diluent Number 0-4
	movwf	decodata+0
	movlw	d'91'                       ; EEPROM address of %O2
	movwf	divemins+0
	bra		menu_diluentgas
diluent_list_edit_gas2:
	movlw	d'1'                        ; Diluent Number 0-4
	movwf	decodata+0
	movlw	d'95'                       ; EEPROM address of %O2
	movwf	divemins+0
	bra		menu_diluentgas
diluent_list_edit_gas3:
	movlw	d'2'                        ; Diluent Number 0-4
	movwf	decodata+0
	movlw	d'99'                       ; EEPROM address of %O2
	movwf	divemins+0
	bra		menu_diluentgas
diluent_list_edit_gas4:
	movlw	d'3'                        ; Diluent Number 0-4
	movwf	decodata+0
	movlw	d'103'                       ; EEPROM address of %O2
	movwf	divemins+0
	bra		menu_diluentgas
diluent_list_edit_gas5:
	movlw	d'4'                        ; Diluent Number 0-4
	movwf	decodata+0
	movlw	d'107'                       ; EEPROM address of %O2
	movwf	divemins+0
;	bra		menu_diluentgas
menu_diluentgas:
	movlw	d'1'
	movwf	menupos
	bcf		menubit4
	bcf		first_FA				; Here: =1: -, =0: +

menu_diluentgas0:
	call	DISP_ClearScreen
    WIN_LEFT    .20
	WIN_TOP		.185
    lfsr    FSR2, letter
	OUTPUTTEXT  .11			; Exit
    STRCAT_PRINT  ""

menu_diluentgas1:
	call	menu_pre_loop_common		; Clear some menu flags, timeout and switches
	call	diluent_title_bar2			; Displays the title bar with the current Gas info

	WIN_TOP		.35
	WIN_LEFT	.20
	lfsr	FSR2,letter
	OUTPUTTEXT	.88			; First Gas?
	PUTC	' '

	movlw	d'115'
	movwf	EEADR
	call	read_eeprom		; Get current startgas 1-5 # into EEDATA
	decf	EEDATA,W		; 0-4
	cpfseq	decodata+0		; =current displayed gas #?
	bra		menu_firstdil0	; no, display three spaces

	OUTPUTTEXT	.96			; Yes
	bsf		second_FA		; Is first gas

	bra		menu_firstdil1

menu_firstdil0:
	bcf		second_FA		; Is not first gas
	STRCAT  "   "           ; 3 spaces.

menu_firstdil1:
	call	word_processor

	WIN_LEFT	.20
	WIN_TOP		.65
	STRCPY  TXT_O2_4

	movf	divemins+0,W
	addlw	0x06
	movwf	EEADR
	call	read_eeprom                 ; O2 value
	movff	EEDATA,lo
	output_8
	STRCAT_PRINT "% "

; Show MOD in m
	WIN_LEFT	.90
    lfsr    FSR2, letter
	OUTPUTTEXTH .297                    ; MOD:

    GETCUSTOM8 .18                      ; ppO2 warnvalue in WREG
	mullw	d'10'
	movff	PRODL,xA+0
	movff	PRODH,xA+1                  ; ppO2 in [0.01bar] * 10
	movf	divemins+0,W
	addlw	0x06
	movwf	EEADR
	call	read_eeprom                 ; O2 value
	movff	EEDATA,xB+0
	clrf	xB+1
	call	div16x16                    ; xA/xB=xC with xA as remainder
	movlw	d'10'
	subwf	xC+0,F                      ; Subtract 10m...
	movff	xC+0,lo
	movlw	d'0'
	subwfb	xC+1,F
	movff	xC+1,hi

#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0					; Enabled ?
	bra		menu_ppO2setup2			; NO
	call	convert_meters_to_feet2	; converts meters to feet, don't check CF#61
	output_16
	STRCAT_PRINT  TXT_FOOT4
	bra		menu_ppO2setup3

menu_ppO2setup2:
#ENDIF
	output_16
	STRCAT_PRINT  TXT_METER3

#IFDEF IMPERIAL
menu_ppO2setup3:
#ENDIF

	WIN_LEFT	.20
	WIN_TOP		.95
	STRCPY  TXT_HE4
	movf	divemins+0,W
	addlw	0x07
	movwf	EEADR
	call	read_eeprom                 ; He value
	movff	EEDATA,lo
	output_8
	STRCAT_PRINT "% "

; Show END in m
    lfsr    FSR2, letter
	WIN_LEFT	.90
	OUTPUTTEXTH .298                    ; END:
	GETCUSTOM8 .18				        ; ppO2 warnvalue in WREG
	mullw	d'10'
	movff	PRODL,xA+0
	movff	PRODH,xA+1		            ; ppO2 in [0.01bar] * 10
   	movf	divemins+0,W
	addlw	0x06
	movwf	EEADR
	call	read_eeprom                 ; O2 value
	movff	EEDATA,xB+0
	clrf	xB+1
	call	div16x16                    ; xA/xB=xC with xA as remainder
	movlw	d'10'
	subwf	xC+0,F                      ; Subtract 10m...
	movff	xC+0,lo
	movlw	d'0'
	subwfb	xC+1,F
	movff	xC+1,hi                     ; lo:hi holding MOD in meters
	movlw	d'10'
	addwf	lo,F
	movlw	d'0'
	addwfc	hi,F                        ; lo:hi holding MOD+10m

	movf	divemins+0,W
	addlw	0x07
	movwf	EEADR
	call	read_eeprom                 ; He value in % -> EEDATA
	movlw	d'100'
	movwf	xA+0
	movf	EEDATA,W                    ; He value in % -> EEDATA
	subwf	xA+0,F                      ; xA+0 = 100 - He Value in %
	clrf	xA+1
	movff	lo,xB+0
	movff	hi,xB+1                     ; Copy MOD+10
	call	mult16x16                   ; xA*xB=xC
	movff	xC+0,xA+0
	movff	xC+1,xA+1
	movlw	d'100'
	movwf	xB+0
	clrf	xB+1
	call	div16x16                    ; xA/xB=xC with xA as remainder
	;	xC:2 = ((MOD+10) * 100 - HE Value in %) / 100
	movlw	d'10'
	subwf	xC+0,F				        ; Subtract 10m...
	movff	xC+0,lo
	movlw	d'0'
	subwfb	xC+1,F
	movff	xC+1,hi
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0					; Enabled ?
	bra		menu_ppO2setup4			; NO
	call	convert_meters_to_feet2	; converts meters to feet, don't check CF#61
	output_16
	STRCAT_PRINT  TXT_FOOT4
	bra		menu_ppO2setup5
menu_ppO2setup4:
#ENDIF
	output_16
	STRCAT_PRINT  TXT_METER3
#IFDEF IMPERIAL
menu_ppO2setup5:
#ENDIF
    WIN_LEFT    .20
	WIN_TOP		.125
	STRCPY  "+/-: "
	movlw	'+'
	btfsc	first_FA
	movlw	'-'
	movwf	POSTINC2
	call	word_processor

	WIN_TOP		.155
	lfsr	FSR2,letter
	OUTPUTTEXT	.89			            ; Default:
	movf	divemins+0,W
	addlw	0x04
	movwf	EEADR
	call	read_eeprom		            ; Default O2 value
	movff	EEDATA,lo
	output_8
	PUTC	'/'
	movf	divemins+0,W
	addlw	0x05
	movwf	EEADR
	call	read_eeprom		            ; Default He value
	movff	EEDATA,lo
	output_8
	STRCAT_PRINT  "  "

	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!
	call	DISP_menu_cursor

diluentgassetup_loop:
	call	check_switches_logbook

	btfsc	menubit3
	bra		diluentgassetup2           ; move cursor

	btfsc	menubit2
	bra		do_diluentgassetup		; call submenu

	btfsc	onesecupdate
	call	menu_check_dive_and_timeout	; "Goto restart" or sets sleepmode flag

	bcf		onesecupdate	; 1 sec. functions done

	btfsc	sleepmode
	bra		exit_menu_const_ppO2

	bra     diluentgassetup_loop

diluentgassetup2:
	incf	menupos,F
	movlw	d'7'
	cpfseq	menupos             ; =7?
	bra		diluentgassetup3	; No
	movlw	d'1'
	movwf	menupos

diluentgassetup3:
	clrf	timeout_counter2
	call	DISP_menu_cursor

	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!

	bcf		menubit3		; clear flag
	bra		diluentgassetup_loop

do_diluentgassetup:
	dcfsnz	menupos,F
    bra     make_first_diluent
	dcfsnz	menupos,F
	bra		adjust_o2_diluent
	dcfsnz	menupos,F
	bra		adjust_he_diluent
	dcfsnz	menupos,F
	bra		toggle_plus_minus_diluentsetup
	dcfsnz	menupos,F
	bra		restore_gas_diluent
exit_diluentgassetup:			; exit...
	movff	decodata+0,menupos
	incf	menupos,F
	bra		menu_diluentsetup_prelist

make_first_diluent:
	movff	decodata+0,EEDATA		; current gas (0-4) into EEDATA
	incf	EEDATA,F				; current gas (1-5) into EEDATA
	movlw	d'115'
	movwf	EEADR
	call	write_eeprom			; store in internal EEPROM
	movlw	d'1'
	movwf	menupos
	bra		menu_diluentgas1	; return

toggle_plus_minus_diluentsetup:
	btg		first_FA
	movlw	d'4'
	movwf	menupos
	bra		menu_diluentgas1	; return

adjust_o2_diluent:
	movf	divemins+0,W			; read current value
	addlw	0x06
	movwf	EEADR
	call	read_eeprom		; Low-value
	movff	EEDATA,lo

	btfsc	first_FA			; Minus?
	bra		adjust_o2_1_diluent			; yes, minus!

	incf	lo,F			; increase O2
	movlw	d'101'
	cpfseq	lo
	bra		adjust_o2_2_diluent
	movlw	d'4'			; LOWER O2 Limit
	movwf	lo
	bra		adjust_o2_2_diluent

adjust_o2_1_diluent:
	decf	lo,F			; decrease O2
	movlw	d'3'
	cpfseq	lo
	bra		adjust_o2_2_diluent

	movf	divemins+0,W
	addlw	0x07
	movwf	EEADR
	call	read_eeprom		; read He value

	movlw	d'100'
	movwf	lo
	movf	EEDATA,W		; He value
	subwf	lo,F			; lo=100% - He%

adjust_o2_2_diluent:				; test if O2+He>100...
	addlw	0x07
	movwf	EEADR
	call	read_eeprom		; read He value
	movf	EEDATA,W		; He value
	addwf	lo,W			; add O2 value
	movwf	hi				; store in temp
	movlw	d'101'
	cpfseq	hi				; O2 and He > 100?
	bra		adjust_o2_3_diluent		; No!

	movlw	d'4'			; LOWER O2 Limit
	movwf	lo

adjust_o2_3_diluent:
	movf	divemins+0,W			; save current value
	addlw	0x06
	movwf	EEADR
	movff	lo,EEDATA
	call	write_eeprom		; Low-value

	movlw	d'2'
	movwf	menupos
	bra		menu_diluentgas1	; return

adjust_he_diluent:
	movf	divemins+0,W			; read current value
	addlw	0x07
    movwf   EEADR			; read current value (He)
	call	read_eeprom		; Low-value
	movff	EEDATA,lo

	btfsc	first_FA			; Minus?
	bra		adjust_he_1_diluent			; yes, minus!

	incf	lo,F
	movlw	d'99'			; He limited to (useless) 99%
	cpfseq	lo
	bra		adjust_he_2_diluent
	clrf	lo
	bra		adjust_he_2_diluent

adjust_he_1_diluent:
	decf	lo,F			; decrease He
	movlw	d'255'
	cpfseq	lo
	bra		adjust_he_2_diluent
	clrf	lo              ; limit to zero

adjust_he_2_diluent:		; test if O2+He>100...
	movf	divemins+0,W
	addlw	0x06
	movwf	EEADR
	call	read_eeprom		; Low-value
	movf	EEDATA,W
	addwf	lo,W			; add He value
	movwf	hi				; store in temp
	movlw	d'101'
	cpfseq	hi                  ; O2 and He > 100?
	bra		adjust_he_3_diluent	; No!
;	clrf	lo				; Yes, clear He to zero
	decf	lo,F			; reduce He again = unchanged after operation

adjust_he_3_diluent:
	movf	divemins+0,W			; save current value
	addlw	0x07
	movwf	EEADR
	movff	lo,EEDATA
	call	write_eeprom	; Low-value

	movlw	d'3'
	movwf	menupos
	bra		menu_diluentgas1;

restore_gas_diluent:
	movf	divemins+0,W			; read Default value
	addlw	0x04
	movwf	EEADR
	call	read_eeprom		; Low-value
	movff	EEDATA,lo
	movf	divemins+0,W
	addlw	0x05
	movwf	EEADR
	call	read_eeprom		; High-value
	movff	EEDATA,hi

	movf	divemins+0,W			; save Default value
	addlw	0x06
	movwf	EEADR
	movff	lo,EEDATA
	call	write_eeprom		; Low-value
	movf	divemins+0,W
	addlw	0x07
	movwf	EEADR
	movff	hi,EEDATA
	call	write_eeprom		; High-value

	movlw	d'5'
	movwf	menupos
	bra		menu_diluentgas1


diluent_title_bar2:
;	call	DISP_topline_box
    call	DISP_divemask_color
	WIN_TOP		.2
	WIN_LEFT	.30
	lfsr	FSR2,letter
    STRCAT  TXT_DIL5            ; Dil.#
	movff	decodata+0,lo
	incf	lo,F
	bsf		leftbind
	output_99
	STRCAT_PRINT ": "

	WIN_TOP		.2
	WIN_LEFT	.80
	lfsr	FSR2,letter

	;He check
	movf	divemins+0,W
	addlw	0x07
	movwf	EEADR
	call	read_eeprom			; He value
	movff	EEDATA,lo			; Move EEData -> lo
	movf	lo,f				; Move lo -> f
	movlw	d'0'				; Move 0 -> WREG
	cpfsgt	lo					; He > 0?
	bra 	diluent_title_bar3	; NO check o2

	; YES Write TX 15/55
    STRCAT  TXT_TX3
	movf	divemins+0,W
	addlw	0x06
	movwf	EEADR
	call	read_eeprom			; O2 value
	movff	EEDATA,lo
	output_8					; Write O2
	PUTC	'/'
	movf	divemins+0,W
	addlw	0x07
	movwf	EEADR
	call	read_eeprom			; He value
	movff	EEDATA,lo
	output_8					; Write He
	bra		diluent_title_bar7

; New v1.44se
diluent_title_bar3:			; O2 Check
	movf	divemins+0,W
	addlw	0x06
	movwf	EEADR
	call	read_eeprom			; O2 value
	movff	EEDATA,lo
	movf	lo,f				; Move lo -> f
	movlw	d'21'				; Move 21 -> WREG
	cpfseq	lo					; o2 = 21
	cpfsgt	lo					; o2 > 21%
	bra 	diluent_title_bar5	; NO AIR
	movlw	d'100'				; Move 100 -> WREG
	cpfslt	lo					; o2 < 100%
	bra		diluent_title_bar4	; NO write O2

	; YES Write NX 32
    STRCAT  TXT_NX3
	output_8
	bra 	diluent_title_bar7

; New v1.44se
diluent_title_bar4:
    STRCAT  TXT_O2_3
	output_8
	bra 	diluent_title_bar7

; New v1.44se
diluent_title_bar5:
	cpfseq	lo					; o2 = 21%
	bra 	diluent_title_bar6

    STRCAT  TXT_AIR4
	output_8
	bra 	diluent_title_bar7

; New v1.44se
diluent_title_bar6:		; ERROR
    STRCAT  TXT_ERR4
	output_8
	;bra 	diluent_title_bar7

diluent_title_bar7:
    STRCAT_PRINT  ""
    call	DISP_standard_color
	return


; ***







menu_const_ppO2_setpoints:          ; Setpoint menu
	movlw	d'1'
	movwf	menupos

	bcf		menubit4
	clrf	decodata+0				; Here: # of SP
	bcf		first_FA				; Here: =1: -, =0: +
	bcf		second_FA				; Here: =1: 1, =0: 10 steps

menu_const_ppO20:
	call	DISP_ClearScreen
    call	DISP_divemask_color
	DISPLAYTEXT	.111		; Constant ppO2 Setup
    call	DISP_standard_color
	

menu_const_ppO21:
	WIN_LEFT 	.20
	WIN_TOP		.35
	lfsr	FSR2,letter
	OUTPUTTEXT	.112				; SP# 
	movff	decodata+0,lo		
	incf	lo,F				
	bsf		leftbind
	output_99
	STRCAT  " ("
	
	OUTPUTTEXT	d'192'				; Dil.
	PUTC	' '

    rcall   get_first_diluent           ; Read first diluent into lo(O2) and hi(He)
	bsf		leftbind
	output_99                           ; O2
	PUTC	'/'
    movff   hi,lo
	output_99                           ; He
	STRCAT_PRINT ")"

	WIN_LEFT 	.20
	WIN_TOP		.65

	lfsr	FSR2,letter
	OUTPUTTEXT	.97			; "Current: "
	movf	decodata+0,W
	addlw	d'36'				; offset in eeprom
	movwf	EEADR
	call	read_eeprom		; ppO2 value
	movff	EEDATA,lo
	clrf	hi
	bsf		leftbind
	output_16dp	d'3'
	STRCAT  TXT_BAR4
	movf	decodata+0,W
	addlw	d'117'				; offset in eeprom
    movwf	EEADR
	call	read_eeprom		; Change depth
	movff	EEDATA,lo
    output_8
	bcf		leftbind
    STRCAT_PRINT    "m "

	WIN_LEFT 	.20
	WIN_TOP		.95
	lfsr	FSR2,letter
	OUTPUTTEXT	d'190'			; ppO2 +
	call	word_processor		

	WIN_LEFT 	.20
	WIN_TOP		.125
	lfsr	FSR2,letter
	OUTPUTTEXT	d'250'			; +1 m
	call	word_processor		

	WIN_LEFT 	.20
	WIN_TOP		.155
	lfsr	FSR2,letter
	OUTPUTTEXT	.251			; -1 m
    call	word_processor

	WIN_LEFT 	.20
	WIN_TOP		.185
	lfsr	FSR2,letter
	OUTPUTTEXT	.89			; "Default: "
	STRCAT_PRINT "1.00 / 0m"

	WIN_LEFT 	.20
	WIN_TOP		.215
    lfsr	FSR2,letter
	OUTPUTTEXT   .11		; Exit
    call	word_processor

	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!
	call	menu_pre_loop_common		; Clear some menu flags, timeout and switches
	call	DISP_menu_cursor

menu_const_ppO2_loop:
	call	check_switches_logbook

	btfsc	menubit3
	bra		menu_const_ppO22	; move cursor

	btfsc	menubit2
	bra		do_menu_const_ppO2		; call submenu

	btfsc	onesecupdate
	call	menu_check_dive_and_timeout	; "Goto restart" or sets sleepmode flag

	bcf		onesecupdate	; 1 sec. functions done

	btfsc	sleepmode
	goto		exit_menu_const_ppO2    ;was bra

	bra		menu_const_ppO2_loop

menu_const_ppO22:
	incf	menupos,F
	
	movlw	d'2'
	cpfseq	menupos				; =2?
	bra		menu_const_ppO22a	; No
	incf	menupos,F			; Skip pos. 2

menu_const_ppO22a:
	movlw	d'8'
	cpfseq	menupos			; =8?
	bra		menu_const_ppO23	; No
	movlw	d'1'
	movwf	menupos

menu_const_ppO23:
	call	menu_pre_loop_common		; Clear some menu flags, timeout and switches
	call	DISP_menu_cursor
	bra		menu_const_ppO2_loop

do_menu_const_ppO2:
	dcfsnz	menupos,F
	bra		next_ppO2
	dcfsnz	menupos,F
	bra		change_ppo2_plus
	dcfsnz	menupos,F
	bra		change_ppo2_plus
	dcfsnz	menupos,F
	bra		change_ppo2_depth_plus
	dcfsnz	menupos,F
	bra		change_ppo2_depth_minus
	dcfsnz	menupos,F
	bra		change_ppo2_reset
#IFDEF CCR_CTRL
    ; Reorder Menu structure for CCR
	movlw	d'3'                        ; Different Return position
#ELSE
	movlw	d'2'
#ENDIF
	movwf	menupos
    goto    menu_const_ppO2_return

change_ppo2_depth_plus:
	movf	decodata+0,W
	addlw	d'117'				; offset in eeprom
    movwf	EEADR
	call	read_eeprom		; Change depth
	movff	EEDATA,lo
    incf    lo,F
	movlw	d'99'
	cpfsgt	lo
	bra		change_ppo2_depth_plus2
    clrf    lo
change_ppo2_depth_plus2:
	movff	lo,EEDATA			; write result
	call	write_eeprom		; save result in EEPROM
	movlw	d'4'
	movwf	menupos
	bra		menu_const_ppO21

change_ppo2_depth_minus:
	movf	decodata+0,W
	addlw	d'117'				; offset in eeprom
    movwf	EEADR
	call	read_eeprom		; Change depth
	movff	EEDATA,lo
    decf    lo,F
	movlw	d'100'
	cpfsgt	lo
	bra		change_ppo2_depth_minus2
    clrf    lo
change_ppo2_depth_minus2:
	movff	lo,EEDATA			; write result
	call	write_eeprom		; save result in EEPROM
	movlw	d'5'
	movwf	menupos
	bra		menu_const_ppO21

change_ppo2_plus:
	movf	decodata+0,W		; read current value 
	addlw	d'36'				; offset in memory
	movwf	EEADR
	call	read_eeprom			; Low-value
	movff	EEDATA,lo
	
    movlw   .10
    addwf   lo,F                ; increase ppO2
#IFDEF CCR_CTRL
	movlw	eCTRL_ADJ_MAX
#ELSE
	movlw	d'200'
#ENDIF
	cpfsgt	lo
	bra		change_ppo2_plus2
#IFDEF CCR_CTRL
	movlw	eCTRL_ADJ_MIN
#ELSE
	movlw	d'30'
#ENDIF
	movwf	lo
change_ppo2_plus2:
	movff	lo,EEDATA			; write result
	call	write_eeprom		; save result in EEPROM
	movlw	d'3'
	movwf	menupos
	bra		menu_const_ppO21

change_ppo2_reset:				; reset to 1.00bar and 0m depth
	movf	decodata+0,W		; read current value 
	addlw	d'36'				; offset in memory
	movwf	EEADR
	movlw	d'100'
	movwf	EEDATA
	call	write_eeprom		; save result in EEPROM

	movf	decodata+0,W
	addlw	d'117'				; offset in eeprom
    movwf	EEADR
    clrf    EEDATA              ; 0m default
    call	write_eeprom		; save result in EEPROM

	movlw	d'6'
	movwf	menupos
	bra	menu_const_ppO21

next_ppO2:
	incf	decodata+0,F
	movlw	d'3'
	cpfseq	decodata+0			; =3?
	bra	next_ppO22
	clrf	decodata+0			; yes, so reset to zero
next_ppO22:	
	movlw	d'1'
	movwf	menupos
	bra		menu_const_ppO21

get_first_diluent:              ; Read first diluent into lo(O2) and hi(He)
    read_int_eeprom d'115'      ; Read First Diluent (1-5)
    movf    EEDATA,W            ; WREG: 0-4
    mullw   .4                  ; PRODL: 0-16
    movlw   .93                 ; Base address for O2
    addwf   PRODL,W
    movwf   EEADR
    call    read_eeprom         ; Read O2
    movff   EEDATA,lo
    incf    EEADR,F
    call    read_eeprom         ; Read He
    movff   EEDATA,hi
    return

