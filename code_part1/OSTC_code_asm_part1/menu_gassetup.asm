
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


; Gas Setup menu
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 10/08/07
; last updated: 080906
; known bugs:
; ToDo: 

menu_gassetup:				
	movlw	d'1'
	movwf	menupos

menu_gassetup_prelist:
	call	DISP_ClearScreen
	call	gassetup_sort_gaslist			; Sorts Gaslist according to change depth
	call	menu_pre_loop_common		; Clear some menu flags, timeout and switches
    call	DISP_divemask_color
	DISPLAYTEXT	.106			; Gas List
    call	DISP_standard_color
	clrf	decodata+0				; Here: # of gas 0-4
	clrf	divemins+0				; Here: # of Gas * 4
	movlw	d'5'
	movwf	waitms_temp		; here: stores row for gas list
	movlw	d'2'
	movwf	wait_temp			; here: stores eeprom address for gas list

; Changed v1.44se
menu_gassetup_list:
	WIN_LEFT	.20
	movlw	d'4'
	addwf	wait_temp,F			; Increase eeprom address for gas list
	movlw	d'30'
	addwf	waitms_temp,F		; Increase row
	movf	waitms_temp,W		; Load row into WREG
	movff	WREG,win_top
	lfsr	FSR2,letter
	PUTC	TXT_GAS_C
	movff	decodata+0,lo		
	incf	lo,F				
	bsf		leftbind
	output_99
	PUTC	':'
	
	movf    decodata+0,W
	call	DISP_grey_inactive_gas			; Sets Greyvalue for inactive gases
	call	word_processor
	WIN_LEFT	.40
	movf	waitms_temp,W		; Load row into WREG
	movff	WREG,win_top
	lfsr	FSR2,letter

	movlw	d'33'
	movwf	EEADR
	call	read_eeprom			; Get current startgas 1-5 # into EEDATA
	decf	EEDATA,W			; 0-4
	cpfseq	decodata+0			; =current displayed gas #?
	bra		menu_gassetup_Tx	; no, do not display *
	PUTC	'*'					; display *

; New v1.44se
menu_gassetup_Tx:
	movf    decodata+0,W
	call	DISP_grey_inactive_gas			; Sets Greyvalue for inactive gases	
	call	word_processor

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
	bra 	menu_gassetup_Nx	; NO check o2

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
	bra 	menu_gassetup_list0

; New v1.44se
menu_gassetup_Nx:
	movff	wait_temp, EEADR	; Gas %O2 - Set address in internal EEPROM
	call	read_eeprom			; Read O2 value from EEPROM
	movff	EEDATA,lo			; Move EEDATA -> lo
	movf	lo,f				; Move lo -> f
	movlw	d'21'				; Move 21 -> WREG
	cpfsgt	lo					; o2 > 21%
	bra 	menu_gassetup_Air	; NO AIR
	movlw	d'100'				; Move 100 -> WREG
	cpfslt	lo					; o2 < 100%
	bra		menu_gassetup_O2	; NO write O2
	
	; YES Write NX 32
    STRCAT  TXT_NX3
	output_8
	bra 	menu_gassetup_list0

; New v1.44se
menu_gassetup_O2:
    STRCAT  TXT_O2_3
	output_8
	bra 	menu_gassetup_list0

; New v1.44se
menu_gassetup_Air:
	cpfseq	lo					; o2 = 21%
	bra     menu_gassetup_Err

    STRCAT  TXT_AIR4
	output_8
	bra 	menu_gassetup_list0

; New v1.44se
menu_gassetup_Err:
    STRCAT  TXT_ERR4
	output_8

; Changed v1.44se
menu_gassetup_list0:
	movf    decodata+0,W
	call	DISP_grey_inactive_gas			; Sets Greyvalue for inactive gases
	call	word_processor

	WIN_LEFT	.105
	movf	waitms_temp,W		; Load row into WREG
	movff	WREG,win_top
	lfsr	FSR2,letter

    STRCAT  TXT_AT4
	movf	decodata+0,W		; read current value 
	addlw	d'28'				; offset in memory
	movwf	EEADR
	call	read_eeprom			; Low-value
	movff	EEDATA,lo
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0					; Enabled ?
	bra		menu_gassetup_list1		; NO
	clrf	hi
	call	convert_meters_to_feet2	; converts meters to feet, don't check CF#61
	output_16_3
	STRCAT_PRINT  TXT_FOOT2
	bra		menu_gassetup_list2
menu_gassetup_list1:
#ENDIF
	output_8
    PUTC	TXT_METER_C
#IFDEF IMPERIAL
menu_gassetup_list2:
#ENDIF
	movf    decodata+0,W
	call	DISP_grey_inactive_gas			; Sets Greyvalue for inactive gases
	call	word_processor	

	call	DISP_standard_color
	
	incf	decodata+0,F
	movlw	d'5'	
	cpfseq	decodata+0
	goto	menu_gassetup_list

	DISPLAYTEXT	.11                     ; Exit
	call	wait_switches               ; Waits until switches are released, resets flag if button stays pressed!
	call	DISP_menu_cursor

gassetup_list_loop:
	call	check_switches_logbook

	btfsc	menubit3
	bra		gassetup_list2              ; move cursor

	btfsc	menubit2
	bra		do_gassetup_list            ; call gas-specific submenu

	btfsc	onesecupdate
	call	menu_check_dive_and_timeout	; "Goto restart" or sets sleepmode flag

	bcf		onesecupdate                ; 1 sec. functions done

	btfsc	sleepmode
	bra		exit_gassetup_list

	bra		gassetup_list_loop

gassetup_list2:
	incf	menupos,F
	movlw	d'7'
	cpfseq	menupos			; =7?
	bra		gassetup_list3	; No
	movlw	d'1'
	movwf	menupos

gassetup_list3:
	clrf	timeout_counter2
	call	DISP_menu_cursor

	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!

	bcf		menubit3		; clear flag
	bra		gassetup_list_loop

exit_gassetup_list:			; exit...
#IFDEF CCR_CTRL
    ; Reorder Menu structure for CCR
	movlw	d'3'            ; Return to position 3
#ELSE
	movlw	d'2'
#ENDIF
	movwf	menupos
	goto	menu2

do_gassetup_list:
	dcfsnz	menupos,F
	bra		gassetup_list_edit_gas1
	dcfsnz	menupos,F
	bra		gassetup_list_edit_gas2
	dcfsnz	menupos,F
	bra		gassetup_list_edit_gas3
	dcfsnz	menupos,F
	bra		gassetup_list_edit_gas4
	dcfsnz	menupos,F
	bra		gassetup_list_edit_gas5
	bra		exit_gassetup_list			; Exit List

gassetup_list_edit_gas1:
	movlw	d'0'
	movwf	decodata+0
	movlw	d'0'
	movwf	divemins+0
	bra		menu_gassetup_page1

gassetup_list_edit_gas2:
	movlw	d'1'
	movwf	decodata+0
	movlw	d'4'
	movwf	divemins+0
	bra		menu_gassetup_page1

gassetup_list_edit_gas3:
	movlw	d'2'
	movwf	decodata+0
	movlw	d'8'
	movwf	divemins+0
	bra		menu_gassetup_page1

gassetup_list_edit_gas4:
	movlw	d'3'
	movwf	decodata+0
	movlw	d'12'
	movwf	divemins+0
	bra		menu_gassetup_page1

gassetup_list_edit_gas5:
	movlw	d'4'
	movwf	decodata+0
	movlw	d'16'
	movwf	divemins+0
	bra		menu_gassetup_page1

menu_gassetup_page1:
	movlw	d'1'
	movwf	menupos
	bcf		gas_setup_page2			; Page 1 of gassetup
	bcf		menubit4
	bcf		first_FA				; Here: =1: -, =0: +

menu_gassetup0:
	call	DISP_ClearScreen
	DISPLAYTEXT	.147		; More...
	DISPLAYTEXT	.11			; Exit

menu_gassetup1:
	call	menu_pre_loop_common		; Clear some menu flags, timeout and switches

	rcall	gassetup_title_bar2			; Displays the title bar with the current Gas info

	WIN_LEFT	.20
	WIN_TOP		.65	
	STRCPY  TXT_O2_4

	movf	divemins+0,W
	addlw	0x06
	movwf	EEADR
	call	read_eeprom                 ; O2 value
	movff	EEDATA,lo
	output_8
#IFNDEF IMPERIAL
	STRCAT_PRINT "% "
; Show MOD in m
#ELSE
	STRCAT_PRINT "%  "
; Show MOD
#ENDIF

; Show MOD in m
	WIN_LEFT	.90
    lfsr    FSR2, letter
	OUTPUTTEXTH .297                    ; MOD:

	rcall	gassetup_get_mod			; compute MOD based on CF18 into lo:hi
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0					; Enabled ?
	bra		menu_gassetup2			; NO
	call	convert_meters_to_feet2	; converts meters to feet, don't check CF#61
	output_16
	STRCAT_PRINT  TXT_FOOT4
	bra		menu_gassetup3

menu_gassetup2:
#ENDIF

	output_16
	STRCAT_PRINT  TXT_METER3

#IFDEF IMPERIAL
menu_gassetup3:
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

#IFNDEF IMPERIAL
; Show END in m
#ELSE
; Show END
#ENDIF
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
	bra		menu_gassetup4			; NO
	call	convert_meters_to_feet2	; converts meters to feet, don't check CF#61
	output_16
	STRCAT_PRINT  TXT_FOOT4
	bra		menu_gassetup5
menu_gassetup4:
#ENDIF
	output_16
	STRCAT_PRINT  TXT_METER3

#IFDEF IMPERIAL
menu_gassetup5:
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

gassetup_loop:
	call	check_switches_logbook

	btfsc	menubit3
	bra		gassetup2	; move cursor

	btfsc	menubit2
	bra		do_gassetup		; call submenu

	btfsc	onesecupdate
	call	menu_check_dive_and_timeout	; "Goto restart" or sets sleepmode flag

	bcf		onesecupdate	; 1 sec. functions done

	btfsc	sleepmode
	bra		exit_gassetup

	bra	gassetup_loop

gassetup2:
	incf	menupos,F
	movlw	d'7'
	cpfseq	menupos			; =7?
	bra		gassetup3	; No
	movlw	d'1'
	movwf	menupos

gassetup3:

	clrf	timeout_counter2
	call	DISP_menu_cursor

	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!

	bcf		menubit3		; clear flag
	bra		gassetup_loop

do_gassetup:
	dcfsnz	menupos,F
	bra		next_gas_page
	dcfsnz	menupos,F
	bra		adjust_o2
	dcfsnz	menupos,F
	bra		adjust_he
	dcfsnz	menupos,F
	bra		toggle_plus_minus_gassetup
	dcfsnz	menupos,F
	bra		restore_gas
exit_gassetup:			; exit...
	movff	decodata+0,menupos
	incf	menupos,F
	bra		menu_gassetup_prelist

toggle_plus_minus_gassetup:
	btg		first_FA
	movlw	d'4'
	movwf	menupos
	bra		menu_gassetup1	; return

next_gas:
	movlw	d'6'
	movwf	menupos
	bra		menu_gassetup0		; incl. clear screen

adjust_o2:
	movf	divemins+0,W			; read current value 
	addlw	0x06
	movwf	EEADR
	call	read_eeprom		; Low-value
	movff	EEDATA,lo

	btfsc	first_FA				; Minus?
	bra		adjust_o2_1			; yes, minus!
	
	incf	lo,F			; increase O2
	movlw	d'101'
	cpfseq	lo
	bra		adjust_o2_2
	movlw	d'4'			; LOWER O2 Limit
	movwf	lo
	bra		adjust_o2_2

adjust_o2_1:
	decf	lo,F			; decrease O2
	movlw	d'3'
	cpfseq	lo
	bra		adjust_o2_2

	movf	divemins+0,W
	addlw	0x07
	movwf	EEADR
	call	read_eeprom		; read He value

	movlw	d'100'
	movwf	lo
	movf	EEDATA,W		; He value
	subwf	lo,F			; lo=100% - He%

adjust_o2_2:				; test if O2+He>100...
	movf	divemins+0,W
	addlw	0x07
	movwf	EEADR
	call	read_eeprom		; read He value
	movf	EEDATA,W		; He value
	addwf	lo,W			; add O2 value
	movwf	hi				; store in temp
	movlw	d'101'
	cpfseq	hi				; O2 and He > 100?
	bra		adjust_o2_3		; No!

	movlw	d'4'			; LOWER O2 Limit
	movwf	lo
	
adjust_o2_3:
	movf	divemins+0,W			; save current value
	addlw	0x06
	movwf	EEADR
	movff	lo,EEDATA
	call	write_eeprom		; Low-value

	movlw	d'2'
	movwf	menupos
	bra		menu_gassetup1	

adjust_he:
	movf	divemins+0,W			; read current value
	addlw	0x07
	movwf	EEADR
	call	read_eeprom		; Low-value
	movff	EEDATA,lo

	btfsc	first_FA			; Minus?
	bra		adjust_he_1			; yes, minus!
	
	incf	lo,F
	movlw	d'92'			; He limited to (useless) 90%
	cpfseq	lo
	bra		adjust_he_2
	clrf	lo
	bra		adjust_he_2

adjust_he_1:
	decf	lo,F			; decrease He
	movlw	d'255'
	cpfseq	lo
	bra		adjust_he_2
	clrf	lo

adjust_he_2:				; test if O2+He>100...
	movf	divemins+0,W
	addlw	0x06
	movwf	EEADR
	call	read_eeprom		; read He value
	movf	EEDATA,W		; He value
	addwf	lo,W			; add O2 value
	movwf	hi				; store in temp
	movlw	d'101'
	cpfseq	hi				; O2 and He > 100?
	bra		adjust_he_3		; No!
;	clrf	lo				; Yes, clear He to zero
	decf	lo,F			; reduce He again = unchanged after operation

adjust_he_3:
	movf	divemins+0,W			; save current value
	addlw	0x07
	movwf	EEADR
	movff	lo,EEDATA
	call	write_eeprom		; Low-value

	movlw	d'3'
	movwf	menupos
	bra		menu_gassetup1	; 

restore_gas:
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
	bra		menu_gassetup1	; 


next_gas_page:
	call	DISP_ClearScreen		
	movlw	d'1'
	movwf	menupos
	bcf		first_FA				; Here: =1: -, =0: +
	bcf		second_FA				; Here: =1: Is first gas
	DISPLAYTEXT	.107		; Depth +/-
	DISPLAYTEXT	.11			; Exit

next_gas_page1:
	call	menu_pre_loop_common		; Clear some menu flags, timeout and switches

	WIN_TOP		.65
	WIN_LEFT	.20
	lfsr	FSR2,letter
	OUTPUTTEXT	.88			; First Gas?
	PUTC	' '

	movlw	d'33'
	movwf	EEADR
	call	read_eeprom		; Get current startgas 1-5 # into EEDATA
	decf	EEDATA,W		; 0-4
	cpfseq	decodata+0		; =current displayed gas #?
	bra		menu_firstgas0	; no, display three spaces

	OUTPUTTEXT	.96			; Yes 
	bsf		second_FA		; Is first gas

; Do not reset change depth (Kind request from Pascal)!
;	movf	decodata+0,W		; read current value 
;	addlw	d'28'				; offset in memory
;	movwf	EEADR
;	call	read_eeprom			; Low-value
;	clrf	EEDATA				; Set change depth to zero
;	call	write_eeprom		; save result in EEPROM
;
	bra		menu_firstgas1

menu_firstgas0:
	bcf		second_FA		; Is not first gas
	STRCAT  "   "           ; 3 spaces.

menu_firstgas1:
	call	word_processor			


	rcall	gassetup_title_bar2			; Displays the title bar with the current Gas info
	WIN_TOP		.125
	WIN_LEFT	.20
	lfsr	FSR2,letter

	OUTPUTTEXT	.108		; Change:

#IFNDEF IMPERIAL
	; lo still holds change depth
#ELSE
	; hi,lo still holds change depth
#ENDIF
	bsf		leftbind
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0					; Enabled ?
	bra		menu_firstgas2			; NO
	output_16_3
	STRCAT_PRINT  TXT_FOOT4
	bra		menu_firstgas3

menu_firstgas2:
#ENDIF
	output_8
    STRCAT_PRINT  TXT_METER2

#IFDEF IMPERIAL
menu_firstgas3:
#ENDIF
; Show ppO2 after change depth
	WIN_TOP		.125
	WIN_LEFT	.110
	lfsr	FSR2,letter
	rcall	gassetup_show_ppO2			; Display the ppO2 of the change depth with the current gas
    clrf    WREG
    movff   WREG,letter+7               ; limit to 7 chars

	movff		xC+0,sub_a+0
	movff		xC+1,sub_a+1
	GETCUSTOM8	d'46'					; color-code ppO2 warning [cbar]
	movwf		sub_b+0
	clrf		sub_b+1
	call		sub16					;  sub_c = sub_a - sub_b	
	btfss		neg_flag
	bra			gassetup_color_code_ppo2_1; too high -> Warning Color!
	call		DISP_standard_color
	bra			gassetup_color_code_ppo2_2
gassetup_color_code_ppo2_1:
	call	DISP_warnings_color
gassetup_color_code_ppo2_2:
	call	word_processor	
	call	DISP_standard_color

	WIN_TOP		.95
	WIN_LEFT	.95
	lfsr	FSR2,letter
	movlw	'+'
	btfsc	first_FA
	movlw	'-'
	movwf	POSTINC2
	call	word_processor	

; Show MOD as "default"
	WIN_TOP		.155
	WIN_LEFT	.20
    lfsr    FSR2, letter

	OUTPUTTEXT	.109		; Default:

	rcall	gassetup_get_mod			; compute MOD based on CF18 into lo:hi
#IFDEF IMPERIAL
	movlw	d'99'
	cpfslt	lo
	movwf	lo						; limit to 99m
	clrf	hi
#ENDIF

	btfsc	second_FA		; Is first gas?
	clrf	lo				; Yes, display 0m
#IFNDEF IMPERIAL
	btfsc	second_FA		; Is first gas?
	clrf	hi				; Yes, display 0m
#ELSE
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0					; Enabled ?
	bra		gassetup_default_mod1	; NO
	call	convert_meters_to_feet2	; converts meters to feet, don't check CF#61
#ENDIF

	output_16
#IFDEF IMPERIAL
	STRCAT_PRINT  TXT_FOOT4
	bra		gassetup_default_mod2

gassetup_default_mod1:
	output_8
#ENDIF
	STRCAT_PRINT  TXT_METER3

#IFDEF IMPERIAL
gassetup_default_mod2:
#ENDIF
	WIN_TOP		.35
	WIN_LEFT	.20
	lfsr	FSR2,letter
	OUTPUTTEXT	.105			; "Active Gas? "

	; Active gas flags in BIT0:4 ....
	movff	decodata+0,lo	; Gas 0-4
	incf	lo,F			; Gas 1-5

	read_int_eeprom		d'27'	; read flag register
active_gas_display:
	rrcf	EEDATA			; roll flags into carry
	decfsz	lo,F			; max. 5 times...
	bra		active_gas_display
	
	btfss	STATUS,C		; test carry
	bra		active_gas_display_no
	
	OUTPUTTEXT	.96			; Yes 
	bra		active_gas_display_end
	
active_gas_display_no:
	STRCAT  "   "                       ; three spaces instead of "Yes"

active_gas_display_end:	
	call	word_processor	

	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!
	call	DISP_menu_cursor

next_gas_page_loop:
	call	check_switches_logbook

	btfsc	menubit3
	bra		next_gas_page2	; move cursor

	btfsc	menubit2
	bra		do_next_gas_page		; call submenu

	btfsc	onesecupdate
	call	menu_check_dive_and_timeout	; "Goto restart" or sets sleepmode flag

	bcf		onesecupdate	; 1 sec. functions done

	btfsc	sleepmode
	bra		exit_gassetup

	bra		next_gas_page_loop

next_gas_page2:
	incf	menupos,F

	movlw	d'7'
	cpfseq	menupos			; =7?
	bra		next_gas_page3	; No
	movlw	d'1'
	movwf	menupos

next_gas_page3:
	clrf	timeout_counter2
	call	DISP_menu_cursor
	
	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!

	bcf		menubit3		; clear flag
	bra		next_gas_page_loop

do_next_gas_page:
	dcfsnz	menupos,F
	bra		toggle_active_gas
	dcfsnz	menupos,F
	bra		make_first_gas
	dcfsnz	menupos,F
	bra		change_gas_depth_plus_minus
	dcfsnz	menupos,F
	bra		change_gas_depth_apply
	dcfsnz	menupos,F
	bra		change_gas_depth_default
	bra		next_gas

make_first_gas:
	movff	decodata+0,EEDATA		; current gas (0-4) into EEDATA
	incf	EEDATA,F				; current gas (1-5) into EEDATA
	movlw	d'33'
	movwf	EEADR
	call	write_eeprom			; store in internal EEPROM

; Set First gas to "Active"
	read_int_eeprom		d'27'		; read flag register
	movff	decodata+0,lo			; selected gas 0-4
	incf	lo,F
	dcfsnz	lo,F
	bsf		EEDATA,0
	dcfsnz	lo,F
	bsf		EEDATA,1
	dcfsnz	lo,F
	bsf		EEDATA,2
	dcfsnz	lo,F
	bsf		EEDATA,3
	dcfsnz	lo,F
	bsf		EEDATA,4
	write_int_eeprom	d'27'		; write flag register

	movlw	d'2'
	movwf	menupos
	bra		next_gas_page1

toggle_active_gas:
	read_int_eeprom		d'27'		; read flag register
	movff	decodata+0,lo			; selected gas 0-4
	incf	lo,F
	dcfsnz	lo,F
	btg		EEDATA,0
	dcfsnz	lo,F
	btg		EEDATA,1
	dcfsnz	lo,F
	btg		EEDATA,2
	dcfsnz	lo,F
	btg		EEDATA,3
	dcfsnz	lo,F
	btg		EEDATA,4
	write_int_eeprom	d'27'		; write flag register
	movlw	d'1'
	movwf	menupos
	bra		next_gas_page1
	
change_gas_depth_apply:			; Apply +1 or -1m
	movf	decodata+0,W		; read current value 
	addlw	d'28'				; offset in memory
	movwf	EEADR
	call	read_eeprom			; Low-value
	movff	EEDATA,lo

	btfsc	first_FA			; Minus?
	bra		change_gas_depth_minus	; yes, minus!
; +1m

	incf	lo,F				; increase depth
	movlw	d'100'				; Change depth limit + 1
	cpfslt	lo					; >99?
	clrf	lo					; Yes, set to zero m

change_gas_depth_plus2:
	movff	lo,EEDATA			; write result
	call	write_eeprom		; save result in EEPROM
	movlw	d'4'
	movwf	menupos
	bra		next_gas_page1

change_gas_depth_minus:
; -1m
	decf	lo,F				; decrease depth
	btfsc	lo,7				; 255?
	clrf	lo					; Yes, stay at zero m
	bra		change_gas_depth_plus2	; exit

change_gas_depth_plus_minus:
	btg		first_FA
	movlw	d'3'
	movwf	menupos
	bra		next_gas_page1

change_gas_depth_default:
	rcall	gassetup_get_mod			; compute MOD based on CF18 into lo:hi

	movlw	d'99'
	cpfslt	lo
	movwf	lo					; limit to 99m

	btfsc	second_FA			; Is first gas?
	clrf	lo					; Yes, set to 0m

	movf	decodata+0,W		; read current value 
	addlw	d'28'				; offset in memory
	movwf	EEADR
;	call	read_eeprom			; Low-value
	movff	lo,EEDATA			; write result
	call	write_eeprom		; save result in EEPROM

	movlw	d'5'
	movwf	menupos
	bra		next_gas_page1


; Changed v1.44se
gassetup_title_bar2:
;	call	DISP_topline_box
    call	DISP_divemask_color
	WIN_TOP		.2
	WIN_LEFT	.0
	lfsr	FSR2,letter
	OUTPUTTEXT	.95				; Gas# 
	movff	decodata+0,lo		
	incf	lo,F				
	bsf		leftbind
	output_99
	STRCAT_PRINT ": "
	
	WIN_TOP		.2
#IFNDEF IMPERIAL
	WIN_LEFT	.50
#ELSE
	WIN_LEFT	.42
#ENDIF
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
	bra 	gassetup_title_bar3	; NO check o2
	
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
	bra		gassetup_title_bar7

; New v1.44se
gassetup_title_bar3:			; O2 Check		
	movf	divemins+0,W
	addlw	0x06
	movwf	EEADR
	call	read_eeprom			; O2 value
	movff	EEDATA,lo	
	movf	lo,f				; Move lo -> f
	movlw	d'21'				; Move 21 -> WREG
	cpfseq	lo					; o2 = 21
	cpfsgt	lo					; o2 > 21%
	bra 	gassetup_title_bar5	; NO AIR
	movlw	d'100'				; Move 100 -> WREG
	cpfslt	lo					; o2 < 100%
	bra		gassetup_title_bar4	; NO write O2

	; YES Write NX 32
    STRCAT  TXT_NX3
	output_8
	bra 	gassetup_title_bar7

; New v1.44se
gassetup_title_bar4:
    STRCAT  TXT_O2_3
	output_8
	bra 	gassetup_title_bar7

; New v1.44se
gassetup_title_bar5:
	cpfseq	lo					; o2 = 21%
	bra 	gassetup_title_bar6

    STRCAT  TXT_AIR4
	output_8
	bra 	gassetup_title_bar7

; New v1.44se
gassetup_title_bar6:		; ERROR
    STRCAT  TXT_ERR4
	output_8
	;bra 	gassetup_title_bar7

gassetup_title_bar7:
    STRCAT  TXT_AT4
	movf	decodata+0,W		; read current value 
	addlw	d'28'				; offset in memory
	movwf	EEADR
	call	read_eeprom			; Low-value
	movff	EEDATA,lo
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0				; Enabled ?
	bra		gassetup_title_bar8	; NO
	clrf	hi
	call	convert_meters_to_feet2	; converts meters to feet, don't check CF#61
	output_16_3
	STRCAT_PRINT  TXT_FOOT3
	bra		gassetup_title_bar9

gassetup_title_bar8:
#ENDIF
	output_8
    STRCAT_PRINT  TXT_METER2
    call	DISP_standard_color
#IFDEF IMPERIAL
gassetup_title_bar9:
#ENDIF
	return

gassetup_show_ppO2:
	movf	divemins+0,W
	addlw	0x06
	movwf	EEADR
	call	read_eeprom			; O2 value
	movff	EEDATA,hi

	movf	decodata+0,W		; read current value 
	addlw	d'28'				; offset in memory
	movwf	EEADR
	call	read_eeprom			; Change depth in m
	movff	EEDATA,lo
	movlw	d'10'
	addwf	lo,F				; Depth+10m=lo
	movf	hi,W
	mulwf	lo					; (Depth+10m)*O2
	movff	PRODL,xA+0
	movff	PRODH,xA+1
	movlw	d'10'
	movwf	xB+0
	clrf	xB+1
	call	div16x16			;xA/xB=xC with xA as remainder 	
	movff	xC+0,lo				; ((Depth+10m)*O2)/10 = [0.01bar] ppO2
	movff	xC+1,hi
	output_16dp	d'3'
	OUTPUTTEXT 	d'150'		; "bar "
	return

gassetup_get_mod:
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
	return



;=============================================================================
; Make sure first gas is marked active.
; Note: - Gas are not soted anymore.
;       - Gas with a depth>0 should not be forced active, or it is impossible
;         to de-activate them.
gassetup_sort_gaslist:

	clrf	EEADRH                  ; Select EEPROM lower page.
	read_int_eeprom		d'33'       ; Get First gas (1-5)
    movff   EEDATA,lo               ; into register lo

	read_int_eeprom		d'27'	    ; Read selected gases

    dcfsnz  lo,F                    ; If lo==1
    bsf     EEDATA,0                ; Select Gas1
    dcfsnz  lo,F                    ; If lo==2
    bsf     EEDATA,1                ; Select Gas2
    dcfsnz  lo,F
    bsf     EEDATA,2
    dcfsnz  lo,F
    bsf     EEDATA,3
    dcfsnz  lo,F
    bsf     EEDATA,4
    
    ; Copy result to register:
    movff   EEDATA,gaslist_active
    
    ; And write to EEPROM too, to survive next reboot:
	write_int_eeprom    d'27'

	return			

;=============================================================================
; EEPROM Locations of Gaslist
; Gas1: 
; O2 Default:4
; He Default:5
; O2 Current:6
; He Current:7
; Change: 28
; Active: 27,0

; Gas2: 
; O2 Default:8
; He Default:9
; O2 Current:10
; He Current:11
; Change: 29
; Active: 27,1

; Gas3: 
; O2 Default:12
; He Default:13
; O2 Current:14
; He Current:15
; Change: 30
; Active: 27,2

; Gas4: 
; O2 Default:16
; He Default:17
; O2 Current:18
; He Current:19
; Change: 31
; Active: 27,3

; Gas5: 
; O2 Default:20
; He Default:21
; O2 Current:22
; He Current:23
; Change: 32
; Active: 27,4
