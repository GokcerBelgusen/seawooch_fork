
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


; routines for display outputs
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 15/01/05
;
; History:
; 2008-06-06 [MH] last updated
; 2010-12-31 [jDG] Multi-page display for GF decoplan
; 2011-01-04 [jDG] Saturation graphs in customview divemode
;
; known bugs:
; ToDo:	More comments

    global   DISP_divemask_color
DISP_divemask_color:
	GETCUSTOM8	d'36'			; Divemask output color
	bra		DISP_standard_color_0

    global  DISP_warnings_color
DISP_warnings_color:
	GETCUSTOM8	d'37'			; Warnings output color
	bra		DISP_standard_color_0

    global  DISP_standard_color
DISP_standard_color:
	GETCUSTOM8	d'35'			; Standard output color
DISP_standard_color_0:			; Common entry point
	movwf	DISPLAY1_temp			; copy
	movlw	d'0'
	cpfseq	DISPLAY1_temp
	bra		DISP_standard_color_1
	bra		DISP_standard_color2
DISP_standard_color_1:
	movlw	d'4'
	cpfseq	DISPLAY1_temp
	bra		DISP_standard_color_2
	bra		DISP_standard_color2
DISP_standard_color_2:
	movlw	d'8'
	cpfseq	DISPLAY1_temp
	bra		DISP_standard_color_3
	bra		DISP_standard_color2
DISP_standard_color_3:
	movlw	d'192'
	cpfseq	DISPLAY1_temp
	bra		DISP_standard_color_4
	bra		DISP_standard_color2
DISP_standard_color_4:
	movlw	d'196'
	cpfseq	DISPLAY1_temp
	bra		DISP_standard_color_5
	bra		DISP_standard_color2
DISP_standard_color_5:
	movlw	d'200'
	cpfseq	DISPLAY1_temp
	bra		DISP_standard_color_6
	bra		DISP_standard_color2
DISP_standard_color_6:
	movf	DISPLAY1_temp,W		; Color should be OK...
	call	DISP_set_color
	return
DISP_standard_color2:
	movlw	0xFF		        ; Force full white.
	call	DISP_set_color
	return

DISP_color_code macro color_code_temp
	movlw	color_code_temp
	call	DISP_color_code1
	endm

DISP_color_code1:				; Color-codes the output, if required
	dcfsnz	WREG
	bra		DISP_color_code_depth		; CF43 [mbar], 16Bit
	dcfsnz	WREG
	bra		DISP_color_code_cns			; CF44 [%]
	dcfsnz	WREG
	bra		DISP_color_code_gf			; CF45 [%]
	dcfsnz	WREG
	bra		DISP_color_code_ppo2		; CF46 [cbar]
	dcfsnz	WREG
	bra		DISP_color_code_velocity	; CF47 [m/min]
	dcfsnz	WREG
	bra		DISP_color_code_ceiling		; Show warning if CF41=1 and current depth>shown ceiling
	dcfsnz	WREG
	bra		DISP_color_code_gaslist		; Color-code current row in Gaslist (%O2 in "EEDATA")


DISP_color_code_gaslist:				; %O2 in "EEDATA"
; Check very high ppO2 manually
    SAFE_2BYTE_COPY amb_pressure,xA
	movlw		d'10'
	movwf		xB+0
	clrf		xB+1
	call		div16x16				; xC=p_amb/10
	movff		xC+0,xA+0
	movff		xC+1,xA+1
	movff		EEDATA,xB+0
	clrf		xB+1
	call		mult16x16				; EEDATA * p_amb/10

	tstfsz		xC+2						; char_I_O2_ratio * p_amb/10 > 65536, ppO2>6,55bar?
	bra			DISP_color_code_gaslist1	; Yes, warn in warning color
; Check if ppO2>3,30bar
	btfsc		xC+1,7
	bra			DISP_color_code_gaslist1	; Yes, warn in warning color

	movff		xC+0,sub_a+0
	movff		xC+1,sub_a+1
	GETCUSTOM8	d'46'					; color-code ppO2 warning [cbar]
	mullw		d'100'					; ppo2_warning_high*100
	movff		PRODL,sub_b+0
	movff		PRODH,sub_b+1
	call		sub16					;  sub_c = sub_a - sub_b	
	btfss		neg_flag
	bra			DISP_color_code_gaslist1; too high -> Warning Color!
	call		DISP_standard_color
	return

DISP_color_code_gaslist1:
	call		DISP_warnings_color
	return

DISP_color_code_ceiling:
	GETCUSTOM8	d'40'			; =1: Warn at all?
	movwf	lo					
	movlw	d'1'
	cpfseq	lo					; =1?
	bra		DISP_color_code_ceiling1	; No, Set to default color

    SAFE_2BYTE_COPY rel_pressure, lo
	call	adjust_depth_with_salinity			; computes salinity setting into lo:hi [mbar]
	movff	hi,xA+1
	movff	lo,xA+0
	movff	char_O_first_deco_depth,lo  ; Ceiling in m
	decf	lo,F	                    ; -1
	movlw	LOW		d'100'
	movwf	xB+0
	clrf	xB+1						; Devide/100 -> xC+0 = Depth in m
	call	div16x16					; xA/xB=xC with xA as remainder 	
	movf	xC+0,W						; Depth in m
	subwf	lo,W
	btfsc	STATUS,C
	bra		DISP_color_code_ceiling2	; Set to warning color
DISP_color_code_ceiling1:
	call	DISP_standard_color
	return
DISP_color_code_ceiling2:
	call	DISP_warnings_color
	return

DISP_color_code_depth:
	movff	hi,hi_temp
	movff	lo,lo_temp
    SAFE_2BYTE_COPY rel_pressure, lo
	call	adjust_depth_with_salinity			; computes salinity setting into lo:hi [mbar]
	movff	lo,sub_a+0
	movff	hi,sub_a+1
	GETCUSTOM15	d'43'				; Depth warn [mbar]
	movff	lo,sub_b+0
	movff	hi,sub_b+1
	call	sub16			;  sub_c = sub_a - sub_b
	btfss	neg_flag
	bra		DISP_color_code_depth2; Set to warning color
	call	DISP_standard_color
	movff	hi_temp,hi
	movff	lo_temp,lo			; Restore hi, lo
	return
DISP_color_code_depth2:
	call	DISP_warnings_color
	movff	hi_temp,hi
	movff	lo_temp,lo			; Restore hi, lo
	return

DISP_color_code_cns:
	movff	char_O_CNS_fraction,lo
	GETCUSTOM8	d'44'			; CNS Warn [%]
	subwf	lo,W
	btfsc	STATUS,C
	bra		DISP_color_code_cns2		; Set to warning color
	call	DISP_standard_color
	return
DISP_color_code_cns2:
	call	DISP_warnings_color
	return

DISP_color_code_gf:
	movff	char_O_gradient_factor,lo		; gradient factor
	GETCUSTOM8	d'45'			; GF Warn [%]
	subwf	lo,W
	btfsc	STATUS,C
	bra		DISP_color_code_gf2		; Set to warning color
	call	DISP_standard_color
	return
DISP_color_code_gf2:
	call	DISP_warnings_color
	return

DISP_color_code_ppo2:
; Check if ppO2>6,55bar
	tstfsz	xC+2					; char_I_O2_ratio * p_amb/10 > 65536, ppO2>6,55bar?
	bra		DISP_color_code_ppo22	; Yes, warn in warning color
; Check if ppO2>3,30bar
	btfsc	xC+1,7
	bra		DISP_color_code_ppo22	; Yes, warn in warning color

	movff	xC+0,sub_a+0
	movff	xC+1,sub_a+1
	GETCUSTOM8	d'46'			; color-code ppO2 warning [cbar]
	mullw	d'100'
	movff	PRODL,sub_b+0
	movff	PRODH,sub_b+1
	call	sub16			  	;  sub_c = sub_a - sub_b
	btfss	neg_flag
	bra		DISP_color_code_ppo22; Set to warning color
	call	DISP_standard_color
	return
DISP_color_code_ppo22:
	call	DISP_warnings_color
	return

DISP_color_code_velocity:
	btfss	neg_flag			; Ignore for ascend!
	bra		DISP_color_code_velocity1		; Skip check!
	movff	divA+0,lo
	GETCUSTOM8	d'47'			; Velocity warn [m/min]
	subwf	lo,W
	btfsc	STATUS,C
	bra		DISP_color_code_velocity2		; Set to warning color
DISP_color_code_velocity1:
	call	DISP_standard_color
	return
DISP_color_code_velocity2:
	call	DISP_warnings_color
	return

ostc_debug	macro value
	movlw	value
	call	ostc_debug1
	endm

ostc_debug1:
	movff	debug_char+4,debug_char+5		; Save for background debugger
	movff	debug_char+3,debug_char+4
	movff	debug_char+2,debug_char+3
	movff	debug_char+1,debug_char+2
	movff	debug_char+0,debug_char+1
	movff	WREG,debug_char+0

	btfss	debug_mode				; Are we in debugmode?
	return							; No, return!

	WIN_TOP		.192
	WIN_LEFT	.100
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color
	lfsr	FSR2,letter
	movf	debug_char+0,W
	movwf 	POSTINC2
	movf	debug_char+1,W
	movwf 	POSTINC2
	movf	debug_char+2,W
	movwf 	POSTINC2
	movf	debug_char+3,W
	movwf 	POSTINC2
	movf	debug_char+4,W
	movwf 	POSTINC2
	movf	debug_char+5,W
	movwf 	POSTINC2
	
	STRCAT_PRINT " "
	return

;=============================================================================
; BlueScreen function.
;
DISP_resetdebugger:
    global DISP_resetdebugger
    global temp10

    movlb   1                       ; For C-code calls
	call	DISPLAY_boot			; DISP boot
	call	DISP_standard_color
	WIN_INVERT	.0					; Init new Wordprocessor

	DISPLAYTEXT	.133
	DISPLAYTEXT	.134
	DISPLAYTEXT	.135
	DISPLAYTEXT	.136				; Display Debug intro
	
	WIN_TOP		.100
	WIN_LEFT	.10

	lfsr	FSR2,letter
	movff	temp10+0,lo             ; Code-stack point at crash time.
	movff	temp10+1,hi             ; Code-stack point at crash time.
	output_16
	movlw	' '
	movwf	POSTINC2
	movf	debug_char+0,W
	movwf 	POSTINC2
	movf	debug_char+1,W
	movwf 	POSTINC2
	movf	debug_char+2,W
	movwf 	POSTINC2
	movf	debug_char+3,W
	movwf 	POSTINC2
	movf	debug_char+4,W
	movwf 	POSTINC2
	movf	debug_char+5,W
	movwf 	POSTINC2
	STRCAT  ". "
	movff	flag1,lo
	output_8		
	PUTC    ' '
	movff	flag2,lo
	output_8		
	call	word_processor

	WIN_TOP		.125

	lfsr	FSR2,letter
	movff	flag3,lo
	output_8		
	PUTC    ' '
	movff	flag4,lo
	output_8		
	PUTC    ' '
	movff	flag5,lo
	output_8		
	PUTC    ' '
	movff	flag6,lo
	output_8		
	PUTC    ' '
	movff	flag7,lo
	output_8		
	call	word_processor

	WIN_TOP		.150

	lfsr	FSR2,letter
	movff	flag8,lo
	output_8		
	PUTC    ' '
	movff	flag9,lo
	output_8		
	PUTC    ' '
	movff	flag10,lo
	output_8		
	PUTC    ' '
	movff	flag11,lo
	output_8		
	PUTC    ' '
	movff	flag12,lo
	output_8		
	call	word_processor

	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!
DISP_resetdebugger_loop:
    bcf     LED_blue            ; Blink blue led every seconds..
    btfss   secs,0
    bsf     LED_blue

	btfss	switch_left
	bra		DISP_resetdebugger_loop		; Loop

    bcf     LED_blue
	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!
	return

DISP_divemode_mask:					; Displays mask in Dive-Mode
	call		DISP_divemask_color	; Set Color for Divemode mask
#IFDEF CCR_CTRL     
    ; Different position of the Max depth.
	btfss	FLAG_const_ppO2_mode
    bra     DISP_divemode_mask1
	DISPLAYTEXTH	.394            ; Max. (at top)
    bra     DISP_divemode_mask2
DISP_divemode_mask1:
	DISPLAYTEXTH	.267            ; Max.
DISP_divemode_mask2:
#ELSE
	DISPLAYTEXTH	.267		; Max.
#ENDIF
	DISPLAYTEXT		.86			; Divetime
	DISPLAYTEXT		.87			; Depth
	call	DISP_standard_color
	return

DISP_clear_customview_divemode:
    WIN_BOX_BLACK   .168, .239, .90, .159		;top, bottom, left, right
	return

DISP_clear_customview_surfmode:
    WIN_BOX_BLACK   .25, .121, .82, .159		;top, bottom, left, right
	return

DISP_clear_decoarea:
    WIN_BOX_BLACK   .54, .168, .90, .159		;top, bottom, left, right
	return

DISP_display_ndl_mask:
#IFDEF SEAWOOCH
    call    DISP_clear_GF
	btfsc	menubit					; Divemode menu active?
	return							; Yes, return

DISP_clear_GF:
	; Clears Gradient Factor
	movlw	d'8'
	movwf	temp1
#IFDEF CCR_CTRL
	WIN_TOP		.119
#ELSE
	WIN_TOP		.145
#ENDIF
	WIN_LEFT	.0
	call	DISP_display_clear_common_y1
	return							; Yes, return
#ELSE
	; Clears Gradient Factor
	movlw	d'8'
	movwf	temp1
#IFDEF CCR_CTRL
	WIN_TOP		.119
#ELSE
	WIN_TOP		.145
#ENDIF
	WIN_LEFT	.0
	call	DISP_display_clear_common_y1

	btfsc	menubit					; Divemode menu active?
	return							; Yes, return
#ENDIF

	; Clear Dekostop and Dekosum
	rcall	DISP_clear_decoarea	
	call	DISP_divemask_color	; Set Color for Divemode mask
	DISPLAYTEXT		d'84'			; NoStop
	call	DISP_standard_color
	return

DISP_display_ndl:
	GETCUSTOM8	d'66'				; Always show GF?
	decfsz		WREG,F				; WREG=1?	
	bra		DISP_display_ndl2		; No
	rcall	DISP_display_gf			; Show GF (If GF > CF08)

DISP_display_ndl2:
	btfsc	menubit					; Divemode menu active?
	return							; Yes, return

	ostc_debug	'z'		; Sends debug-information to screen if debugmode active
	
	WIN_TOP		.136
	WIN_LEFT	.118
	WIN_FONT 	FT_MEDIUM
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color

	lfsr	FSR2,letter
	movff	char_O_nullzeit,lo				; NDL in minutes
	output_8
	STRCAT_PRINT    "'"
	WIN_FONT 	FT_SMALL
	return

DISP_display_deko_mask:
	btfsc	menubit					; Divemode menu active?
	return							; Yes, return

    rcall	DISP_clear_decoarea	
    ; total deco time word
	bcf			show_safety_stop	; Clear safety stop flag
    call		DISP_divemask_color	; Set Color for Divemode mask
    DISPLAYTEXT	d'85'			; TTS
	DISPLAYTEXT	d'82'			; DEKOSTOP
    call	DISP_standard_color
    return

DISP_display_deko:
	btfsc	menubit					; Divemode menu active?
	bra		DISP_display_deko1		; Yes, do not display deco, only GF (if required)

	ostc_debug	'y'		; Sends debug-information to screen if debugmode active
	WIN_TOP		.80
#IFNDEF IMPERIAL
	WIN_LEFT	.94
#ENDIF
	WIN_FONT 	FT_MEDIUM
	WIN_INVERT	.0                      ; Init new Wordprocessor
	bcf     leftbind
	DISP_color_code		warn_ceiling    ; Color-code Output
    btfsc   decoplan_invalid            ; The decoplan needs to updated...
    call    DISP_grey                   ; .. so set the color to grey
	lfsr	FSR2,letter
	movff	char_O_first_deco_depth,lo  ; Ceiling in m
#IFDEF IMPERIAL
	bcf     leftbind
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0					; Enabled ?
	bra		DISP_display_deko3		; NO
	WIN_LEFT	.80
	clrf	hi
	call	convert_meters_to_feet2	; converts meters to feet, don't check CF#61
	output_16_3
	call word_processor

	WIN_TOP		.91
	WIN_LEFT	.115
	WIN_FONT 	FT_SMALL
	lfsr	FSR2,letter
	STRCPY_PRINT TXT_FOOT2          ; Print 'ft' in small font

	WIN_TOP		.80
	WIN_LEFT	.130
	WIN_FONT 	FT_MEDIUM
	lfsr	FSR2,letter
	bra		DISP_display_deko4

DISP_display_deko3:
	WIN_LEFT	.94
#ENDIF
	output_99
	PUTC    TXT_METER_C
#IFNDEF IMPERIAL
	movff	char_O_first_deco_time,lo   ; length of first stop in m
#ELSE
DISP_display_deko4:	
	movff	char_O_first_deco_time,lo   ; length of first stop in min
#ENDIF
	output_99
	STRCAT_PRINT "'"
	WIN_FONT 	FT_SMALL
	
	ostc_debug	'x'		; Sends debug-information to screen if debugmode active
	
	WIN_TOP		.136
	WIN_LEFT	.140 - 6*7 - 4          ; let space for sign + 5 digits + '
	WIN_FONT 	FT_MEDIUM
	WIN_INVERT	.0					    ; Init new Wordprocessor

	call	DISP_standard_color
    btfsc   decoplan_invalid            ; The decoplan needs to updated...
    call    DISP_grey                   ; .. so set the color to grey
	lfsr	FSR2,letter
	movff	int_O_ascenttime+0,lo       ; TTS
	movff	int_O_ascenttime+1,hi       ; on 16bits
	output_16
	STRCAT_PRINT    "'"
	call	DISP_standard_color

DISP_display_deko1:
	rcall	DISP_display_gf				; Show GF (If GF > CF08)
	return								; Done.

DISP_display_gf:
	movff	char_O_gradient_factor,lo	; gradient factor
	GETCUSTOM8	d'8'					; threshold for display
	cpfslt	lo							; show value?
	bra		DISP_display_deko2			; Yes
	; No
#IFDEF SEAWOOCH
    call    DISP_clear_GF
#ELSE
	; Clears Gradient Factor
	movlw	d'8'
	movwf	temp1
#IFDEF CCR_CTRL
	WIN_TOP		.119
#ELSE
	WIN_TOP		.145
#ENDIF
	WIN_LEFT	.0
	call	DISP_display_clear_common_y1	
#ENDIF
	return

DISP_display_deko2:
	ostc_debug	'w'		; Sends debug-information to screen if debugmode active
;GF
#IFDEF CCR_CTRL
	WIN_TOP		.119
#ELSE
	WIN_TOP		.145
#ENDIF
	WIN_LEFT	.0
	WIN_FONT 	FT_SMALL
	DISP_color_code		warn_gf		; Color-code Output

	STRCPY  TXT_GF3
	movff	char_O_gradient_factor,lo		; gradient factor
	output_8
	STRCAT_PRINT  "% "
	call	DISP_standard_color
	return

DISP_show_safety_stop:
	tstfsz	safety_stop_countdown			; Countdown at zero?
	bra		DISP_show_safety_stop2			; No, show stop

	bcf		show_safety_stop				; Clear flag

	btfsc	safety_stop_active				; Displayed?
    rcall	DISP_clear_decoarea				; Yes, Clear stop
	bcf		safety_stop_active				; Clear flag
	bra		DISP_display_ndl_mask			; Show NDL again

DISP_show_safety_stop2:
	btfsc	safety_stop_active				; Displayed?
	bra		DISP_show_safety_stop3			; Yes.

	bsf		safety_stop_active				; Set flag
 
	btfsc	menubit							; Divemode menu active?
	bra		DISP_show_safety_stop3			; Yes, do not display now but countdown

	call	DISP_divemask_color				; Set Color for Divemode mask
	DISPLAYTEXT	d'227'						; Safety stop

DISP_show_safety_stop3:
	decf	safety_stop_countdown,F			; Reduce countdown
	btfsc	menubit							; Divemode menu active?
	return									; Yes, do not show
	movff	safety_stop_countdown,lo
	call	DISP_standard_color
	WIN_TOP		.80
	WIN_LEFT	.104
	WIN_FONT 	FT_MEDIUM
	WIN_INVERT	.0                      	; Init new Wordprocessor
	lfsr	FSR2,letter
	clrf	hi
	call	convert_time					; converts hi:lo in seconds to mins (hi) and seconds (lo)
	movf	hi,W
	movff	lo,hi
	movwf	lo								; exchange lo and hi
	output_99
	PUTC    ':'
	movff	hi,lo
	output_99x
	STRCAT_PRINT ""
	WIN_FONT 	FT_SMALL
	call	DISP_standard_color
	return

DISP_show_gas_change_countdown:
    btfss   gaschange_cnt_active
    return
    decf	apnoe_surface_secs,F			; Reduce countdown
    btfss   STATUS,N
    bra     DISP_show_gas_change_countdown2
    movlw   .59
    movwf   apnoe_surface_secs
    decf    apnoe_surface_mins,F
    btfss   STATUS,N
    bra     DISP_show_gas_change_countdown2
    bcf     gaschange_cnt_active            ; Clear flag
	WIN_TOP		.91
	WIN_LEFT	.64
	WIN_FONT 	FT_SMALL
	movlw	d'4'
	movwf	temp1
	call    DISP_display_clear_common_y1
    ; Reload countdown
    GETCUSTOM8  d'55'
    movwf   apnoe_surface_mins
    clrf    apnoe_surface_secs
    return

DISP_show_gas_change_countdown2:
	btfsc	menubit							; Divemode menu active?
	return									; Yes, do not show
   	movlw	color_yellow                    ; show in yellow
    call	DISP_set_color
	WIN_TOP		.91
	WIN_LEFT	.64
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0                      	; Init new Wordprocessor
	lfsr	FSR2,letter
	movff	apnoe_surface_mins,lo
    bsf     leftbind
	output_8
	PUTC    ':'
	movff	apnoe_surface_secs,lo
	output_99x
	STRCAT_PRINT ""
	call	DISP_standard_color
    bcf     leftbind
	return


;=============================================================================
; Update simulator menu with time/depth
; Note: because translations might change a bit the string length, we reprint
;       that part of the mask to be sure the numbers fit in the right places.
DISP_simulator_data:
	WIN_LEFT	.20
	WIN_FONT 	FT_SMALL
	call	DISP_standard_color

    ;---- Updates interval line ----------------------------------------------
	WIN_TOP    .35
	lfsr	    FSR2,letter
	OUTPUTTEXTH .307                    ; Interval:

	movff	    char_I_dive_interval,lo
    movf        lo,W
    bnz         DISP_simulator_data_1
    OUTPUTTEXTH .308                    ; Now
    clrf        POSTINC2                ; End buffer.
    bra         DISP_simulator_data_2

DISP_simulator_data_1:
	bsf		leftbind
	output_8
	STRCAT      TXT_0MIN5

DISP_simulator_data_2:
    call        word_processor

    ;---- Updates bottom time line -------------------------------------------
	WIN_TOP		.95
	lfsr        FSR2,letter
	OUTPUTTEXTH .277                    ; Bottom Time:

	movff	sim_btm_time,lo
	bsf		leftbind
	output_8
	STRCAT_PRINT  TXT_MIN4

    ;---- Updates depth line -------------------------------------------------
	WIN_TOP		.125
	lfsr	FSR2,letter
	OUTPUTTEXTH .278                    ; Max. Depth:

	movff	sim_btm_depth,lo
	bsf		leftbind
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0					; Enabled ?
	bra		DISP_simulator_data_3	; NO
	clrf	hi
	call	convert_meters_to_feet2	; converts meters to feet, don't check CF#61
	output_16_3
	STRCAT_PRINT  TXT_FOOT4
	bcf		leftbind
	return

DISP_simulator_data_3:
#ENDIF
	output_8
	STRCAT_PRINT  TXT_METER3

	bcf		leftbind
	return

;=============================================================================

DISP_divemode_timeout2:
	WIN_TOP		.54
	WIN_LEFT	.110
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.1
	call    DISP_warnings_color
	STRCPY  0x94					; "End of dive" icon
	movff	timeout_counter, lo
	movff	timeout_counter2, hi
	call	convert_time				; converts hi:lo in minutes to hours (hi) and minutes (lo)
	movf	hi,W
	movff	lo,hi
	movwf	lo							; exchange lo and hi
	output_99x
	PUTC    ':'
	movff	hi,lo
	output_99x
	STRCAT_PRINT " "
	bsf		timeout_display				; Set Flag
	call	DISP_standard_color
	WIN_INVERT	.0
	return

DISP_divemode_timeout:
	WIN_TOP		.54
	WIN_LEFT	.110
	WIN_FONT 	FT_SMALL
	call	DISP_standard_color
	STRCPY  0x94						; "End of dive" icon
	GETCUSTOM15	d'2'					; diveloop_timeout
	movff	lo,sub_a+0
	movff	hi,sub_a+1
	movff	timeout_counter, sub_b+0
	movff	timeout_counter2, sub_b+1	; Divemode timeout
	call	sub16						;  sub_c = sub_a - sub_b
	movff	sub_c+0,lo
	movff	sub_c+1,hi
	call	convert_time				; converts hi:lo in minutes to hours (hi) and minutes (lo)
	movf	hi,W
	movff	lo,hi
	movwf	lo							; exchange lo and hi
	output_99x
	PUTC    ':'
	movff	hi,lo
	output_99x
	STRCAT_PRINT " "
	bsf		timeout_display				; Set Flag
	return

DISP_divemode_timeout_clear:
	btfsc		dekostop_active				; Is a deco stop displayed?
	call		DISP_display_deko_mask		; Yes, redraw mask

	WIN_TOP		.54
	WIN_LEFT	.110
	movlw		d'6'
	movwf		temp1
	bcf			timeout_display				; Clear flag
	bra			DISP_display_clear_common_y1

DISP_display_velocity_graph_clr:
	WIN_BOX_BLACK	 .20, .90, .65, .75		; Clear graphic display
	bra		DISP_display_velocity			; Continue with normal output

DISP_display_velocity_graphical:
	btfss	neg_flag
	bra		DISP_display_velocity_graph_clr
	bsf		DISP_velocity_display
	; divA+0 holding the ascend speed in m/min
	movff	divA+0,hi	; Copy
	WIN_BOX_BLACK	 .20, .90, .65, .75		; Clear graphic display
	GETCUSTOM8		d'36'					; Divemode mask
    WIN_FRAME_COLOR   .20, .90, .65, .75	; Outer frame
	GETCUSTOM8		d'36'					; Divemode mask
	WIN_FRAME_COLOR   .20+.10, .90-.10, .65, .75	; Inner frames
	GETCUSTOM8		d'36'					; Divemode mask
	WIN_FRAME_COLOR   .20+.20, .90-.20, .65, .75	;
	GETCUSTOM8		d'36'					; Divemode mask
	WIN_FRAME_COLOR   .20+.30, .90-.30, .65, .75	;
	
	GETCUSTOM8		d'47'					; color_warn_celocity_mmin	
	movwf	xA+0
	clrf	xA+1
	movlw	.5
	movwf	xB+0							; Threshold for color warning (5 color normal + 2 color warning)
	clrf	xB+1
	call	div16x16						;xA/xB=xC with xA as remainder 	
	; xC+0 holds stepsize in m/min (e.g. =3 for 15m/min warning treshold)
	movff	hi,xA+0							; Velocity in m/min
	clrf	xA+1
	movff	xC+0,xB+0						; Step size
	clrf	xB+1
	call	div16x16						;xA/xB=xC with xA as remainder 	
	; xC+0 now holds amount of segments to show

	movff	hi,divA+0	; Copy back for numeric output
	movlw	d'7'
	cpfslt	xC+0
	bra		DISP_graph_vel_7
	movlw	d'6'
	cpfslt	xC+0
	bra		DISP_graph_vel_6
	movlw	d'5'
	cpfslt	xC+0
	bra		DISP_graph_vel_5
	movlw	d'4'
	cpfslt	xC+0
	bra		DISP_graph_vel_4
	movlw	d'3'
	cpfslt	xC+0
	bra		DISP_graph_vel_3
	movlw	d'2'
	cpfslt	xC+0
	bra		DISP_graph_vel_2
	movlw	d'1'
	cpfslt	xC+0
	bra		DISP_graph_vel_1
	bra		DISP_graph_vel_0			; Should not happen...

DISP_graph_vel_7:
	GETCUSTOM8		d'37'					; Color warning
    WIN_BOX_COLOR   .22, .22+.6, .67, .73	; Fill box
DISP_graph_vel_6:
	GETCUSTOM8		d'37'					; Color warning
    WIN_BOX_COLOR   .32, .32+.6, .67, .73	; Fill box
DISP_graph_vel_5:
    WIN_BOX_STD   	.42, .42+.6, .67, .73	; Fill box
DISP_graph_vel_4:
    WIN_BOX_STD   	.52, .52+.6, .67, .73	; Fill box
DISP_graph_vel_3:
    WIN_BOX_STD   	.62, .62+.6, .67, .73	; Fill box
DISP_graph_vel_2:
    WIN_BOX_STD   	.72, .72+.6, .67, .73	; Fill box
DISP_graph_vel_1:
    WIN_BOX_STD   	.82, .82+.6, .67, .73	; Fill box
DISP_graph_vel_0:

DISP_display_velocity:
	ostc_debug	'v'		; Sends debug-information to screen if debugmode active
#IFNDEF IMPERIAL
	WIN_TOP		.90
#ELSE
	WIN_TOP		.91
#ENDIF
	WIN_LEFT	.0
	WIN_FONT 	FT_SMALL
	DISP_color_code		warn_velocity		; Color code output
	lfsr	FSR2,letter
	movlw	'-'
	btfsc	neg_flag
	movlw	'+'
	movwf	POSTINC2
	movff	divA+0,lo
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0                  ; Enabled ?
	bra     DISP_display_velocity2  ; NO
	clrf    hi
	call    convert_meters_to_feet2 ; converts meters to feet, don't check CF#61
	output_16_3
	OUTPUTTEXTH d'384'              ; ft/min
	bra     DISP_display_velocity3
 
DISP_display_velocity2:
#ENDIF
	output_99
	OUTPUTTEXT	d'83'			; m/min
	call	word_processor
#IFDEF IMPERIAL
DISP_display_velocity3:
#ENDIF
	call	DISP_standard_color
	bsf		DISP_velocity_display
	return

DISP_display_velocity_clear:
	GETCUSTOM8	d'60'			; use graphic velocity (=1)?
	movwf	lo
	movlw	d'1'
	cpfseq	lo					; =1?
	bra		DISP_display_velocity_clear1	; No, clear text display
	WIN_BOX_BLACK	 .20, .90, .65, .75		; Clear graphic display

DISP_display_velocity_clear1:
#IFNDEF IMPERIAL
	movlw	d'8'
#ELSE
	movlw	d'10'
#ENDIF
	movwf	temp1
	WIN_TOP		.90
	WIN_LEFT	.0
	bcf		DISP_velocity_display
	bra		DISP_display_clear_common_y1

DISP_display_wait_clear:
    WIN_BOX_BLACK   .0, .25, .0, .159		;top, bottom, left, right
	return

DISP_display_clear_common_y2:				; Clears with y-scale=2
	WIN_FONT 	FT_MEDIUM
	bra		DISP_display_clear_common1

DISP_display_clear_common_y1:				; Clears with y-scale=1
	WIN_FONT 	FT_SMALL
DISP_display_clear_common1:
	lfsr	FSR2,letter
DISP_display_clear_common2:
	PUTC    ' '
	decfsz	temp1,F
	bra 	DISP_display_clear_common2
	call	word_processor
	WIN_FONT 	FT_SMALL
	return


DISP_diveclock:
	call	DISP_divemask_color	; Set Color for Divemode mask
	DISPLAYTEXT		d'255'			; Time:
	call	DISP_standard_color

DISP_diveclock2:
	WIN_TOP		.192
	WIN_LEFT	.123
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color
	lfsr	FSR2,letter
	movff	hours,lo
	output_99x
	PUTC    ':'
	movff	mins,lo
	output_99x
	call	word_processor

DISP_diveclock3:                    ; Update end of divetime only
    return
	WIN_TOP		.216
	WIN_LEFT	.116
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0
	call	DISP_standard_color

    btfss	dekostop_active         ; Already in nodeco mode ?
	bra     DISP_diveclock4         ; No, overwrite with some spaces

	STRCPY  0x94					; "End of dive" icon
    movff	hours,WREG
    mullw   .60
    movf    mins,W
    addwf   PRODL
    movlw   .0
    addwfc  PRODH
	movff	PRODL, lo
	movff	PRODH, hi

    ; Add TTS
    movff	int_O_ascenttime+0,WREG     ; TTS
    addwf   lo,F
	movff	int_O_ascenttime+1,WREG     ; TTS is 16bits
    addwfc  hi,F

	call	convert_time				; converts hi:lo in minutes to hours (hi) and minutes (lo)
	movf	hi,W
	movff	lo,hi
	movwf	lo							; exchange lo and hi
	output_99x
	PUTC    ':'
	movff	hi,lo
	output_99x
	STRCAT_PRINT ""
	return

DISP_diveclock4:
    STRCPY_PRINT "      "
    return

DISP_clock:
	ostc_debug	'c'
	WIN_TOP		.50
	WIN_LEFT	.1
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color
	lfsr	FSR2,letter
	movff	hours,lo
	output_99x
	PUTC    ':'
	movff	mins,lo
	output_99x
	PUTC    ':'
	movff	secs,lo
	output_99x
	STRCAT_PRINT ""
	return

DISP_interval:
	WIN_TOP		.75
	WIN_LEFT	.90
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color
	lfsr	FSR2,letter

	movff	surface_interval+0,lo
	movff	surface_interval+1,hi
	call	convert_time			; lo=mins, hi=hours

	movf	hi,W
	movff	lo,hi
	movwf	lo					; exchange lo and hi
	output_99x
	PUTC    ':'
	movff	hi,lo
	output_99x
	STRCAT_PRINT " "
	return


DISP_show_deco_gas:                 ; Show the next decogas
	WIN_LEFT	.90
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
    call    DISP_divemask_color     ; Set Color for Divemode mask
    DISPLAYTEXTH .270               ; Decogas

DISP_show_deco_gas1:
	WIN_LEFT	.90
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
    call    DISP_standard_color
	WIN_TOP		.192
    movff       char_O_deco_gas+0,lo
    tstfsz      lo              ; =0?
    bra         $+4             ; No
    bra     DISP_show_deco_gas2
    incf        lo,F            ;+1
    STRCPY  TXT_GAS1            ; "G"
    bsf     leftbind
    output_8
    STRCAT  TXT_AT4             ; " at "
    movlw   .27                 ; 28=Gas 1
    addwf   lo,W
    movwf   EEADR
    clrf    EEADRH              ; Get Change depth
    call    read_eeprom
    movff   EEDATA,lo
    tstfsz      lo              ; =0?
    bra     $+4                 ; No
    bra     DISP_show_deco_gas2
    output_8                    ; Change depth
    bcf     leftbind
    STRCAT  "m "
    clrf    WREG
    movff   WREG,letter+.9       ; Limit to 9 chars
    STRCAT_PRINT  ""

	WIN_TOP		.216
    movff       char_O_deco_gas+1,lo
    tstfsz      lo              ; =0?
    bra     $+4                 ; No
    bra     DISP_show_deco_gas3
    incf        lo,F            ;+1
    STRCPY  TXT_GAS1            ; "G"
    bsf     leftbind
    output_8
    STRCAT  TXT_AT4             ; " at "
    movlw   .27                 ; 28=Gas 1
    addwf   lo,W
    movwf   EEADR
    clrf    EEADRH              ; Get Change depth
    call    read_eeprom
    movff   EEDATA,lo
    tstfsz      lo              ; =0?
    bra     $+4                 ; No
    bra     DISP_show_deco_gas3
    output_8                    ; Change depth
    bcf     leftbind
    STRCAT  "m "
    clrf    WREG
    movff   WREG,letter+.9       ; Limit to 9 chars
    STRCAT_PRINT  ""
    return

DISP_show_deco_gas2:
    WIN_BOX_BLACK   .192, .239, .90, .159		;top, bottom, left, right
    return

DISP_show_deco_gas3:
    WIN_BOX_BLACK   .216, .239, .90, .159		;top, bottom, left, right
    return

DISP_show_gf_customview:
	WIN_LEFT	.93
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
    call    DISP_divemask_color     ; Set Color for Divemode mask
    DISPLAYTEXTH .268               ;"Gradient Factors"

   	GETCUSTOM8	d'64'               ; Set to grey
	call	DISP_set_color
    btfss   use_aGF
    call    DISP_standard_color

	WIN_TOP		.192
    STRCPY  TXT_GF3             ; "GF:"
    GETCUSTOM8  d'32'            ; GF_lo
    movwf   lo
	bsf		leftbind
	output_8
	STRCAT  "/"
    GETCUSTOM8  d'33'            ; GF_hi
    movwf   lo
    output_8
    STRCAT_PRINT  ""

   	GETCUSTOM8	d'64'               ; Set to grey
	call	DISP_set_color
    btfsc   use_aGF
    call    DISP_standard_color

	WIN_TOP		.216
    STRCPY  TXT_aGF4             ; "aGF:"
    GETCUSTOM8  d'67'            ; aGF_lo
    movwf   lo
    bsf		leftbind
	output_8
	STRCAT  "/"
    GETCUSTOM8  d'68'            ; aGF_hi
    movwf   lo
	output_8
    STRCAT_PRINT  ""
    bcf		leftbind

    call    DISP_standard_color
    return

DISP_show_cf11_cf12_cf29:; Display saturations/desaturation multiplier and last deco in the customview field
	WIN_TOP		.25
	WIN_LEFT	.90
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color
	STRCPY  TXT_BSAT5

	GETCUSTOM8	d'11'
	movwf	lo
	bsf		leftbind
	output_8
	STRCAT_PRINT  "%"

	WIN_TOP		.50
	STRCPY  TXT_BDES5

	GETCUSTOM8	d'12'
	movwf	lo
	bsf		leftbind
	output_8
	STRCAT_PRINT  "%"

DISP_show_cf11_cf12_cf29_2:
	WIN_TOP		.75
    STRCPY  TXT_LAST5
	GETCUSTOM8	d'29'
	movwf	lo
	bsf		leftbind
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0						; Enabled ?
	bra		DISP_show_cf11_cf12_cf29_3	; NO
	clrf	hi
	call	convert_meters_to_feet2
	output_16
	STRCAT_PRINT  TXT_FOOT2
	bcf		leftbind
	return

DISP_show_cf11_cf12_cf29_3:
#ENDIF
	output_8
	STRCAT_PRINT  TXT_METER1

	bcf		leftbind
	return

DISP_show_cf32_cf33_cf62_cf63:	; Display GF_LOW, GF_HIGH, pSCR ratio and drop in the customview field
	WIN_TOP		.25
	WIN_LEFT	.90
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color
	GETCUSTOM8	d'32'				; GF_lo
	movwf	lo
    STRCPY  TXT_GFLO6
	bsf		leftbind
	output_8
	STRCAT_PRINT  "%"

	WIN_TOP		.50
	GETCUSTOM8	d'33'				; GF_hi
	movwf	lo
    STRCPY  TXT_GFHI6
	bsf		leftbind
	output_8
	STRCAT_PRINT  "%"

	WIN_TOP		.75
	lfsr        FSR2,letter
	GETCUSTOM8  d'62'		; O2 Drop in percent
	movwf		lo
	bsf			leftbind
	output_8

	STRCAT		 "% 1/"
	GETCUSTOM8  d'63'		; Counter lung ratio in 1/X
	movwf		lo
	output_8
	bcf			leftbind
    STRCAT_PRINT ""
	return



DISP_show_cf32_cf33_cf29:; Display GF_LOW, GF_HIGH and last deco in the customview field
	WIN_TOP		.25
	WIN_LEFT	.90
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color
	GETCUSTOM8	d'32'				; GF_lo
	movwf	lo

    STRCPY  TXT_GFLO6
	bsf		leftbind
	output_8
	STRCAT_PRINT  "%"

	WIN_TOP		.50
	GETCUSTOM8	d'33'				; GF_hi
	movwf	lo
    STRCPY  TXT_GFHI6
	bsf		leftbind
	output_8
	STRCAT_PRINT  "%"

	bra		DISP_show_cf11_cf12_cf29_2		; Display CF29 in the third row and RETURN


DISP_logbook_cursor:

DISP_menu_cursor:
    WIN_BOX_BLACK   .35, .239, .0, .16		;top, bottom, left, right

	WIN_LEFT	.0
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color

	movff	menupos,temp1
	dcfsnz	temp1,F
	movlw	d'35'
	dcfsnz	temp1,F
	movlw	d'65'
	dcfsnz	temp1,F
	movlw	d'95'
	dcfsnz	temp1,F
	movlw	d'125'
	dcfsnz	temp1,F
	movlw	d'155'
	dcfsnz	temp1,F
	movlw	d'185'
	dcfsnz	temp1,F
	movlw	d'215'
	
	movff	WREG,win_top
	STRCPY_PRINT "\xB7"
	return

DISP_menu_mask:
    call	DISP_divemask_color
	DISPLAYTEXT	.5			; Menu:
	call	DISP_standard_color
#IFDEF CCR_CTRL     
    ; Reorder Menu ittems
	DISPLAYTEXT .110		; CCR Setup Menu
	DISPLAYTEXT .6			; Logbook
	DISPLAYTEXT .7			; Gas Setup
#ELSE
	DISPLAYTEXT .6			; Logbook
	DISPLAYTEXT .7			; Gas Setup
	DISPLAYTEXT .9			; Reset all
#ENDIF
	DISPLAYTEXT .10			; Setup...
	DISPLAYTEXT	.142		; More...
	DISPLAYTEXT .11			; Exit

; Write OSTC serial in Main Menu
	WIN_TOP		.215
	WIN_LEFT	.47
	GETCUSTOM8	d'64'					; Write header in blue when
    call    DISP_set_color              ; compiled in DEBUG mode...
	lfsr	FSR2,letter
	OUTPUTTEXTH		d'262'              ; "OSTC "
	clrf	EEADRH
	clrf	EEADR                       ; Get Serial number LOW
	call	read_eeprom                 ; read byte
	movff	EEDATA,lo
	incf	EEADR,F                     ; Get Serial number HIGH
	call	read_eeprom                 ; read byte
	movff	EEDATA,hi
	bsf		leftbind
	output_16
	call	word_processor
	call	DISP_standard_color
	return	

DISP_setup_menu_mask:
    call	DISP_divemask_color
	DISPLAYTEXT	.98			; Setup Menu:
    call    DISP_standard_color
	DISPLAYTEXT .99			; Custom FunctionsI
	DISPLAYTEXT	.153		; Custom FunctionsII
	DISPLAYTEXTH .295		; Custom FunctionsIII
	DISPLAYTEXT .100		; Decotype:
	DISPLAYTEXT	.142		; More...
	DISPLAYTEXT .11			; Exit
	return	

DISP_ccr_setup_menu_mask:
    call	DISP_divemask_color
    DISPLAYTEXT	.111		; CCR Setup Menu
	call    DISP_standard_color
#IFDEF CCR_CTRL
	DISPLAYTEXTH .387		; CCR Controller Control
#ENDIF
	DISPLAYTEXT .229        ; Diluent Setup
	DISPLAYTEXT	.230		; Setpoint Setup
    DISPLAYTEXT .234        ; SP Mode:
#IFDEF CCR_CTRL	
	DISPLAYTEXTH .389		; CCR Controller Setup
#ENDIF
	DISPLAYTEXT .11			; Exit
	return


DISP_more_setup_menu_mask:
    call	DISP_divemask_color
	DISPLAYTEXTH	.258	; Setup Menu 2:
	call	DISP_standard_color
	DISPLAYTEXTH	.257	; Date format:
	DISPLAYTEXT		.129	; Debug: 
	DISPLAYTEXT		.187	; Show License
	DISPLAYTEXTH 	.276	; Salinity:
	DISPLAYTEXTH	.280 	; Brightness:
	DISPLAYTEXT .11			; Exit
	return	

DISP_more_menu_mask:
    call	DISP_divemask_color
	DISPLAYTEXT	.144		; Menu 2:
	call	DISP_standard_color
	DISPLAYTEXT .8			; Set Time
#IFDEF CCR_CTRL
	DISPLAYTEXT	.9  		; Reset Menu
#ELSE
	DISPLAYTEXT	.110		; CCR Setup Menu
#ENDIF
	DISPLAYTEXT	.113		; Battery Info
	DISPLAYTEXT	.247		; Simulator
	DISPLAYTEXTH .287		; Altimeter
	DISPLAYTEXT .11			; Exit
	return

DISP_reset_menu_mask:
    call	DISP_divemask_color
	DISPLAYTEXT	.28				; Reset Menu
	call	DISP_standard_color
	DISPLAYTEXT	.21				; Cancel Reset
	DISPLAYTEXT	.245			; Reset CF,Gases & Deco
	DISPLAYTEXTH .284			; Reset Logbook
	DISPLAYTEXTH .285			; Reboot OSTC
	DISPLAYTEXTH .286			; Reset Decodata
	DISPLAYTEXT .11			; Exit
	return

DISP_simulator_mask:
    call	DISP_divemask_color
	DISPLAYTEXT	.248		; OSTC Simulator
	call	DISP_standard_color
    DISPLAYTEXTH    .307                ; Interval:
	DISPLAYTEXT	    .249                ; Start Dive
	DISPLAYTEXTH	.277                ; Bottom Time:
	DISPLAYTEXTH	.278                ; Max. Depth:
	DISPLAYTEXTH	.279                ; Calculate Deco
	DISPLAYTEXT     .11                 ; Exit
	return
	
DISP_temp_surfmode:
	ostc_debug	'e'
    SAFE_2BYTE_COPY    temperature, last_temperature
	WIN_TOP		.100
	WIN_LEFT	.1
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0                      ; Init new Wordprocessor
	call	DISP_standard_color

	movff	last_temperature+1,hi
	movff	last_temperature+0,lo
	lfsr	FSR2,letter
#IFDEF IMPERIAL
	call    convert_celsius_to_fahrenheit
#ENDIF
	call	DISP_convert_signed_temperature	; converts lo:hi into signed-short and adds '-' to POSTINC2 if required
	movlw	d'3'
	movwf	ignore_digits
	bsf		leftbind			; left orientated output
	output_16dp	d'2'
	bcf		leftbind
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0                          ; Enabled ?
	bra DISP_temp_surfmode1                         ; NO
	STRCAT_PRINT  "°F "
	return
DISP_temp_surfmode1:
#ENDIF
	STRCAT_PRINT  "°C "
	return

DISP_temp_divemode:
	ostc_debug	'u'		; Sends debug-information to screen if debugmode active

; temperature
    SAFE_2BYTE_COPY temperature, last_temperature

	WIN_TOP		.216
	WIN_LEFT	.50
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color

	movff	last_temperature+1,hi
	movff	last_temperature+0,lo

	lfsr	FSR2,letter
#IFDEF IMPERIAL
	call    convert_celsius_to_fahrenheit
#ENDIF
	call	DISP_convert_signed_temperature	; converts lo:hi into signed-short and adds '-' to POSTINC2 if required
	movlw	d'3'
	movwf	ignore_digits
	bsf		leftbind			; left orientated output
	output_16dp	d'2'
	bcf		leftbind
    STRCAT "° "
    clrf    WREG				; Allow up to 5 chars to avoid
    movff   WREG,letter+5		; collision with sat graphs
    call    word_processor
	return

DISP_show_ppO2:					; Show ppO2 (ppO2 stored in xC)
	ostc_debug	't'		; Sends debug-information to screen if debugmode active
	WIN_TOP		.117
	WIN_LEFT	.0
	WIN_FONT 	FT_SMALL
	DISP_color_code		warn_ppo2		; Color-code output (ppO2 stored in xC)
    STRCPY  TXT_PPO2_5

; Check very high ppO2 manually
	tstfsz		xC+2					; char_I_O2_ratio * p_amb/10 > 65536, ppO2>6,55bar?
	bra			DISP_show_ppO2_3		; Yes, display fixed Value!

	movff	xC+0,lo
	movff	xC+1,hi
	bsf		ignore_digit4
	output_16dp	d'1'
	bcf		ignore_digit4
DISP_show_ppO2_2:
    STRCAT_PRINT " "
	call	DISP_standard_color
	return

DISP_show_ppO2_3:
    STRCAT  ">6.6"
	bra		DISP_show_ppO2_2

DISP_show_ppO2_clear:					; Clear ppO2
	movlw	d'10'
	movwf	temp1
	WIN_TOP		.117
	WIN_LEFT	.0
	call	DISP_display_clear_common_y1
	return

DISP_active_gas_clear:					; clears active gas!
	WIN_TOP		.192
	WIN_LEFT	.50
	movlw	d'5'
	movwf	temp1
#IFDEF	SEAWOOCH
	goto		DISP_display_clear_common_y1; also returns!
#ELSE
	bra		DISP_display_clear_common_y1; also returns!
#ENDIF

DISP_active_gas_divemode:				; Displays current gas (e.g. 40/20) if a) He>0 or b) O2>Custom9
	btfsc	FLAG_apnoe_mode				; Ignore in Apnoe mode
	return
#IFDEF BOTTOM
; Disable Active Gas display in Gauge mode
	btfsc	gauge_mode					
	return
#ENDIF
	WIN_INVERT	.0					; Init new Wordprocessor	
	call	DISP_active_gas_divemode_show	; Show gas (Non-Inverted in all cases)

	btfss	better_gas_available	;=1: A better gas is available and a gas change is advised in divemode
	return					; Done.

; Check if Gas Output should blink when a better gas is available...
	GETCUSTOM8	d'42'			; =1 if gas should blink
	movwf	lo
	movlw	d'1'
	cpfseq	lo					; =1?
	return						; No, Done.

	btg		blinking_better_gas		; Toggle blink bit...
	btfss	blinking_better_gas		; blink now?
	return							; No, Done.
	movlw	color_yellow			; Blink in yellow
    call	DISP_set_color
	WIN_INVERT	.1					; Init new Wordprocessor	
	call	DISP_active_gas_divemode_show1	; Show gas (Non-Inverted in all cases)
	WIN_INVERT	.0					; Init new Wordprocessor	
	call	DISP_standard_color
	return							; Done.

DISP_active_gas_divemode_show:
	call	DISP_standard_color
DISP_active_gas_divemode_show1:
	ostc_debug	's'		; Sends debug-information to screen if debugmode active
; gas
	WIN_TOP		.192
	WIN_LEFT	.50
	WIN_FONT 	FT_SMALL

	movlw	d'100'						; 100% in the tank
	movff	char_I_N2_ratio,lo			; minus N2
	bsf		STATUS,C					; set borrow bit
	subfwb	lo,W
	movff	char_I_He_ratio,lo			; minus He
	bsf		STATUS,C					; set borrow bit
	subfwb	lo,F						; =% O2
	GETCUSTOM8		d'9'				; get oxygen treshold
	movff	char_I_He_ratio,hi			; He ratio
	cpfsgt	lo
	bra		DISP_active_gas_divemode2		; Check He
	bra		DISP_active_gas_divemode3		; Skip He check, display gas
	
DISP_active_gas_divemode2:
	tstfsz	hi							; He = 0 %
	bra		DISP_active_gas_divemode3	; display gas

    call    DISP_warnings_color         ; O2 below treshold, He=0 : Bad stuff !
    bra     DISP_active_gas_divemode4	

DISP_active_gas_divemode3:
	movlw	d'21'
	cpfseq	lo				; Air? (O2=21%)
	bra		DISP_active_gas_divemode4 ; No!
	tstfsz	hi				; Air? (He=0%)
	bra		DISP_active_gas_divemode4 ; No!
	
							; Yes, display "Air" instead of 21/0
	lfsr	FSR2,letter
	OUTPUTTEXTH		d'264'			;"Air  "
	PUTC	' '
    clrf    WREG				; Allow up to 5 chars to avoid
    movff   WREG,letter+5		; collision with sat graphs
	bcf		leftbind
	call	word_processor
DISP_active_better_gas:
	WIN_TOP		.192
	WIN_LEFT	.43
	WIN_FONT 	FT_SMALL
	lfsr	FSR2,letter
	movlw	' '
	btfsc	better_gas_available	;=1: A better gas is available and a gas change is advised in divemode
	movlw	'*'
	movwf	POSTINC2
	call	word_processor
	return

DISP_active_gas_divemode4:
	lfsr	FSR2,letter
	bsf		leftbind			; left orientated output
	output_8					; O2 ratio is still in "lo"
	PUTC    '/'
	movff	char_I_He_ratio,lo		; copy He ratio into lo
	output_8
	PUTC	' '
    clrf    WREG				; Allow up to 5 chars to avoid
    movff   WREG,letter+5		; collision with sat graphs
	bcf		leftbind
	call	word_processor
	rcall	DISP_active_better_gas	; show *, if required
    call    DISP_standard_color ; Back to normal (if O2<21 and He=0)
	return

;-----------------------------------------------------------------------------
; Set color to grey when gas is inactive
; Inputs: WREG : gas# (0..4)
; Trashes: lo
; New v1.44se
DISP_grey_inactive_gas:
	movwf	lo		                    ; copy gas number 0-4
	incf	lo,F				        ; 1-5

    read_int_eeprom		d'33'       	; Get First gas (1-5)
    movf    EEDATA,W            
    subwf   lo,W                        ; Compare with current
    bz      DISP_white_gas              ; First is always on.

    movlw   .28-1                       ; Depth for gas# is at idx+28
    addwf   lo,W
    movwf   EEADR                       ; address in EEPROM.
    call    read_eeprom                 ; Read depth
    clrf    WREG                
    cpfsgt  EEDATA                      ; is depth > 0 ?
    bra     DISP_grey_gas

    clrf    EEADRH                      ; Lower page of EEPROM.
    read_int_eeprom		d'27'	        ; read flag register
DISP_grey_inactive_gas1:
	rrcf	EEDATA			            ; roll flags into carry
	decfsz	lo,F			            ; max. 5 times...
	bra		DISP_grey_inactive_gas1
	bnc		DISP_grey_gas               ; test carry

DISP_white_gas:
	GETCUSTOM8	d'35'		            ;movlw	color_white	
	goto	DISP_set_color	            ; grey out inactive gases!
    ; return

DISP_grey_gas:
DISP_grey:
	GETCUSTOM8	d'64'					;movlw	color_grey
	goto	DISP_set_color	            ; grey out inactive gases!
    ; return

DISP_bailoutgas:        ; Show the first bailout gas
	WIN_TOP		.25
	WIN_LEFT	.90
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color
    lfsr	FSR2,letter
	OUTPUTTEXT	.137			; Bailout
    call	word_processor
	WIN_TOP		.50
	WIN_LEFT	.90
	read_int_eeprom 	d'33'	; Read start gas (1-5)
	movff	EEDATA,hi       ; Store start gas
    bsf		leftbind
    lfsr	FSR2,letter
	STRCPY  TXT_GAS1
	movff	hi,lo			; copy gas number
	output_8				; display gas number
	STRCAT  ": "
    movlw   .4
    mulwf   hi              ; 1-5
    movf    PRODL,W
	addlw   .2              ; Gas #x: %O2 - Set address in internal EEPROM
    movwf   EEADR
	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	output_8				; outputs into Postinc2!
	PUTC    '/'
	incf	EEADR,F			; Gas #hi: %He - Set address in internal EEPROM
	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	output_8				; outputs into Postinc2!
    bcf		leftbind
    call	word_processor
    return

DISP_bailoutlist:       ; Show the bailout list:
    bra     DISP_pre_dive_screen2

;-----------------------------------------------------------------------------
; Display Pre-Dive Screen

DISP_pre_dive_screen:			
	; List active gases/Setpoints

	btfsc	FLAG_const_ppO2_mode		; in ppO2 mode?
	bra		DISP_pre_dive_screen3		; Yes, display SetPoint/Sensor result list

DISP_pre_dive_screen2:
	ostc_debug	'm'		; Sends debug-information to screen if debugmode active

	WIN_LEFT	.90
	WIN_FONT	FT_SMALL
	bsf		leftbind
	movlw	d'2'
	movwf	wait_temp			; here: stores eeprom address for gas list
	movlw	d'0'
	movwf	waitms_temp         ; here: stores row for gas list
	clrf	hi					; here: Gas counter

DISP_pre_dive_screen2_loop:
	incf	hi,F				; Increase Gas
	movlw	d'4'
	addwf	wait_temp,F			; Increase eeprom address for gas list
	
	STRCPY  TXT_GAS1
	movff	hi,lo			; copy gas number
	output_8				; display gas number
	STRCAT  ": "
	movff	wait_temp, EEADR; Gas #hi: %O2 - Set address in internal EEPROM
	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	output_8				; outputs into Postinc2!
	PUTC    '/'
	incf	EEADR,F			; Gas #hi: %He - Set address in internal EEPROM
	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	output_8				; outputs into Postinc2!

    decf    hi,W            ; Gas # in 0..4
	call    DISP_grey_inactive_gas

	read_int_eeprom 	d'33'	; Read start gas (1-5)
	movf	EEDATA,W
	cpfseq	hi                  ; Current Gas the start gas?
	bra		DISP_pre_dive_screen2a  ; Yes
	bra		DISP_pre_dive_screen2b  ; No

DISP_pre_dive_screen2a:
	movlw	d'25'
	addwf	waitms_temp,F		; Increase row
	WIN_LEFT	.90
	movff	waitms_temp,win_top ; Set Row
	call	word_processor      ; Display gas

DISP_pre_dive_screen2b:
	movlw	d'5'                ; list all four (remaining) gases
	cpfseq	hi                  ; All gases shown?
	bra		DISP_pre_dive_screen2_loop	; No
	
	return						; No, return (OC mode)

DISP_pre_dive_screen3:	
	WIN_LEFT	.90
	WIN_FONT	FT_SMALL
	bsf		leftbind
	call    DISP_standard_color

	; list three SP in Gaslist
	movlw	d'35'				; 36 = current SP position in EEPROM
	movwf	wait_temp			; here: stores eeprom address for gas list
	movlw	d'0'
	movwf	waitms_temp			; here: stores row for gas list
	clrf 	apnoe_mins          ; here: SP counter

DISP_pre_dive_screen3_loop:
	incf	wait_temp,F			; EEPROM address
	incf	apnoe_mins,F	    ; Increase SP

	movlw	d'25'
	addwf	waitms_temp,F		; Increase row
	WIN_LEFT	.90
	movff	waitms_temp,win_top ; Set Row
	
	STRCPY  TXT_SP2
	movff	apnoe_mins,lo       ; copy gas number
	output_8			        ; display gas number
	STRCAT  ": "
	movff	wait_temp, EEADR    ; SP #hi position
	call	read_eeprom		    ; get byte (stored in EEDATA)
	movff	EEDATA,lo		    ; copy to lo
	clrf	hi
	output_16dp	d'3'		    ; outputs into Postinc2!
	call	word_processor	

	movlw	d'3'		        ; list all three SP
	cpfseq	apnoe_mins          ; All gases shown?
	bra		DISP_pre_dive_screen3_loop	;no


    call    get_first_diluent           ; Read first diluent into lo(O2) and hi(He)
	WIN_LEFT	.90
	WIN_TOP		.100
	STRCPY  TXT_DIL4
	output_8				; O2 Ratio
	PUTC    '/'
	movff	hi,lo
	output_8				; He Ratio
	call	word_processor		

	bcf		leftbind
	return				; Return (CC Mode)

DISP_active_gas_surfmode:				; Displays start gas/SP 1
	ostc_debug	'q'		; Sends debug-information to screen if debugmode active
		
	btfsc	FLAG_apnoe_mode				; In Apnoe mode?
	return								; Yes, return

	btfsc	gauge_mode					; In Gauge mode?
	return								; Yes, return

	btfss	FLAG_const_ppO2_mode	; are we in const. ppO2 mode?	
	bra		DISP_active_gas_surfmode2	; No, display gases

; In CC Mode
	WIN_TOP		.135
	WIN_LEFT	.90
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color

	lfsr	FSR2,letter
	read_int_eeprom	d'36'
	movff	EEDATA,lo				; copy to lo
	clrf	hi
	output_16dp	d'3'		; outputs into Postinc2!
	STRCAT_PRINT  TXT_BAR3

    bsf     leftbind
    call    get_first_diluent           ; Read first diluent into lo(O2) and hi(He)
	WIN_TOP		.160
	WIN_LEFT	.104
	lfsr	FSR2,letter
	output_8				; O2 Ratio
	PUTC    '/'
	movff	hi,lo
	output_8				; He Ratio
	STRCAT_PRINT  ""
	bcf		leftbind
	return								; Done.

DISP_active_gas_surfmode2:
	clrf	EEADRH                  ; Select EEPROM lower page.
	read_int_eeprom		d'33'       ; Get First gas (1-5)
    movff   EEDATA,hi               ; into register hi

	WIN_TOP		.175
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0                      ; Init new Wordprocessor
    movlw   .0
	call	DISP_grey_inactive_gas			; Sets Greyvalue for inactive gases
	WIN_LEFT	.115
    STRCPY_PRINT    "1"
    movlw   .1
    cpfseq  hi                              ; Is "first gas"?
    bra     DISP_active_gas_surfmode3       ; No, skip box
	WIN_FRAME_STD   .174, .196, .114, .122    ;top, bottom, left, right
DISP_active_gas_surfmode3:
    movlw   .1
	call	DISP_grey_inactive_gas			; Sets Greyvalue for inactive gases
	WIN_LEFT	.124
    STRCPY_PRINT    "2"
    movlw   .2
    cpfseq  hi                              ; Is "first gas"?
    bra     DISP_active_gas_surfmode4       ; No, skip box
	WIN_FRAME_STD   .174, .196, .123, .131    ;top, bottom, left, right
DISP_active_gas_surfmode4:
    movlw   .2
	call	DISP_grey_inactive_gas			; Sets Greyvalue for inactive gases
	WIN_LEFT	.133
    STRCPY_PRINT    "3"
    movlw   .3
    cpfseq  hi                              ; Is "first gas"?
    bra     DISP_active_gas_surfmode5       ; No, skip box
	WIN_FRAME_STD   .174, .196, .132, .140    ;top, bottom, left, right
DISP_active_gas_surfmode5:
    movlw   .3
	call	DISP_grey_inactive_gas			; Sets Greyvalue for inactive gases
	WIN_LEFT	.142
    STRCPY_PRINT    "4"
    movlw   .4
    cpfseq  hi                              ; Is "first gas"?
    bra     DISP_active_gas_surfmode6       ; No, skip box
	WIN_FRAME_STD   .174, .196, .141, .149    ;top, bottom, left, right
DISP_active_gas_surfmode6:
    movlw   .4
	call	DISP_grey_inactive_gas			; Sets Greyvalue for inactive gases
	WIN_LEFT	.151
    STRCPY_PRINT    "5"
    movlw   .5
    cpfseq  hi                              ; Is "first gas"?
    bra     DISP_active_gas_surfmode7       ; No, skip box
	WIN_FRAME_STD   .174, .196, .150, .158    ;top, bottom, left, right
DISP_active_gas_surfmode7:

	WIN_TOP		.130
	WIN_LEFT	.100
	WIN_FONT 	FT_MEDIUM
	call	DISP_standard_color

	read_int_eeprom 	d'33'			; Read byte (stored in EEDATA)
	movff	EEDATA,active_gas			; Read start gas (1-5)

	decf	active_gas,W				; Gas 0-4
	mullw	d'4'
	movf	PRODL,W			
	addlw	d'7'						; = address for He ratio
	movwf	EEADR
	call	read_eeprom					; Read He ratio
	movff	EEDATA,char_I_He_ratio		; And copy into hold register

	decf	active_gas,W				; Gas 0-4
	mullw	d'4'
	movf	PRODL,W			
	addlw	d'6'						; = address for O2 ratio
	movwf	EEADR
	call	read_eeprom					; Read O2 ratio
	movff	EEDATA, char_I_O2_ratio		; O2 ratio
	movff	char_I_He_ratio, wait_temp	; copy into bank1 register
	bsf		STATUS,C					; Borrow bit
	movlw	d'100'						; 100%
	subfwb	wait_temp,W					; minus He
	bsf		STATUS,C					; Borrow bit
	subfwb	EEDATA,F					; minus O2
	movff	EEDATA, char_I_N2_ratio		; = N2!

	movlw	d'100'						; 100% in the tank
	movff	char_I_N2_ratio,lo			; minus N2
	bsf		STATUS,C					; set borrow bit
	subfwb	lo,W
	movff	char_I_He_ratio,lo			; minus He
	bsf		STATUS,C					; set borrow bit
	subfwb	lo,F						; =% O2

	movff	char_I_He_ratio,hi			; Copy into Bank1 register

	movlw	d'21'
	cpfseq	lo							; Air? (O2=21%)
	bra		DISP_active_gas_surfmode8 	; No!
	tstfsz	hi							; Air? (He=0%)
	bra		DISP_active_gas_surfmode8 	; No!
	
							; Yes, display "Air" instead of 21/0
	DISPLAYTEXTH		d'265'		;"Air  ", y-scale=2
	return								; Done.

DISP_active_gas_surfmode8:
	lfsr	FSR2,letter
	bsf		leftbind			; left orientated output
	output_99					; O2 ratio is still in "lo"
	movff	char_I_He_ratio,lo	; copy He ratio into lo
	tstfsz	lo					; He>0?
	bra		DISP_active_gas_surfmode9	; Yes.
	bra		DISP_active_gas_surfmode10	; No, skip He
DISP_active_gas_surfmode9:
	PUTC    '/'
	output_99
DISP_active_gas_surfmode10:
	bcf		leftbind
	call	word_processor

	rcall	DISP_mainscreen_show_nx
	tstfsz	lo					; He>0?
	rcall	DISP_mainscreen_show_tx	; Yes
	return								; Done.

DISP_mainscreen_show_tx:
	WIN_LEFT	.85
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
   	WIN_TOP		.127

   	STRCPY_PRINT TXT_TX1
  	WIN_TOP		.148
	STRCPY_PRINT TXT_TX2
	return
DISP_mainscreen_show_nx:
	WIN_LEFT	.85
   	WIN_TOP		.127
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor

   	STRCPY_PRINT TXT_NX1
  	WIN_TOP		.148
	STRCPY_PRINT TXT_NX2
	return


DISP_confirmbox:
    WIN_BOX_BLACK   .68, .146, .34, .101		;top, bottom, left, right
	WIN_FRAME_STD   .70, .144, .35, .100

    call	DISP_divemask_color
	DISPLAYTEXT	.143			; Confirm:
    call	DISP_standard_color
	DISPLAYTEXT	.145			; Cancel
	DISPLAYTEXT	.146			; OK!

	movlw		d'1'
	movwf		menupos

DISP_confirmbox2:
    WIN_BOX_BLACK   .96, .143, .39, .51		;top, bottom, left, right

	movff	menupos,temp1
	movlw	d'96'
	dcfsnz	temp1,F
	movlw	d'96'
	dcfsnz	temp1,F
	movlw	d'120'
	movff	WREG,win_top
	WIN_LEFT	.39
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color

    STRCPY_PRINT "\xB7"				; Cursor

	bcf			sleepmode					; clear some flags
	bcf			menubit2
	bcf			menubit3
	bcf			switch_right
	bcf			switch_left
	clrf		timeout_counter2
	WAITMS		d'100'

DISP_confirmbox_loop:
	call		check_switches_logbook
	
	btfsc		menubit3					; SET/MENU?
	bra			DISP_confirmbox_move_cursor; Move Cursor
	btfsc		menubit2					; ENTER?
	bra			DISP_confirmbox_menu_do		; Do task

	btfsc		onesecupdate
	call		timeout_surfmode			; timeout

	btfsc		onesecupdate
	call		set_dive_modes				; check, if divemode must be entered
	bcf			onesecupdate				; one second update

	btfsc		sleepmode					; Timeout?
	bra			DISP_confirmbox_cancel		; back with cancel
	btfsc		divemode
	bra			DISP_confirmbox_cancel		; back with cancel

	bra			DISP_confirmbox_loop		; wait for something to do

DISP_confirmbox_cancel:
	retlw	.0
DISP_confirmbox_ok:
	retlw	.1

DISP_confirmbox_menu_do:
	dcfsnz	menupos,F
	bra		DISP_confirmbox_cancel
	dcfsnz	menupos,F
	bra		DISP_confirmbox_ok
	bra		DISP_confirmbox_cancel

DISP_confirmbox_move_cursor:
	incf	menupos,F
	movlw	d'3'						; number of menu options+1
	cpfseq	menupos						; =limit?
	bra		DISP_confirmbox_move_cursor2	; No!
	movlw	d'1'							; Yes, reset to position 1!
	movwf	menupos
DISP_confirmbox_move_cursor2:
	bra		DISP_confirmbox2		; Return to Profile Menu, also updates cursor


DISP_depth:
;	ostc_debug	'r'		; Sends debug-information to screen if debugmode active
    SAFE_2BYTE_COPY rel_pressure, lo
	call	adjust_depth_with_salinity			; computes salinity setting into lo:hi [mbar]
#IFDEF IMPERIAL
	call	convert_meters_to_feet		; converts meters to feet
#ENDIF

	movlw	.039
	cpfslt	hi
#IFNDEF IMPERIAL
    bra		depth_greater_99_84mtr

	btfsc	depth_greater_100m			; Was depth>100m during last call
#ELSE
    bra		DISP_depth_full_units_only

	btfsc	depth_greater_100			; Was depth>100 during last call
#ENDIF

	call	DISP_clear_depth			; Yes, clear depth area
#IFNDEF IMPERIAL
	bcf		depth_greater_100m			; Do this once only...
#ELSE
	bcf		depth_greater_100			; Do this once only...
#ENDIF

	lfsr	FSR2,letter

	movlw	HIGH	d'1000'
	movwf	sub_a+1
	movlw	LOW		d'1000'
	movwf	sub_a+0
	movff	hi,sub_b+1
	movff	lo,sub_b+0
	incf	sub_b+0,F
	movlw	d'0'
	addwfc	sub_b+1,F				; Add 1mbar offset
	call	sub16					; sub_c = sub_a - sub_b
#IFNDEF IMPERIAL
	btfss	neg_flag				; Depth lower then 10m?
	rcall	depth_less_10mtr		; Yes, add extra space
#ELSE
	btfss	neg_flag				; Depth lower then 10?
	rcall	depth_less_10   		; Yes, add extra space
#ENDIF

	WIN_TOP		.24
	WIN_LEFT	.0
	WIN_FONT 	FT_LARGE
	WIN_INVERT	.0					; Init new Wordprocessor
	DISP_color_code	warn_depth		; Color-code the output

	movlw	HIGH	d'99'
	movwf	sub_a+1
	movlw	LOW		d'99'
	movwf	sub_a+0
	movff	hi,sub_b+1
	movff	lo,sub_b+0
	call	sub16					; sub_c = sub_a - sub_b
#IFNDEF IMPERIAL
	btfss	neg_flag				; Depth lower then 1m?
#ELSE
	btfss	neg_flag				; Depth lower then 1?
#ENDIF
	bra		DISP_depth2				; Yes, display manual Zero

	bsf		leftbind
	bsf		ignore_digit4
	output_16						; Full meters in Big font
	bcf		leftbind
	bra		DISP_depth3

DISP_depth2:
	PUTC	'0'

DISP_depth3:
	call	word_processor
	bcf		ignore_digit4

#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
;	btfsc   WREG,0                  ; Enabled ?
	btfss   WREG,0                  ; Enabled ?
	bra		DISP_depth6             ; YES, don't print any fractions

#ENDIF
	WIN_FONT 	FT_MEDIUM
	WIN_TOP		.50
	WIN_LEFT	.40
	DISP_color_code	warn_depth		; Color-code the output

    SAFE_2BYTE_COPY rel_pressure, lo
	call	adjust_depth_with_salinity			; computes salinity setting into lo:hi [mbar]
	
	STRCPY  "."

	movlw	HIGH	d'20'			; Display 0.0m if lower then 20cm
	movwf	sub_a+1
	movlw	LOW		d'20'
	movwf	sub_a+0
	movff	hi,sub_b+1
	movff	lo,sub_b+0
	call	sub16					; sub_c = sub_a - sub_b
	btfss	neg_flag				; Depth lower then 0.3m?
	bra		DISP_depth4				; Yes, display manual Zero

	movlw	d'4'
	movwf	ignore_digits
	bsf		ignore_digit5
	output_16dp	d'0'
	bra		DISP_depth5

DISP_depth4:
	PUTC	'0'

DISP_depth5:
	call	word_processor			; decimeters in medium font
	bcf		ignore_digit5
#IFDEF IMPERIAL
DISP_depth6:
#ENDIF
	WIN_FONT 	FT_SMALL
	return

#IFNDEF IMPERIAL
depth_greater_99_84mtr:			; Display only in full meters
	btfss	depth_greater_100m			; Is depth>100m already?
#ELSE
DISP_depth_full_units_only:			; Display only in full meters or feet
	btfss	depth_greater_100			; Is depth>100 already?
#ENDIF
	call	DISP_clear_depth			; No, clear depth area and set flag
	; Depth is already in hi:lo
#IFNDEF IMPERIAL
	; Show depth in Full meters
#ELSE
	; Show depth in Full units
#ENDIF
	; That means ignore figure 4 and 5
	lfsr	FSR2,letter
	WIN_TOP		.24
	WIN_LEFT	.0
	WIN_FONT 	FT_LARGE
	WIN_INVERT	.0					; Init new Wordprocessor
	DISP_color_code	warn_depth		; Color-code the output

	bsf		ignore_digit4
	bsf		leftbind
	output_16
	bcf		leftbind
	call	word_processor
	bcf		ignore_digit4
	WIN_FONT 	FT_SMALL
	return
	
#IFNDEF IMPERIAL
depth_less_10mtr:
#ELSE
depth_less_10:
#ENDIF
	PUTC    ' '
	return

DISP_clear_depth			; No, clear depth area and set flag
    WIN_BOX_BLACK   .24, .90, .0, .90		;top, bottom, left, right
#IFNDEF IMPERIAL
	bsf		depth_greater_100m			; Set Flag
#ELSE
	bsf		depth_greater_100			; Set Flag
#ENDIF
	return

DISP_desaturation_time:	
	movff		int_O_desaturation_time+0,lo
	movff		int_O_desaturation_time+1,hi		; Copy
	tstfsz		lo									; =0?
	bra			DISP_desaturation_time2				; No!
	tstfsz		hi									; =0?
	bra			DISP_desaturation_time2				; No!
	return											; Do not display Desat
	
DISP_desaturation_time2:
	ostc_debug	'h'
	WIN_TOP		.150
	WIN_LEFT	.1
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color

	lfsr	FSR2,letter
	OUTPUTTEXT	d'14'				; Desat
	PUTC    ' '
	movff		desaturation_time_buffer+0,lo			; divide by 60...
	movff		desaturation_time_buffer+1,hi

	call		convert_time				; converts hi:lo in minutes to hours (hi) and minutes (lo)
	bsf			leftbind
	movf		lo,W
	movff		hi,lo
	movwf		hi							; exchange lo and hi...
	output_8								; Hours
	PUTC        ':'
	movff		hi,lo						; Minutes
	output_99x
	bcf		leftbind
	PUTC	' '
;   clrf    WREG							; Allow up to 5 chars to avoid
;   movff   WREG,letter+6					; collision with decotype letters
	call	word_processor
	return

DISP_nofly_time:	
 	movf    nofly_time+0,W              ; Is nofly null ?
    iorwf   nofly_time+1,W
    bnz     DISP_nofly_time2            ; No...
	return
 
DISP_nofly_time2:
	ostc_debug	'g'
	WIN_TOP		.125
	WIN_LEFT	.1
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color

	lfsr	FSR2,letter
	OUTPUTTEXT	d'35'				; NoFly
	PUTC    ' '
	movff		nofly_time+0,lo			; divide by 60...
	movff		nofly_time+1,hi
	call		convert_time			; converts hi:lo in minutes to hours (hi) and minutes (lo)
	bsf			leftbind
	movf		lo,W
	movff		hi,lo
	movwf		hi                      ; exchange lo and hi...
	output_8                            ; Hours
	PUTC        ':'
	movff		hi,lo					; Minutes
	output_99x
	bcf		leftbind
	PUTC	' '
;   clrf    WREG							; Allow up to 5 chars to avoid
;   movff   WREG,letter+6					; collision with decotype letters
	call	word_processor
	return


update_surf_press:
	btfsc	premenu		; Do not update when "Menu?" is displayed!
	return

	ostc_debug	'b'		; Sends debug-information to screen if debugmode active
	WIN_TOP		.25
	WIN_LEFT	.1
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor

;	btfss	high_altitude_mode		; In high altitude mode?
	call	DISP_standard_color		; No
;	btfsc	high_altitude_mode		; In high altitude mode?
;	call	DISP_warnings_color		; Yes, display ambient pressure in red

    SAFE_2BYTE_COPY amb_pressure, lo
	lfsr	FSR2,letter

	movff	lo,sub_a+0
	movff	hi,sub_a+1
	movff	last_surfpressure_30min+0,sub_b+0
	movff	last_surfpressure_30min+1,sub_b+1
	call	sub16					; sub_c = sub_a - sub_b
	btfsc	neg_flag				; Pressure lower?
	rcall	update_surf_press2		; Yes, test threshold

	tstfsz	sub_c+1					; >255mbar difference?
	bra		update_surf_press_common; Yes, display!
	movlw	d'5'
	subwf	sub_c+0,W
	btfsc	STATUS,C
	bra		update_surf_press_common; Yes, display!
;	PUTC	'+'						; For debug only
    SAFE_2BYTE_COPY last_surfpressure_30min, lo	; Overwrite with stable value...

update_surf_press_common:
	bsf		leftbind
	output_16
	bcf		leftbind
	STRCAT_PRINT  TXT_MBAR5
	call	DISP_standard_color		; Reset color
	return

update_surf_press2:
	movff	lo,sub_b+0
	movff	hi,sub_b+1
	movff	last_surfpressure_30min+0,sub_a+0
	movff	last_surfpressure_30min+1,sub_a+1
	call	sub16					; sub_c = sub_a - sub_b
;	PUTC	'-'						; For debug only
	return

update_batt_voltage_divemode:
	call	DISP_warnings_color
	DISPLAYTEXT		d'246'		; LowBatt!
	call	DISP_standard_color
	return

update_batt_voltage_clear:   ; Clear every two seconds if on_time_seconds:3 > CF74*60
    movff   on_time_seconds+0,xC+0
    movff   on_time_seconds+1,xC+1
    movff   on_time_seconds+2,xC+2
    clrf    xC+4
    movlw   .60
    movwf   xB+0
    clrf    xB+1
    call    div32x16  ; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder
    movff   xC+0,sub_b+0
    movff   xC+1,sub_b+1
    GETCUSTOM15     d'74'
    movff   lo,sub_a+0
    movff   hi,sub_a+1
    call    sub16       ;  sub_c = sub_a - sub_b
    btfss   neg_flag
    bra     update_batt_voltage0    ; Normal display
    WIN_BOX_BLACK   .174, .194, .1, .35			;top, bottom, left, right
    return

update_batt_get_percent_in_lo:
    ; percent = ontime [m] * 100 / CF74
    movff   on_time_seconds+0,xC+0
    movff   on_time_seconds+1,xC+1
    movff   on_time_seconds+2,xC+2
    clrf    xC+4
    movlw   .60
    movwf   xB+0
    clrf    xB+1
    call    div32x16  ; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder
    movff   xC+0,xA+0
    movff   xC+1,xA+1   ; On-Time in full minutes
    movlw   .100
    movwf   xB+0
    clrf    xB+1
    call    mult16x16		;xA*xB=xC
    GETCUSTOM15     d'74'
    movff   lo,xB+0
    movff   hi,xB+1
    call    div32x16  ; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder
	movff	xC+0,lo     ; Copy result

    ; Limit to 100
    movlw   .100
    cpfslt  lo
    movwf   lo
	; lo will be between 0 (Full) and 100 (empty)
	movf	lo,W
	sublw	.100
	movwf	lo

	movlw	.100
	cpfslt	lo
	movwf	lo
	; lo will be between 100 (Full) and 0 (empty)
    return

update_batt_voltage:
	ostc_debug	'f'
    btfss   secs,0
    bra     update_batt_voltage_clear   ; Clear every two seconds if on_time_seconds:3 > CF74*60

update_batt_voltage0:
	GETCUSTOM8	d'31'			; =1 if battery voltage should be visible
	movwf	lo
	movlw	d'1'
	cpfseq	lo					; =1?
	bra		update_batt_voltage2	; No, show symbol

	WIN_TOP		.175
	WIN_LEFT	.1
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor

	GETCUSTOM8	d'35'			; Standard output color
	btfsc	cc_active
    movlw   color_orange        ; CC active
	btfsc	charge_done
	movlw	color_green			; Charge done.
    call	DISP_set_color

    call    update_batt_get_percent_in_lo   ; 100 - 0
	lfsr	FSR2,letter
    STRCPY  TXT_BATT            ; Batt:
	bsf		leftbind
	output_8
	bcf		leftbind
	STRCAT_PRINT  "%"
	call	DISP_standard_color
	return

update_batt_voltage2:
    WIN_FRAME_STD .174, .194, .1, .33

    call    update_batt_get_percent_in_lo   ; 100 - 0
    movf    lo,W
    mullw   .10         ; PRODH:PRODL = 1000 - 0
    movff   PRODH,divA+1
    movff   PRODL,divA+0
    movlw   .5
    movwf   divB
    call    div16       ; divA=divA/2^divB (divB: 8Bit only!)
; xC is between 0 and 32
	movff	divA+0,wait_temp				;save value

	movlw	d'3'
	cpfsgt	wait_temp
	movwf	wait_temp					; Minimum = 3

update_batt_voltage2a:
    WIN_BOX_STD .181, .187, .33, .35    ; Battery nose

update_batt_voltage3:
	GETCUSTOM8	d'34'			; Color battery
	btfsc	cc_active
;	movlw	color_yellow		; CC active
    movlw   color_orange        ; CC active
	btfsc	charge_done
	movlw	color_green			; Charge done.
    call	DISP_set_color

	movlw   .175
	movff   WREG,win_top		; row top (0-239)
	movlw   .19
	movff   WREG,win_height
	movlw   .2
	movff   WREG,win_leftx2		; column left (0-159)
    movff   wait_temp,win_width	; column right (0-159)
	call    DISP_box
	call    DISP_standard_color
	return

;update_batt_voltage2_empty:
;	movlw	d'1'
;	movwf	wait_temp
;	bra		update_batt_voltage2a
;
;update_batt_voltage2_full:
;	movlw	d'30'
;	movwf	wait_temp
;	bra		update_batt_voltage2a

DISP_convert_signed_temperature:
   	btfss   	hi,7                    ; Negative temperature ?
    return								; No, return
; Yes, negative temperature!
	PUTC		'-'                     ; Display "-"
    comf    	hi                      ; Then, 16bit sign changes.
    negf    	lo
    btfsc   	STATUS,C
    incf    	hi
	return								; and return

DISP_convert_date:	; converts into "DD/MM/YY" or "MM/DD/YY" or "YY/MM/DD" in postinc2
	read_int_eeprom d'91'			; Read date format (0=MMDDYY, 1=DDMMYY, 2=YYMMDD)
	tstfsz	EEDATA
	bra		DISP_convert_date1

; Use MMDDYY
	movff	convert_value_temp+0,lo			;month
	bsf		leftbind
	output_99x
	bcf		leftbind
	PUTC    '.'
	movff	convert_value_temp+1,lo			;day
	bra 	DISP_convert_date1_common		;year

DISP_convert_date1:
	read_int_eeprom d'91'			; Read date format (0=MMDDYY, 1=DDMMYY, 2=YYMMDD)
	decfsz	EEDATA,F
	bra		DISP_convert_date2

; Use DDMMYY
	movff	convert_value_temp+1,lo			;day
	bsf		leftbind
	output_99x
	bcf		leftbind
	PUTC    '.'
	movff	convert_value_temp+0,lo			;month

DISP_convert_date1_common:
	bsf		leftbind
	output_99x
	bcf		leftbind
	PUTC    '.'
	movff	convert_value_temp+2,lo			;year
	bsf		leftbind
	output_99x
	return

DISP_convert_date2:
; Use YYMMDD
	movff	convert_value_temp+2,lo			;year
	bsf		leftbind
	output_99x
	bcf		leftbind
    PUTC    '.'
	movff	convert_value_temp+0,lo			;month
	bsf		leftbind
	output_99x
	bcf		leftbind
    PUTC    '.'
	movff	convert_value_temp+1,lo			;day
	bsf		leftbind
	output_99x
	return

DISP_convert_date_short:	; converts into "DD/MM" or "MM/DD" or "MM/DD" in postinc2
	read_int_eeprom d'91'			; Read date format (0=MMDDYY, 1=DDMMYY, 2=YYMMDD)
	tstfsz	EEDATA
	bra		DISP_convert_date_short1

; Use MMDDYY
DISP_convert_date_short_common:
	movff	convert_value_temp+0,lo			;month
	bsf		leftbind
	output_99x
	bcf		leftbind
    PUTC    '.'
	movff	convert_value_temp+1,lo			;day
	bsf		leftbind
	output_99x
	bcf		leftbind
	return

DISP_convert_date_short1:
	read_int_eeprom d'91'			; Read date format (0=MMDDYY, 1=DDMMYY, 2=YYMMDD)
	decfsz	EEDATA,F
	bra		DISP_convert_date_short_common	; Use YYMMDD

; Use DDMMYY
	movff	convert_value_temp+1,lo			;day
	bsf		leftbind
	output_99x
	bcf		leftbind
    PUTC    '.'
	movff	convert_value_temp+0,lo			;month
	bsf		leftbind
	output_99x
	bcf		leftbind
	return

update_date:
	ostc_debug	'd'
	WIN_TOP		.75
	WIN_LEFT	.1
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color

	lfsr	FSR2,letter

	movff	month,convert_value_temp+0
	movff	day,convert_value_temp+1
	movff	year,convert_value_temp+2
	call	DISP_convert_date		; converts into "DD/MM/YY" or "MM/DD/YY" or "YY/MM/DD" in postinc2	
	call	word_processor
	return

#IFDEF BOTTOM
;Clear arrea at top after menu-like message dispayed
DISP_pre_gauge_clear:
    WIN_BOX_BLACK   .0, .26, .50, .120			;top, bottom, left, right
	return
#ENDIF

DISP_menu_clear:
    WIN_BOX_BLACK   .0, .26, .50, .100			;top, bottom, left, right

#IFDEF CCR_CTRL
	btfss	FLAG_const_ppO2_mode
	return
#ENDIF

DISP_max_pressure:
	ostc_debug	'p'		; Sends debug-information to screen if debugmode active
	movff	max_pressure+0,lo
	movff	max_pressure+1,hi
	call	adjust_depth_with_salinity			; computes salinity setting into lo:hi [mbar]

#IFDEF IMPERIAL
	call	convert_meters_to_feet			; converts meters to feet

;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
;	btfsc	WREG,0                          ; Enabled ?
	btfss	WREG,0                          ; Enabled ?
	bra		DISP_maxdepth_full_units_only   ; YES, don't print any feet fractions
#ENDIF

	movlw	.039
	cpfslt	hi
#IFNDEF IMPERIAL
    bra		maxdepth_greater_99_84mtr
#ELSE
    bra		DISP_maxdepth_full_units_only
#ENDIF


; Display normal "xx.y"
	lfsr	FSR2,letter
	call	DISP_standard_color

#IFDEF CCR_CTRL
	btfss	FLAG_const_ppO2_mode
    bra     DISP_max_pressure1
	WIN_TOP		.0
	WIN_LEFT	.50
	WIN_INVERT	.0					; Init new Wordprocessor
	bsf		leftbind
	bsf		ignore_digit5		; do not display 1cm depth
	output_16dp	d'3'
	bcf		leftbind
	bcf		show_last3
	call	word_processor
	return
DISP_max_pressure1:
#ENDIF
	WIN_TOP		.207
	WIN_LEFT	.0
	WIN_FONT 	FT_MEDIUM
	WIN_INVERT	.0					; Init new Wordprocessor
	bsf		leftbind
	bsf		ignore_digit5		; do not display 1cm depth
	output_16dp	d'3'
	bcf		leftbind
	bcf		show_last3
	call	word_processor
	WIN_FONT 	FT_SMALL
	return

#IFNDEF IMPERIAL
maxdepth_greater_99_84mtr:			; Display only in full meters
	btfss	maxdepth_greater_100m	; Is max.depth>100m already?
#ELSE
DISP_maxdepth_full_units_only:		; Display only in full units (meters or feet)
	btfss	maxdepth_greater_100	; Is max.depth>100 already?
#ENDIF
	call	DISP_clear_maxdepth		; No, clear maxdepth area and set flag
	; max Depth is already in hi:lo
#IFNDEF IMPERIAL
	; Show max depth in Full meters
#ELSE
	; Show max depth in Full units
#ENDIF
	; That means ignore figure 4 and 5
	lfsr	FSR2,letter
	call	DISP_standard_color
	WIN_TOP		.207
	WIN_LEFT	.0
	WIN_FONT 	FT_MEDIUM
#IFDEF CCR_CTRL
	btfss	FLAG_const_ppO2_mode
    bra     DISP_max_pressure1a
	WIN_TOP		.0
	WIN_LEFT	.50
	WIN_FONT 	FT_SMALL
DISP_max_pressure1a:
#ENDIF
	WIN_INVERT	.0					; Init new Wordprocessor

	bsf		ignore_digit4
	bsf		leftbind
	output_16
#IFDEF CCR_CTRL
    PUTC    ' '
#ENDIF
	bcf		leftbind
	call	word_processor
	bcf		ignore_digit4
	WIN_FONT 	FT_SMALL
	return

DISP_clear_maxdepth:
    WIN_BOX_BLACK   .207, .239, .0, .41		;top, bottom, left, right
#IFNDEF IMPERIAL
	bsf		maxdepth_greater_100m	; Set Flag
#ELSE
	bsf		maxdepth_greater_100	; Set Flag
#ENDIF
	return

DISP_divemins:
	btfsc	menubit					; Divemode menu active?
	return							; Yes, do not update divetime

	ostc_debug	'A'		; Sends debug-information to screen if debugmode active

#IFNDEF BOTTOM
	btfsc	gauge_mode				; different display in gauge mode
	bra		DISP_divemins_gauge
#ENDIF

	btfsc	FLAG_apnoe_mode			; different display in apnoe mode
	bra		DISP_divemins_apnoe

	GETCUSTOM8	d'38'		; Show seconds (=1?)
	movwf	lo
	movlw	d'1'
	cpfseq	lo					; =1?
	bra		DISP_divemins2		; No, minutes only
	bra		DISP_divemins_gauge	; Yes, use Gauge routine
	
DISP_divemins2:
	movff	divemins+0,lo
	movff	divemins+1,hi
	bcf		leftbind
	lfsr	FSR2,letter
	output_16_3	; displays only last three figures from a 16Bit value (0-999)
	WIN_TOP		.20
	WIN_LEFT	.120
	WIN_FONT	FT_MEDIUM
	call	DISP_standard_color
	call	word_processor
	WIN_FONT	FT_SMALL
	return

DISP_display_apnoe_surface:
	btfsc	menubit					; Divemode menu active?
	return							; Yes, do not display surface mode timeout

	call		DISP_divemask_color	; Set Color for Divemode mask
	DISPLAYTEXT	d'140'			; "SURFACE"
	call	DISP_standard_color

	WIN_TOP		.85
	WIN_LEFT	.90
	WIN_FONT	FT_MEDIUM
	call	DISP_standard_color


	movff	apnoe_surface_mins,lo
	bcf		leftbind
	lfsr	FSR2,letter
	output_8
    PUTC    ':'
	movff	apnoe_surface_secs,lo
	output_99x
	call	word_processor
	WIN_FONT	FT_SMALL
	return

DISP_apnoe_clear_surface:
	; Clear Surface timer....
	WIN_BOX_BLACK   .60, .119, .90, .159			;top, bottom, left, right
	return


DISP_display_apnoe_descent:
	btfsc	menubit					; Divemode menu active?
	return							; Yes, do not display/update descent time

	call		DISP_divemask_color	; Set Color for Divemode mask
	DISPLAYTEXT		d'139'			; "Descent"
	call	DISP_standard_color


	WIN_TOP		.145
	WIN_LEFT	.90
	WIN_FONT	FT_MEDIUM
	call	DISP_standard_color


	movff	apnoe_mins,lo
	lfsr	FSR2,letter
	output_8
    PUTC    ':'
	movff	apnoe_secs,lo
	output_99x
	call	word_processor
	WIN_FONT	FT_SMALL
	return
	
DISP_divemins_apnoe:

DISP_divemins_gauge:
	movff	divemins+0,lo
	movff	divemins+1,hi
	bcf		leftbind
	bsf		show_last3
	lfsr	FSR2,letter
	output_16_3					;Displays only 0...999
    PUTC    ':'
	movff	divesecs,lo
	output_99x
	WIN_TOP		.20
	WIN_LEFT	.90
	WIN_FONT	FT_MEDIUM
	call	DISP_standard_color

	call	word_processor
	bcf		show_last3
	WIN_FONT	FT_SMALL
	return

DISP_stopwatch_show:
	; Stopwatch
	call		DISP_divemask_color	; Set Color for Divemode mask
	DISPLAYTEXTH	d'283'			; Stopwatch

DISP_stopwatch_show2:
	call	DISP_standard_color
	ostc_debug	'V'		; Sends debug-information to screen if debugmode active
	WIN_TOP		.192
	WIN_LEFT	.110
	WIN_FONT	FT_SMALL
	call	DISP_standard_color

	lfsr	FSR2,letter
#IFNDEF BOTTOM
	movff	average_divesecs+0,lo				; Stopwatch
	movff	average_divesecs+1,hi				; Stopwatch
#ELSE
	movff	average_divesecs_timer+0,lo				; Stopwatch separatelly resettable
	movff	average_divesecs_timer+1,hi				; Stopwatch separatelly resettable
DISP_stopwatch_timer_2:
#ENDIF
	movlw	d'2'
	subwf	lo,F
	movlw	d'0'
	subwfb	hi,F						; Subtract 2 seconds

	call	convert_time				; converts hi:lo in seconds to mins (hi) and secs (lo)

	movff	lo,wait_temp
	movff	hi,lo
	clrf	hi	

	movlw	d'0'
	bcf		leftbind
	bsf		show_last3
	output_16_3					;Displays only 0...999
    PUTC    ':'
	movff	wait_temp,lo
	output_99x
	call	word_processor

	ostc_debug	'U'				; Sends debug-information to screen if debugmode active

	WIN_TOP		.216
#IFNDEF IMPERIAL
	WIN_LEFT	.110
#ENDIF
	WIN_FONT	FT_SMALL
	call	DISP_standard_color

	lfsr	FSR2,letter
	movff	avr_rel_pressure+0,lo
	movff	avr_rel_pressure+1,hi
	call	adjust_depth_with_salinity			; computes salinity setting into lo:hi [mbar]
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0					; Enabled ?
	bra DISP_stopwatch_show3		; NO
	WIN_LEFT	.124
	call	convert_meters_to_feet2	; converts meters to feet, don't check CF#61
	bsf		ignore_digit4			; do not display feet fractions
	output_16
	bcf		ignore_digit4
	bcf		leftbind
	STRCAT_PRINT TXT_FOOT2
	return

DISP_stopwatch_show3:
	WIN_LEFT	.110
#ENDIF
	bsf		ignore_digit5		; do not display 1cm depth
	output_16dp	d'3'
	bcf		leftbind
	STRCAT_PRINT TXT_METER1
	return

DISP_stopwatch_show_gauge:
	btfsc	menubit					; Divemode menu active?
	return							; Yes, return
	; BIG Stopwatch
	call	DISP_divemask_color	; Set Color for Divemode mask
#IFDEF BOTTOM
	btfss	show_timer
	bra		DISP_stopwatch_timer_5
#ENDIF
	DISPLAYTEXTH	d'310'			; Stopwatch
#IFDEF BOTTOM
DISP_stopwatch_timer_5:
#ENDIF
	DISPLAYTEXTH	d'309'			; Average
	call	DISP_standard_color
	ostc_debug	'V'		; Sends debug-information to screen if debugmode active
#IFNDEF BOTTOM
	WIN_TOP		.80
#ELSE
	WIN_TOP		.86
;Will we show Timer?
	btfss	show_timer
	bra		DISP_stopwatch_timer_4
#ENDIF
	WIN_LEFT	.90
	WIN_FONT	FT_MEDIUM
	call	DISP_standard_color

	lfsr	FSR2,letter
#IFNDEF BOTTOM
	movff	average_divesecs+0,lo				; Stopwatch
	movff	average_divesecs+1,hi				; Stopwatch
#ELSE
;Show Resettable Timer with MIDDLE font
	movff	average_divesecs_timer+0,lo				; Stopwatch separatelly resettable
	movff	average_divesecs_timer+1,hi				; Stopwatch separatelly resettable
#ENDIF
	movlw	d'2'
	subwf	lo,F
	movlw	d'0'
	subwfb	hi,F						; Subtract 2 seconds
	call	convert_time				; converts hi:lo in seconds to mins (hi) and secs (lo)
	movff	lo,wait_temp
	movff	hi,lo
	clrf	hi	
	movlw	d'0'
	bcf		leftbind
	bsf		show_last3
	output_16_3					;Displays only 0...999
    PUTC    ':'
	movff	wait_temp,lo
	output_99x
	call	word_processor
#IFDEF BOTTOM
DISP_stopwatch_timer_4:
#ENDIF

	ostc_debug	'U'				; Sends debug-information to screen if debugmode active
#IFNDEF IMPERIAL
	WIN_TOP		.136
	WIN_LEFT	.90
#ELSE
	WIN_TOP		.150
#ENDIF
#IFNDEF	BOTTOM 
	WIN_FONT	FT_MEDIUM
#ELSE
;Show Resettable Average Depth with LARGE font
	WIN_FONT	FT_LARGE
#ENDIF
	call	DISP_standard_color
	lfsr	FSR2,letter
	movff	avr_rel_pressure+0,lo
	movff	avr_rel_pressure+1,hi
	call	adjust_depth_with_salinity		; computes salinity setting into lo:hi [mbar]
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0							; Enabled ?
	bra DISP_stopwatch_show_gauge2			; NO
#IFNDEF BOTTOM
	WIN_LEFT	.110
#ELSE
	WIN_LEFT	.83
#ENDIF
	call	convert_meters_to_feet2			; converts meters to feet, don't check CF#61
	bsf		ignore_digit4					; do not display feet fractions
	output_16
	bcf		ignore_digit4
	bcf		leftbind
	call	word_processor

#IFNDEF BOTTOM
	WIN_TOP		.149
#ELSE
	WIN_TOP		.178
#ENDIF
	WIN_LEFT	.145
	WIN_FONT	FT_SMALL
	lfsr	FSR2,letter
 	STRCPY_PRINT TXT_FOOT2
;#IFDEF BOTTOM
;	WIN_FONT	FT_SMALL				; Reset...
;#ENDIF
#IFNDEF BOTTOM
	return
#ELSE
	WIN_TOP		.216
	bra DISP_total_average_show2a
#ENDIF

DISP_stopwatch_show_gauge2:
#ENDIF
#IFNDEF BOTTOM
	bsf		ignore_digit5					; do not display 1cm depth
	output_16dp	d'3'
	bcf		leftbind
	STRCAT_PRINT TXT_METER1
	WIN_FONT	FT_SMALL				; Reset...
	return
#ELSE
; Display Average Depth in XXX.x format
;	movlw	.038
;	cpfslt	hi								; > 100m?
;	bra DISP_stopwatch_show_gauge3			; NO
	; display XX
	WIN_LEFT	.70	;RBT
;    WIN_BOX_BLACK   .136, .203, .90, .150		;top, bottom, left, right
	bsf		ignore_digit4					; do not display fractions
	output_16
	bcf		ignore_digit4
	bcf		leftbind
	call	word_processor
	bcf		ignore_digit4
	; display .x
	WIN_FONT 	FT_MEDIUM
	WIN_TOP		.172	;137+25
	WIN_LEFT	.130	;80+60

;	movff	avr_rel_pressure+0,lo
;	movff	avr_rel_pressure+1,hi
;	call	adjust_depth_with_salinity		; computes salinity setting into lo:hi [mbar]
	
	STRCPY  "."

	movlw	d'4'
	movwf	ignore_digits
	bsf		ignore_digit5
	output_16dp	d'0'
;	call	word_processor			; decimeters in medium font
	bcf		ignore_digit5
;	bra DISP_stopwatch_show_gauge4

;DISP_stopwatch_show_gauge3:
; Display depth > 100m without decimals.
;	WIN_LEFT	.90	;RBT
;	bsf		ignore_digit4					; do not display fractions
;	output_16
;	bcf		ignore_digit4

;DISP_stopwatch_show_gauge4:
	bcf		leftbind
;	call	word_processor
	STRCAT_PRINT TXT_METER1
	WIN_FONT	FT_SMALL				; Reset...
;Show Non-resettable Average Depth with SMALL font
;	return
	WIN_TOP		.216
	bra DISP_total_average_show2a
#ENDIF

DISP_total_average_show:
	; Non-Resettable Average
	call		DISP_divemask_color	; Set Color for Divemode mask
	DISPLAYTEXTH	d'281'			; Avr.Depth

DISP_total_average_show2:
#IFNDEF IMPERIAL
	WIN_TOP		.192
	WIN_LEFT	.110
#ELSE
	WIN_TOP		.193
#ENDIF
#IFDEF SEAWOOCH
DISP_total_average_show2a:
#ENDIF
	WIN_FONT	FT_SMALL
	call	DISP_standard_color

	lfsr	FSR2,letter
	movff	avr_rel_pressure_total+0,lo
	movff	avr_rel_pressure_total+1,hi
	call	adjust_depth_with_salinity			; computes salinity setting into lo:hi [mbar]
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0                          ; Enabled ?
	bra DISP_total_average_show3            ; NO
	WIN_LEFT	.124
	call	convert_meters_to_feet2			; converts meters to feet, don't check CF#61
	bsf		ignore_digit4					; do not display feet fractions
	bcf		leftbind
	output_16
	bcf		ignore_digit4
	STRCAT_PRINT TXT_FOOT2
	return

DISP_total_average_show3:
	WIN_LEFT	.110
#ENDIF
	bsf		ignore_digit5		; do not display 1cm depth
	bcf		leftbind
	output_16dp	d'3'
	STRCAT_PRINT TXT_METER1
	return

;=============================================================================
; Writes OSTC #Serial and Firmware version in surfacemode
;
DISP_serial:			
	ostc_debug	'a'		; Sends debug-information to screen if debugmode active
	WIN_TOP		.0
	WIN_LEFT	.1
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0                      ; Init new Wordprocessor
	
  ifdef __DEBUG
	GETCUSTOM8	d'64'					; Write header in blue when
    call    DISP_set_color              ; compiled in DEBUG mode...
  else
	call	DISP_standard_color
  endif

	lfsr	FSR2,letter
	read_int_eeprom d'34'		; Read deco data
	tstfsz	EEDATA
	bra		show_decotype_mainscreen2
	OUTPUTTEXT	.101			; ZH-L16 OC =0
	bra		show_decotype_mainscreen8	; Done.
show_decotype_mainscreen2:
	decfsz	EEDATA,F
	bra		show_decotype_mainscreen3
	OUTPUTTEXT	.102			; Gauge	=1
	bra		show_decotype_mainscreen8	; Done.
show_decotype_mainscreen3:
	decfsz	EEDATA,F
	bra		show_decotype_mainscreen4
	OUTPUTTEXT	.104			; ZH-L16 CC =2
	bra		show_decotype_mainscreen8	; Done.
show_decotype_mainscreen4:
	decfsz	EEDATA,F
	bra		show_decotype_mainscreen5
	OUTPUTTEXT	.138			; Apnoe	=3
	bra		show_decotype_mainscreen8	; Done.
show_decotype_mainscreen5:
	decfsz	EEDATA,F
	bra		show_decotype_mainscreen6
	OUTPUTTEXT	.152			; L16-GF OC	=4
	bra		show_decotype_mainscreen8	; Done.
show_decotype_mainscreen6:
	decfsz	EEDATA,F
	bra		show_decotype_mainscreen7
	OUTPUTTEXT	.236			; L16-GF CC	=5
	bra		show_decotype_mainscreen8	; Done.
show_decotype_mainscreen7:
	decfsz	EEDATA,F
	bra		show_decotype_mainscreen8	; Done.
	OUTPUTTEXT	.226			; pSCR-GF =6
show_decotype_mainscreen8:
	STRCAT  " \x90\x91 V"               ; Scribble logo...
	movlw	softwareversion_x
	movwf	lo
	bsf		leftbind
	output_8
    PUTC    '.'
	movlw	softwareversion_y
	movwf	lo
	bsf		leftbind
	output_99x
	bcf		leftbind

#IFDEF SEAWOOCH
    PUTC    '+'							; Seawooch
#ENDIF

  ifdef __DEBUG
    STRCAT_PRINT "-Dbg"    
  else
	call	word_processor

	movlw	softwareversion_beta        ; =1: Beta, =0: Release
	decfsz	WREG,F
	return                              ; Release version -> Return

	call	DISP_warnings_color
	DISPLAYTEXT		d'243'              ; beta
	call	DISP_standard_color
  endif

	return

;=============================================================================

DISP_divemode_menu_mask_first:			; Write Divemode menu1 mask
	ostc_debug	'o'		; Sends debug-information to screen if debugmode active
	call	DISP_menu_clear			; clear "Menu?"
	call    DISP_standard_color

#IFDEF BOTTOM
;Do not show original Menu ittems
	btfsc	gauge_mode
	return
#ENDIF

	btfsc	FLAG_const_ppO2_mode	; are we in ppO2 mode?
	bra		DISP_divemode_menu_mask_first2

; in OC Mode
	DISPLAYTEXT	.32					;"Gaslist"
	DISPLAYTEXT	.31					;"Decoplan"
	bra		DISP_divemode_menu_mask_first3

DISP_divemode_menu_mask_first2:
; in CC Mode
	DISPLAYTEXT	.238				;"SetPoint"
	DISPLAYTEXT	.31					;"Decoplan"

DISP_divemode_menu_mask_first3:
; In all modes
	call	customview_menu_entry3	; Show customview-dependent menu entry
	DISPLAYTEXT	.241				;"Display"
	DISPLAYTEXT	.34					;"Exit"
	return							

DISP_divemode_set_xgas:				; Displayes the "Set Gas" menu
	WIN_LEFT	.100
	WIN_TOP		.0
	WIN_FONT	FT_SMALL
	call	DISP_standard_color

    STRCPY  TXT_G6_3
	read_int_eeprom	d'24'			; Get Gas6 %O2
	movff	EEDATA,lo
	bcf		leftbind
    movlw   .100
    cpfseq  lo                      ; 100% O2?
    bra     DISP_divemode_set_xgas2 ; No
    STRCAT  "100/0"                 ; Draw 100/0 manually
    bra     DISP_divemode_set_xgas3
DISP_divemode_set_xgas2:
	output_99					; outputs into Postinc2!
    PUTC    '/'
	read_int_eeprom	d'25'			; Get Gas6 %He
	movff	EEDATA,lo
	output_99					; outputs into Postinc2!
DISP_divemode_set_xgas3:
	call	word_processor
	DISPLAYTEXT	.123			; O2 +
	DISPLAYTEXT	.124			; O2 -
	DISPLAYTEXT	.125			; He +
	DISPLAYTEXT	.126			; He -
	DISPLAYTEXTH	d'300'		; Active? (Enable/Disable Gas underwater)
	return

DISP_divemode_simulator_mask:
	    call    DISP_standard_color
        DISPLAYTEXT	.254			; Close
#IFDEF IMPERIAL
;		GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
		GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
		btfsc   WREG,0                          ; Enabled ?
		bra DISP_divemode_simulator_mask2       ; NO
        DISPLAYTEXTH .380			; +  1ft
        DISPLAYTEXTH .381			; -  1ft
        DISPLAYTEXTH .382			; + 10ft
        DISPLAYTEXTH .383			; - 10ft
		DISPLAYTEXTH .306			; Quit Sim
        return

DISP_divemode_simulator_mask2:
#ENDIF
        DISPLAYTEXT	.250			; + 1m
        DISPLAYTEXT	.251			; - 1m
        DISPLAYTEXT	.252			; +10m
        DISPLAYTEXT	.253			; -10m
		DISPLAYTEXTH .306			; Quit Sim
        return

;-----------------------------------------------------------------------------
; Draw a stop of the deco plan (simulator or dive).
; Inputs: lo      = depth. Range 3m...93m
;                 + 80 if this is a switch-gas stop.
;         hi      = minutes. range 1'..240'.
;         win_top = line to draw on screen.
; Trashed: hi, lo, win_height, win_leftx2, win_width, win_color*,
;          WREG, PROD, TBLPTR TABLAT.
;
DISP_decoplan_show_stop:
        ;---- Print depth ----------------------------------------------------
        WIN_LEFT .100
        
        btfss   lo,7                    ; Bit set ?
        bra     DISP_decoplan_std_stop  ; No : Just an usual stop

        bcf     lo,7                    ; cleanup depth
		
		GETCUSTOM8	d'55'				; Load gas switch [min] in wreg
		tstfsz	WREG					; =0?
		bra		DISP_decoplan_show_stop1; No: Show gas switch stop
		bra     DISP_decoplan_std_stop  ; Yes: Just an usual stop
		
DISP_decoplan_show_stop1:		
        movlw   color_yellow
        call    DISP_set_color			; Show in yellow for gas switch
        bra     DISP_decoplan_nstd_stop

DISP_decoplan_std_stop:
	    call    DISP_standard_color

DISP_decoplan_nstd_stop:        
	    lfsr	FSR2,letter
	    bsf     leftbind
#IFDEF IMPERIAL
;		GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
		GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
		btfsc   WREG,0						; Enabled ?
		bra     DISP_decoplan_nstd_stop2    ; NO
		movff   hi,temp11					; save hi
		clrf    hi
		call    convert_meters_to_feet2		; converts meters to feet, don't check CF#61
		output_16_3
        STRCAT_PRINT "  "
		movff   temp11,hi					; restore hi
		bra     DISP_decoplan_nstd_stop3

DISP_decoplan_nstd_stop2:
#ENDIF
	    output_8					    ; outputs into Postinc2!
        STRCAT_PRINT TXT_METER2
#IFDEF IMPERIAL
DISP_decoplan_nstd_stop3:
#ENDIF
        ;---- Print duration -------------------------------------------------
	    WIN_LEFT	.139
	    lfsr	FSR2,letter
	    
	    movf    lo,W                    ; Swap hi & lo
	    movff   hi,lo
	    movwf   hi

        bsf     leftbind
	    output_99					    ; Allow up to 99'
        STRCAT_PRINT "'"                ; 1 to 3 chars for depth.

	    movf    lo,W                    ; Swap back hi & lo
	    movff   hi,lo
	    movwf   hi

        ;---------------------------------------------------------------------
        ; Draw the bar graph used for deco stops (decoplan in simulator or dive).
        movff   win_top,WREG            ; Increment win_top (BANK SAFE)
        incf    WREG
        movff   WREG,win_top
        movlw	d'18'+1                 ; 19 --> height (bank safe !)
        movff   WREG,win_height
        movlw	.122
        movff	WREG,win_leftx2    		; column left (0-159)
        movlw	.16
        movff	WREG,win_width    		; column max width.

        ; Draw used area (hi = minutes):
        movlw	d'16'                   ; Limit length (16min)
        cpfslt	hi
        movwf	hi
        movff   hi,win_bargraph         ; Active width, the rest is cleared.

        call    DISP_standard_color
        call	DISP_box

        ; Restore win_top
        call    DISP_standard_color
        movff   win_top,WREG            ; decf win_top (BANK SAFE)
        decf    WREG
        movff   WREG,win_top
        return

;-----------------------------------------------------------------------------
; Clear unused area below last stop
; Inputs: win_top : last used area...
DISP_decoplan_clear_bottom:
        movff   win_top,WREG            ; Get back from bank0
        btfsc   divemode                ; In dive mode ?
        sublw   .168                    ; Yes: bottom row in divemode
        btfss   divemode                ; In dive mode ?
        sublw   .240                    ; No: bottom row in planning
        movff   WREG,win_height

        WIN_LEFT .85                    ; Full divemenu width
        movlw   .159-.85
        movff   WREG,win_width

        clrf    WREG                    ; Fill with black
        movff   WREG,win_color1
        movff   WREG,win_color2
        movff   WREG,win_color3
        movff   WREG,win_color4
        movff   WREG,win_color5
        movff   WREG,win_color6
        goto	DISP_box

;-----------------------------------------------------------------------------
; Display the decoplan (simulator or divemode) for GF model
; Inputs: char_O_deco_table (array of stop times, in minutes)
;         decoplan_page = page number. Displays 5 stop by page.
;
#define decoplan_index  apnoe_mins          ; within each page
#define decoplan_gindex apnoe_secs          ; global index
#define decoplan_last   apnoe_max_pressure  ; Depth of last stop (CF#29)
#define decoplan_max    apnoe_max_pressure+1; Number of lines per page. 7 in planning, 5 in diving.

DISP_decoplan:
        ostc_debug	'n'		; Sends debug-information to screen if debugmode active

        WIN_INVERT 0

        ;---- Is there deco stops ? ------------------------------------------
    	movff   char_O_first_deco_depth,WREG
    	iorwf   WREG
        bnz		DISP_decoplan_1

        ;---- No Deco --------------------------------------------------------
        call    DISP_standard_color
        DISPLAYTEXT	d'239'              ;"No Deco"
        bsf     last_ceiling_gf_shown
        return

DISP_decoplan_1:
    	lfsr	FSR0,char_O_deco_depth  ; Initialize indexed addressing.
	    lfsr	FSR1,char_O_deco_time

        movlw   .8                      ; 8 lines/page in decoplan
        btfsc   divemode
        movlw   .6                      ; 6 lines/page in divemode.
        movwf   decoplan_max

        clrf   decoplan_index           ; Start with index = 0
        clrf	WREG
        movff	WREG,win_top            ; and row = 0

        ; Read stop parameters, indexed by decoplan_index and decoplan_page
        movf    decoplan_page,W         ; decoplan_gindex = 6*decoplan_page + decoplan_index
        mulwf   decoplan_max
        movf    decoplan_index,W
        addwf   PRODL,W
        movwf   decoplan_gindex         ; --> decoplan_gindex
        
        bcf     last_ceiling_gf_shown   ; Not finished yet...

DISP_decoplan_2:
        btfsc   decoplan_gindex,5       ; Reached table length (32) ?
        bra     DISP_decoplan_99        ; YES: finished...

        ; Read stop parameters, indexed by decoplan_index
        movf    decoplan_gindex,W       ; index
    	movff	PLUSW1,hi               ; char_O_deco_time [gindex] --> hi
	    movff	PLUSW0,lo               ; char_O_deco_depth[gindex]
        movf    lo,W
        bz      DISP_decoplan_99        ; depth == 0 : finished.

        ; Display the stop line
    	call	DISP_decoplan_show_stop

        ; Next
        movff   win_top,WREG            ; row: += 24
	    addlw	.24
        movff   WREG,win_top
	    incf	decoplan_index,F        ; local index += 1
	    incf	decoplan_gindex,F       ; global index += 1

        ; Max number of lines/page reached ?
    	movf    decoplan_max,W          ; index+1 == max ?
    	cpfseq	decoplan_index
    	bra		DISP_decoplan_2         ; NO: loop

    	; Check if next stop if end-of-list ?
    	movf    decoplan_gindex,W
	    movff	PLUSW0,WREG             ; char_O_deco_depth[gindex]
	    iorwf   WREG
    	bz      DISP_decoplan_99        ; End of list...

        ; Display the message "more..."
        bcf		last_ceiling_gf_shown	; More page to display...

        rcall   DISP_decoplan_clear_bottom  ; Clear from next line

    	WIN_LEFT .130 - 7*3
    	call    DISP_standard_color
    	lfsr    FSR2,letter
    	OUTPUTTEXT .142                 ; More...
    	goto    word_processor

DISP_decoplan_99:
        bsf		last_ceiling_gf_shown   ; Nothing more in table to display.
        rcall   DISP_decoplan_clear_bottom  ; Clear from next line
        return
;-----------------------------------------------------------------------------
; Toggle gas activity flag during dive.
; 
; Input: gaslist_active
;        Gaslist from eeprom[2...]
;
; Output: gaslist_active
;
; Note: Gas with a zero depth cannot be used in deco simulation, hence
;       should not be displayed as selected here...
;
DISP_de_activelist:			; show (de)active gaslist
	call	DISP_standard_color	
    DISPLAYTEXT	.254			; Close

	WIN_LEFT	.100
	WIN_FONT	FT_SMALL
	bsf		leftbind
	
	movlw	d'2'
	movwf	wait_temp			; here: stores eeprom address for gas list
	movlw	d'0'
	movwf	waitms_temp			; here: stores row for gas list
	clrf	hi					; here: Gas counter

DISP_de_activelist_loop:
	incf	hi,F				; Increase Gas
	movlw	d'4'
	addwf	wait_temp,F			; Increase eeprom address for gas list
	movlw	d'25'
	addwf	waitms_temp,F		; Increase row
	WIN_LEFT	.100
	movff	waitms_temp,win_top ; Set Row
	
  	lfsr	FSR2,letter
	movff	wait_temp, EEADR; Gas #hi: %O2 - Set address in internal EEPROM
	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	output_8				; outputs into Postinc2!
    PUTC    '/'
	incf	EEADR,F			; Gas #hi: %He - Set address in internal EEPROM
	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	output_8				; outputs into Postinc2!
    PUTC    '@'

	movlw	d'27'
	addwf	hi,W
	movwf	EEADR			; Point to Change depth

	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	output_8				; outputs into Postinc2!
	
	movf    lo,w            ; Gas with a zero depth
	bz      DISP_de_activelist_grey ; should be displayed inactive.
	
; Check if gas needs to be greyed-out (inactive)	
	movff	gaslist_active,EEDATA	; Get flag register
	movff	hi,lo			; copy gas number
DISP_de_activelist_loop1:
	rrcf	EEDATA			; roll flags into carry
	decfsz	lo,F			; max. 5 times...
	bra		DISP_de_activelist_loop1
    bc      DISP_de_activelist_white

DISP_de_activelist_grey:    ; grey out inactive gases!
	GETCUSTOM8	d'64'					;movlw	color_grey
	call	DISP_set_color

DISP_de_activelist_white:
	call	word_processor	
	call	DISP_standard_color	

	movlw	d'5'			; list all five gases
	cpfseq	hi				; All gases shown?
	bra		DISP_de_activelist_loop	; No

	return					;  return 

DISP_show_change_depth:		; Yes, show change depth for gas #menupos
	btfsc	display_set_setpoint	; In Setpoint list?
	return							; Yes, return.
	movlw	color_yellow			; Blink in yellow
    call	DISP_set_color
#IFDEF IMPERIAL
	WIN_LEFT	.82
#ELSE
	WIN_LEFT	.95
#ENDIF
	WIN_TOP		.148
	WIN_FONT	FT_SMALL

	movlw	.6
	cpfslt	menupos							; <6?
	bra		DISP_show_change_depth_clear	; Yes!

	bsf		leftbind
	STRCPY  TXT_GAS1
	movff	menupos,lo
	output_8					; Show gas number
    STRCAT  TXT_AT4				; " at "
	decf	menupos,W
	addlw	d'28'				; offset in memory
	movwf	EEADR
	call	read_eeprom			; Low-value
	movff	EEDATA,lo
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0						; Enabled ?
	bra     DISP_show_change_depth2     ; NO
	clrf    hi
	call    convert_meters_to_feet2		; converts meters to feet, don't check CF#61
	output_16_3
	STRCAT_PRINT TXT_FOOT4
	bcf		leftbind
	call	DISP_standard_color
	return

DISP_show_change_depth2:
#ENDIF
	output_8					; Show gas number
    STRCAT  "m "
    clrf    WREG
    movff   WREG,letter+.9       ; Limit to 9 chars
    STRCAT_PRINT  ""
	bcf		leftbind
	call	DISP_standard_color
	return

DISP_show_change_depth_clear:
#IFNDEF IMPERIAL
	STRCPY_PRINT  "         "
#ELSE
	STRCPY_PRINT  "          "
#ENDIF
	return

DISP_diluent_list:
	ostc_debug	'm'		; Sends debug-information to screen if debugmode active
	WIN_LEFT	.100
	WIN_FONT	FT_SMALL
	bsf		leftbind
    movlw	d'93'
	movwf	wait_temp			; here: stores eeprom address for diluent list (96-2)
	movlw	d'231'
	movwf	waitms_temp			; here: stores row for gas list
	clrf	hi					; here: Diluent counter

DISP_diluent_list_loop:
   	incf	hi,F				; Increase Diluent
    movlw   d'4'
	addwf   wait_temp,F			; Increase eeprom address for gas list
	movlw	d'25'
	addwf	waitms_temp,F		; Increase row
	WIN_LEFT	.100
	movff	waitms_temp,win_top ; Set Row
	STRCPY  TXT_DIL_C
	movff	hi,lo			; copy dil number
	output_8				; display dil number
    PUTC    ':'
	movff	wait_temp, EEADR; Dil #hi: %O2 - Set address in internal EEPROM
	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	output_8				; outputs into Postinc2!
    PUTC    '/'
	incf	EEADR,F			; Dil #hi: %He - Set address in internal EEPROM
	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	output_8				; outputs into Postinc2!

	decf	EEADR,F			; Dil #hi: %O2 - Set address in internal EEPROM
	call	read_eeprom		; get byte (stored in EEDATA)
	DISP_color_code		warn_gas_in_gaslist		; Color-code output	(%O2 in "EEDATA")
    
	call	word_processor
	call	DISP_standard_color

	movlw	d'5'			; list all five Diluents
	cpfseq	hi				; All diluents shown?
	bra		DISP_diluent_list_loop	; No
	return					;  return


DISP_gas_list:
	ostc_debug	'm'		; Sends debug-information to screen if debugmode active

	WIN_LEFT	.100
	WIN_FONT	FT_SMALL
	bsf		leftbind
	
	movlw	d'2'
	movwf	wait_temp			; here: stores eeprom address for gas list
	movlw	d'231'
	movwf	waitms_temp			; here: stores row for gas list
	clrf	hi					; here: Gas counter

DISP_gas_list_loop:
	incf	hi,F				; Increase Gas
	movlw	d'4'
	addwf	wait_temp,F			; Increase eeprom address for gas list
	movlw	d'25'
	addwf	waitms_temp,F		; Increase row
	WIN_LEFT	.100
	movff	waitms_temp,win_top ; Set Row
	STRCPY  TXT_GAS1
	movff	hi,lo			; copy gas number
	output_8				; display gas number
    PUTC    ':'
	movff	wait_temp, EEADR; Gas #hi: %O2 - Set address in internal EEPROM
	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	output_8				; outputs into Postinc2!
    PUTC    '/'
	incf	EEADR,F			; Gas #hi: %He - Set address in internal EEPROM
	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	output_8				; outputs into Postinc2!

	decf	EEADR,F			; Gas #hi: %O2 - Set address in internal EEPROM
	call	read_eeprom		; get byte (stored in EEDATA)
	DISP_color_code		warn_gas_in_gaslist		; Color-code output	(%O2 in "EEDATA")

; Check if the "better gas" should be highlighted

	WIN_INVERT	.0					; Init new Wordprocessor	
	movf	better_gas_number,W	; better gas 1-5?
	cpfseq	hi					; compare with gas#
	bra		DISP_gas_list_loop2	; No equal, skip
	
	movlw	color_yellow			; Blink in yellow
    call	DISP_set_color
	WIN_INVERT	.1					; Init new Wordprocessor	
	
DISP_gas_list_loop2:
; Check if gas needs to be greyed-out (inactive)
	movff	gaslist_active, EEDATA		; Work with sorted list
	movff	hi,lo			; copy gas number
DISP_gas_list_loop1:
	rrcf	EEDATA			; roll flags into carry
	decfsz	lo,F			; max. 5 times...
	bra		DISP_gas_list_loop1

	btfss	STATUS,C		; test carry
	rcall	DISP_gas_list_grey
		
	call	word_processor	
	call	DISP_standard_color	

	movlw	d'5'			; list all five gases
	cpfseq	hi				; All gases shown?
	bra		DISP_gas_list_loop	; No

    WIN_INVERT	.0
	DISPLAYTEXT		d'122'		; More
	return					;  return (OC mode)

DISP_gas_list_grey:
	GETCUSTOM8	d'64'					;movlw	color_grey
	call	DISP_set_color	; grey out inactive gases!
	return

DISP_splist_start:	
	WIN_LEFT	.100
	WIN_FONT	FT_SMALL
	bsf		leftbind
	call    DISP_standard_color

	; list three SP in Gaslist
	movlw	d'35'				; 36 = current SP position in EEPROM
	movwf	wait_temp			; here: stores eeprom address for gas list
	movlw	d'0'
	movwf	waitms_temp			; here: stores row for gas list
	clrf 	decoplan_index	    ; here: SP counter

DISP_splist_loop:
	incf	wait_temp,F			; EEPROM address
	incf	decoplan_index,F	; Increase SP

	movlw	d'25'
	addwf	waitms_temp,F		; Increase row
	movff	waitms_temp,win_top ; Set Row
	WIN_LEFT	.100
	
	STRCPY  TXT_SP2
	movff	decoplan_index,lo	; copy gas number
	output_8				; display gas number
    PUTC    ':'
	movff	wait_temp, EEADR; SP #hi position
	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	clrf	hi
	output_16dp	d'3'		; outputs into Postinc2!
	call	word_processor	

	movlw	d'3'		; list all three SP
	cpfseq	decoplan_index          ; All gases shown?
	bra		DISP_splist_loop	; No

	bcf		leftbind
	return						; no, return

DISP_clear_divemode_menu:
#IFDEF IMPERIAL
	btfsc   divemode
	bra		DISP_clear_divemode_menu1
#ENDIF
    WIN_BOX_BLACK   .0, .168, .85, .159		;top, bottom, left, right
	return

#IFDEF IMPERIAL
;; Clear larger area to accomodate for 3 digit decostop
;; when depth is shown in feet
DISP_clear_divemode_menu1:
    WIN_BOX_BLACK   .0, .168, .81, .160		;top, bottom, left, right
	return

#ENDIF
DISP_divemenu_cursor:
	ostc_debug	'l'		; Sends debug-information to screen if debugmode active

    WIN_BOX_BLACK   .0, .150, .85, .95			;top, bottom, left, right

	WIN_TOP		.0
	WIN_LEFT	.85
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call	DISP_standard_color

	movff	menupos,temp1
	movlw	d'0'
	dcfsnz	temp1,F
	movlw	d'0'
	dcfsnz	temp1,F
	movlw	d'25'
	dcfsnz	temp1,F
	movlw	d'50'
	dcfsnz	temp1,F
	movlw	d'75'
	dcfsnz	temp1,F
	movlw	d'100'
	dcfsnz	temp1,F
	movlw	d'125'

	movff	WREG,win_top
    STRCPY_PRINT "\xB7"				; Cursor
	return


;=============================================================================
; Draw saturation graph, is surface mode or in dive mode.
;
DISP_tissue_saturation_graph:
	ostc_debug	'i'		; Sends debug-information to screen if debugmode active

    ;---- Draw Frame ---------------------------------------------------------
    btfsc   divemode
    bra     DISP_tsg_1
    
    WIN_FRAME_STD   .25, .120, .82, .159    ; Surfmode
    bra     DISP_tsg_2
DISP_tsg_1:    
    WIN_FRAME_STD   .169, .239, .90, .159   ; Divemode
DISP_tsg_2:

    ;---- Draw grid ----------------------------------------------------------
    btfss   divemode
    bra     DISP_no_graph_grid
    
	GETCUSTOM8	d'64'					;movlw	color_grey
    call	DISP_set_color

    movlw   .169+.1                     ; divemode
	movff	WREG,win_top
    movlw   .239-.169-.1                ; divemode
	movff	WREG,win_height

    movlw   1
    movff   WREG,win_width

    movlw   .122
    movff   WREG,win_leftx2
    call    DISP_box
    movlw   .131
    movff   WREG,win_leftx2
    call    DISP_box
    movlw   .140
    movff   WREG,win_leftx2
    call    DISP_box
    movlw   .149
    movff   WREG,win_leftx2
    call    DISP_box
DISP_no_graph_grid:
    
    ;---- Draw N2 Tissues ----------------------------------------------------
	lfsr	FSR2, char_O_tissue_N2_saturation
	movlw	d'16'
	movwf	wait_temp                   ; 16 tissues
	clrf	waitms_temp                 ; Row offset

	movlw	.1
	movff	WREG,win_height             ; row bottom (0-239)
	movlw	.82+.18                     ; surfmode
    btfsc   divemode
    movlw   .90+.18                     ; divemode
	movff	WREG,win_leftx2             ; column left (0-159)
    movlw	.57                         ; surfmode: max width 57pix
    btfsc   divemode
	movlw	.57-8                       ; divemode: 8pix less...
	movff   WREG,win_width

DISP_tissue_saturation_graph3:
    call    DISP_standard_color         ; Reset color foreach iteration

	movlw	.25+3                       ; surfmode: 3pix below top border
    btfsc   divemode
    movlw   .169+3                      ; divemode
	addwf	waitms_temp,W
	movff	WREG,win_top                ; row top (0-239)

	incf	waitms_temp,F
	incf	waitms_temp,F

	movf	POSTINC2,W
	bcf		STATUS,C                    ; Clear carry
	rrcf	WREG                        ; And divide by 4
	bcf		STATUS,C
	rrcf	WREG
	movwf   temp1

	movff   win_width,WREG              ; Max width.
	cpfslt	temp1                       ; skip if 57 (WREG) < win_width
	movwf	temp1
	movff   temp1,win_bargraph

	call	DISP_box	

	decfsz	wait_temp,F
	bra		DISP_tissue_saturation_graph3

    ;---- Draw He Tissues ----------------------------------------------------
	lfsr	FSR2, char_O_tissue_He_saturation
	movlw	d'16'
	movwf	wait_temp                   ; 16 tissues
	clrf	waitms_temp                 ; Row offset

DISP_tissue_saturation_graph2:
    call    DISP_standard_color         ; Reset color foreach iteration

	movlw	.120-.33                    ; surfmode : 33pix above bottom border
    btfsc   divemode
    movlw   .239-.33                    ; divemode
	addwf	waitms_temp,W
	movff	WREG,win_top                ; row top (0-239)

	incf	waitms_temp,F
	incf	waitms_temp,F

	movf	POSTINC2,W
	bcf		STATUS,C                    ; Clear carry
	rrcf	WREG                        ; And divide by 4
	bcf		STATUS,C
	rrcf	WREG
	movwf   temp1

	movff   win_width,WREG              ; Max width.
	cpfslt	temp1                       ; skip if 57 (WREG) < win_width
	movwf	temp1
	movff   temp1,win_bargraph

	call	DISP_box	

	decfsz	wait_temp,F
	bra		DISP_tissue_saturation_graph2

    ;---- Draw N2/He Text ----------------------------------------------------
    call    DISP_standard_color         ; Reset color after last iterarion.

	movlw	.82+2                       ; surfmode: 2pix right of left border
    btfsc   divemode
    movlw   .90+2                       ; divemode
    movff   WREG,win_leftx2

	movlw	.25+7                       ; surfmode: 7pix below top border
    btfsc   divemode
    movlw   .169+7                      ; divemode
    movff   WREG,win_top
	STRCPY_PRINT  TXT_N2_2

	movlw	.120-.30                    ; surfmode: 30pix above bottom border
    btfsc   divemode
    movlw   .239-.30                    ; divemode
    movff   WREG,win_top
	STRCPY_PRINT  TXT_HE2
	
    ;---- Draw scale and O2[16]% ---------------------------------------------
    btfsc   divemode
    return

	movff	char_O_gtissue_no,wait_temp			; used as temp

	lfsr	FSR1,char_O_tissue_N2_saturation
	movf	wait_temp,W			; W <- 0-15
	movff	PLUSW1,lo			; lo <- FSR1[W]

	WIN_TOP		.62
	WIN_FONT	FT_SMALL
	lfsr	FSR2,letter
	bsf		leftbind
	output_8
	bcf		leftbind

	STRCAT_PRINT  "% "

    ;---- Draw Scale ---------------------------------------------------------
    WIN_BOX_STD .73, .74, .121, .157
    WIN_BOX_STD .61, .84, .121, .122
    WIN_BOX_STD .65, .80, .130, .131
    WIN_BOX_STD .65, .80, .139, .140
    WIN_BOX_STD .65, .80, .148, .149
    WIN_BOX_STD .61, .84, .157, .158
	return

;=============================================================================

DISP_startupscreen1:
    call	DISP_divemask_color
	DISPLAYTEXT d'3'			; "HeinrichsWeikamp"
	call	DISP_standard_color
	DISPLAYTEXT	.68				; Licence 1/2
	DISPLAYTEXT	.69
	DISPLAYTEXT	.70
	DISPLAYTEXT	.71
	DISPLAYTEXT	.72
	DISPLAYTEXT	.73
	DISPLAYTEXT	.74
	return

DISP_startupscreen2:
    call	DISP_divemask_color
	DISPLAYTEXT d'3'			; "HeinrichsWeikamp"
	call	DISP_standard_color
	DISPLAYTEXT	.75				; Licence 2/2
	DISPLAYTEXT	.76
	DISPLAYTEXT	.77
	DISPLAYTEXT	.78
	DISPLAYTEXT	.79
	DISPLAYTEXT	.80
	DISPLAYTEXT	.81
	return

DISP_new_cf_warning:
    call	DISP_divemask_color
	DISPLAYTEXTH	.271		; New CF added!
	call	DISP_standard_color
	DISPLAYTEXTH .272		; New CustomFunctions
	DISPLAYTEXTH .273		; were added! Check
	DISPLAYTEXTH .274		; CF I and CF II Menu
	DISPLAYTEXTH .275		; for Details!
	return

DISP_const_ppO2_value:
	ostc_debug	'j'		; Sends debug-information to screen if debugmode active
	
#IFDEF CCR_CTRL
    btfss   ext_ppO2_enable         ; Check for eCTRL mode
    bra     DISP_const_ppO2_value_3

;Show Controlled Setpoint
	call		DISP_standard_color
	WIN_TOP		.144
	WIN_LEFT 	.50
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor

	lfsr	FSR2,letter
	bsf		leftbind

	btfss	ext_ppO2_valid
    bra     DISP_const_ppO2_value_1
	btfsc	ext_ppo2_abandon
    bra     DISP_const_ppO2_value_1
    ;Show Setpoint from eCTRL
	movff	eCTRL_Setpoint,lo
	output_16dp	d'3'
	STRCAT_PRINT  " "				; Display Setpoint with trailing zero
    bra     DISP_const_ppO2_value_2
DISP_const_ppO2_value_1:
    ;Show "----"
    STRCPY  "---- "
DISP_const_ppO2_value_2:
	call	word_processor
	bcf		leftbind
DISP_const_ppO2_value_3:
#ENDIF

	WIN_TOP		.168
	WIN_LEFT 	.50
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor

	lfsr	FSR2,letter
	movff	char_I_const_ppO2,lo
	
	tstfsz	lo						; In Bailout mode (char_I_const_ppO2=0)?
	bra		DISP_const_ppO2_value2	; No, display Setpoint

; Yes, Display "Bail"
	call		DISP_standard_color
	OUTPUTTEXTH		d'263'			;"Bail "
	call	word_processor
	return

DISP_const_ppO2_value2:				; Display SetPoint
;Show fixed SP value
    SAFE_2BYTE_COPY amb_pressure, xA

	movlw		d'10'
	movwf		xB+0
	clrf		xB+1
	;xA/xB=xC with xA as remainder 	
	call		div16x16				; xC+0=p_amb/10

	; char_I_const_ppO2 > p_amb/10 -> Not physically possible! -> Display actual value!
	tstfsz	xC+1				; xC>255
	setf	xC+0				; Yes, set xC+0 to 2,55bar ppO2

	movff		ppO2_setpoint_store,WREG
	cpfslt		xC+0							; Setpoint value possible?
	bra			DISP_const_ppO2_value1			; Yes

	clrf		hi								; Reset hi
	movff		xC+0,char_I_const_ppO2			; No, Overwrite with actual value
	bra			DISP_const_ppO2_value1a

DISP_const_ppO2_value1:
	; char_I_const_ppO2 < ppO2[Diluent] -> Not physically possible! -> Display actual value!
    SAFE_2BYTE_COPY amb_pressure, xA
	movlw		d'10'
	movwf		xB+0
	clrf		xB+1
	call		div16x16				; xC=p_amb/10
	movff		xC+0,xA+0
	movff		xC+1,xA+1
	movff		char_I_O2_ratio,xB+0
	clrf		xB+1
	call		mult16x16				; xC:2=char_I_O2_ratio * p_amb/10

	movff		xC+0,sub_b+0
	movff		xC+1,sub_b+1
	movff		ppO2_setpoint_store,WREG; Setpoint
	mullw		d'100'					; Setpoint*100
	movff		PRODL,sub_a+0
	movff		PRODH,sub_a+1
	call		sub16					; sub_c = sub_a - sub_b

	btfss		neg_flag
	bra			DISP_const_ppO2_value11	; Value in range (lower then fix Setpoint)

	; char_I_const_ppO2 < ppO2[Diluent] -> Not physically possible! -> Display actual value!

	movff		xC+0,xA+0				; xC=p_amb/10
	movff		xC+1,xA+1
	movlw		d'100'
	movwf		xB+0
	clrf		xB+1
	call		div16x16				;xA/xB=xC with xA as remainder 	

	movff		xC+0,char_I_const_ppO2	; No, Overwrite with actual value
	movff		xC+1,hi					; For test if ppO2>2,55bar
	
	GETCUSTOM8	d'39'					; Adjust fixed SP?
	dcfsnz		WREG,F
	bra			DISP_const_ppO2_value1a	; Yes!
	; Do not adjust -> restore original SetPoint

DISP_const_ppO2_value11:
; Setpoint in possible limits
	movff		ppO2_setpoint_store,char_I_const_ppO2		; Restore Setpoint
	clrf		hi
	
DISP_const_ppO2_value1a:
	movff	char_I_const_ppO2,lo

	movff	lo,WREG					; copy to WREG
	mullw	.100
	movff	PRODH,xC+1
	movff	PRODL,xC+0				; For color code
	DISP_color_code		warn_ppo2	; Color-code output (ppO2 stored in xC)	

	tstfsz	hi						; >2,55bar?
	rcall	DISP_const_ppO2_too_hi	; Yes

	bsf		leftbind
	output_16dp	d'3'
	bcf		leftbind
	STRCAT_PRINT  " "				; Display Setpoint with trailing zero
	call	DISP_standard_color		; Reset color
	return

DISP_const_ppO2_too_hi:
	PUTC	'>'
	setf	lo						; show ">2.55"
	clrf	hi						; clear hi
	call	DISP_warnings_color		; Set Warning color
	return


DISP_show_ceiling:
    call		DISP_divemask_color ; Set Color for Divemode mask
	WIN_FONT	FT_SMALL
    DISPLAYTEXT	d'233'              ; Ceiling
DISP_show_ceiling_1:
    call	DISP_standard_color
    WIN_FONT	FT_MEDIUM
	WIN_LEFT	.100
	WIN_TOP		.195
    lfsr        FSR2,letter
    movff       int_O_ceiling+0,lo
    movff       int_O_ceiling+1,hi
    call        adjust_depth_with_salinity			; computes salinity setting into lo:hi [mbar]
#IFDEF IMPERIAL
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0							; Enabled ?
	bra DISP_show_ceiling_1a    			; NO
	call	convert_meters_to_feet2			; converts meters to feet, don't check CF#61
	bcf		leftbind
	bsf		ignore_digit4					; do not display feet fractions
	output_16
	bcf		ignore_digit4
    bra     DISP_show_ceiling_1b
DISP_show_ceiling_1a:
#ENDIF
    bsf         ignore_digit5         ; no cm
    output_16dp  .3               ; yxz.a
    bcf         ignore_digit5
#IFDEF IMPERIAL
DISP_show_ceiling_1b:
#ENDIF
    STRCAT_PRINT ""
    return

;=============================================================================
; Display EAD/END computed in calc_hauptroutine_update_tissues() every 2sec.
;
DISP_show_end_ead_divemode:
	call		DISP_divemask_color     ; Set Color for Divemode mask

	WIN_FONT	FT_SMALL
	WIN_LEFT	.95
	WIN_TOP		.192
	lfsr	FSR2,letter
	OUTPUTTEXTH	.299                    ; EAD:
	call        word_processor

	WIN_TOP		.216
	lfsr        FSR2,letter
	OUTPUTTEXTH	.298                    ; END:
	call        word_processor

	call        DISP_standard_color     ; Back to white.
	WIN_LEFT	.125
	WIN_TOP		.192
	lfsr        FSR2,letter
	movff       char_O_EAD,lo
	bsf         leftbind
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc       WREG,0						; Enabled ?
	bra         DISP_show_end_ead_divemode2 ; NO
	clrf        hi							; clear hi
	call        convert_meters_to_feet2		; converts meters to feet, don't check CF#61
	output_16								; Print EAD w/o leading space.
	bcf	        leftbind
	STRCAT_PRINT TXT_FOOT2

	WIN_TOP     .216
	lfsr        FSR2,letter
	movff       char_O_END,lo
	clrf        hi							; clear hi
	call        convert_meters_to_feet2		; converts meters to feet, don't check CF#61
	bsf	        leftbind
	output_16	                            ; Print END w/o leading space.
	bcf	        leftbind
	STRCAT_PRINT TXT_FOOT2
	bra         DISP_show_end_ead_divemode3

DISP_show_end_ead_divemode2:
#ENDIF
	output_8                            ; Print EAD w/o leading space.
	STRCAT_PRINT TXT_METER2

	WIN_TOP		.216
	lfsr        FSR2,letter
	movff       char_O_END,lo
	output_8                            ; Print END w/o leading space.
	bcf	        leftbind
	STRCAT_PRINT TXT_METER2

#IFDEF IMPERIAL
DISP_show_end_ead_divemode3:
#ENDIF
; Show ppO2[Flush] iff in CCR mode & not in Bailout:
	btfsc       is_bailout              ; In bailout mode?
	return                              ; Yes: done.

	btfss       FLAG_const_ppO2_mode    ; In (true) CCR mode ?
	return                              ; No: done.

	WIN_LEFT	.95
	WIN_TOP		.168
	call        DISP_divemask_color     ; Set Color for Divemode mask
	STRCPY_PRINT TXT_PPO2_5                ; ppO2 of diluent

	movff       char_O_flush_ppO2,WREG  ; copy to WREG
	mullw       .100
	movff       PRODH,xC+1
	movff       PRODL,xC+0              ; For color code
	DISP_color_code		warn_ppo2		; Color-code output (ppO2 stored in xC)	

	WIN_LEFT	.130
	WIN_TOP		.168

    movff       char_O_flush_ppO2, lo
    incf        lo,W                    ; ppO2 == 2.55 ?
    bnz         DISP_show_end_ead_divemode_1

    STRCPY_PRINT "----"                 ; YES: mark overflow.
	goto        DISP_standard_color     ; Back to white.

DISP_show_end_ead_divemode_1:    
	lfsr		FSR2,letter
    clrf        hi
	bsf		leftbind
	output_16dp	d'3'					; Show ppO2 w/o leading zero
	bcf		leftbind
    PUTC    " "
    clrf    WREG
    movff   WREG,letter+4               ; limit to five chars
	STRCAT_PRINT  ""					; Display ppO2[Diluent]
	goto    DISP_standard_color         ; Back to white.

;=============================================================================
; Display TTS after extra time at the same depth.
;
DISP_show_@5:
	WIN_FONT    FT_SMALL
    WIN_LEFT    .159-.69                ; 10 chars aligned right.
    WIN_TOP     .170
	call		DISP_divemask_color     ; Set Color for Divemode mask
    lfsr        FSR2,letter

    OUTPUTTEXTH .305                    ; "Future TTS"
    call        word_processor

    WIN_LEFT	.97
    WIN_TOP     .194
    STRCPY      "@"
	GETCUSTOM8  d'58'
	movwf       lo
	bsf         leftbind
	output_8
	bcf         leftbind
	STRCAT_PRINT "': "
    
	WIN_LEFT    .97+7*5                 ; "@10':" is 5 chars long
	call        DISP_standard_color 
	lfsr        FSR2,letter

	movff       int_O_extra_ascenttime+0,lo
    movff       int_O_extra_ascenttime+1,hi
    movf        lo,W
	iorwf       hi,W                    ; extra_ascenttime == 0 ?
	bz          DISP_show_@5_nodeco
	movf        lo,W                    ; extra_ascenttime == 0xFFFF ?
	andwf       hi,W
	incf        WREG,w
	bz          DISP_show_@5_wait

	bsf         leftbind
	output_16
	bcf         leftbind
	STRCAT      "'  "                  ; From "none" to "1'" we need 2 trailing spaces
    movff       WREG,letter+3          ; limit to four chars
	STRCAT_PRINT ""
	return

DISP_show_@5_nodeco:
DISP_show_@5_wait:
    STRCPY_PRINT "--- "
    return

;=============================================================================

compute_pscr_ppo2:
; (Pressure[mbar]*char_I_O2_ratio)-(100-char_I_O2_ratio)*CF61*CF62*10	
	movff	char_I_O2_ratio,WREG
	sublw	.100			; 100-char_I_O2_ratio -> WREG
	mullw	.10				; (100-char_I_O2_ratio)*10 -> PROD:2
	movff	PRODL,xA+0
	movff	PRODH,xA+1
	GETCUSTOM8  d'62'		; O2 Drop
	movff	WREG,xB+0
	clrf	xB+1
	call	mult16x16	;xA*xB=xC -> (100-char_I_O2_ratio)*10*CF61
	movff	xC+0,xA+0
	movff	xC+1,xA+1
 	GETCUSTOM8  d'63'		; Lung ratio
	movff	WREG,xB+0
	clrf	xB+1
	call	mult16x16	;xA*xB=xC -> (100-char_I_O2_ratio)*10*CF61*CF62

	movlw	.10
	movwf	xB+0
	clrf	xB+1
	call	div32x16	  ; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder
	; store xC:2 in lo:hi
	movff	xC+0,lo
	movff	xC+1,hi

	SAFE_2BYTE_COPY amb_pressure, xA
	movff	char_I_O2_ratio,xB+0
	clrf	xB+1
	call	mult16x16	;xA*xB=xC -> xC:4 = Pressure[mbar]*char_I_O2_ratio

	movlw	.10
	movwf	xB+0
	clrf	xB+1
	call	div32x16	  ; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder

	; store xC:2 in sub_a
	movff	xC+0,sub_a+0
	movff	xC+1,sub_a+1
	; reload result from lo:hi
	movff	lo,sub_b+0
	movff	hi,sub_b+1

	call	subU16		;sub_c = sub_a - sub_b (with UNSIGNED values)
	return

; Display pSCR ppO2
DISP_show_pSCR_ppO2:
	WIN_FONT    FT_SMALL
    WIN_LEFT    .159-.63                ; 9 chars aligned right.
    WIN_TOP     .170
	call		DISP_divemask_color     ; Set Color for Divemode mask
    lfsr        FSR2,letter
    OUTPUTTEXTH .266                    ; "pSCR Info"
    call        word_processor

	WIN_FONT	FT_SMALL
	WIN_LEFT	.95
	WIN_TOP		.192
	lfsr	FSR2,letter
	STRCPY_PRINT TXT_PPO2_5             ; ppO2:
	rcall   compute_pscr_ppo2		; pSCR ppO2 into sub_c:2
	movff	sub_c+0,xC+0
	movff	sub_c+1,xC+1
	clrf	xC+2
	clrf	xC+3			; For color coding
	DISP_color_code		warn_ppo2		; Color-code output (ppO2 stored in xC)	
	WIN_LEFT	.130
	WIN_TOP		.192
	lfsr    FSR2,letter
	movff	xC+0,lo
	movff	xC+1,hi
	bsf		ignore_digit4
	output_16dp	d'1'
	bcf		ignore_digit4
    PUTC    " "
    movlw   .0
    movff   WREG,letter+.4         ; Limit to 4chars
    STRCAT_PRINT ""
	call        DISP_standard_color     ; Back to white.
; Show O2 drop and counter lung ration in second row
	WIN_LEFT	.98
	WIN_TOP		.216
	lfsr        FSR2,letter
	GETCUSTOM8  d'62'		; O2 Drop in percent
	movwf		lo
	bsf			leftbind
	output_8
	STRCAT		 "% 1/"
	GETCUSTOM8  d'63'		; Counter lung ratio in 1/X
	movwf		lo
	output_8
	bcf			leftbind
    PUTC    " "
    movlw   .0
    movff   WREG,letter+.9           ; Limit to 9chars
    STRCAT_PRINT ""
	return

;=============================================================================
; Display cave consomation prediction (and warning).
;
DISP_show_cave_bailout:
	WIN_FONT    FT_SMALL
    WIN_LEFT    .159-.69                ; 10 chars aligned right.
    WIN_TOP     .170
	call		DISP_divemask_color     ; Set Color for Divemode mask
    lfsr        FSR2,letter

    OUTPUTTEXTH .311                    ; "Cave Bail."
    call        word_processor
    
;   WIN_TOP     .240 - 24               ; DO NOT display liter units, as this
;   WIN_LEFT    .160 - 7                ; can be Bars also...
;   STRCPY_PRINT "l"

	WIN_FONT    FT_MEDIUM
    WIN_LEFT	.90
    WIN_TOP     .201                    ; 170 + 24 + 14/2 + 32 + 14/2 = 240.
	call        DISP_standard_color 
	lfsr        FSR2,letter
    
    ;---- Retrieve divetime in seconds (since last reset)
	movff	    average_divesecs+0,xA+0
	movff	    average_divesecs+1,xA+1
	
	;---- Multiply by SAC, and divide by 60 (SAC inliters per minutes)
    GETCUSTOM8	d'56'			        ; Get bottom SAC
    movwf       xB+0
    clrf        xB+1
	call	    mult16x16               ; xC:4=xA:2*xB:2

	movlw       LOW(.60)
	movwf       xB+0
	movlw       HIGH(.60)
	movwf       xB+1
	call	    div32x16                ; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder

    ;---- Multiply by average pressure [absolute, in bar]
    movff       xC+0,xA+0               ; Get result (in xC+0, noy xC+2 !) into xA
    movff       xC+1,xA+1
    
    movf        avr_rel_pressure+0,W    ; Add surface pressure to get absolute pressure
    addwf       last_surfpressure_30min+0,W
    movwf       xB+0
    movf        avr_rel_pressure+1,W
    addwfc      last_surfpressure_30min+1,W
    movwf       xB+1                    ; --> Into xB

	call	    mult16x16               ; xC:4=xA:2*xB:2
	
	movlw       LOW(.1000)              ; Pressure was in milibar, so divide by 1000.
	movwf       xB+0
	movlw       HIGH(.1000)
	movwf       xB+1
	call	    div32x16                ; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder

    ;---- Go RED when limit is exceeded
    movff       xC+0,sub_a+0
    movff       xC+1,sub_a+1
    GETCUSTOM15 d'59'			        ; Get Cave bailout alarm threshold
    movff       lo, sub_b+0
    movff       hi, sub_b+1
    call        sub16                   ; Computes prediction - limit
    btfss       neg_flag                ; Negativ ?
    call        DISP_warnings_color     ; NO: go RED.
    
    ;---- Then display...
	movff       xC+0,lo
	movff       xC+1,hi

    bcf         leftbind
    output_16
    call        word_processor
	WIN_FONT    FT_SMALL
	return

;=============================================================================

DISP_show_leading_tissue:
	call		DISP_divemask_color	; Set Color for Divemode mask
	DISPLAYTEXTH	.282		; L. Tissue:
DISP_show_leading_tissue_2:
	call	deco_calc_desaturation_time	; calculate desaturation time
	movlb	b'00000001'						; select ram bank 1

    STRCPY  "#"
	movff	char_O_gtissue_no,lo
	movff	char_O_gtissue_no,wait_temp			; used as temp
	bsf		leftbind
	output_8
	STRCAT  " ("
	
	movlw	d'16'
	cpfslt	wait_temp
	bra		DISP_show_leading_tissue_he
	STRCAT  TXT_N2_2
	bra		DISP_show_leading_tissue2

DISP_show_leading_tissue_he:	
    STRCAT  TXT_HE2

DISP_show_leading_tissue2:	
	WIN_LEFT	.95
	WIN_TOP		.192
	WIN_FONT	FT_SMALL
	call	DISP_standard_color

    STRCAT_PRINT  ") "

	lfsr	FSR1,char_O_tissue_N2_saturation
	movf	wait_temp,W			; W <- 0-15
	movff	PLUSW1,lo			; lo <- FSR1[W]

	WIN_LEFT	.95
	WIN_TOP		.216
	WIN_FONT	FT_SMALL

	lfsr	FSR2,letter
	output_8	
	STRCAT_PRINT  "% "
	bcf		leftbind
	return

DISP_marker_set:
	WIN_LEFT	.105
    WIN_TOP		.170
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
    call    DISP_divemask_color     ; Set Color for Divemode mask
    SAFE_2BYTE_COPY marker_depth, lo
	call	adjust_depth_with_salinity			; computes salinity setting into lo:hi [mbar]
    lfsr	FSR2,letter
   	bsf		leftbind
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0                          ; Enabled ?
	bra DISP_marker_set_m           ; NO
	call	convert_meters_to_feet2			; converts meters to feet, don't check CF#61
	bsf		ignore_digit4					; do not display feet fractions
	output_16
	bcf		ignore_digit4
	STRCAT_PRINT TXT_FOOT2
	bra DISP_marker_set_m2

DISP_marker_set_m:
	bsf		ignore_digit5		; do not display 1cm depth
	output_16dp	d'3'
    PUTC    TXT_METER_C
    bcf		show_last3
	call	word_processor
DISP_marker_set_m2:
#ENDIF

	WIN_LEFT	.105
    WIN_TOP		.192
	WIN_INVERT	.0					; Init new Wordprocessor
	movff	marker_time+0,lo
	movff	marker_time+1,hi
	bsf		leftbind
	lfsr	FSR2,letter
	output_16_3	; displays only last three figures from a 16Bit value (0-999)
    PUTC    ':'
	movff	marker_time+2,lo
	output_99x
	call	word_processor
	bcf		leftbind
	call	DISP_standard_color
    return

DISP_topline_box_clear:			; Writes an empty box
	movlw	.0
	bra		DISP_topline_box2
DISP_topline_box:				; Writes a filled box...
	GETCUSTOM8		d'35'		; ... with the standard color
DISP_topline_box2:
    WIN_BOX_COLOR   .0, .26, .0, .159	
	call    DISP_standard_color	; Reset to standard color in case of unreadable color
	return

DISP_display_cns:
	btfsc	gauge_mode			; Do not display in gauge mode
	 return

	btfsc	FLAG_apnoe_mode		; Do not display in apnoe mode
	 return

	btfsc	DISP_velocity_display	; Is velocity displayed?`
	 return							; Yes, do not overwrite until DISP_velocity_clear was called

	ostc_debug	'k'				; Sends debug-information to screen if debugmode active

	WIN_TOP		.090
	WIN_LEFT	.0
	WIN_FONT 	FT_SMALL
	DISP_color_code		warn_cns		; Color-code CNS output
	
	STRCPY  TXT_CNS4
	movff	char_O_CNS_fraction,lo
	bsf		leftbind
	output_8
	bcf		leftbind
	STRCAT_PRINT "%"
	return

;-----------------------------------------------------------------------------
;
DISP_display_cns_surface:
; Check if CNS should be displayed
	movff	char_O_CNS_fraction,lo		; copy into bank1
	GETCUSTOM8	d'15'					; cns_display_high_surfacemode
	subwf	lo,W
	btfss	STATUS,C
	return								; Do not show...
	; Show CNS

	ostc_debug	'W'				; Sends debug-information to screen if debugmode active

	WIN_TOP		.175
	WIN_LEFT	.45
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	DISP_color_code		warn_cns		; Color-code CNS output
	
	STRCPY  TXT_CNS4
	movff	char_O_CNS_fraction,lo
	bsf		leftbind
	output_8
	bcf		leftbind
	STRCAT  "% "
    movlw   .0
    movff   WREG,letter+.8          ; Limit to 8 chars
    STRCAT_PRINT  ""
	return

;-----------------------------------------------------------------------------
; Display GF at furface, if > CF8.
;
DISP_display_gf_surface:
        movff	char_O_gradient_factor,lo   ; gradient factor
        GETCUSTOM8	d'8'                ; threshold for display
        cpfslt	lo                      ; show value?
        bra		DISP_display_gf_surf_1  ; YES: do it.
        return

DISP_display_gf_surf_1:
        WIN_TOP	    .175
        WIN_LEFT	.45
        WIN_FONT 	FT_SMALL
        DISP_color_code		warn_gf		; Color-code Output

        STRCPY  TXT_GF3
        movff   char_O_gradient_factor,lo		; gradient factor
        output_8
        STRCAT  "%  "
        movlw   .0
        movff   WREG,letter+.8          ; Limit to 8 chars
        STRCAT_PRINT  ""
        goto    DISP_standard_color

;-----------------------------------------------------------------------------

DISP_custom_text:
	read_int_eeprom	d'64'
	movlw	d'1'
	cpfseq	EEDATA						; Custom text active?
	bra		DISP_custom_text_serial		; No, show serial instead
	WIN_TOP		.200
	WIN_LEFT	.0
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call		DISP_divemask_color	; Set Color for Divemode mask

	lfsr	FSR2,letter	
	movlw	d'64'
	movwf	lo
	movlw	d'24'
	movwf	hi					; counter

DISP_custom_text1:
	incf	lo,F
	call	DISP_get_custom_letter			; Get one letter for the custom text
	movlw	'}'							; End marker found?
	cpfseq	EEDATA
	bra		DISP_custom_text2			; No
	bra		DISP_custom_text3
DISP_custom_text2:
	movff	EEDATA,POSTINC2				; Copy into Postinc

	decfsz	hi,F				; Max. numbers?
	bra		DISP_custom_text1			; No, get next letters

DISP_custom_text3:
	call	word_processor
	call	DISP_standard_color
	return

DISP_get_custom_letter:
	movff	lo,EEADR			; Address for next custom text letter
	call	read_eeprom					; Read letter
	return

DISP_custom_text_serial:
	WIN_TOP		.200
	WIN_LEFT	.50
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.0					; Init new Wordprocessor
	call		DISP_divemask_color	; Set Color for Divemode mask

	lfsr	FSR2,letter
	OUTPUTTEXTH		d'262'              ; "OSTC "
	clrf	EEADRH
	clrf	EEADR                       ; Get Serial number LOW
	call	read_eeprom                 ; read byte
	movff	EEDATA,lo
	incf	EEADR,F                     ; Get Serial number HIGH
	call	read_eeprom                 ; read byte
	movff	EEDATA,hi
	bsf		leftbind
	output_16
	call	word_processor
	call	DISP_standard_color
	return

DISP_simdata_screen:			;Display Pre-Dive Screen
	; List active gases/Setpoints
	btfsc	FLAG_const_ppO2_mode		; in ppO2 mode?
	bra		DISP_simdata_screen3		; Yes, display SetPoint/Sensor result list

DISP_simdata_screen2:
	ostc_debug	'm'		; Sends debug-information to screen if debugmode active

	WIN_LEFT	.0
	WIN_FONT	FT_SMALL
	bsf		leftbind
	
	movlw	d'2'
	movwf	wait_temp			; here: stores eeprom address for gas list
	movlw	d'10'
	movwf	waitms_temp		; here: stores row for gas list
	clrf	hi					; here: Gas counter

DISP_simdata_screen2_loop:
	incf	hi,F				; Increase Gas
	movlw	d'4'
	addwf	wait_temp,F			; Increase eeprom address for gas list
	
	STRCPY  TXT_GAS1
	movff	hi,lo			; copy gas number
	output_8				; display gas number
	PUTC    ':'
	movff	wait_temp, EEADR            ; Gas #hi: %O2 - Set address in internal EEPROM
	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	output_8				; outputs into Postinc2!
	PUTC    '/'
	incf	EEADR,F			; Gas #hi: %He - Set address in internal EEPROM
	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	output_8				; outputs into Postinc2!
	PUTC    ' '
	movf	hi,W			; Gas number
	addlw	d'27'			; -> Adress of change depth register
	call	read_int_eeprom_1
	movff	EEDATA,lo		; Change depth in m
	movff	lo,divemins		; Store for grey-out
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0						; Enabled ?
	bra     DISP_simdata_screen2_loop3  ; NO
	movff	hi,temp11					; save hi
	clrf    hi							; clear hi
	call    convert_meters_to_feet2		; converts meters to feet, don't check CF#61
	output_16_3							; Print 
	STRCAT	TXT_FOOT2
	movff	temp11,hi					; restore hi
	bra DISP_simdata_screen2_loop4

DISP_simdata_screen2_loop3:
#ENDIF
	output_99				; outputs into Postinc2!
    PUTC    TXT_METER_C

#IFDEF IMPERIAL
DISP_simdata_screen2_loop4:
#ENDIF
    ; Check if gas is first gas ?
	read_int_eeprom d'33'	            ; First gas (1-5)?
	movf	hi,W                        ; Current gas in WREG
	cpfseq	EEDATA				        ; Is equal first gas?
	bra		DISP_simdata_screen2_loop2	; No : more tests...

	bra		DISP_simdata_white	        ; Yes

DISP_simdata_screen2_loop2:	
    ; Check if gas is inactive ?
	read_int_eeprom d'27'	            ; read flag register
	movff	hi,lo			            ; copy gas number
DISP_simdata_screen2_loop1:
	rrcf	EEDATA			            ; roll flags into carry
	decfsz	lo,F		            	; max. 5 times...
	bra		DISP_simdata_screen2_loop1

	btfss	STATUS,C		            ; test inactive flag
	bra		DISP_simdata_grey	        ; Is inactive!

	tstfsz	divemins		            ; Test change depth=0?
	bra		DISP_simdata_white      	; Is not zero

DISP_simdata_grey:
	GETCUSTOM8	d'64'					;movlw	color_grey
	call	DISP_set_color	            ; grey out inactive gases!
	bra		DISP_simdata_color_done

DISP_simdata_white:
	call	DISP_standard_color

DISP_simdata_color_done:	
	movlw	d'25'
	addwf	waitms_temp,F		; Increase row
	WIN_LEFT	.0
	movff	waitms_temp,win_top ; Set Row
	call	word_processor		; display gas

DISP_simdata_screen2b:
	call		DISP_standard_color

	movlw	d'5'			; list all five gases
	cpfseq	hi				; All gases shown?
	bra		DISP_simdata_screen2_loop	; No
	
	return							; No, return (OC mode)

DISP_simdata_screen3:	
	WIN_LEFT	.0
	WIN_FONT	FT_SMALL
	bsf		leftbind

	; list three SP in Gaslist
	movlw	d'35'				; 36 = current SP position in EEPROM
	movwf	wait_temp			; here: stores eeprom address for gas list
	movlw	d'10'
	movwf	waitms_temp			; here: stores row for gas list
	clrf 	decoplan_index		; here: SP counter

DISP_simdata_screen3_loop:
	incf	wait_temp,F			; EEPROM address
	incf	decoplan_index,F    ; Increase SP

	movlw	d'25'
	addwf	waitms_temp,F		; Increase row
	WIN_LEFT	.0
	movff	waitms_temp,win_top ; Set Row
	
	STRCPY  TXT_SP2
	movff	decoplan_index,lo   ; copy gas number
	output_8				; display gas number
	STRCAT  ": "
	movff	wait_temp, EEADR; SP #hi position
	call	read_eeprom		; get byte (stored in EEDATA)
	movff	EEDATA,lo		; copy to lo
	clrf	hi
	output_16dp	d'3'		; outputs into Postinc2!
	call	word_processor	

	movlw	d'3'		; list all three SP
	cpfseq	decoplan_index		; All gases shown?
	bra		DISP_simdata_screen3_loop	;no

    ; Show Diluent
    call    get_first_diluent           ; Read first diluent into lo(O2) and hi(He)
	WIN_LEFT	.0
	WIN_TOP		.110
	STRCPY  TXT_DIL4
	output_8				; O2 Ratio
	STRCAT  "/"
	movff	hi,lo
	output_8				; He Ratio
	call	word_processor		

	bcf		leftbind
	return				; Return (CC Mode)

adjust_depth_with_salinity:			; computes salinity setting into lo:hi [mbar]

	btfsc	simulatormode_active	; Do apply salinity in Simulatormode
	return

	read_int_eeprom	d'26'			; Read Salinity from EEPROM
	movff	EEDATA, wait_temp		; salinity
	
	movlw	d'105'					; 105% ?
	cpfslt	wait_temp				; Salinity higher limit
	return							; Out of limit, do not adjust lo:hi
	
	movlw	d'99'					; 99% ?
	cpfsgt	wait_temp				; Salinity lower limit
	return							; Out of limit, do not adjust lo:hi

	movff	lo,xA+0
	movff	hi,xA+1

	movlw	d'102'					; 0,98bar/10m
	movwf	xB+0
	clrf	xB+1
	
	call	mult16x16				;xA*xB=xC (lo:hi * 100)
	
	movff	wait_temp,xB+0			; Salinity
	clrf	xB+1
							
	call	div32x16  				; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder

	movff	xC+0,lo
	movff	xC+1,hi					; restore lo and hi with updated value
	
	return
#IFDEF IMPERIAL
convert_meters_to_feet:             ; convert value in lo:hi from meters to feet

;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
    btfsc   WREG,0                  ; Enabled ?
    return                          ; NO: return

convert_meters_to_feet2:
	movff	lo,xA+0
	movff	hi,xA+1

	movlw	LOW d'328'              ; 328feet/100m
	movwf	xB+0
	movlw	HIGH d'328'
	movwf	xB+1

	call	mult16x16			    ; xA*xB=xC (lo:hi * 328)

	movlw	d'50'                   ; round up
	addwf	xC+0,F
	movlw	0
	addwfc	xC+1,F
	addwfc	xC+2,F
	addwfc	xC+3,F

	movlw	d'100'					
	movwf	xB+0
	clrf	xB+1

	call	div32x16  			    ; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder

	movff	xC+0,lo
	movff	xC+1,hi				    ; restore lo and hi with updated value

	return

convert_celsius_to_fahrenheit:		; convert value in lo:hi from celsius to fahrenheit

;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
    btfsc   WREG,0                  ; Enabled ?
    return                          ; NO: return

	movff	lo,xA+0
	movff	hi,xA+1

	movlw	d'18'                   ; 1C = 1.8F
	movwf	xB+0
	clrf	xB+1

	call	mult16x16               ;xA*xB=xC (lo:hi * 18)

	movlw	d'10'
	movwf	xB+0
	clrf	xB+1

	call	div32x16                ; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder

    movlw	LOW d'320'              ; 0C = 32F
	addwf	xC+0,F
	movlw	HIGH d'320'
	addwfc	xC+1,F

	movff	xC+0,lo
	movff	xC+1,hi                 ; restore lo and hi with updated value

	return
#ENDIF

#IFDEF CCR_CTRL
DISP_BT_EN_text:
    btfss     CTRL_BT_mode               ;Check Flag
    bra DISP_BT_EN_text_no
	STRCPY  "BT"                    ; Draw "BT" when it is ON
    bra     DISP_BT_EN_text_cont
DISP_BT_EN_text_no:
	STRCPY  "  "
DISP_BT_EN_text_cont:
	movlw	color_blue              ; blue
	call DISP_set_color
	WIN_TOP		.220
	WIN_LEFT	.116
    call	word_processor
	call    DISP_standard_color
    return

DISP_ppo2_sensor_mask:
    call	DISP_divemask_color
	DISPLAYTEXTH	.388            ; CCR Controller Setup:
	call    DISP_standard_color

    ; Switch setpoint manually
    call    DISP_SP_manual

DISP_ppo2_sensor_mask3:
	DISPLAYTEXTH .391               ; "Calibrate" [item 4]
	WIN_LEFT	.90
	WIN_TOP		.125
	lfsr	FSR2,letter
    GETCUSTOM8  eCTRL_CF_CALO2      ; persentage of the O2 for calibration ECTRL_CF
	movff	WREG,lo
	bsf		leftbind
	output_8
	bcf		leftbind
    STRCAT_PRINT "% O2 ?"
	call	word_processor

	DISPLAYTEXTH	.396    ; Turn eCTRL OFF?
	DISPLAYTEXT .11			; Exit [item 6]
	return

DISP_ppo2_setup_mask:
    call	DISP_divemask_color
	DISPLAYTEXTH	.390		; CCR Controller Setup:
	call    DISP_standard_color

	call DISP_scruber_life
    call DISP_Bluetooth
	DISPLAYTEXT .11			; Exit [item 6]
	return

DISP_SP_manual:
    ; Read SP from EEPROM and display it
	WIN_LEFT	.20
	WIN_TOP		.95
	lfsr	FSR2,letter
    STRCAT "Test SP["
    movff   eCTRL_SP_manual, WREG       ; Load current SP #
	addlw	d'36'						; offset in memory
	movwf	EEADR
	call	read_eeprom					; Read SetPoint
	movff	EEDATA, lo                  ; Use SetPoint
	clrf    hi
    bsf     leftbind
	output_16dp	3
    STRCAT "] next?"
	call	word_processor              ; Display SP
	return

DISP_scruber_life:
; Read Scruber life from EEPROM and display it
	WIN_LEFT	.20
	WIN_TOP		.65
    bcf     leftbind
    STRCPY "Scr.Time "
    rcall    read_sctuber_life
	output_16
    STRCAT " RES?"
	call	word_processor			; Display Scruber life
	return

DISP_Bluetooth:
    ; Read Scruber life from EEPROM and display it
	WIN_LEFT	.20
	WIN_TOP		.35
    bcf     leftbind
    STRCPY "Bluetooth: turn "
    movff   eCTRL_BT_timer, WREG    ;Load current timer value
    tstfsz  WREG
    bra     DISP_Bluetooth_off
    STRCAT "ON?"
DISP_Bluetooth_exit:
	call	word_processor			; Display Scruber life
	return
DISP_Bluetooth_off:
    STRCAT "OFF?"
    bra     DISP_Bluetooth_exit

DISP_divemode_02sensor_all:
	; Display Sensors in Dive mode
    WIN_FONT	FT_MEDIUM
	WIN_LEFT	.0
	WIN_TOP     .142
	bsf		leftbind

    ;Copy Sensor Status to tmp for color processing
    movff   eCTRL_StatLo,eCTRL_color_tmp

	lfsr	FSR2,letter
	movff	eCTRL_ppo2_s1,lo		; copy measured value
	call	DISP_ppO2_sensor_dive

	WIN_TOP		.175
	lfsr	FSR2,letter
	movff	eCTRL_ppo2_s2,lo		; copy measured value
	call	DISP_ppO2_sensor_dive

	WIN_TOP		.208
	lfsr	FSR2,letter
	movff	eCTRL_ppo2_s3,lo		; copy measured value
	call	DISP_ppO2_sensor_dive
	WIN_FONT	FT_SMALL

	return

o2_sensor_color_show:
    ; Sensors color marking
    movff	eCTRL_color_tmp,hi          ; copy status
	movlw	.3                          ; mask lower bits
    andwf   hi,F                        ; Mask two low bits
    incf    hi,F                        ; +1

	dcfsnz	hi,F
    bra o2_sensor_color_show_no         ; 0: Sensor disqualified
	dcfsnz	hi,F
    bra o2_sensor_color_show_alarm      ; 1: Alarm
	dcfsnz	hi,F
    bra o2_sensor_color_show_warning    ; 2: Warning

o2_sensor_color_show_ok:                ; 3: Ok
	movlw   color_green                 ; green
	call DISP_set_color;
    bra o2_sensor_color_show_exit

o2_sensor_color_show_alarm:
	movlw	color_red                   ; red
	call DISP_set_color;
    bra o2_sensor_color_show_exit

o2_sensor_color_show_warning:
	movlw	color_yellow                ; yellow
	call DISP_set_color;
    bra o2_sensor_color_show_exit

o2_sensor_color_show_no:
	movlw	color_blue                  ; blue
	call DISP_set_color;
    bra o2_sensor_color_show_exit

o2_sensor_color_show_exit:
    ;prepare for next sensor processing
    movff	eCTRL_color_tmp,hi          ; copy status
	rrncf	hi                          ; shift to right
	rrncf	hi                          ; shift to right
	movff	hi, eCTRL_color_tmp         ; store

	return

;Sensors color marking
o2_sensor_decomp:
    clrf    hi
    movlw   .200
    cpfsgt  lo  ;Less 200 - do nothing
	return
    ; Decompress data
    subwf   lo,f    ; -200
	movff   lo,xA+0
	clrf    xA+1
	movlw	.10
	movwf	xB+0
	clrf    xB+1
	call	mult16x16		;xA*xB=xC	-> xC:4 = (v-200)*10
	movlw   .200
    addwf	xC+0,F			    ; Add 200 to low byte
	clrf    WREG
	addwfc	xC+1,F			    ; and high byte.
	movff	xC+0,lo
	movff	xC+1,hi

	return

;-----------------------------------------------------------------------------
; Display ppO2 values for 3 oxy sensors in Surface mode

DISP_ppO2_sensors_surf:
	WIN_FONT	FT_SMALL
	WIN_LEFT	.90
	WIN_TOP     .25
	bsf		leftbind

    ;Copy Sensor Status to tmp for color processing
    movff   eCTRL_StatLo,eCTRL_color_tmp

	STRCPY  "O1"
	movff	eCTRL_ppo2_s1,lo		; copy measured value
	call	DISP_ppO2_sensor_surf

	WIN_TOP		.50
	STRCPY  "O2"
	movff	eCTRL_ppo2_s2,lo		; copy measured value
	call	DISP_ppO2_sensor_surf

	WIN_TOP		.75
	STRCPY  "O3"
	movff	eCTRL_ppo2_s3,lo		; copy measured value
	call	DISP_ppO2_sensor_surf

	WIN_TOP		.100
	STRCPY  "BAT "
    call   show_bat_value
    return

DISP_ppO2_sensor_dive_no_col:
	call    DISP_standard_color
    bra     DISP_ppO2_sensor_surf_1
DISP_ppO2_sensor_dive:
    rcall   o2_sensor_color_show
    bra     DISP_ppO2_sensor_surf_1
DISP_ppO2_sensor_surf_no_col:
	call    DISP_standard_color
    bra     DISP_ppO2_sensor_surf_0
DISP_ppO2_sensor_surf:
    rcall   o2_sensor_color_show
DISP_ppO2_sensor_surf_0:
    STRCAT  ": "
DISP_ppO2_sensor_surf_1:
    btfsc	ext_ppO2_valid
    bra     DISP_ppO2_sensor_do     ; show ppO2
    ; Data don't available - show dots instead ppO2
	STRCAT  "...."
    movff   win_font, lo
    tstfsz  lo                      ; If FONT MEDIUM (non 0)
    bra     DISP_ppO2_sensor_2
    bra     DISP_ppO2_sensor_3
DISP_ppO2_sensor_2:
	STRCAT  "..."                   ; Add dots for Divemode big font
DISP_ppO2_sensor_3:
	movlw	color_blue                          ; blue
	call    DISP_set_color;
    bra     DISP_ppO2_sensor_end

DISP_ppO2_sensor_do:
    rcall   o2_sensor_decomp
	output_16dp	d'3'				; outputs into Postinc2!

DISP_ppO2_sensor_end:
	call	word_processor
    return

DISP_show_sensor_mv:
	WIN_FONT    FT_SMALL
    WIN_LEFT    .159-.63                ; 9 chars aligned right.
    WIN_TOP     .170
	call		DISP_divemask_color     ; Set Color for Divemode mask
	bcf		leftbind
    clrf    hi
	STRCPY  "O1"
	movff	eCTRL_mv_s1,lo		; copy measured value
    call    DISP_sen_mv

	WIN_TOP		.192
	STRCPY  "O2"
	movff	eCTRL_mv_s2,lo		; copy measured value
    call    DISP_sen_mv

	WIN_TOP		.216
	STRCPY  "O3"
	movff	eCTRL_mv_s3,lo		; copy measured value
    call    DISP_sen_mv

	return

DISP_sen_mv:
	STRCAT  ": "
    btfsc	ext_ppO2_valid
    bra     DISP_sen_mv_do   ;show mv

	movlw	color_blue                          ; blue
	call    DISP_set_color;
	STRCAT  ".."
    bra     DISP_sen_mv_end

DISP_sen_mv_do:
	output_99				; outputs into Postinc2!

DISP_sen_mv_end:
	STRCAT  "mV"
	call	word_processor
    return

DISP_show_scrub_life:
	WIN_FONT    FT_SMALL
    WIN_LEFT    .159-.63                ; 9 chars aligned right.
    WIN_TOP     .170
	call		DISP_divemask_color     ; Set Color for Divemode mask
	bcf		leftbind
    clrf    hi
	STRCPY  "Scub.Time"
	call	word_processor

    WIN_TOP		.192
	lfsr	FSR2,letter

	call	DISP_standard_color
    GETCUSTOM15 eCTRL_CF_SCRLIF		; Scrubber alarm threshold
    movff   lo, sub_b+0
    movff   hi, sub_b+1
    call    calc_scrub_life_plus_divemins
    movff   lo,sub_a+0
    movff   hi,sub_a+1
    call    sub16                   ; Computes prediction - limit
    btfss   neg_flag                ; Negativ ?
    call    DISP_warnings_color     ; NO: go RED.
   	output_16       ;display value
	call	word_processor
	return

calc_scrub_life_plus_divemins:
    rcall    read_sctuber_life
    ;add actual dive time
    movff   divemins+0,WREG
    addwf   lo,F
    movff   divemins+1,WREG
    addwfc  hi,F
    return

read_sctuber_life:
    ;Load storred in internal eeprom values
	clrf    EEADRH                      ; Make sure to select eeprom bank 0
    read_int_eeprom     d'120'
	movff	EEDATA, lo
    read_int_eeprom     d'121'
	movff	EEDATA, hi
    return

DISP_more_list:
	call	DISP_standard_color
	WIN_FONT	FT_SMALL
#IFDEF CCR_CTRL_DBG_P
    call    DISP_more_p_plus
    call    DISP_more_p_minus
#ELSE
    DISPLAYTEXTH	.395                ; Increase SP4 menu item
    call    DISP_more_SP4
#ENDIF
    call    DISP_more_ppo2
    call    DISP_more_sol
    DISPLAYTEXTH	.397                ; Close

    return

DISP_more_SP4:
    ;Display newly adjusted SP#4
    bsf		leftbind
	WIN_LEFT	.100
	WIN_TOP		.25
	STRCPY  "SP4:"
	movff	eCTRL_SP3_adj,lo		; copy to lo
	clrf	hi
	output_16dp	d'3'		; outputs into Postinc2!
	call	word_processor
    bcf		leftbind
    return

DISP_more_ppo2:
	WIN_LEFT	.100
	WIN_TOP		.50
	STRCPY  "ppO2 "
    btfsc   ext_ppo2_abandon
    bra     DISP_more_ppo2a
	STRCAT  "EXT"
    bra     DISP_more_ppo2b
DISP_more_ppo2a:
	movlw		color_red					;
	call		DISP_set_color				; Set Color...
	STRCAT  "MAN"
DISP_more_ppo2b:
	call	word_processor
    call DISP_standard_color
    return

DISP_more_sol:
	WIN_LEFT	.100
	WIN_TOP		.75
	STRCPY  "SOL "
    btfsc   CTRL_sol_off_mode
    bra     DISP_more_sola
	STRCAT  "ON  "
    bra     DISP_more_solb
DISP_more_sola:
	movlw		color_red					;
	call		DISP_set_color				; Set Color...
	STRCAT  "OFF "
DISP_more_solb:
	call	word_processor
    call DISP_standard_color
    return

#IFDEF CCR_CTRL_DBG_P
DISP_more_p_plus:
	WIN_LEFT	.100
	WIN_TOP		.0
	STRCPY  "P "
    movff   eCTRL_P, lo
    output_8
	STRCAT  "+"
	call	word_processor
    return

DISP_more_p_minus:
	WIN_LEFT	.100
	WIN_TOP		.25
	STRCPY  "P "
    movff   eCTRL_P, lo
    output_8
	STRCAT  "-"
	call	word_processor
    return
#ENDIF

#ENDIF