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


; Menu "Custom Functions", Custom Functions checker (Displays permanent warning if critical custom functions are altered)
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 051030
; last updated: 120421
; known bugs:
; ToDo: 

;First Bank of Custom Functions:
; The custom functions are stored in the internal EEPROM after 0x80
; any custom function occupies 4 byte:
; 2 byte (low:high) store the default value, reset from menu "reset"
; if bit16=1 then the custrom function is 15bit value, if not it's a 8bit value
; 2 byte (low:high) store the actual value
; defaults for custom functions are in menu "reset"
; get values with GETCUSTOM8	.x with x=0...32 for 8 Bit values (stored in WREG)
; or with GETCUSTOM15	.x with x=0...32 for 15 Bit values (stored in lo and hi)

;Second Bank of Custom Functions:
; The custom functions are stored in the internal EEPROM after 0x180
; any custom function occupies 4 byte:
; 2 byte (low:high) store the default value, reset from menu "reset"
; if bit16=1 then the custrom function is 15bit value, if not it's a 8bit value
; 2 byte (low:high) store the actual value
; defaults for custom functions are in menu "reset"
; get values with GETCUSTOM8	.x with x=0...32 for 8 Bit values (stored in WREG)
; or with GETCUSTOM15	.x with x=0...32 for 15 Bit values (stored in lo and hi)

;Third Bank of Custom Functions:
; The custom functions are stored in the internal EEPROM after 0x280
; any custom function occupies 4 byte:
; 2 byte (low:high) store the default value, reset from menu "reset"
; if bit16=1 then the custrom function is 15bit value, if not it's a 8bit value
; 2 byte (low:high) store the actual value
; defaults for custom functions are in menu "reset"
; get values with GETCUSTOM8	.x with x=0...32 for 8 Bit values (stored in WREG)
; or with GETCUSTOM15	.x with x=0...32 for 15 Bit values (stored in lo and hi)

; [jDG] 2010-11-30 More fancy displsy of the various CF types
; data types. When we do have a 8bit data (bit16=0), the high byte serves to
; define the display format. Also stores min/max bounds into the PROM table.
; And provides surfacemode checking of all parameters.

CF_INT8		EQU	0       ; Default display, 8 or 15 bits values.
CF_PERCENT	EQU	1	    ; Displays 110%
CF_DECI		EQU	2	    ; Displays 1.6
CF_CENTI	EQU	3	    ; Displays 1.50
CF_MILI		EQU	4	    ; Displays 1.015
CF_BOOL		EQU	5	    ; Displays ON/OFF
CF_SEC		EQU	6	    ; Displays 4:00
CF_COLOR	EQU	7	    ; Display 240 plus a color watch (inverse video space)
;
CF_TYPES    EQU 0x0F    
CF_MAX_BIT  EQU 6       ; Default is the highest safe value.
CF_MAX      EQU (1<<CF_MAX_BIT)
CF_MIN_BIT  EQU 5       ; Default is the lowest safe value.
CF_MIN      EQU (1<<CF_MIN_BIT)
CF_NEG_BIT  EQU 4       ; Allow negativ values.
CF_NEG      EQU (1<<CF_NEG_BIT)
;
CF_INT15	EQU	0x80; Default display. Flag for 15bit, typeless values.

;=============================================================================
; Overlay our tmp data in ACCESS0 bank
    CBLOCK tmp                  ; Into safe (from C library) area.
        cf32_x4             ; CF# modulus 32, time 4.
        cf_type             ; Type of the edited CF
        cf_default:2
        cf_value:2   
        cf_min     
        cf_max     
        cf_step             ; Value ad add/substract: 1, 10, 100
		cf_page_number		; CF page number (0: 0-31, 1: 32-63)
		cf_title_text		; # of text for title
		cf_descriptor_text	; # of descriptor text offset
    ENDC

;=============================================================================

GETCUSTOM8	macro	custom8
	movlw	custom8
	call	getcustom8_1
	endm

getcustom8_1:
	; # number of requested custom function in wreg
	movwf	customfunction_temp1

	clrf	EEADRH	
	movlw	d'31'
	cpfsgt	customfunction_temp1
	bra		getcustom8_3			; bank 0

	movlw	d'1'
	movwf	EEADRH					; bank 1
	movlw	d'32'
	subwf	customfunction_temp1,F
	movlw	d'31'
	cpfsgt	customfunction_temp1
	bra		getcustom8_3

	movlw	d'2'
	movwf	EEADRH					; bank 2
	movlw	d'32'
	subwf	customfunction_temp1,F
getcustom8_3:
	movf	customfunction_temp1,W
	mullw	d'4'
	movf	PRODL,W			; x4 for adress
	addlw	d'130'
	movwf	EEADR			; +130 for LOW Byte of value
	call	read_eeprom		; Lowbyte
	movf	EEDATA,W		; copied into wreg
	clrf	EEADRH
	return					; return

GETCUSTOM15	macro	number
	movlw	number
	call	getcustom15
	endm

    global  getcustom15
getcustom15:
	; # number of requested custom function in wreg
	movwf	customfunction_temp1
	
	clrf	EEADRH	
	movlw	d'31'
	cpfsgt	customfunction_temp1
	bra		getcustom15_3			; bank 0

	movlw	d'1'
	movwf	EEADRH					; bank 1
	movlw	d'32'
	subwf	customfunction_temp1,F
	movlw	d'31'
	cpfsgt	customfunction_temp1
	bra		getcustom15_3			; bank 1

	movlw	d'2'
	movwf	EEADRH					; bank 2
	movlw	d'32'
	subwf	customfunction_temp1,F
getcustom15_3:
	movf	customfunction_temp1,W
	mullw	d'4'
	movf	PRODL,W			; x4 for adress
	addlw	d'130'
	movwf	EEADR			; +130 for LOW Byte of value
	call	read_eeprom		; Lowbyte
	movff	EEDATA,lo
	incf	EEADR,F
	call	read_eeprom		; Highbyte
	movff	EEDATA,hi
	clrf	EEADRH
	return					; return

menu_custom_functions_page3:
	movlw	.2
	movff	WREG,cf_page_number		; CF page number (0: 0-31, 1: 32-63)
	movlw	.225
	movff	WREG,cf_title_text		; # of text for title
	movlw	.193
	movff	WREG,cf_descriptor_text	; # of descriptor text offset
	bra		menu_custom_functions0

menu_custom_functions_page2:
	movlw	.1
	movff	WREG,cf_page_number		; CF page number (0: 0-31, 1: 32-63)
	movlw	.186
	movff	WREG,cf_title_text		; # of text for title
	movlw	.154
	movff	WREG,cf_descriptor_text	; # of descriptor text offset
	bra		menu_custom_functions0

menu_custom_functions:
	movlw	.0
	movff	WREG,cf_page_number		; CF page number (0: 0-31, 1: 32-63)
	movlw	.27
	movff	WREG,cf_title_text		; # of text for title
	movlw	.36
	movff	WREG,cf_descriptor_text	; # of descriptor text offset
	
menu_custom_functions0:
	bsf		leftbind
	call	DISP_ClearScreen
	movlw	d'1'
	movwf	menupos

	bcf		menubit4
	bcf		sleepmode
	clrf	decodata+0				; here: # of CustomFunction
	clrf	cf32_x4                 ; here: # of CustomFunction*4
	bcf		first_FA				; here: =1: -, =0: +
    movlw   1                       ; Stepsize: 1, 10, or 100.
    movwf   cf_step

    call	DISP_divemask_color
	movff	cf_title_text,WREG		; Title text in low bank
	call	displaytext_1_low

menu_custom_functions1:
	call	DISP_standard_color         ; Trash EEADRH...

	movff	cf_page_number,EEADRH		; CF page number (0: 0-31, 1: 32-63)

	clrf	timeout_counter2
	bcf		menubit2
	bcf		menubit3
	WIN_LEFT 	.20
	WIN_TOP		.35
	lfsr	FSR2,letter
	movff	decodata+0,lo               ; decodata == CF number % 32

	movff	cf_page_number,WREG			; CF page number (0: 0-31, 1: 32-63)
	mullw	.32							; CF page number * 32 -> PRODL:PRODH
	movf	PRODL,W
	addwf	lo,F						; Add offset for display in CF menu

	output_99x
	STRCAT_PRINT ": "
	movff	cf_descriptor_text,WREG		; start of custom function descriptors		
	addwf	decodata+0,W				; add # of current custom function, place result in wreg
	call	displaytext_1_low           ; shows descriptor

; Read default, type and min/max from reset table.
    rcall    cf_read_default

	movf	cf_type,W					; Is it a ON/OFF flag ?
	xorlw	CF_BOOL
	bnz		menu_custom_functions10a	; Not a  binary CF selected

menu_custom_functions10c:
    ; Erase unused lines when editing boolean...
	WIN_LEFT 	.20
	WIN_TOP		.65
	lfsr	FSR2,letter					; Make a string of 8 spaces
	call    cf_fill_line
	call	word_processor				; Clear +/- line

	WIN_TOP		.95
	call	word_processor				; Clear 1/10 line
 
	bra		menu_custom_functions10b

menu_custom_functions10a:
	WIN_LEFT 	.20
	WIN_TOP		.65
	STRCPY  "+/-: "
	movlw	'+'
	btfsc	first_FA
	movlw	'-'
	movwf	POSTINC2
	call	word_processor		

	WIN_TOP		.95
	STRCPY  TXT_STEP5
    clrf    hi
    movff   cf_step,lo
	call	display_formated	        ; Typed display, w/o fill line.
	STRCAT_PRINT "   "                   ; 2 spaces for "0.01"->"1"

menu_custom_functions10b:
	WIN_LEFT 	.20
	WIN_TOP		.125
	lfsr	FSR2,letter
	OUTPUTTEXT	d'89'				    ; "Default:"

    movff   cf_default+0,lo
    movff   cf_default+1,hi
	call	display_customfunction	    ; Typed display.

	WIN_LEFT 	.20
	WIN_TOP		.155
	lfsr	FSR2,letter
	OUTPUTTEXT	d'97'			        ; "Current:"

	movf	cf32_x4,W
	addlw	0x82
	movwf	EEADR
	call	read_eeprom				; Lowbyte
	movff   EEDATA,cf_value+0

	movf	cf32_x4,W
	addlw	0x83
	movwf	EEADR
	call	read_eeprom				; Highbyte
	movff	EEDATA,cf_value+1

	call    DISP_standard_color     ; Changed by color swatches, but trash EEADRH...
    movff   cf_value+0,lo
    movff   cf_value+1,hi
	call	display_customfunction

; End of mask: min/max and the exit line...
	rcall display_minmax
	DISPLAYTEXT	.11					; Exit

	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!
	call	DISP_menu_cursor

customfunctions_loop:
	call    check_switches_logbook

	btfsc   menubit3
	bra     customfunctions2		    ; Move cursor or generate next page

	btfsc   menubit2
	bra     do_customfunction		    ; call subfunction

	btfsc	divemode
	goto	restart					    ; dive started during cf menu

	btfsc	onesecupdate
	call	timeout_surfmode

	btfsc	onesecupdate
	call	set_dive_modes

	bcf		onesecupdate			    ; end of 1sek. tasks

	btfsc	sleepmode
	bra     exit_customfunctions

	bra     customfunctions_loop

customfunctions2:
	incf	menupos,F
	movf    cf_type,W               ; Are we editing a boolean value ?
	xorlw   CF_BOOL
	bnz     customfunctions2a       ; NO : don't skip lines 2/3.
	
	movlw   d'4'                    ; Just after increment,
	cpfsgt  menupos                 ; Is current position < 4 ?
	movwf   menupos                 ; NO: skip set to 4.

customfunctions2a:
	movlw	d'7'
	cpfseq	menupos					; =7?
	bra		customfunctions3		; No
	movlw	d'1'
	movwf	menupos

customfunctions3:
	clrf	timeout_counter2
	call	DISP_menu_cursor
	
	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!

	bcf		menubit3				; clear flag
	bra		customfunctions_loop

;-----------------------------------------------------------------------------
; Read default value, type, and constraints
;
; Input: cf32_x4
; Output: cf_default, cf_type, cf_min, cf_max.
; Trashes: TBLPTR

cf_read_default:
    movlw   LOW(cf_default_table0)      ; Get 24bit PROM pointer. SKIP 
    movwf   TBLPTRL
    movlw   HIGH(cf_default_table0)
    movwf   TBLPTRH
    movlw   UPPER(cf_default_table0)
    movwf   TBLPTRU

	movff	cf_page_number,WREG			; CF page number (0: 0-31, 1: 32-63)
	mullw	0x80						; CF page number * 0x80 -> PRODL:PRODH

	movf	PRODL,W
	addwf   cf32_x4,W                   ; Add 4 x (CF index modulus 32)
    addwf   TBLPTRL,F                   ; And to a 8+16 add into TBLPTR
	movf	PRODH,W
    addwfc  TBLPTRH,F                   ; Propagate to 16bit (but not 24bits).

    tblrd*+
    movff   TABLAT,cf_default+0         ; Low byte
    tblrd*+
    movff   TABLAT,cf_default+1         ; High byte
    btfss   cf_default+1,7              ; 15bit ?
    clrf    cf_default+1                ; NO: clear extra type flags
    bcf     cf_default+1,7              ; clear 15bit flag
    
    movff   TABLAT,cf_type              ; type (high byte) --> cf_type

    tblrd*+
    movff   TABLAT,cf_min               ; Then get optional min/max
    tblrd*+
    movff   TABLAT,cf_max

    return

;-----------------------------------------------------------------------------
; Display a 8/15bit value, plus optional min and max bound.
; Input : hi:lo = data to display.
;         cf_type = the type.
;         cf_min, cf_max : the optional min/max.
;         FSR2  = current string pointer.
; Trash : hi:lo (when displaying min/max)

display_customfunction:
	movff	EEADRH, FSR1H		; Backup...

	rcall   display_formated
	incf    WREG                        ; Color swatch drawn ?
	bz      display_customfunction_1    ; YES: skip line fill...
	
    rcall   cf_fill_line
	call	word_processor

display_customfunction_1:
	movff	FSR1H, EEADRH
	return

;-----------------------------------------------------------------------------
; Display optional min/max values.
; Inputs: cf_value, cf_min, cf_max (and cf_type to display min/max).
; Trashed: hi:lo while display min and max values.
display_minmax:
; Min/max unsupported for 15bit values yet...
	movff	EEADRH, FSR1H		; Backup...

; Display min line
    WIN_TOP  .65
    WIN_LEFT .100
    lfsr    FSR2, letter

    btfsc   cf_type,7           ; A 15bit value ?
    bra     cf_no_min           ; Don't display, hence clear line...

    btfss   cf_type,CF_MIN_BIT  ; A min value exists ?
    bra     cf_no_min

    btfss   cf_type,CF_NEG_BIT
    bra     cf_min_unsigned

    ; Uses 16bit sub for checking signed min value.
    movff   cf_value,sub_a+0    ; A <- value
    clrf    sub_a+1
    btfsc   cf_value,7          ; extend sign if value < 0
    setf    sub_a+1

    movff   cf_min,sub_b+0      ; B <- min (with signed extend)
    setf    sub_b+1             ; min have to be negativ.
    call    sub16               ; Compute (A-B)

    btfss   neg_flag            ; Result < 0 ?
    bra     cf_min_passed       ; NO
    bra     cf_min_failed       ; YES

cf_min_unsigned:
    movf    cf_min,W            ; Retrieve current 8b value
    subwf   cf_value,W          ; Compute (value-min)
    bc      cf_min_passed       ; Ok if CARRY, ie. min >= lo

cf_min_failed:
    call    DISP_warnings_color
    WIN_INVERT  1

cf_min_passed:    
    STRCAT  "> "                ; A min value follows
    movff   cf_min, lo
    rcall   display_formated

cf_no_min:
    rcall   cf_fill_line        ; Fill buffer
    lfsr    FSR2, letter+.7     ; Limit to 8 chars btw.
	call	word_processor

; Display max line
    WIN_TOP  .95
    call    DISP_standard_color
    WIN_INVERT 0
    lfsr    FSR2, letter

    btfsc   cf_type,7           ; A 15bit value ?
    bra     cf_no_max           ; Don't display, hence clear line too...

    btfss   cf_type, CF_MAX_BIT ; A max value exists ?
    bra     cf_no_max

    btfss   cf_type,CF_NEG_BIT
    bra     cf_max_unsigned

    ; Uses 16bit sub for checking signed min value.
    movff   cf_max,sub_a+0      ; A <- max (with signed extend)
    clrf    sub_a+1             ; max have to be positiv.

    movff   cf_value,sub_b+0    ; B <- value
    clrf    sub_b+1
    btfsc   cf_value,7          ; extend sign if value < 0
    setf    sub_b+1
    call    sub16               ; Compute (A-B)

    btfss   neg_flag            ; Result < 0 ?
    bra     cf_max_passed       ; NO
    bra     cf_max_failed       ; YES

cf_max_unsigned:
    movf    cf_value,W          ; Retrieve current max bound
    subwf   cf_max,W            ; Compute (max-lo)
    bc      cf_max_passed       ; Ok if no carry, ie. max <= lo

cf_max_failed:
    call    DISP_warnings_color
    WIN_INVERT  1

cf_max_passed:    
    STRCAT  "< "                ; A max value follows
    movff   cf_max, lo
    rcall   display_formated

cf_no_max:
    rcall   cf_fill_line        ; Fill buffer
    lfsr    FSR2, letter+.7     ; Limit to 8 chars btw.
	call	word_processor

cf_minmax_done:
    call    DISP_standard_color
    WIN_INVERT  0
	movff	FSR1H, EEADRH
	return

;-----------------------------------------------------------------------------
; Display a single 8/15 bit value, according to cf_type.
; Input : hi:lo = data to display.
;         cf_type = the type.
;         cf_min, cf_max : the optional min/max.
;         FSR2  = current string pointer.
display_formated:
	movf	cf_type,W			; Just set N flags
	bn		cf_type_80			; Keep 15bits value in old format.

    ;---- handle signed values -----------------------------------------------
    ; NOTE: only 8bit values can have a negativ flag right now.
    btfss   cf_type,CF_NEG_BIT  ; Signed value ?
    bra     cf_type_unsigned    ; NO: display unsigned as-is

    btfss   lo,7                ; Negativ value ?
    bra     cf_type_pos         ; NO: display positives with a + sign.
    
    PUTC    '-'                 ; YES: display with a - sign.
    negf    lo                  ; and correct the said value.
    bra     cf_type_unsigned

cf_type_pos:
    PUTC    '+'

	;---- decode type --------------------------------------------------------
cf_type_unsigned:
	; Jump table:               ; test the value with cleared flags...
    movf    cf_type,W
    andlw   CF_TYPES            ; Look just at types
	bz		cf_type_00			; 8bit standard mode

	dcfsnz	WREG
	bra		cf_type_01
	dcfsnz	WREG
	bra		cf_type_02
	dcfsnz	WREG
	bra		cf_type_03
	dcfsnz	WREG
	bra		cf_type_04
	dcfsnz	WREG
	bra		cf_type_05
	dcfsnz	WREG
	bra		cf_type_06
	dcfsnz	WREG
	bra		cf_type_07
	bra		cf_type_00			; Default to 8bit mode...

cf_type_01:						; Type == 1 is CF_PERCENT mode
    bcf     leftbind
	output_8
	PUTC    '%'
	retlw   0

cf_type_02:						; Type == 2 is CF_DECI mode.
    clrf    hi
    bsf     leftbind
	output_16dp	4
	retlw   0

cf_type_03:						; Type == 3 is CF_CENTI mode.
    clrf    hi
    bsf     leftbind
	output_16dp	3
	retlw   0

cf_type_04:						; Type == 4 is CF_MILI mode
	output_16dp	2
	retlw   0

cf_type_05:						; Type == 5 is CF_BOOL mode.
	movf	lo,W				; Get flag value...
	bz		cf_type_off
	OUTPUTTEXT	d'130'			; ON
	retlw   0

cf_type_off:
	OUTPUTTEXT	d'131'			; OFF
	retlw   0

cf_type_06:						; Type == 6 is CF_SECS mode (mm:ss or hh:mm)
    clrf    hi
	call	convert_time		; Convert to min:sec into hi:low.
	movff	lo,TABLAT			; Save seconds for later.
	movff	hi,lo				; Get minutes
    bsf     leftbind            ; Skip leading space(s).
	output_8					; Print them
	PUTC	':'					; Separator
	movff	TABLAT,lo			; Get back seconds
	output_99x					; lo in 2 digits with trailing zeros.
	retlw   0

cf_type_07:						; Type == 7 is CF_COLOR swatch.
    bcf     leftbind            ; Keep leading space (better alignement)
	output_8

    movff   win_top,WREG        ; Is it the step value ?
    xorlw   .95                 ; Line for "Step:"
    btfsc   STATUS,Z            
    retlw   -1                  ; YES : return

	STRCAT_PRINT " "
	movf	lo,W				; Get color.
	call    DISP_set_color
	movlw	.23
	movff	WREG,win_height		; row bottom (0-239)
	movlw	.110
	movff	WREG,win_leftx2		; column left (0-159)
	movlw	.148-.110+1	
	movff	WREG,win_width		; column right (0-159)

	call	DISP_box
    retlw   -1  				; wp already done. Skip it...

cf_type_00:						; 8bit mode. Or unrecognized type...
	clrf	hi
    bsf     leftbind

cf_type_80: 					; 15bit mode.
	bcf		hi,7
	output_16
	retlw   0

;-----------------------------------------------------------------------------

cf_fill_line:                   ; Mattias: No flicker if u clear just what you need...
	movf    FSR2L,W             ; How many chars lefts ?
	sublw   letter + .18        ; Remaining chars to fill: (letter + 18) - PTR
	btfsc   STATUS,N            ; Add chars until none left...
	return
	PUTC   ' '
	bra     cf_fill_line

;-----------------------------------------------------------------------------

do_customfunction:
	movff	cf_page_number,EEADRH		; CF page number (0: 0-31, 1: 32-63)

	dcfsnz	menupos,F
	bra		next_customfunction
	dcfsnz	menupos,F
	bra		toggle_plusminus
	dcfsnz	menupos,F
	bra		toggle_oneorten
	dcfsnz	menupos,F
	bra		restore_cfn_value
	dcfsnz	menupos,F
	bra		adjust_cfn_value

;-----------------------------------------------------------------------------

exit_customfunctions:
	movff	cf_page_number,menupos	; CF page number (0: 0-31, 1: 32-63)
	incf	menupos,F
	clrf	EEADRH					; Clear EEADRH !
	goto	setup_menu2				; exit...

;-----------------------------------------------------------------------------

next_customfunction:
	incf	decodata+0
	btfsc	decodata+0,5			;>31?
	clrf	decodata+0				;Yes, so reset to zero
	
	movf	decodata+0,W
	mullw	d'4'
	movff	PRODL, cf32_x4          ; 12bit address for correct addressing

	movlw	d'1'
	movwf	menupos
	bra		menu_custom_functions1  ; also debounces switches

;-----------------------------------------------------------------------------

toggle_plusminus:
	btg		first_FA
	movlw	d'2'
	movwf	menupos
	bra		menu_custom_functions1	; also debounces switches

;-----------------------------------------------------------------------------

toggle_oneorten:
    movlw   .10                     ; Multiply step by 10,
    mulwf   cf_step                 ; Result in PROD low.

    movf    PRODH,W                 ; Check 1000
    bz      toggle_oneorten_1       ; HIGH(new step) null == no overflow
    movlw   .1                      ; Cycle to 1.
    movwf   cf_step
    bra     toggle_oneorten_3

toggle_oneorten_1:                  ; Special case for mm:ss
    movf    cf_type,W               ; Get type
    andlw   CF_TYPES                ; w/o min/max/neg flags.
    xorlw   CF_SEC                  ; Check for mm:ss ?
    bnz     toggle_oneorten_2       ; no: continue
    movlw   .100                    ; Step = 100 ?
    xorwf   PRODL,W
    bnz     toggle_oneorten_2       ; no: continue
    movlw   .60                     ; yes: replace by 1:00
    movff   WREG,cf_step
    bra     toggle_oneorten_3       ; Done.

toggle_oneorten_2:
    movff   PRODL,cf_step           ; Just keep result.
toggle_oneorten_3:
	movlw	d'3'
	movwf	menupos
	bra		menu_custom_functions1	; also debounces switches

;-----------------------------------------------------------------------------

restore_cfn_value:
	movf	cf32_x4,W               ; store default value
	addlw	0x82
	movwf	EEADR
	movff	cf_default+0,EEDATA
	movff	cf_default+0,cf_value+0
	call	write_eeprom			; Lowbyte
	movf	cf32_x4,W
	addlw	0x83
	movwf	EEADR
	movff	cf_default+1,EEDATA
	movff	cf_default+1,cf_value+1
	call	write_eeprom			; Highbyte

	movlw	d'4'
	movwf	menupos
	bra		menu_custom_functions1	; also debounces switches

;-----------------------------------------------------------------------------
; Adjust current value.
adjust_cfn_value:
	movf	cf32_x4,W               ; get current value
	addlw	0x82
	movwf	EEADR
	call	read_eeprom				; Lowbyte
	movff	EEDATA,lo
	movf	cf32_x4,W
	addlw	0x83
	movwf	EEADR
	call	read_eeprom				; Highbyte
	movff	EEDATA,hi

    movf    cf_type,W
    xorlw   CF_BOOL
    bnz      adjust_cfn_value1

	btg     lo,0			            ; Change lower bit.
	bra		adjust_cfn_value3		    ; Store result

adjust_cfn_value1:
    movf    cf_step,W                   ; 1, 10, 100 ?
	
	btfss	first_FA				    ; Minus?
	bra		adjust_cfn_value2	    	; No, Plus

	subwf	lo,F					    ; substract value
	movlw	d'0'
	btfsc	cf_type,7			        ; 8Bit value
	subwfb	hi,F

	movlw	b'01111111'
	btfsc	hi,7				        ; >32768?
	movwf	hi						

	bra		adjust_cfn_value3
	
adjust_cfn_value2:
	addwf	lo,F					    ; add value
	movlw	d'0'
	btfsc	cf_type,7			        ; 8Bit value?
	addwfc	hi,F

	btfsc	hi,7					    ; >32768?
	clrf	hi

adjust_cfn_value3:
	movf	cf32_x4,W                   ; Store current value
	addlw	0x82
	movwf	EEADR
	movff	lo,EEDATA
	call	write_eeprom			    ; Lowbyte
	movf	cf32_x4,W
	addlw	0x83
	movwf	EEADR
	movff	hi,EEDATA
	call	write_eeprom			    ; Highbyte 
	movlw	d'5'
	movwf	menupos
	bra		menu_custom_functions1	    ; also debounces switches

;-----------------------------------------------------------------------------
; Check all CF values that have max and min constraints.
; Input: cf_checker_counter.
; Output: Pop warning with the first CF number if bad.
;        cf_checker_counter incremented.
; Trashes: TBLPTR, EEADR, PROD
;
; Note: the trick here is to do two sweep over the 64 CF values, to make sure
;       they are all ok.

check_customfunctions:
;	movlw	max_custom_number+1			; Defined in definitions.asm
    movlw   .63                         ; mH: CF checker does currently not work for Bank2
	cpfseq	cf_checker_counter			; All tested?
	bra		check_customfunctions1		; No, continue
	clrf	cf_checker_counter			; clear counter
	return								; YES: just do nothing.

check_customfunctions1:
	; Setup cf_page_number
	movlw	.0
	movff	WREG,cf_page_number
	movlw	d'31'
	cpfsgt	cf_checker_counter
	bra		check_customfunctions2	; CF I

	movlw	.1
	movff	WREG,cf_page_number
	movlw	d'63'
	cpfsgt	cf_checker_counter
	bra		check_customfunctions2	; CF II

	movlw	.2
	movff	WREG,cf_page_number		; CF III
	
check_customfunctions2:
    ; Setup cf32_x4
    movf    cf_checker_counter,W
    rlcf    WREG                        ; x4
    rlcf    WREG
    andlw   4*.31
    movwf   cf32_x4
    
    ; Do the check
    rcall   check_one_cf
    iorwf   WREG                        ; Test return value.
    bz      check_failed                ; 0 == FAILED.
    
    ; Passed: Simple loop until 128 is reached:
    incf    cf_checker_counter,F        ; Next CF to check.
    bra     check_customfunctions       ; And loop until 128 reached (ie twice)
    
check_failed:
    movlw   .63                         ; Make sure number is back to range 0..63
    andwf   cf_checker_counter,F

    ; Went wrong: draw the warning line...
	WIN_TOP		.0
	WIN_LEFT	.125
	WIN_FONT 	FT_SMALL
	WIN_INVERT	.1					    ; Init new Wordprocessor
	call    DISP_warnings_color
	
	STRCPY  TXT_CF2
    movff   cf_checker_counter,lo
    output_99x
	STRCAT_PRINT "!"
    WIN_INVERT	.0					    ; Init new Wordprocessor
    
    ; When failed, increment counter modulo 64, to restart checks.
    incf    cf_checker_counter,W
    andlw   .63                     ; Modulo 64
    movwf   cf_checker_counter
    return

; Check one CF value ---------------------------------------------------------
check_one_cf:
    rcall   cf_read_default             ; Sets cf_value, cf_type, cf_min, cf_max.
    
    btfsc   cf_type,7                   ; A 15bit type ?
    bra     check_cf_check              ; Then we have to check it...

    movf    cf_type,W                   ; 8bit MIN or MAX set ?
    andlw   (CF_MIN + CF_MAX)
    bnz     check_cf_check              ; yes: do something.
    retlw   -1                          ; no: no problem then.

; It does have bound:
check_cf_check:
    movf    cf_checker_counter,W        ; Get current CF number
    andlw   .63                         ; Keep range 0..63

    btfss   cf_type,7                   ; 15 or 8 bit value ?
    bra     cf_check_8bit
    
; Implement the 15bit check, even if not displayed...
    rcall   getcustom15                 ; Read into hi:lo

    movf    cf_min,W                    ; Compute (bound-value) -> hi:lo
    subwf   lo,F
    movf    cf_max,W
    bcf     WREG,7                      ; Clear min/max bit
    subwfb  hi,F

    movf    lo,W                        ; Is it a 0 result ?
    iorwf   hi,W
    bnz     cf_15_not_equal             ; NO: check sign.
    retlw   -1                          ; YES: then it is ok.

cf_15_not_equal:
    btfss   cf_max,7                    ; Checking min or max ?
    btg     hi,6                        ; exchange wanted sign
    
    btfss   hi,6                        ; Is sign correct ?
    retlw   0                           ; NO: return failed.
    retlw   -1                          ; YES: return passed.

; Do a 8bit check
cf_check_8bit:
    rcall   getcustom8_1                ; Read into WREG
    movwf   lo                          ; Save it.

    btfss   cf_type,CF_MIN_BIT
    bra     check_no_min

    btfss   cf_type,CF_NEG_BIT
    bra     check_min_unsigned

    ; Uses 16bit sub for checking signed min value.
    movff   lo,sub_a+0          ; A <- value
    clrf    sub_a+1
    btfsc   lo,7                ; extend sign if value < 0
    setf    sub_a+1

    movff   cf_min,sub_b+0      ; B <- min (with signed extend)
    setf    sub_b+1             ; min have to be negativ.
    call    sub16               ; Compute (A-B)

    btfss   neg_flag            ; Result < 0 ?
    bra     check_no_min        ; NO
    retlw   0                   ; YES = FAILED

check_min_unsigned:
    cpfsgt  cf_min                      ; Compare to cf_min
    bra     check_no_min                ; PASSED: continue.
    retlw   0                           ; NO: return failed.

check_no_min:
    btfss   cf_type,CF_MAX_BIT          ; Is there a MAX bound ?
    retlw   -1                          ; No check: return OK.

    btfss   cf_type,CF_NEG_BIT
    bra     check_max_unsigned

    ; Uses 16bit sub for checking signed min value.
    movff   cf_max,sub_a+0      ; A <- max (with signed extend)
    clrf    sub_a+1             ; max have to be positiv.

    movff   lo,sub_b+0          ; B <- value
    clrf    sub_b+1
    btfsc   lo,7                ; extend sign if value < 0
    setf    sub_b+1
    call    sub16               ; Compute (A-B)

    btfss   neg_flag            ; Result < 0 ?
    retlw   -1                  ; NO
    retlw   0                   ; YES

check_max_unsigned:
    movf    lo,W                        ; Compute value-max
    cpfslt  cf_max
    retlw   -1                          ; Bound met: return OK.
    retlw   0                           ; NO: return failed.
