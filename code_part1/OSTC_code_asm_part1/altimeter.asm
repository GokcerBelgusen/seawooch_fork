;=============================================================================
;
;    File altimeter.asm
;
;    Altimeter function prototype.
;
;    This program is free software: you can redistribute it and/or modify
;    it under the terms of the GNU General Public License as published by
;    the Free Software Foundation, either version 3 of the License, or
;    (at your option) any later version.
;
;    This program is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;    GNU General Public License for more details.
;
;    You should have received a copy of the GNU General Public License
;    along with this program.  If not, see <http://www.gnu.org/licenses/>.
;
;    Copyright (c) 2010, JD Gascuel.
;=============================================================================
; HISTORY
;  2010-12-15 : [jDG] First prototype with quadratic polynomial ant tp°.
;  2010-12-28 : [jDG] Use MPLAB Math and C libraries for FP32 computations.
;  2011-01-02 : [jDG] Edit reference pressure by 0.25 mbar.
;  2011-01-31 : [jDG] Better menu: default 1013mbar, and editing by +/- 1mbar.
;  2011-02-23 : [jDG] Fix restart after sleepmode.
;
; Known bug: Simulator reset altitude and reference...

altimeter_calc:
        movlb   HIGH(pressureAvg)       ; Altimeter data in bank 0.
        
        movlw   HIGH(4*.900)            ; Is presure ref bigger than 900mbar
        cpfsgt  pressureRef+1
        bra     altimeter_reset         ; No: Should do a reset now.
        
        movlw   HIGH(4*.1100)           ; Is ref pressure bigger than 1100mbar ?
        cpfsgt  pressureRef+1
        bra     altimeter_1             ; No: ok it is valid...

; Reset calibration value to default.
altimeter_reset:
        movlb   HIGH(pressureAvg)       ; Altimeter data in bank 0.
        movlw   LOW(4*.1013+1)          ; Init see level at 1013,25 mbar.
        movwf   pressureRef+0
        movlw   HIGH(4*.1013+1)
        movwf   pressureRef+1

; Restart averaging. Eg. after a sleep, enables to faster restart with correct
; values...
altimeter_restart:
        movlb   HIGH(pressureAvg)       ; Altimeter data in bank 0.
        clrf    pressureSum+0           ; Init averaging area
        clrf    pressureSum+1
        clrf    pressureCount

        SAFE_2BYTE_COPY amb_pressure, pressureAvg   ; And init first average.

        movlw   4                       ; And multiply AVG by 16 to be coherent.
altimeter_reset_1:
        bcf     STATUS,C
        rlcf    pressureAvg+0
        rlcf    pressureAvg+1
        decfsz  WREG
        bra     altimeter_reset_1
        
        rcall   compute_altitude

        movlb   1                       ; Back to normal bank1.
        return

altimeter_1:
        ;---- Do a bank-safe 16bit summing -----------------------------------
        SAFE_2BYTE_COPY amb_pressure, lo   ; And init first average.

        movff   lo,WREG
        addwf   pressureSum+0,F
        movff   hi,WREG
        addwfc  pressureSum+1,F

        incf    pressureCount           ; Increment count too.
        movlw   .32                     ; Already 32 done ?
        subwf   pressureCount,W
        bnz     altimeter_4             ; NO: skip update.

        ;---- Update altitude every 32 pressure measures --------------------
        bcf     STATUS,C                ; Divide by 2, to store pressure x16
        rrcf    pressureSum+1
        rrcf    pressureSum+0
        
        movff   pressureSum+0,pressureAvg+0
        movff   pressureSum+1,pressureAvg+1

        rcall   compute_altitude        ; Compute from the averaged value...

        clrf    pressureSum+0           ; And reset sum zone for next averaging.
        clrf    pressureSum+1
        clrf    pressureCount

altimeter_4:
        movlb   1                       ; make sure to be in normal bank1
        return

        ;---- Display result -------------------------------------------------
altimeter_display:
        GETCUSTOM8  .49                 ; Check CF#49
        btfss   WREG,0                  ; Enabled ?
        return                          ; NO: return

        WIN_TOP     .35                 ; Custom view drawing zone...
        WIN_LEFT    .82
        WIN_FONT    FT_SMALL
        call    DISP_standard_color

        STRCPY  TXT_ALT5

        movff   altitude+0,lo           ; BANK-SAFE read altitude
        movff   altitude+1,hi
        btfss   hi,7                    ; Is altitude negative ?
        bra     altimeter_2             ; No: just print it

        PUTC    '-'                     ; Yes: print the minus sign
        comf    hi                      ; And do a 16bit 2-complement.
        comf    lo
        infsnz  lo
        incf    hi

altimeter_2:
        bsf     leftbind
#IFDEF IMPERIAL
;		GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
		GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
		btfsc   WREG,0					; Enabled ?
		bra		altimeter_3				; NO
		call	convert_meters_to_feet2	; converts meters to feet, don't check CF#61
		output_16
		bcf		leftbind
		STRCAT_PRINT TXT_FOOT5
        return
altimeter_3:
#ENDIF
        output_16
        bcf     leftbind
        STRCAT  TXT_METER5
        clrf    WREG
        movff   WREG,letter+.11         ;limit to 12chars
        STRCAT_PRINT    ""
        return

;=============================================================================
; Compute altitude, using the formula:
; H(P) = 18.787 log10(P0/P)  (Log base 10)

;---- Interface the the Math library -----------------------------------------
        extern  __AARGB2                ; A float in fA2, fA1, fA0, fAExo
        extern  __BARGB2                ; B float in fB2, fB1, fB0, fBExo
        extern  FLO1632U                ; convert uint16 fA+1 --> fp32 fA
        extern  FPD32                   ; fp32 divide fA/fB --> fA
        extern  FPM32                   ; fp32 multiply fA*fB --> fA
        extern  INT3216                 ; convert fp32 fA --> int16 fA+1
#define fA __AARGB2
#define fB __BARGB2

;---- Interface to the C library ---------------------------------------------
        extern  __AARGB3
        extern  log10                   ; float32 log(auto float32)
#define fRet __AARGB3

compute_altitude:
        ; Setup C-code stack, to enable calling the log() function.
        lfsr    FSR1, c_code_data_stack
        lfsr    FSR2, c_code_data_stack
        
        ; Convert pressure to float, --> fB
        movff   pressureAvg+0, fA+1 
        movff   pressureAvg+1, fA+2
        call    FLO1632U                ; u16 fA[1:2] --> fp32 fA
        movff   fA+0, fB+0
        movff   fA+1, fB+1
        movff   fA+2, fB+2
        movff   fA+3, fB+3

        ; Convert sea-level reference pressure to float, --> fB
        movff   pressureRef+0, fA+1     ; Get sea level pressure.
        movff   pressureRef+1, fA+2
        bcf     STATUS,C                ; Multiply by 4.
        rlcf    fA+1
        rlcf    fA+2
        bcf     STATUS,C
        rlcf    fA+1
        rlcf    fA+2
        call    FLO1632U                ; to float: u16 fA[1:2] --> fp32 fA

        ; Divide
        call    FPD32                   ; fp32 X/Y --> X

        ; log10()
        movff   fA+0, POSTINC1          ; Push X to stack
        movff   fA+1, POSTINC1
        movff   fA+2, POSTINC1
        movff   fA+3, POSTINC1
        call    log10                   ; log(P0/P)

        movf    POSTDEC1,F,ACCESS       ; pop argument
        movf    POSTDEC1,F,ACCESS            
        movf    POSTDEC1,F,ACCESS     
        movf    POSTDEC1,F,ACCESS

        ; Move log10(P0/P) to fB
        movff   fRet+0,fB+0             ; move result to fB
        movff   fRet+1,fB+1
        movff   fRet+2,fB+2 
        movff   fRet+3,fB+3

        ; Multiply by scaling factor for meters, and standatd atmosphere.
        movlw   LOW(.18787)
        movff   WREG, fA+1
        movlw   HIGH(.18787)
        movff   WREG, fA+2
        call    FLO1632U                ; u16 fA[1:2] --> fp32 fA
        call    FPM32                   ; altitute --> fp32 fA

        ; Convert result to int16 --> altitude.
        call    INT3216                 ; fp32 fA --> int16 fA+1
        movff   fA+1, altitude+0
        movff   fA+2, altitude+1

        return

;=============================================================================
; Altimeter menu
;
; Edit reference (where altitude = 0) pressure, while displaying corresponding
; altitude.
;
altimeter_menu:
        movff   pressureRef+0,WREG     ; Make sure it is initialized...
        movff   pressureRef+1,fA
        iorwf   fA
        bnz     altimeter_menu_1       ; Yes: skip reset...
        rcall   altimeter_reset

altimeter_menu_1:
        call    DISP_ClearScreen        ; Menu header.
        call	DISP_divemask_color
        DISPLAYTEXTH .288               ; Title bar
        call    DISP_standard_color

        movlw       2                   ; Start menu on line 2.
        movwf       menupos

altimeter_menu_2:
        WIN_FONT    0
        WIN_LEFT    .20                 ; First line:
        WIN_TOP     .35
        lfsr        FSR2,letter
        OUTPUTTEXTH .289               ; Sea ref:

        movff       pressureRef+0, lo
        movff       pressureRef+1, hi
        bcf         STATUS,C            ; Divide ref pressure by 4
        rrcf        hi                  ; to get the integer part of it:
        rrcf        lo
        bcf         STATUS,C
        rrcf        hi
        rrcf        lo
        bsf         leftbind
        output_16
        
        STRCAT_PRINT	TXT_MBAR7
        
        WIN_TOP     .65                 ; Action enable
        lfsr        FSR2, letter
        OUTPUTTEXTH .290
        GETCUSTOM8  .49
        btfss       WREG,0
        bra         alt_menu_1
        OUTPUTTEXT  .130                ; ON
        bra         alt_menu_2
alt_menu_1:
        OUTPUTTEXT  .131                ; OFF
alt_menu_2:
        call word_processor
        
        DISPLAYTEXTH .291               ; Action reset
        DISPLAYTEXTH .292               ; Action add
        DISPLAYTEXTH .293               ; Action sub
        DISPLAYTEXT  .011               ; Action exit
#IFNDEF IMPERIAL
        WIN_LEFT    .80                 ; Bottom right. 
#ELSE
        WIN_LEFT    .72                 ; Bottom right. 
#ENDIF
        lfsr    FSR2, letter
        OUTPUTTEXTH .294                ; "Alt: "

        movff       altitude+0, lo
        movff       altitude+1, hi
        btfss       hi,7                ; Is altitude negativ ?
        bra         altimeter_menu_3    ; No: just print it

        PUTC        '-'                 ; Yes: print the minus sign
        comf        hi                  ; And do a 16bit 2-complement.
        comf        lo
        infsnz      lo
        incf        hi

altimeter_menu_3:
        bsf         leftbind
#IFDEF IMPERIAL
;		GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
		GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
		btfsc		WREG,0					; Enabled ?
		bra			altimeter_menu_4		; NO
		call		convert_meters_to_feet2	; converts meters to feet, don't check CF#61
		output_16
		bcf			leftbind
		STRCAT_PRINT TXT_FOOT5
		bra			alt_menu_loop
altimeter_menu_4:
#ENDIF
        output_16
        bcf         leftbind
        STRCAT  TXT_METER5
        clrf    WREG
        movff   WREG,letter+.11         ;limit to 12chars
        STRCAT_PRINT    ""

alt_menu_loop:
        call        DISP_menu_cursor    ; Display cursor
		call		wait_switches		; Waits until switches are released, resets flag if button stays pressed!

alt_menu_loop1:                         ; Wait for button.
    	btfsc	uart_dump_screen        ; Asked to dump screen contains ?
    	call	dump_screen     		; Yes!

    	btfsc   	switch_right        ; [[MENU]] button
    	bra	        alt_menu_next

    	btfsc	    switch_left         ;[[ENTER]] button
    	bra	        alt_menu_do_it

    	btfsc	    divemode            ; Diving stared ?
    	goto	    restart             ; YES: quit this menu !

    	btfsc	    onesecupdate        ; Check what should be every 1sec.
    	call    	timeout_surfmode

    	btfsc	    onesecupdate
    	call	    set_dive_modes

    	bcf		    onesecupdate		; end of 1sek. tasks

    	btfsc	    sleepmode           ; Sleep mode entered ?
    	bra	        alt_menu_exit

    	bra	        alt_menu_loop1

;---- Move to next line ------------------------------------------------------

alt_menu_next:
        incf        menupos             ; next line.
        movlw       .7
        cpfseq      menupos             ; Below last line ?
        bra         alt_menu_loop
        movlw       .2                  ; Yes: back to line no 2.
        movwf       menupos
        bra         alt_menu_loop

;----- Execute menu line -----------------------------------------------------

alt_menu_do_it:
        movf        menupos,W           ; test line value
        addlw       -2
        bz          alt_menu_enable     ; 2 --> reset
        dcfsnz      WREG
        bra         alt_menu_reset      ; 3 --> +1
        dcfsnz      WREG
        bra         alt_menu_plus1      ; 4 --> +1
        dcfsnz      WREG
        bra         alt_menu_minus1     ; 5 --> -1
        bra         alt_menu_exit       ; else --> exit

;---- Toggle altimeter (CF#49) -----------------------------------------------        
alt_menu_enable:
        GETCUSTOM8  .49                 ; Read CF#49
        btg         WREG,0              ; Toggle boolean value
    	movwf	    EEDATA              
    	movlw	    d'1'				; Upper EEPROM Bank
    	movwf	    EEADRH
        movlw       4*(.49-.32) + 0x82  ; CF#49 low byte address in EEPROM
    	movwf	    EEADR
    	call	    write_eeprom
    	clrf	    EEADRH				; Reset EEADRH for compatibility
    	bra         altimeter_menu_2

;---- Reset sea level pressure to reference ----------------------------------
alt_menu_reset:
        rcall       altimeter_reset
        bra         altimeter_menu_2
        
;---- Increment sea level pressure -------------------------------------------        
alt_menu_plus1:
        movlb       HIGH(pressureRef)   ; Setup our own ram bank
        movlw       4
        addwf       pressureRef+0,F     ; 16bit inc.
        movlw       0
        addwfc      pressureRef+1,F
        bra         alt_menu_recompute  ; then recompute altitude.

;---- Decrement sea level pressure -------------------------------------------        
alt_menu_minus1:
        movlb       HIGH(pressureRef)   ; Setup our own ram bank
        movlw       -4
        addwf       pressureRef+0,F     ; 16bit decrement
        movlw       -1
        addwfc      pressureRef+1,F

alt_menu_recompute:
        rcall       compute_altitude    ; Recompute altitude
        movlb       1                   ; Go back to normal bank1
        bra         altimeter_menu_2

;---- Exit altimeter menu ----------------------------------------------------
alt_menu_exit:
    	movlw	.5                      ; reset position to Altimeter line.
    	movwf	menupos					; 
    	goto	more_menu2				; in the More... menu.
