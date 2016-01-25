; OSTC Mk.2, 2N and 2C - diving computer code
; Copyright (C) 2009 HeinrichsWeikamp GbR

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


; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 090801
; History:
; 2009-08-30: [MH] last updated.
; 2011-01-07: [jDG] Added flip_screen option
; known bugs: pixel-write (loogbok curves) not done yet...
; ToDo:

WIN_FONT 	macro	win_font_input
			movlw	win_font_input
			movff	WREG,win_font
			endm
			
WIN_TOP 	macro 	win_top_input
			movlw	win_top_input
			movff	WREG,win_top
			endm
			
WIN_LEFT 	macro 	win_left_input
			movlw	win_left_input
			movff	WREG,win_leftx2
			endm
			
WIN_INVERT	macro 	win_invert_input
			movlw	win_invert_input
			movff	WREG,win_invert
			endm

WIN_COLOR	macro 	win_color_input
			movlw	win_color_input
			call	DISP_set_color
			endm

;=============================================================================

word_processor:						; word_processor:
    clrf	POSTINC2				; Required, to mark end of string.
    call	aa_wordprocessor
    movlb	b'00000001'				; Back to Rambank1
    return

;=============================================================================
; Macro to provides our own interface code.
;
PIXEL_WRITE macro   colRegister, rowRegister
        movff   colRegister,win_leftx2
        movff   rowRegister,win_top
        call    pixel_write
        endm

INIT_PIXEL_WRITE macro colRegister
        movff   colRegister,win_leftx2
        call    init_pixel_write
        endm

HALF_PIXEL_WRITE macro rowRegister
        movff   rowRegister,win_top
        call    half_pixel_write
        endm

;-----------------------------------------------------------------------------
; Init for half_pixel_write
; Set column register on DISPLAY device, and current color.
; Inputs: win_leftx2
; Outputs: win_color:2
; Trashed: WREG, PROD
init_pixel_write:
        movff   win_leftx2,WREG
        mullw   2
        rcall   pixel_write_col320      ; Start Address Vertical (.0 - .319)
        goto    DISP_standard_color
    
;-----------------------------------------------------------------------------
; Writes two half-pixels at position (win_top,win_leftx2)
; Inputs: win_leftx2, win_top, win_color:2
; Trashed: WREG, PROD
pixel_write:
        movff   win_leftx2,WREG
        mullw   2
        rcall   pixel_write_col320      ; Start Address Vertical (.0 - .319)
        rcall   half_pixel_write        ; Write this half-one.

        movff   win_leftx2,WREG         ; Address of next one
        mullw   2
        infsnz  PRODL                   ; +1
        incf    PRODH
    	rcall   pixel_write_col320
    	bra     half_pixel_write        ; Note: Cmd 0x20 is mandatory, because
    	                                ; of the autoincrement going vertical

        ;---- Do the 16bit 319-X-->X, if needed, and send to DISPLAY ------------
pixel_write_col320:
        movff   win_flags,WREG          ; BEWARE: bank0 bit-test
        btfsc   WREG,1                  ; Display1
        bra     pixel_write_noflip_H_display1 ; Yes.
        btfss   WREG,0                  ; 180° rotation ?
        bra     pixel_write_noflip_H

        movf    PRODL,W                 ; 16bits 319 - PROD --> PROD
        sublw   LOW(.319)               ; 319-W --> W
        movwf   PRODL
        movf    PRODH,W
        btfss   STATUS,C                ; Borrow = /CARRY
        incf    WREG
        sublw   HIGH(.319)
        movwf   PRODH

pixel_write_noflip_H:
    	movlw	0x21				    ; Start Address Vertical (.0 - .319)
    	rcall	DISP_CmdWrite
    	bra     DISP_DataWrite_PROD     ; And return...

pixel_write_noflip_H_display1:
        movlw	0x06
        rcall	DISP_CmdWrite
        movf    PRODH,W
        rcall	DISP_DataWrite
        movlw	0x07
        rcall	DISP_CmdWrite
        movf    PRODL,W
        rcall	DISP_DataWrite

        incf    PRODL,F
        movlw   .0
        addwfc  PRODH,F             ;+1

        movlw	0x08
        rcall	DISP_CmdWrite
        movf    PRODH,W
        rcall	DISP_DataWrite
        movlw	0x09
        rcall	DISP_CmdWrite
        movf    PRODL,W
        bra 	DISP_DataWrite      ; And return...
        
    
;-----------------------------------------------------------------------------
; Writes a vertical line of half-pixel at position (win_top,win_leftx2,win_height).
; Inputs: win_leftx2, win_top, win_height, win_color:2
; Trashed: WREG, PROD, TABLAT, TBLPTRL
half_vertical_line:
        clrf    TABLAT                  ; Loop index.

half_vertical_line_loop:
        movff   win_leftx2,WREG         ; Init X position.
        mullw   2
        movf    TABLAT,W                ; Get loop index
        andlw   1                       ; Just low bit
        xorwf   PRODL,F                 ; And use it to jitter current X position
        rcall   pixel_write_col320      ; Start Address Vertical (.0 - .319)

        movff   win_height,WREG         ; Index reached height (Bank0 read) ?
        xorwf   TABLAT,W
        btfsc   STATUS,Z                ; Equals ?
        return                          ; Yes: done.
        movff   win_top,WREG            ; Y = top + index (Bank0 read)
        addwf   TABLAT,W
        rcall   half_pixel_write_1
        incf    TABLAT,F                ; index++
        bra     half_vertical_line_loop

;-----------------------------------------------------------------------------
; Writes one half-pixel at position (win_top,win_leftx2).
; Inputs: win_leftx2, win_top, win_color:2
; Trashed: WREG, PROD
half_pixel_write:
    	movff  	win_top,WREG            ; d'0' ... d'239'
; Variant with Y position in WREG.
half_pixel_write_1:
        movff   win_flags,PRODL          ; Display1? win_flags is in bank0...
        btfsc   PRODL,1
        bra     half_pixel_write_1_display1 ; Yes.

    	btfsc   PRODL,0                 ; 180° rotation ?
    	sublw   .239                    ; 239-Y --> Y
    	mullw   1                       ; Copy row to PRODL (PRODH=0)

    	movlw	0x20			        ; Horizontal Address START:END
    	rcall	DISP_CmdWrite
    	rcall   DISP_DataWrite_PROD
    
    	movlw	0x22					; Start Writing Data to GRAM
    	rcall	DISP_CmdWrite
    	bsf		DISPLAY_rs				; Data!
    	movff	win_color1, PORTD
    	bcf		DISPLAY_rw
    	bsf		DISPLAY_rw				; Upper
    	movff	win_color2, PORTD
    	bcf		DISPLAY_rw
    	bsf		DISPLAY_rw				; Lower
        return                          ; Done.

half_pixel_write_1_display1:
    	mullw   1                       ; Copy row to PRODL (PRODH=0)
    ; Row address start
        movlw	0x02
        rcall	DISP_CmdWrite
        movlw   .0
        rcall	DISP_DataWrite
        movlw	0x03
        rcall	DISP_CmdWrite
        movf    PRODL,W
        rcall 	DISP_DataWrite

        incf    PRODL,F

        movlw	0x04
        rcall	DISP_CmdWrite
        movlw   .0
        rcall	DISP_DataWrite
        movlw	0x05
        rcall	DISP_CmdWrite
        movf    PRODL,W
        rcall 	DISP_DataWrite

    	movlw	0x22					; Start Writing Data to GRAM
    	rcall	DISP_CmdWrite
    	bsf		DISPLAY_rs				; Data!
    	movff	win_color4, PORTD
    	bcf		DISPLAY_rw
    	bsf		DISPLAY_rw				; Upper
    	movff	win_color5, PORTD
    	bcf		DISPLAY_rw
    	bsf		DISPLAY_rw				; High
    	movff	win_color6, PORTD
    	bcf		DISPLAY_rw
    	bsf		DISPLAY_rw				; Lower
    	return


; -----------------------------
; DISP Display Off
; -----------------------------
DISP_DisplayOff:
    rcall   DISP_ClearScreen
	clrf	PORTD
	bcf		DISPLAY_hv
	bcf		DISPLAY_vdd
	bcf		DISPLAY_cs
	bcf		DISPLAY_e_nwr	
	bcf		DISPLAY_rw
    bcf		DISPLAY_rs
	bcf		DISPLAY_nreset
	return

;=============================================================================
; Fast macros to write to DISPLAY display.
; Adding a call/return adds 3 words and a pipeline flush, hence make it
; nearly twice slower...
;
; Input	 : commande as macro parameter.
; Output : NONE
; Trash  : WREG
;
AA_CMD_WRITE macro cmd
		movlw	cmd
		rcall   DISP_CmdWrite			; slow but saves a lot of bytes in flash
		endm
;
; Input	 : data as macro parameter.
; Output : NONE
; Trash  : WREG
;
AA_DATA_WRITE macro data
        movlw   data
        rcall   DISP_DataWrite
        endm
;
; Input	 : PRODH:L as 16bits data.
; Output : NONE
; Trash  : NONE
;
AA_DATA_WRITE_PROD	macro
        rcall   DISP_DataWrite_PROD	; slow but saves a lot of bytes in flash
		endm

;=============================================================================
; Output DISPLAY Window Address commands.
; Inputs : win_top, win_leftx2, win_height, aa_width.
; Output : PortD commands.
; Trashed: PROD
;
DISP_box_write:
        movff   win_flags,WREG          ; Display1? win_flags is in bank0...
        btfsc   WREG,1                  ; Display1?
        bra     DISP_box_write_display1 ; Yes

		movff	win_leftx2,WREG         ; Compute left = 2*leftx2 --> PROD
		mullw	2

        movff   win_flags,WREG          ; BEWARE: bank0 bit-test
    	btfsc   WREG,0                  ; 180° rotation ?
    	bra     DISP_box_flip_H         ; YES: 

        ;---- Normal horizontal window ---------------------------------------
        ; Output 0x35 left,
        ;        0x36 right ==  left + width - 1.
		AA_CMD_WRITE	0x35		    ; this is the left border
		AA_DATA_WRITE_PROD              ; Output left
		AA_CMD_WRITE	0x21            ; Also the horizontal first pix coord.
		AA_DATA_WRITE_PROD
		
		movf	aa_width+0,W,ACCESS	    ; right = left + width - 1
		addwf	PRODL,F
		movf	aa_width+1,W,ACCESS
		addwfc	PRODH,F
		decf	PRODL,F,A			    ; decrement result
		btfss   STATUS,C
		decf	PRODH,F,A

		AA_CMD_WRITE	0x36		    ; Write and the right border
		AA_DATA_WRITE_PROD

		bra     DISP_box_noflip_H

        ;---- Flipped horizontal window --------------------------------------
DISP_box_flip_H:
        ; Output 0x36 flipped(left)  = 319-left
        ;        0x35 flipped(right) = 319-right = 320 - left - width
        movf    PRODL,W                 ; 16bits 319 - PROD --> PROD
        sublw   LOW(.319)               ; 319-W --> W
        movwf   PRODL
        movf    PRODH,W
        btfss   STATUS,C                ; Borrow = /CARRY
        incf    WREG
        sublw   HIGH(.319)
        movwf   PRODH
		AA_CMD_WRITE	0x36		    ; this is the left border
		AA_DATA_WRITE_PROD              ; Output left
		AA_CMD_WRITE	0x21
		AA_DATA_WRITE_PROD

        movf    aa_width+0,W            ; 16bits PROD - width --> PROD
        subwf   PRODL,F                 ; PRODL - WREG --> PRODL
        movf    aa_width+1,W
        subwfb  PRODH,F
        infsnz  PRODL                   ; PROD+1 --> PROD
        incf    PRODH
		AA_CMD_WRITE	0x35		    ; this is the left border
		AA_DATA_WRITE_PROD              ; Output left

DISP_box_noflip_H:
        movff   win_flags,WREG          ; BEWARE: bank0 bit-test
    	btfsc   WREG,0                  ; 180° rotation ?
    	bra     DISP_box_flip_V

        ;---- Normal vertical window -----------------------------------------
        ; Output 0x37 (top) (bottom)
		movff	win_top,PRODH           ; top --> PRODH (first byte)
		movff   win_height,WREG
		addwf   PRODH,W
		decf	WREG
		movwf	PRODL                   ; top+height-1 --> PRODL (second byte)

		AA_CMD_WRITE	0x37
		AA_DATA_WRITE_PROD

        movff   PRODH,PRODL
        clrf    PRODH                   ; Start pixel V coord == top.
		AA_CMD_WRITE	0x20
		AA_DATA_WRITE_PROD

		return

        ;---- Flipped vertical window ----------------------------------------
        ; Output 0x37 flipped(bottom) = 239-bottom = 240 - top - height
        ;             flipped(top)    = 239-top
DISP_box_flip_V:
		movff   win_top,PRODL
		movff   win_height,WREG
		addwf   PRODL,W
		sublw   .240                    ; 240 - top - height
		movwf   PRODH                   ; First byte

		movf	PRODL,W
		sublw   .239                    ; 249-top
		movwf   PRODL                   ; --> second byte.

		AA_CMD_WRITE	0x37
		AA_DATA_WRITE_PROD

        clrf    PRODH                   ; Start pixel V coord.
		AA_CMD_WRITE	0x20
		AA_DATA_WRITE_PROD

		return

DISP_box_write_display1:
    	movff	win_leftx2,WREG         ; Compute left = 2*leftx2 --> PROD
		mullw	2

        movlw	0x06
        rcall	DISP_CmdWrite
        movf    PRODH,W
        rcall	DISP_DataWrite
        movlw	0x07
        rcall	DISP_CmdWrite
        movf    PRODL,W
        rcall	DISP_DataWrite

		movf	aa_width+0,W,ACCESS	    ; right = left + width - 1
		addwf	PRODL,F
		movf	aa_width+1,W,ACCESS
		addwfc	PRODH,F
		decf	PRODL,F,A			    ; decrement result
		btfss   STATUS,C
		decf	PRODH,F,A

        movlw	0x08
        rcall	DISP_CmdWrite
        movf    PRODH,W
        rcall	DISP_DataWrite
        movlw	0x09
        rcall	DISP_CmdWrite
        movf    PRODL,W
        rcall	DISP_DataWrite

        ;---- Normal vertical window -----------------------------------------
        ; Output  (top) (bottom)
		movff	win_top,PRODH           ; top --> PRODH (first byte)
		movff   win_height,WREG
		addwf   PRODH,W
		decf	WREG
		movwf	PRODL                   ; top+height-1 --> PRODL (second byte)

        movlw	0x02
        rcall	DISP_CmdWrite
        movlw   0x00
        rcall	DISP_DataWrite
        movlw	0x03
        rcall	DISP_CmdWrite
        movf    PRODH,W
        rcall	DISP_DataWrite

        movlw	0x04
        rcall	DISP_CmdWrite
        movlw   0x00
        rcall	DISP_DataWrite
        movlw	0x05
        rcall	DISP_CmdWrite
        movf    PRODL,W
        rcall	DISP_DataWrite
		return

;=============================================================================
; DISP_frame : draw a frame around current box with current color.
; Inputs:  win_top, win_leftx2, win_height, win_width, win_color1, win_color2
; Outputs: (none)
; Trashed: WREG, PROD, aa_start:2, aa_end:2, win_leftx2, win_width:1
    global  DISP_frame
DISP_frame:
    movff   win_top,aa_start+0          ; Backup everything.
    movff   win_height,aa_start+1
    movff   win_leftx2,aa_end+0
    movff   win_width,aa_end+1

    ;---- TOP line -----------------------------------------------------------
    movlw   1                           ; row ~ height=1
    movff   WREG,win_height
    rcall   DISP_box

    ;---- BOTTOM line --------------------------------------------------------
    movff   aa_start+0,PRODL             ; Get back top,
    movff   aa_start+1,WREG              ; and height
    addwf   PRODL,W                      ; top+height
    decf    WREG                         ; top+height-1
    movff   WREG,win_top                 ; top+height-1 --> top
    rcall   DISP_box                        

    ;---- LEFT column --------------------------------------------------------
    movff   aa_start+0,win_top              ; Restore top/height.
    movff   aa_start+1,win_height
    movlw   1                               ; column ~ width=1
    movff   WREG,win_width
    rcall   DISP_box

    ;---- RIGHT column -------------------------------------------------------
    movff   aa_end+0,WREG
    movff   aa_end+1,PRODL
    addwf   PRODL,W
    decf    WREG
    movff   WREG,win_leftx2
    bra     DISP_box

;=============================================================================
; DISP_box : fills current box with current color.
; Inputs:  win_top, win_leftx2, win_height, win_width, win_color1, win_color2
; Outputs: (none)
; Trashed: WREG, PROD

    global  DISP_box
DISP_box:
    ;---- Define Window ------------------------------------------------------
	movff	win_width,WREG
	bcf     STATUS,C
	rlcf    WREG
	movwf   aa_width+0
	movlw   0
	rlcf    WREG
	movwf   aa_width+1
	rcall   DISP_box_write

    ;---- Fill Window --------------------------------------------------------
	movlw	0x22                        ; Start Writing Data to GRAM
	rcall	DISP_CmdWrite

	clrf	PRODH                       ; Column counter.
	bsf		DISPLAY_rs                  ; Data!
DISP_box2:                              ; Loop height times
	movff	win_height,PRODL
    
DISP_box3:                              ; loop width times
    movff   win_flags,WREG              ; Display1? win_flags is in bank0...
    btfsc   WREG,1                      ; Display1?
    bra     DISP_box3aa                 ; Yes

	movff	win_color1,PORTD
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw                  ; Upper
	movff	win_color2,PORTD
    bra     DISP_box3a                  ; Done.
DISP_box3aa:
    movff	win_color4,PORTD
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw                  ; Upper
	movff	win_color5,PORTD
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw                  ; Lower/High
	movff	win_color6,PORTD

DISP_box3a:
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw                  ; Lower/High

    movff   win_flags,WREG              ; Display1? win_flags is in bank0...
    btfsc   WREG,1                      ; Display1?
    bra     DISP_box3ab                 ; Yes

	movff	win_color1,PORTD
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw                  ; Upper
	movff	win_color2,PORTD
    bra     DISP_box3b

DISP_box3ab:
    movff	win_color4,PORTD
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw                  ; Upper
	movff	win_color5,PORTD
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw                  ; High
	movff	win_color6,PORTD

DISP_box3b:
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw                  ; Lower
	decfsz	PRODL,F                     ; row loop finished ?
	bra		DISP_box3                   ; No: continue.

    incf    PRODH,F                     ; column count ++

    movff   win_bargraph,WREG           ; current column == bargraph ?
    cpfseq  PRODH
    bra     DISP_box4                   ; No: just loop.

    clrf    WREG                        ; Yes: switch to black
    movff   WREG,win_color1
    movff   WREG,win_color2
    movff   WREG,win_color3
    movff   WREG,win_color4
    movff   WREG,win_color5
    movff   WREG,win_color6
DISP_box4:
    movff   win_width,WREG
    cpfseq  PRODH
    bra     DISP_box2

	setf    WREG                        ; Reset bargraph mode...
	movff   WREG,win_bargraph

    movff   win_flags,WREG          ; Display1? win_flags is in bank0...
    btfsc   WREG,1                  ; Display1?
    return                              ; Yes, done.

	movlw	0x00                        ; NOP, to stop window mode
	bra     DISP_CmdWrite               ; Returns....

;=============================================================================
; DISP_ClearScreen: An optimized version of PLEX_box, for full screen black.
; Trashed: WREG, PROD

    global  DISP_ClearScreen
DISP_ClearScreen:
    movff   win_flags,WREG          ; Display1? win_flags is in bank0...
    btfsc   WREG,1                  ; Display1?
    bra     DISP_ClearScreen_display1; Yes

	movlw	0x35				; VerticalStartAddress HIGH:LOW
	rcall	DISP_CmdWrite
	mullw   0
	rcall	DISP_DataWrite_PROD

	movlw	0x36				; VerticalEndAddress HIGH:LOW
	rcall	DISP_CmdWrite
	movlw	0x01
	rcall	DISP_DataWrite		
	movlw	0x3F
	rcall	DISP_DataWrite		

	movlw	0x37				; HorizontalAddress START:END
	rcall	DISP_CmdWrite
	movlw	0x00
	rcall	DISP_DataWrite		
	movlw	0xEF
	rcall	DISP_DataWrite		

	movlw	0x20				; Start Address Horizontal (.0 - .239)
	rcall	DISP_CmdWrite
	rcall	DISP_DataWrite_PROD

	movlw	0x21				; Start Address Vertical (.0 - .319)
	rcall	DISP_CmdWrite
	rcall	DISP_DataWrite_PROD

	movlw	0x22                ; Start Writing Data to GRAM
	rcall	DISP_CmdWrite

	; See Page 101 of DISPLAY Driver IC Datasheet how to handle rs/rw clocks
	bsf		DISPLAY_rs             ; Data!
	clrf	PORTD
	movlw	.160
	movwf	PRODH
DISP_ClearScreen2:
	movlw	.240
	movwf	PRODL
DISP_ClearScreen3:
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw             ; Upper
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw             ; Lower
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw             ; Upper
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw             ; Lower
	decfsz	PRODL,F
	bra		DISP_ClearScreen3
	decfsz	PRODH,F
	bra		DISP_ClearScreen2
	movlw	0x00					; NOP, to stop Address Update Counter
	bra     DISP_CmdWrite           ; And return...

DISP_ClearScreen_display1:
    ; Column Address start
	movlw	0x02
	rcall	DISP_CmdWrite
	movlw	0x00
	rcall	DISP_DataWrite
	movlw	0x03
	rcall	DISP_CmdWrite
	movlw	0x00
	rcall	DISP_DataWrite

; Column Address end
	movlw	0x04
	rcall	DISP_CmdWrite
	movlw	0x00
	rcall	DISP_DataWrite
	movlw	0x05
	rcall	DISP_CmdWrite
	movlw	0xEF
	rcall	DISP_DataWrite

; Row address start
	movlw	0x06
	rcall	DISP_CmdWrite
	movlw	0x00
	rcall	DISP_DataWrite
	movlw	0x07
	rcall	DISP_CmdWrite
	movlw	0x00
	rcall	DISP_DataWrite

; Row address end
	movlw	0x08
	rcall	DISP_CmdWrite
	movlw	0x01
	rcall	DISP_DataWrite
	movlw	0x09
	rcall	DISP_CmdWrite
	movlw	0x3F
	rcall	DISP_DataWrite

	movlw	0x22                ; Start Writing Data to GRAM
	rcall	DISP_CmdWrite

	bsf		DISPLAY_rs             ; Data!

	movlw	.160
	movwf	PRODH
DISP_ClearScreen2_display1:
	movlw	.240
	movwf	PRODL
	clrf	PORTD
DISP_ClearScreen3_display1:
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw             ; Upper
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw             ; High
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw             ; Lower
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw             ; Upper
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw             ; High
    bcf		DISPLAY_rw
	bsf		DISPLAY_rw             ; Lower
	decfsz	PRODL,F
	bra		DISP_ClearScreen3_display1
	decfsz	PRODH,F
	bra		DISP_ClearScreen2_display1
    return


; -----------------------------
; DISP Write Cmd via W
; -----------------------------
DISP_CmdWrite:
	bcf		DISPLAY_rs					; Command!
	movwf	PORTD					; Move Data to PORTD
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw
	return

; -----------------------------
; DISP Write Display Data via W
; -----------------------------
DISP_DataWrite:
	bsf		DISPLAY_rs					; Data!
	movwf	PORTD					; Move Data to PORTD
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw
	return

; -----------------------------
; DISP Data Cmd via W
; -----------------------------
DISP_DataWrite_PROD:
	bsf		DISPLAY_rs					; Data!
	movff	PRODH,PORTD				; Move high byte to PORTD (DISPLAY is bigendian)
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw
	movff	PRODL,PORTD				; Move low byte to PORTD
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw
    movff   win_flags,WREG          ; Display1? win_flags is in bank0...
    btfss   WREG,1                  ; Display1?
    return                          ; No, done.
    movff   win_color3,PORTD        ; Move low(est) byte to PORTD
	bcf		DISPLAY_rw
	bsf		DISPLAY_rw
	return


; -----------------------------
; DISP Read data into WREG
; -----------------------------
; NOTE: you should "setf TRISD" before calling this function,
;       to make PortD an input port...
DISP_DataRead:
    movff   win_flags,WREG          ; Display1? win_flags is in bank0...
    btfsc   WREG,1                  ; Display1?
    return                          ; Yes, done.
	bsf		DISPLAY_rs					; Data register.
	bcf		DISPLAY_e_nwr              ; Read enable.
	nop
	nop
	nop
	nop
	movf	PORTD,W				    ; Read byte.
	bsf		DISPLAY_e_nwr              ; release bus.
	return

; -----------------------------
; DISP boot
; -----------------------------
DISPLAY_boot:
    movlw   LOW     0x17FDC
    movwf   TBLPTRL
    movlw   HIGH    0x17FDC
    movwf   TBLPTRH
    movlw   UPPER   0x17FDC
    movwf   TBLPTRU
    TBLRD*
    movlw   0x01
    cpfseq  TABLAT      ; Display1?
    bra     display0_init ; No, Display0

    banksel win_flags
    bsf     win_flags,0
    bsf     win_flags,1
    banksel flag1
    bcf     DISPLAY_hv     ; Backlight off
    nop
	bcf		DISPLAY_vdd
    WAITMS	d'10'
	bsf		DISPLAY_vdd
    WAITMS	d'100'
	bsf		DISPLAY_rw
	nop
	bcf		DISPLAY_cs
	nop
	bsf		DISPLAY_nreset
	WAITMS	d'1'
	bcf		DISPLAY_nreset
	WAIT10US	d'2'
	bsf		DISPLAY_nreset
	WAITMS	d'120'
    bsf		DISPLAY_e_nwr              ; release bus.
    rcall   display1_init           ; Init sequence
	rcall	DISP_ClearScreen
	WAITMS	d'60'
	bsf		DISPLAY_hv     ; Backlight on
	return

display1_init:
    movlw   LOW     display1_config_table
    movwf   TBLPTRL
    movlw   HIGH    display1_config_table
    movwf   TBLPTRH
    movlw   UPPER   display1_config_table
    movwf   TBLPTRU
display1_init_loop:
    TBLRD*+
    movlw   0xFF
    cpfseq  TABLAT
    bra     display1_config_write    ; Write Config pair to Display
    ; Delay ms or quit (return)
    TBLRD*+
    tstfsz  TABLAT                  ; End of config?
    bra     $+4                     ; No
    return                          ; Done.
    movf    TABLAT,W
    call    WAITMSX                 ; Wait WREG milliseconds
    bra     display1_init_loop       ; Loop

display1_config_write:               ; With command in WREG
    movf    TABLAT,W
    rcall	DISP_CmdWrite           ; Write command
    TBLRD*+                         ; Get config
    movf    TABLAT,W
	rcall	DISP_DataWrite          ; Write config
    bra     display1_init_loop       ; Loop


display1_config_table:
    ; Reg, Dat or 0xFF, Delay or 0xFF, 0x00 (End)
    db  0x96,0x01
    db  0x19,0x87
    db  0xFF,.10
    db  0x26,0x80
    db  0x1B,0x0C
    db  0x43,0x00
    db  0x20,0x00
    db  0x1F,0x07
    db  0x44,0x7F
    db  0x45,0x14
    db  0x1D,0x05
    db  0x1E,0x00
    db  0x1C,0x04
    db  0x1B,0x14
    db  0xFF,.40
    db  0x43,0x80
    db  0x42,0x08
    db  0x23,0x95
    db  0x24,0x95
    db  0x25,0xFF
    db  0x21,0x10
    db  0x2B,0x00
    db  0x95,0x01
    db  0x1A,0x00
    db  0x93,0x0F
    db  0x70,0x66
    db  0x18,0x01
    db  0x46,0x86
    db  0x47,0x60
    db  0x48,0x01
    db  0x49,0x67
    db  0x4A,0x46
    db  0x4B,0x13
    db  0x4C,0x01
    db  0x4D,0x67
    db  0x4E,0x00
    db  0x4F,0x13
    db  0x50,0x02
    db  0x51,0x00
    db  0x38,0x00
    db  0x39,0x00
    db  0x27,0x02
    db  0x28,0x03
    db  0x29,0x08
    db  0x2A,0x08
    db  0x2C,0x08
    db  0x2D,0x08
    db  0x35,0x09
    db  0x36,0x09
    db  0x91,0x14
    db  0x37,0x00
    db  0x01,0x06
    db  0x3A,0xA1
    db  0x3B,0xA1
    db  0x3C,0xA1
    db  0x3D,0x00
    db  0x3E,0x2D
    db  0x40,0x03
    db  0x41,0xCC
    db  0x0A,0x00
    db  0x0B,0x00
    db  0x0C,0x01
    db  0x0D,0x3F
    db  0x0E,0x00
    db  0x0F,0x00
    db  0x10,0x01
    db  0x11,0x40
    db  0x12,0x00
    db  0x13,0x00
    db  0x14,0x00
    db  0x15,0x00
    db  0x02,0x00
    db  0x03,0x00
    db  0x04,0x00
    db  0x05,0xEF
    db  0x06,0x00
    db  0x07,0x00
    db  0x08,0x01
    db  0x09,0x3F
    db  0x16,0x88
    db  0x72,0x00
    db  0x22,0x60
    db  0x94,0x0A
    db  0x90,0x7F
    db  0x26,0x84
    db  0xFF,.40
    db  0x26,0xA4
    db  0x26,0xAC
    db  0xFF,.40
    db  0x26,0xBC
    db  0x96,0x00
    db  0xFF,0x00   ; End of table pair


display0_init:          ; Display0
    banksel win_flags
    bcf     win_flags,1
    banksel flag1
	bcf		DISPLAY_hv
    btfsc   DISPLAY_hv
    bra     $-4
	WAITMS	d'32'
	bsf		DISPLAY_vdd
	nop
	bcf		DISPLAY_cs
	nop
	bsf		DISPLAY_nreset
	WAITMS	d'250'			; Standard wake-up
	bsf		DISPLAY_e_nwr	
	nop
	bcf		DISPLAY_nreset
	WAIT10US	d'2'
	bsf		DISPLAY_nreset
	WAITMS	d'10'

	movlw	0x24				; 80-System 8-Bit Mode
	rcall	DISP_CmdWrite

	movlw	0x02				; RGB Interface Control (S6E63D6 Datasheet page 42)
	rcall	DISP_CmdWrite
	movlw	0x00				; X X X X X X X RM
	rcall	DISP_DataWrite
	movlw	0x00				; DM X RIM1 RIM0 VSPL HSPL EPL DPL
	rcall	DISP_DataWrite		; System Interface: RIM is ignored, Internal Clock

	movlw	0x03				; Entry Mode (S6E63D6 Datasheet page 46)
	rcall	DISP_CmdWrite
	movlw	0x00				; CLS MDT1 MDT0 	BGR 	X  	X  	X  	SS  65k Color

#IFDEF SEAWOOCH
	btfsc	seawooch_OLED2
	movlw	0x01				; CLS MDT1 MDT0 	BGR 	X  	X  	X  	SS  65k Color
#ENDIF

	rcall	DISP_DataWrite

	; Change direction for block-writes of pixels
    lfsr    FSR0,win_flags
    movlw   b'00000000'         ; [flipped] X  	X 	 I/D1 	I/D0 	X  	X  	X 	AM
	btfss   INDF0,0             ; BANK-SAFE bit test.
	movlw	b'00110000'			; [normal]  X  	X 	 I/D1 	I/D0 	X  	X  	X 	AM
	rcall	DISP_DataWrite

    movlw   LOW     display0_config_table
    movwf   TBLPTRL
    movlw   HIGH    display0_config_table
    movwf   TBLPTRH
    movlw   UPPER   display0_config_table
    movwf   TBLPTRU
    rcall   display0_init_loop
    rcall	DISP_brightness_full        ; Gamma settings...
	rcall	DISP_ClearScreen
	bsf		DISPLAY_hv                  ; OLED volatages on
	return


display0_config_table:
    ; Reg, Dat0, Dat1 or 0xFF,0x00,0x00 for end
    db  0x18,0x00,0x28,0xF8,0x00,0x0F
    db  0xF9,0x00,0x0F,0x10,0x00,0x00
    db  0x05,0x00,0x01,0xFF,0x00,0x00


DISP_brightness_full: ; Choose between Eco and High...
	btfsc	DISPLAY_brightness_high		; DISPLAY brightness (=0: Eco, =1: High)
	bra		DISP_brightness_full_high
; Mid
    bsf     PORTB,7
    nop
    bcf     PORTB,6
#IFDEF CCR_CTRL                     ; Set Maximal Brightness for HUD
	movff	eCTRL_HUD,WREG          ; Load HUD config
    andlw   eCTRL_HUD_BR_MSKN       ; Reset brightness bits
    iorlw   eCTRL_HUD_BR_MAX        ; Maximal HUD Brightness
	movff	WREG,eCTRL_HUD          ; Save HUD config
#ENDIF
    movff   win_flags,WREG          ; Display1? win_flags is in bank0...
    btfsc   WREG,1                  ; Display1?
    return                          ; Yes, done.

    movlw   LOW     display0_gamma_high_table
    movwf   TBLPTRL
    movlw   HIGH    display0_gamma_high_table
    movwf   TBLPTRH
    movlw   UPPER   display0_gamma_high_table
    movwf   TBLPTRU
    bra     display0_init_loop          ; And return...


DISP_brightness_full_high:
; Full
    bsf     PORTB,7
    nop
    bsf     PORTB,6
#IFDEF CCR_CTRL                     ; Set Maximal Brightness for HUD
	movff	eCTRL_HUD,WREG          ; Load HUD config
    andlw   eCTRL_HUD_BR_MSKN       ; Reset brightness bits
    iorlw   eCTRL_HUD_BR_MAX        ; Maximal HUD Brightness
	movff	WREG,eCTRL_HUD          ; Save HUD config
#ENDIF
    movff   win_flags,WREG          ; Display1? win_flags is in bank0...
    btfsc   WREG,1                  ; Display1?
    return                          ; Yes, done.

    movlw   LOW     display0_gamma_full_table
    movwf   TBLPTRL
    movlw   HIGH    display0_gamma_full_table
    movwf   TBLPTRH
    movlw   UPPER   display0_gamma_full_table
    movwf   TBLPTRU
    bra     display0_init_loop          ; And return...


DISP_brightness_low:
;Low
    bcf     PORTB,7
    nop
    bcf     PORTB,6
#IFDEF CCR_CTRL                     ; Set Minimal Brightness for HUD
	movff	eCTRL_HUD,WREG          ; Load HUD config
    andlw   eCTRL_HUD_BR_MSKN       ; Reset brightness bits
    iorlw   eCTRL_HUD_BR_DIM        ; Reduced HUD Brightness
	movff	WREG,eCTRL_HUD          ; Save HUD config
#ENDIF
    movff   win_flags,WREG          ; Display1? win_flags is in bank0...
    btfsc   WREG,1                  ; Display1?
    return                          ; Yes, done.
    movlw   LOW     display0_gamma_low_table
    movwf   TBLPTRL
    movlw   HIGH    display0_gamma_low_table
    movwf   TBLPTRH
    movlw   UPPER   display0_gamma_low_table
    movwf   TBLPTRU
    bra     display0_init_loop          ; And return...

display0_init_loop:
    TBLRD*+
    movlw   0xFF
    cpfseq  TABLAT
    bra     display0_config_write    ; Write Config pair to Display
    ; Delay ms or quit (return)
    TBLRD*+
    tstfsz  TABLAT                  ; End of config?
    bra     $+4                     ; No
    return                          ; Done.
    movf    TABLAT,W
    call    WAITMSX                 ; Wait WREG milliseconds
    bra     display0_init_loop       ; Loop

display0_config_write:               ; With command in WREG
    movf    TABLAT,W
    rcall	DISP_CmdWrite           ; Write command
    TBLRD*+                         ; Get config
    movf    TABLAT,W
	rcall	DISP_DataWrite          ; Write config
    TBLRD*+                         ; Get config
    movf    TABLAT,W
	rcall	DISP_DataWrite          ; Write config
    bra     display0_init_loop       ; Loop


display0_gamma_high_table:
    ; Reg, Dat0, Dat1 or 0xFF,0x00,0x00 for end
    db  0x70,0x1B,0x80,0x71,0x1F,0x00
    db  0x72,0x22,0x00,0x73,0x17,0x11
    db  0x74,0x1A,0x0E,0x75,0x1D,0x15
    db  0x76,0x18,0x11,0x77,0x1E,0x18
    db  0x78,0x1D,0x11,0xFF,0x00,0x00

display0_gamma_full_table:
    ; Reg, Dat0, Dat1 or 0xFF,0x00,0x00 for end
    db  0x70,0x1F,0x00,0x71,0x23,0x80
    db  0x72,0x2A,0x80,0x73,0x15,0x11
    db  0x74,0x1C,0x11,0x75,0x1B,0x15
    db  0x76,0x1A,0x15,0x77,0x1C,0x18
    db  0x78,0x21,0x15,0xFF,0x00,0x00

display0_gamma_low_table:
    ; Reg, Dat0, Dat1 or 0xFF,0x00,0x00 for end
    db  0x70,0x14,0x00,0x71,0x17,0x00
    db  0x72,0x15,0x80,0x73,0x15,0x11
    db  0x74,0x14,0x0B,0x75,0x1B,0x15
    db  0x76,0x13,0x0E,0x77,0x1C,0x18
    db  0x78,0x15,0x0E,0xFF,0x00,0x00


DISP_set_color:;Converts 8Bit RGB b'RRRGGGBB' into 16Bit RGB b'RRRRRGGG GGGBBBBB'
	movwf	DISPLAY1_temp				; Get 8Bit RGB b'RRRGGGBB'
	movwf	DISPLAY2_temp				; Copy
    movff   WREG,win_color6             ; Another (bank-safe) copy

    ; Display0
	; Mask Bit 7,6,5,4,3,2
	movlw	b'00000011'
	andwf	DISPLAY2_temp,F

	movlw	b'00000000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'01010000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'10100000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'11111000'
	movwf	DISPLAY3_temp				; Blue done.

	movff	DISPLAY1_temp,	DISPLAY2_temp	; Copy
	; Mask Bit 7,6,5,1,0
	movlw	b'00011100'
	andwf	DISPLAY2_temp,F
	rrncf	DISPLAY2_temp,F
	rrncf	DISPLAY2_temp,F

	movlw	b'00000000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00000100'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00001000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00001100'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00010000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00010100'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00100000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00111111'
	movwf	DISPLAY4_temp			

	rrcf	DISPLAY4_temp,F
	rrcf	DISPLAY3_temp,F

	rrcf	DISPLAY4_temp,F
	rrcf	DISPLAY3_temp,F

	rrcf	DISPLAY4_temp,F
	rrcf	DISPLAY3_temp,F		; DISPLAY3_temp (b'GGGBBBBB') done.

	movff	DISPLAY1_temp,	DISPLAY2_temp	; Copy
	clrf	DISPLAY1_temp

	rrcf	DISPLAY4_temp,F
	rrcf	DISPLAY1_temp,F

	rrcf	DISPLAY4_temp,F
	rrcf	DISPLAY1_temp,F

	rrcf	DISPLAY4_temp,F
	rrcf	DISPLAY1_temp,F		; Green done.

	; Mask Bit 4,3,2,1,0
	movlw	b'11100000'
	andwf	DISPLAY2_temp,F

	rrncf	DISPLAY2_temp,F
	rrncf	DISPLAY2_temp,F
	rrncf	DISPLAY2_temp,F
	rrncf	DISPLAY2_temp,F
	rrncf	DISPLAY2_temp,F

	movlw	b'00000000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00000100'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00001000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00001100'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00010000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00010100'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00011000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00011111'
	movwf	DISPLAY4_temp			

	rrcf	DISPLAY4_temp,F
	rrcf	DISPLAY1_temp,F

	rrcf	DISPLAY4_temp,F
	rrcf	DISPLAY1_temp,F	

	rrcf	DISPLAY4_temp,F
	rrcf	DISPLAY1_temp,F

	rrcf	DISPLAY4_temp,F
	rrcf	DISPLAY1_temp,F

	rrcf	DISPLAY4_temp,F
	rrcf	DISPLAY1_temp,F		; Red done.

	movff	DISPLAY1_temp,win_color1
	movff	DISPLAY3_temp,win_color2	; Set Bank0 Color registers...

    movff   win_flags,WREG          ; Display1? win_flags is in bank0...
    btfss   WREG,1                  ; Display1?
	return                          ; No

DISP_set_color_display1:;Converts 8Bit RGB b'RRRGGGBB' into 24Bit RGB b'00RRRRRR 00GGGGGG 00BBBBBB'
    movff   win_color6,DISPLAY1_temp
    movff   win_color6,DISPLAY2_temp

	; Mask Bit 7,6,5,4,3,2
	movlw	b'00000011'
	andwf	DISPLAY2_temp,F

	movlw	b'00000000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'01010000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'10100000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'11111000'
    movff   WREG,win_color6              ; B

	movff	DISPLAY1_temp,	DISPLAY2_temp	; Copy
	; Mask Bit 7,6,5,1,0
	movlw	b'00011100'
	andwf	DISPLAY2_temp,F
	rrncf	DISPLAY2_temp,F
	rrncf	DISPLAY2_temp,F

	movlw	b'00000000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00010000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00100000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'00110000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'01000000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'01010000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'10000000'
	dcfsnz	DISPLAY2_temp,F
	movlw	b'11111100'
    movff   WREG,win_color5              ; G

	; Mask Bit 4,3,2,1,0
	movlw	b'11100000'
	andwf	DISPLAY1_temp,F

	rrncf	DISPLAY1_temp,F
	rrncf	DISPLAY1_temp,F
	rrncf	DISPLAY1_temp,F
	rrncf	DISPLAY1_temp,F
	rrncf	DISPLAY1_temp,F

	movlw	b'00000000'
	dcfsnz	DISPLAY1_temp,F
	movlw	b'00010000'
	dcfsnz	DISPLAY1_temp,F
	movlw	b'00100000'
	dcfsnz	DISPLAY1_temp,F
	movlw	b'00110000'
	dcfsnz	DISPLAY1_temp,F
	movlw	b'01000000'
	dcfsnz	DISPLAY1_temp,F
	movlw	b'01010000'
	dcfsnz	DISPLAY1_temp,F
	movlw	b'10000000'
	dcfsnz	DISPLAY1_temp,F
	movlw	b'11111100'
    movff   WREG,win_color4              ; R
	return
