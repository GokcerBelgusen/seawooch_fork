;=============================================================================
;
;    File aa_wordprocessor.asm
;
;    Anti-aliased word processor
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
;  2010-11-22 : [jDG] Creation.
;  2010-12-01 : [jDG] Adding 3bits antialiased fonts.
;  2010-12-30 : [jDG] Revised to put temp into ACCESSRAM0
;
; BUGS :
;  * If the three fonts are not in the same half of the PROM memory, TBLPTRU
;    will be badly set, and font48 or font90 will display giberish...
;=============================================================================
;
; MEMORY FOOTPRINT:
;------------------
;
; wp_wordprocessor : 8KB, including fonts.
; aa_wordprocessor : 0.5KB code
;                  + 3.5KB aa_font28 (reduced to 99 chars)
;                  + 1.6KB aa_font48
;                  + 2.2KB aa_font90
;                  = 7.9 KB including fonts...
;
; Input registers:
;	letter:26				String to print.
;	win_font				Font size (0=small, 1=medium, 2=large)
;	win_color1:2			16bits unpacked color
;	win_top, win_leftx2		Position on screen
;	win_inverse				Inverse video mode.
;
; Available general purpose registers:
;   PRODH, PRODL	(needed for array indexing)
;   FSRx            12bits. Usefull as RAM pointers.
;=============================================================================
; Temporary variables are overlayed in bank 0 ACCESS, used also by C-code
; (p2_deco), MPLAB math and stdlib libraries.

    CBLOCK 0x000                        
        	aa_flags                ; Various flags for aa_wordprocessor
        	aa_width:2              ; Width in pixels (0..319)
        	aa_bitlen               ; Count of pixels when decoding bitmaps.
        	aa_start:2              ; PROM ptr to start of encoded bitmap
        	aa_end:2                ; and end of it.
        	aa_temp:2               ; Current color, divided by 2 or 4
    ENDC

; Flags allocation:
#define		aa_antialias	aa_flags,0
#define		aa_color_quart	aa_flags,1
#define		aa_color_half	aa_flags,2

;------------------------------------------------------------------------------
; Setup pointers for a char:
; Inputs	WREG = char to draw, win_font
; Output	aa_start, aa_end, win_height, aa_flags
; Trashed	PRODH, PRODL, TBLPTR, TABLAT
;
aa_char_setup:
		movwf	PRODL,A				    ; save char into PROD for now.

		movf	win_font,W,BANKED	    ; Get font number (updates Z flag)
		bnz		aa_char_1

		; SMALL font ---------------------------------------------------------
		; Font SMALL character folding...
aa_char_0:
		movlw	LOW aa_font28_block
		movwf	TBLPTRL,A
		movlw	HIGH aa_font28_block
		movwf	TBLPTRH,A
		bra		aa_char_3

		; MEDIUM font --------------------------------------------------------
aa_char_1:
		decfsz	WREG,A				    ; This is medium font ???
		bra		aa_char_2

		; Font MEDIUM block:
		movlw	LOW aa_font48_block
		movwf	TBLPTRL,A
		movlw	HIGH aa_font48_block
		movwf	TBLPTRH,A
		bra		aa_char_3

		; LARGE font ---------------------------------------------------------
aa_char_2:
		decfsz	WREG,A				    ; This is medium font ???
		bra		aa_char_2a

		; Font LARGE block:
		movlw	LOW aa_font90_block
		movwf	TBLPTRL,A
		movlw	HIGH aa_font90_block
		movwf	TBLPTRH,A
        bra		aa_char_3

		; HUGE font ---------------------------------------------------------
aa_char_2a:
		; Font HUGE block:
		movlw	LOW aa_font120_block
		movwf	TBLPTRL,A
		movlw	HIGH aa_font120_block
		movwf	TBLPTRH,A


		; Execute font block -------------------------------------------------
aa_char_3:
        ; This is safe iff the three fonts are in the same code segment
        ; (and that segment do not span the 64K edge...)
		movlw	UPPER aa_font28_block
		movwf	TBLPTRU,A

        ; Proceed to character substitutions
aa_char_30:
		tblrd*+						    ; Read FROM char
		movf	TABLAT,W			    ; Get it, and set Z,N
		bz		aa_char_32			    ; Break at end of translations
		
		tblrd*+						    ; Read TO char
		cpfseq	PRODL,A				    ; FROM == current char ? 
		bra		aa_char_30		    	; Different: loop
		movff	TABLAT, PRODL		    ; make substitution
		bra		aa_char_30			    ; Loop.

        ; Make sure char is in the available range
aa_char_32:
		tblrd*+						    ; Read first char
		movf	TABLAT,W			    ; get it.
		subwf	PRODL,F				    ; (char - first) --> PRODL

		tblrd*+						    ; Read nb chars
		movf	TABLAT,W			    ; nbchars --> WREG
		tblrd*+						    ; Read default char
		cpfslt	PRODL				    ; if char > WREG ?
		movff	TABLAT,PRODL		    ; replace PRODL

        ; Decode font height and anti-aliasing mode
		clrf	aa_flags,ACCESS         ; Default to no AA
		tblrd*+						    ; Read font height + AA flag
		movf	TABLAT,W			    ; into WREG
		bnn		aa_char_34			    ; High bit set ?
		bsf		aa_antialias,ACCESS     ; YES : then the font is AA.
aa_char_34:
		andlw	0x7F				    ; Keep just font height,
		movwf	win_height,BANKED	    ; then save it (its a register)

        ; Set PROM pointer to the char index
		movf	PRODL,W				    ; Read back char
		mullw	2					    ; PROD = 2*(char - base), TBLPTR=idx
		movf	PRODL,W
		addwf	TBLPTRL,F			    ; Add into TBLPTR (low byte)
		movf	PRODH,W
		addwfc	TBLPTRH,F			    ; and high byte.

        ; Read start and stop pointers
		tblrd*+						    ; aa_start = PROM16(*tblptr++)
		movff	TABLAT,aa_start+0       ; Read low byte
		tblrd*+
		movff	TABLAT,aa_start+1       ; and high byte

		tblrd*+						    ; aa_end = PROM16(*tblptr++)
		movff	TABLAT,aa_end+0         ; Read low byte
		tblrd*+
		movff	TABLAT,aa_end+1         ; and high byte

		return

;------------------------------------------------------------------------------
; Character width
; Inputs	aa_start, aa_end, aa_width, win_height, aa_flags
; Output	width added to aa_width
; Trashed	aa_bitlen, TBLPTR, TABLAT
;
aa_char_width:
		movff	aa_start+0, TBLPTRL     ;	TBLPTR = aa_start
		movff	aa_start+1, TBLPTRH
		clrf	aa_bitlen,ACCESS	    ; clear reminders...

		; Read bitmap byte, and decode length:
aa_char_width_1:
    ifdef AA_BYTE_SWAP
		btg		TBLPTRL,0,A			    ; Toggle low ptr bit.
		tblrd*
		movf	TABLAT,W,A			    ; Store to WREG
		btg		TBLPTRL,0,A			    ; Get is back
		tblrd*+						    ; then increment (but trash TABLAT)
		movwf	TABLAT,A			    ; Then restore copy to TABLAT.
    else
		tblrd*+						    ; Normal read...
		movf	TABLAT,W,A			    ; Store copy to WREG
    endif
		btfss	aa_antialias,ACCESS     ; Antialiased font ?
		bra		aa_char_width_10	    ; No: always 7 bits count

		bn		aa_char_width_10	    ; Non-white pixels ?
		andlw	0x1F				    ; Yes : 5 bits count.
aa_char_width_10:
		andlw	0x7F				    ; No: 7 bit count.
		incf	WREG,A				    ; WREG = repetition count
		addwf	aa_bitlen,F,ACCESS	    ; Add remaining pixels from last code.
		
		movf	win_height,W,BANKED	    ; WREG = - height
		negf	WREG,A

		; This is a hand-made division by successive substraction of height
aa_char_width_2:
		addwf	aa_bitlen,F,ACCESS      ; Try to substract win_height
		bn		aa_char_width_3		    ; If neg it was a bad idea...

		infsnz	aa_width+0,F,ACCESS     ; Succeded: do a 16bit increment
		incf	aa_width+1,F,ACCESS     ; on the aa_width counter.
		bra		aa_char_width_2		    ; and loop.

aa_char_width_3:
		negf	WREG,A				    ; WREG = +height
		addwf	aa_bitlen,F,ACCESS      ; Restore true reminder.

		; Are we done ?
		movf	TBLPTRL,W,A			    ; Compare TBLPTR to aa_end
		cpfseq	aa_end+0,ACCESS
		bra		aa_char_width_1		    ; Loop if LOW is different
		movf	TBLPTRH,W,A
		cpfseq	aa_end+1,ACCESS         ; Loop to if HIGH is different
		bra		aa_char_width_1

		return

;------------------------------------------------------------------------------
; String width
; Inputs	letter (SHOULD BE NULL TERMINATED)
; Output	aa_width, win_height
; Trashed	PROD, TBLPTR, FSR2, aa_bitlen, aa_start, aa_end, aa_flags
;
aa_string_width:
		lfsr	FSR2, letter		    ; FSR2 pointer to start of string.

		clrf	aa_width+0,ACCESS       ; Clear width sum.
		clrf	aa_width+1,ACCESS       ; (16 bit counter)

aa_string_width_1:
		movf	POSTINC2,W,A		    ; WREG = *FSR2++
		bz		aa_string_width99	    ; Exit if null byte encountered.

		rcall	aa_char_setup		    ; setup aa_start / aa_end
		rcall	aa_char_width		    ; sum-up width into aa_width
		bra		aa_string_width_1	    ; and loop.

aa_string_width99:
		return

;------------------------------------------------------------------------------
; Decode a compressed char.
; Inputs	aa_start, aa_end, win_height, win_invert, win_color1, win_color2
; Output	none
; Trashed	TBLPTR, TABLAT, PROD, aa_bitlen, aa_flags, aa_colorDir:2
;
aa_decode_char:
		movff	aa_start+0, TBLPTRL     ; TBLPTR = aa_start
		movff	aa_start+1, TBLPTRH

		; Read bitmap byte, and decode color & length
aa_decode_1:
    ifdef AA_BYTE_SWAP
		btg		TBLPTRL,0,A			    ; Toggle low ptr bit.
		tblrd*
		movf	TABLAT,W,A			    ; Store to WREG
		btg		TBLPTRL,0,A			    ; Get is back
		tblrd*+						    ; then increment (but trash TABLAT)
		movwf	TABLAT,A			    ; Then restore copy to TABLAT.
    else
		tblrd*+						    ; Normal read...
		movf	TABLAT,W,A			    ; Store copy to WREG
    endif
		btfss	aa_antialias,ACCESS     ; Antialiased font ?
		bra		aa_decode_10		    ; No: always 7 bits count
		bn		aa_decode_10		    ; Non-white pixels ?
		andlw	0x1F				    ; Yes : 5 bits count.
aa_decode_10:
		andlw	0x7F				    ; No: 7 bit count.
		incf	WREG,A				
		movwf	aa_bitlen,ACCESS        ; repetition count --> aa_bitlen

		;---- COLOR DECODING -------------------------------------------------
		;
		;   Code    Normal    Inverse
		;   1xx        0%      100%	: Managed by aa_decode_13
		;   011       25%       75%
		;   010       50%       50%
		;   001       75%       25%
		;   000      100%        0% : Managed by aa_decode_13 too.
		;
		movf	TABLAT,W,A			    ; Get back code
		btfss	aa_antialias,ACCESS     ; Antialiased font ?
		bra		aa_decode_13		    ; NO: 1bit case

		; Asymetry test: 1xx code is another case for 1bit color.
		; This have to be done before inverse video, because
		; of the asymetric processing !
		bn		aa_decode_13		    ; decode as not-aa

		; Manage 000 special case too:
		andlw	0xE0				    ; Select color bits
		bz		aa_decode_13		    ; That's a 000 !

		; Apply reverse video, in a reversed way
		btfss	win_invert,0		    ; Inverse video mode ?
		sublw	0x80

		; Move the two bits to aa_color_half and aa_color_quarter:
		swapf	WREG				    ; --> 0000.0LL0 byte
		iorlw	b'001'				    ; We are in AA mode, don't forget it !
		movwf	aa_flags,ACCESS         ; save that to aa_color_(half/quad)/AA flags.

		;---- 2 bit x RGB(16bits) computation --------------------------------
		clrf	PRODL				    ; We will accumulate result here...
		clrf	PRODH

		; Take color div 2 into aa_temp. Max red = 15/31
		rrcf	win_color1,W,BANKED	    ; xRRRRxGG
		andlw	b'01111011'			    ; 0RRRR0GG (don't change C)
		movwf	aa_temp+0,ACCESS
		rrcf	win_color2,W,BANKED	    ; GGGxBBBB
		andlw	b'11101111'			    ; GGG0BBBB
		movwf	aa_temp+1,ACCESS

		btfss	aa_color_half,ACCESS
		bra		aa_decode_12

		movff	aa_temp+0,PRODH         ; Add color/2 if bit set.
		movff	aa_temp+1,PRODL         ; DISPLAY is big endian, so swap here.
aa_decode_12:
		btfss	aa_color_quart,ACCESS
		bra		aa_decode_12b

		; Divide it once again by 2. Max red = 7/31.
		rrcf	aa_temp+0,W,ACCESS      ; xxRRRxxG
		andlw	b'00111001'             ; 00RRR00G (don't change C)
		movwf	aa_temp+0,ACCESS
		rrcf	aa_temp+1,W,ACCESS      ; GGGxxBBB
		andlw	b'11100111'             ; GGG00BBB
		movwf	aa_temp+1,ACCESS

		movf	aa_temp+1,W,ACCESS      ; Add color/4
		addwf	PRODL,F				    ; NOTE: 7/31+15/31=22/31,
		movf	aa_temp+0,W,ACCESS      ; hence composants won't overlap.
		addwfc	PRODH,F				    ; In right order, to propagate carry.

aa_decode_12b:
        movff   win_flags,WREG          ; BEWARE: bank0 bit-test
        btfss   WREG,1                  ; Display1?
        bra		aa_decode_3			    ; No, Done.

        ; Convert 16Bit RGB b'RRRRRGGG GGGBBBBB' into 24Bit RGB b'RRRRRR00 GGGGGG00 BBBBBB00'
                            ; PRODH     PRODL                  win_color4 win_color5 win_color6
        ; Blue
        movff   PRODL,win_color6
        bcf     STATUS,C
        rlcf    win_color6,F            ; = UUBBBBB0
        bcf     STATUS,C
        rlcf    win_color6,F            ; = UBBBBB00
        bcf     STATUS,C
        rlcf    win_color6,F            ; = BBBBB000
        btfsc   win_color6,7
        bsf     win_color6,2

        ; Green
        rrcf    PRODH,F
        rrcf    PRODL,F
        rrcf    PRODH,F
        rrcf    PRODL,F
        rrcf    PRODH,F                 ; = UUURRRRR
        rrcf    PRODL,F                 ; = GGGGGGUU
        bcf     PRODL,1                 ; = GGGGGG0U
        bcf     PRODL,0                 ; = GGGGGG00
        movff   PRODL,win_color5

        ; Red
        bcf     STATUS,C
        rlcf    PRODH,F                 ; = UURRRRR0
        bcf     STATUS,C
        rlcf    PRODH,F                 ; = URRRRR00
        bcf     STATUS,C
        rlcf    PRODH,F                 ; = RRRRR000
        btfsc   PRODH,7
        bsf     PRODH,2
        movff   PRODH,win_color4
		bra		aa_decode_3			    ; Done.

		; ---- Simple BLACK and WHITE cases ------------------------------
aa_decode_13:							; Got a 1xx or a 000 code...
		btfsc	win_invert,0		    ; Inverse video mode ?
		xorlw	0x80				    ; YES: invert levels.
		bn		aa_decode_2			    ; Then test high bit.

		; WHITE pixel (ie. full color)
		movff	win_color1,PRODH	    ; current draw color
		movff	win_color2,PRODL	    ; (rem: DISPLAY is big endian)
		bra		aa_decode_12b

aa_decode_2:
		clrf	PRODH,A				    ; BLACK pixel
		clrf	PRODL,A
        clrf    win_color4
        clrf    win_color5
        clrf    win_color6

aa_decode_3:
		;---- PIXEL WRITE LOOP -----------------------------------------------
;		AA_DATA_WRITE_PROD
    	bsf		DISPLAY_rs					; Data!

        movff   win_flags,WREG          ; Display1? win_flags is in bank0...
        btfsc   WREG,1                  ; Display1?
        bra     aa_decode_3_display1    ; Yes.

    	movff	PRODH,PORTD				; Move high byte to PORTD (DISPLAY is bigendian)
        bcf		DISPLAY_rw
        bsf		DISPLAY_rw
        movff	PRODL,PORTD				; Move low byte to PORTD
        bra     aa_decode_3_done

aa_decode_3_display1:
    	movff   win_color4,PORTD		; Move high byte to PORTD (DISPLAY is bigendian)
        bcf		DISPLAY_rw
        bsf		DISPLAY_rw
        movff   win_color5,PORTD		; Move low byte to PORTD
        bcf		DISPLAY_rw
        bsf		DISPLAY_rw
        movff   win_color6,PORTD        ; Move low(est) byte to PORTD
aa_decode_3_done:
        bcf		DISPLAY_rw
        bsf		DISPLAY_rw

		decf	aa_bitlen,F,ACCESS
		bnz		aa_decode_3

		;---- BYTE-CODE LOOP -------------------------------------------------
		; Are we done ?
		movf	TBLPTRL,W,A			    ; Compare TBLPTR to aa_end
		cpfseq	aa_end+0,ACCESS
		bra		aa_decode_1             ; Loop if LOW is different
		movf	TBLPTRH,W,A
		cpfseq	aa_end+1,ACCESS         ; Loop to if HIGH is different
		bra		aa_decode_1
		
		return

;------------------------------------------------------------------------------
; Setup pointers for a char:
; Inputs : letter : string to print (SHOULD BE NULL TERMINATED)
; Output : DISPLAY commands on port D + clocks.
; 
        global  aa_wordprocessor        ; Callable from C-code.
aa_wordprocessor:
		; Make sure context is well known
		movlb	HIGH win_top            ; Switch to bank 0...

		rcall	aa_string_width		    ; Set win_height, compute win_width
		call	DISP_box_write		    ; Use that for the box.

		; Restart the loop for each char to print
		lfsr	FSR2, letter		    ; FSR2 pointer to start of string.

		; DATA bloc commande:
		AA_CMD_WRITE	0x22

aa_wordprocessor_1:
		movf	POSTINC2,W,A		    ; WREG = *FSR2++
		bz		aa_wordprocessor_99	    ; Exit if null byte encountered.

		rcall	aa_char_setup		    ; setup aa_start / aa_end
		rcall	aa_decode_char		    ; write pixels to screen
		bra		aa_wordprocessor_1	    ; and loop.

aa_wordprocessor_99:
		; END of bloc commande
		AA_CMD_WRITE	0x00

		return
