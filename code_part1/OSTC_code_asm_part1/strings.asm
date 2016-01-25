;=============================================================================
;
;    File strings.asm
;
;    Implementation code various string functions.
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
;  2010-12-02 : [jDG] Creation...
;
; See strings.inc for doc and public calling macros.

VARARGS_BEGIN   macro
        movff   TOSL, TBLPTRL
        movff   TOSH, TBLPTRH
        movff   TOSU, TBLPTRU
        endm

VARARGS_GET8    macro   register
        tblrd*+
        movff   TABLAT, register
        endm

VARARGS_GET16   macro   register
        tblrd*+
        movff   TABLAT, register+0
        tblrd*+
        movff   TABLAT, register+1
        endm

VARARGS_ALIGN   macro
        btfss   TBLPTRL,0
        local   no_tblptr_align
        bra     no_tblptr_align
        incf    TBLPTRL
        movlw   0
        addwfc  TBLPTRH
no_tblptr_align:
        endm
        
VARARGS_END macro
        ; Compute string length (modulo 256):
        movf    TOSL,W
        subwf   TBLPTRL,W
        
		; Then 24bit add to return address
		addwf	TOSL,F
		movlw	0			            ; Clear WREG, but keep carry
		addwfc	TOSH,F
		addwfc	TOSU,F
		endm

;=============================================================================
; Variants that call word_processor at the end:
strcpy_block_print:
        lfsr    FSR2, letter
strcat_block_print:
        clrf    PRODL,A
        bra     strings_common

; Variants that do not call word_processor at end:
strcpy_block:
        lfsr    FSR2, letter
strcat_block:
        setf    PRODL,A
        
; Common part: append the string from PROM return address:
strings_common:
        VARARGS_BEGIN

strcpy_loop:
        tblrd*+
        movf    TABLAT,W
        movwf   POSTINC2
        bnz     strcpy_loop
        movf    POSTDEC2,W               ; rewind one char in string buffer.

        VARARGS_ALIGN
        VARARGS_END
		
		btfsc   PRODL,0,A               ; Should we print afterward ?
		return                          ; NO: then return straight away.
		goto    word_processor          ; ELSE: print it...

;=============================================================================

start_small_block:
        clrf        WREG,A
        movff       WREG, win_font      ; Need a bank-safe move here !
        movff       WREG, win_invert
        bra         start_common

start_small_invert_block:
        clrf        WREG,A
        movff       WREG, win_font      ; Need a bank-safe move here !
        setf        WREG,A
        movff       WREG, win_invert
        bra         start_common

start_medium_block:
        movlw       1
        movff       WREG, win_font      ; Need a bank-safe move here !
        clrf        WREG,A
        movff       WREG, win_invert
        bra         start_common

start_medium_invert_block:
        movlw       1
        movff       WREG, win_font      ; Need a bank-safe move here !
        movff       WREG, win_invert
        bra         start_common

start_large_block:
        movlw       2
        movff       WREG, win_font      ; Need a bank-safe move here !
        clrf        WREG,A
        movff       WREG, win_invert
        bra         start_common

start_large_invert_block:
        movlw       2
        movff       WREG, win_font      ; Need a bank-safe move here !
        movlw       1
        movff       WREG, win_invert
        bra         start_common

start_common:
        VARARGS_BEGIN
            VARARGS_GET8    win_leftx2
            VARARGS_GET8    win_top
        VARARGS_END
        return

;=============================================================================

box_std_block:                          ; Use standard color (CF#35)
        call    DISP_standard_color
        bra     box_common
box_color_block:                        ; Use color from WREG
        call	DISP_set_color
        bra     box_common
box_black_block:                        ; Use black color
        clrf    WREG
        movff   WREG,win_color1         ; Bank-safe addressing.
        movff   WREG,win_color2
        movff   WREG,win_color3
        movff   WREG,win_color4
        movff   WREG,win_color5
        movff   WREG,win_color6
box_common:
        VARARGS_BEGIN
            VARARGS_GET8    win_top
            VARARGS_GET8    win_height
            VARARGS_GET8    win_leftx2
            VARARGS_GET8    win_width
        VARARGS_END
        goto    DISP_box

box_frame_std:
        call    DISP_standard_color

box_frame_common:
        VARARGS_BEGIN
            VARARGS_GET8    win_top
            VARARGS_GET8    win_height
            VARARGS_GET8    win_leftx2
            VARARGS_GET8    win_width
        VARARGS_END
        goto    DISP_frame

box_frame_color:
      	call	DISP_set_color
		bra		box_frame_common
