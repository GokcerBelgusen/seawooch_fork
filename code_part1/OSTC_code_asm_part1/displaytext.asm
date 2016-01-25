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


; Displays from text_table_vx.asm
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 10/30/05
; last updated: 100730
; known bugs:
; ToDo:


; These macros output to POSTINC only
OUTPUTTEXT	macro	n                   ; For Texts 1-255
    If n>.255
        Error "Bad low text number", n
    Endif
	movlw	n
	call	displaytext0_low
	endm

OUTPUTTEXTH	macro	n		            ; For Texts 256-511
    If n<.256
        Error "Bad low text number", n
    Endif
	movlw	n-.256			            ; Use only Lower 8 Bit
	call	displaytext0_high
	endm

displaytext0_high:
	bsf		displaytext_high            ; Highbit set
	bra     displaytext0
	
displaytext0_low:
	bcf		displaytext_high            ; Highbit clear
displaytext0:
	bsf     output_to_postinc_only
	bra     displaytext

; These macros output to letter[], and call the wordprocessor
DISPLAYTEXT	macro	n
	movlw	n
	call	displaytext_1_low
	endm

DISPLAYTEXTH	macro	n
	movlw   LOW	n			        ; Use only Lower 8 Bit
	call    displaytext_1_high
	endm

displaytext_1_high:
	bsf     displaytext_high		; Highbit set
	bra     displaytext

displaytext_1_low:
    bcf     displaytext_high

displaytext:
    movwf   textnumber
    movlw   LOW(text_pointer-4)
    movwf   TBLPTRL
    movlw   HIGH(text_pointer-4)
    movwf   TBLPTRH
    movlw   UPPER(text_pointer-4)
    movwf   TBLPTRU

    movlw   4                           ; textnumber * 4 --> PROD
    mulwf   textnumber

    btfsc   displaytext_high            ; If high text, add 4*256 to PROD
    addwf   PRODH

    movf    PRODL,W                     ; Add PROD to TBLPTR
    addwf   TBLPTRL,F
    movf    PRODH,W
    addwfc  TBLPTRH,F
    movlw   0
    addwfc  TBLPTRU

	TBLRD*+
	movff   TABLAT,textaddress+0        ; textaddress:2 holds address for first character
	TBLRD*+
	movff   TABLAT,textaddress+1
	
	btfsc   output_to_postinc_only      ; output to postinc only?
	bra     displaytext2
	
	TBLRD*+
	movff	TABLAT,win_leftx2			; No, write coordinates

	TBLRD*+
	movff	TABLAT,win_top              ; No, write coordinates

displaytext2:
    clrf    WREG                        ; Reset to small font
    movff   WREG,win_font               ; (BANK 0)

    movff   textaddress+0,TBLPTRL
    movff   textaddress+1,TBLPTRH
	btfss	output_to_postinc_only		; output to postinc2 only?
	lfsr	FSR2,letter					; no!

displaytext3:
	TBLRD*+
	movf    TABLAT,W
	bz      display_text_exit           ; Text finished?
	movwf   POSTINC2
	bra     displaytext3

display_text_exit:
	btfss	output_to_postinc_only		; output to postinc only?
	bra     display_text_exit2
	
	bcf     output_to_postinc_only
	return

display_text_exit2:
    clrf    WREG
    movff   WREG,letter+.22
    goto    word_processor
