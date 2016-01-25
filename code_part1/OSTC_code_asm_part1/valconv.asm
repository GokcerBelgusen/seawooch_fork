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


; converts hex values to dez values
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 13/10/07
; 10/12/2010 jDG: optimize macro size.
; last updated: 10/12/2010
; known bugs:
; ToDo: clean up!!!


output_16_3 macro			; displays only last three figures from a 16Bit value (0-999)
	call 	output16_3_call
	endm

output_16dp macro temp4		; 16Bit with decimal point
	movlw	temp4			; Temp4 stores position for decimal point
	call 	output16		
	endm					

output_16 macro				; 16Bit Normal
	call 	output16_call
	endm

output_8 macro				; 8 Bit Normal
	call output8_call
	endm

output_99 macro				; displays only last two figures from a 8Bit value (0-99)
	call 	output99_call
	endm

output_99x macro			; displays only last two figures from a 8Bit value with leading zero (00-99) 
	call 	output99x_call
	endm

output99_call:
	clrf	ignore_digits
	incf	ignore_digits,F
	clrf	temp4

output99:
	movlw	d'99'
	cpfslt	lo
	movwf	lo							; Limit to 99
	movff	lo,lo_temp		
	clrf	hi_temp
	bcf		pre_zero_flag	; do not display leading zeros

LCD_val99_2:	
	movlw	b'00001010'	; 10
	movwf	temp2		
	clrf	temp3		
	rcall	DEC2ASCII

	movlw	b'00000001'	; 1
	movwf	temp2		
	clrf	temp3		
	bsf		pre_zero_flag ; last figure, display zero (0)
	rcall	DEC2ASCII
	RETURN

output99x_call:
	clrf	ignore_digits
	incf	ignore_digits,F
	clrf	temp4

	movlw	d'99'
	cpfslt	lo
	movwf	lo							; Limit to 99
	movff	lo, lo_temp		
	clrf	hi_temp
	bsf		pre_zero_flag		; display leading zeros
	bra		LCD_val99_2			
	
output8_call:	
    clrf	ignore_digits
	incf	ignore_digits,F
	clrf	temp4

output8:
	movff	lo, lo_temp		
	clrf	hi_temp
	bcf		pre_zero_flag	; do not display leading zeros
	
	movlw	b'01100100'	; 100
	movwf	temp2		
	clrf	temp3		
	rcall	DEC2ASCII
	bra		LCD_val99_2			

output16_3_call:
	clrf	ignore_digits
	incf	ignore_digits,F
	bsf		show_last3	
    ; Limit to 3
    movlw   .4
    cpfslt  hi
    bra     output16_3_call_2
    movlw   .3
    cpfseq  hi          ; =3?
    bra     output16_3_call_3   ; No, done.
    movlw   .231                ; Limit to 231(+768=999...)
    cpfslt  lo
    movwf   lo
    bra     output16_3_call_3   ; done.
output16_3_call_2:  ; Set to .999
    movlw   LOW     .999
    movwf   lo
    movlw   HIGH    .999
    movwf   hi
output16_3_call_3:
	clrf	WREG
	bra     output16

output16_call:
	clrf	ignore_digits
	incf	ignore_digits,F
	clrf	WREG

output16:
	movwf	temp4           ; Passed from output16dp macro, cleared by others.

	bcf		all_zeros_flag	; do not display any zero from here unless there was at least one figure /zero

	bsf		leading_zeros
	incf	temp4,1
	decfsz	temp4,F
	bcf		leading_zeros

	bsf		DP_done2		
	incf	temp4,1
	decfsz	temp4,F
	bcf		DP_done2		; decimal point not yet set
	
	movff	lo, lo_temp		
	movff	hi, hi_temp		
	bcf		pre_zero_flag	; do not display leading zeros
	
	movlw	b'00010000'	; 10000s
	movwf	temp2		 
	movlw	b'00100111'
	movwf	temp3		 
	btfss	show_last3		; display only last three figures?
	rcall	DEC2ASCII
	
	movlw	b'11101000'	; 1000s
	movwf	temp2		 
	movlw	b'00000011'
	movwf	temp3		 
	btfsc	DP_done2			; Is there a decimal point at all?
	bra		output16_2			; no, use normal display mode

	btfsc	all_zeros_flag		; display any zero from here
	bra		output16_1			; there was a figure /zero already

	bsf		pre_zero_flag		; display figure if zero?
	decfsz	temp4,W		
	bcf		pre_zero_flag		; No

output16_1:
	btfsc	DP_done				; Decimal point set already?
	bsf		pre_zero_flag		; Yes, so display the rest
output16_2:
	btfss	show_last3			; display only last three figures?
	rcall	DEC2ASCII
	bcf		show_last3			; No, so display the rest

	movlw	b'01100100'	; 100s
	movwf	temp2		 
	clrf	temp3		

	btfsc	ignore_digit3		; Ignore 3rd-5th digit?
	bra		output16_5			; Yes, skip the rest

	btfsc	DP_done2			; Is there a decimal point at all?
	bra		output16_3			; no, use normal display mode

	btfsc	all_zeros_flag		; display any zero from here
	bra		output16_2_1		; there was a figure /zero already

	bsf		pre_zero_flag		; display figure if zero?
	decfsz	temp4,W		
	bcf		pre_zero_flag		; No

output16_2_1:
	btfsc	DP_done				; Decimal point set already?
	bsf		pre_zero_flag		; Yes, so display the rest
	btfsc	DP_done2			; Is there a decimal point at all?
	bsf		pre_zero_flag		; No, so display the rest
output16_3:
	rcall	DEC2ASCII

	movlw	b'00001010'	; 10s
	movwf	temp2		
	clrf	temp3		
	btfsc	DP_done2	
	bra		output16_4	

	btfsc	all_zeros_flag		; display any zero from here
	bra		output16_3_1		; there was a figure /zero already

	bsf		pre_zero_flag
	decfsz	temp4,W		
	bcf		pre_zero_flag

output16_3_1:
	btfsc	DP_done		
	bsf		pre_zero_flag
	btfsc	DP_done2	
	bsf		pre_zero_flag		
output16_4:
	btfsc	ignore_digit4		; Ignore 4-5th digit?
	bra		output16_5			; Yes, skip the rest
	rcall	DEC2ASCII

	movlw	b'00000001'	; 1s
	movwf	temp2		
	clrf	temp3		 
	bsf		pre_zero_flag
	btfss	ignore_digit5		; Ignore 5th digit?
	rcall	DEC2ASCII			; No!
	bcf		ignore_digit5		; yes, and clear flag
output16_5:
	bcf		ignore_digit3		; Clear flag
	clrf	ignore_digits
	incf	ignore_digits,F
	bcf		DP_done
	RETURN
	
DEC2ASCII	clrf	temp1		; converts into ASCII code
DEC2ASCII_2	movf	temp3,W
	subwf	hi_temp,0
	btfss	STATUS,C	
	bra		DEC2ASCII_4	
	bnz		DEC2ASCII_3		

	movf	temp2,W	
	subwf	lo_temp,0
	btfss	STATUS,C
	bra		DEC2ASCII_4
	
DEC2ASCII_3	movf	temp3,W
	subwf	hi_temp,1
	movf	temp2,W
	subwf	lo_temp,1
	btfss	STATUS,C
	decf	hi_temp,1
	incf	temp1,1	
	bsf		pre_zero_flag
	bra		DEC2ASCII_2

DEC2ASCII_4
	decfsz	ignore_digits,F
	return
	incf	ignore_digits,F
	movlw	'0'				; Offset for Ascii-value
	addwf	temp1,0	
	btfsc	pre_zero_flag	; is this a leading zero?
	bra		DEC2ASCII_4_1	; no
	btfsc	leftbind
	bra		DEC2ASCII_6
	movlw	' '				; instead of leading zeros a space!
	bra		DEC2ASCII_5

DEC2ASCII_4_1:
	bsf		all_zeros_flag	; display any zero from here
DEC2ASCII_5
	movwf	POSTINC2
DEC2ASCII_6	
	decfsz	temp4,F			; Set decimal point?
	RETURN					; No
	movlw	"."				; Yes
	movwf	POSTINC2
	bsf		DP_done
	RETURN