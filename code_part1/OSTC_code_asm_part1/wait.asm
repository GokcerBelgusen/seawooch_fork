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
; Wait routines
; written by: chsw, info@heinrichsweikamp.com
; written: 01/31/04
; last updated: 11/05/07
; known bugs:
; ToDo: 
	
; ==========================================================
; 	WAIT 10 MICROSECONDS  -  16 Mhz and 32 Mhz
; ==========================================================

	IFDEF	SPEED_16MHz
WAIT10US 	macro	wait_temp
			movlw	wait_temp
			call	WAIT10USX
			endm
	ENDIF

	IFDEF	SPEED_32MHz
WAIT10US 	macro	wait_temp
			movlw	wait_temp
			call	WAIT10USX
			movlw	wait_temp
			call	WAIT10USX
			endm
	ENDIF

WAIT10USX	movwf	wait_temp
			goto	JumpIn10us
WAIT10USX2	nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
JumpIn10us:
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			decfsz	wait_temp,1
			goto	WAIT10USX2
			return
; ==========================================================
; 	WAIT 1 MILLISECOND  -  Working with TMR1
; ==========================================================
WAITMS		macro	waitms_temp
			movlw	waitms_temp
			call 	WAITMSX
			endm

WAITMSX     movwf	waitms_temp			; Holds number of ms to wait

            ASSERT_BANK1    .10002      ; Make sure we are in bank1
    Ifdef TESTING
            return                      ; In emulation, just skips waits
    Endif
WAITMSX1:
			movf	TMR1L,W
			addlw	d'66'				; 66*15,26µs ~ 1ms

WAITMSX2	cpfseq	TMR1L
			bra		WAITMSX2			; loop here...

			decfsz	waitms_temp,F
			bra		WAITMSX1
			return