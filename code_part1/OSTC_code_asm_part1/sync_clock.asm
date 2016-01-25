
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


; Syncs RTC with PC
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 13/10/07
; last updated: 08/08/31
; known bugs:
; ToDo: 

; routine echoes the "b" command as ready signal
; PC has to send 6 bytes
; Byte1: hours
; Byte2: minutes
; Byte3: seconds
; Byte4: month
; Byte5: day
; Byte6: year
; All bytes will be checked for plausibility and the clock will be set
; after a timeout of about 20ms, the routine ends

sync_clock:
	bcf		uart_settime				; clear flag
	bcf		PIE1,RCIE					; no interrupt for UART
	bsf		LED_blue					; LEDusb ON
	bcf		PIR1,RCIF					; clear flag

	movlw	"b"							; send echo
	movwf	TXREG						
	call	rs232_wait_tx					; wait for UART
	
	call	rs232_get_byte					; hours
	movff	RCREG, hours

	movlw	d'24'
	cpfslt	hours
	clrf	hours

	call	rs232_get_byte					; minutes
	movff	RCREG, mins

	movlw	d'60'
	cpfslt	mins
	clrf	mins

	call	rs232_get_byte					; seconds
	movff	RCREG, secs

	movlw	d'60'
	cpfslt	secs
	clrf	secs

	call	rs232_get_byte					; month
	movff	RCREG, month

	movlw	d'12'
	cpfsgt	month
	bra	sync_clock0
	movwf	month					

sync_clock0:
	call	rs232_get_byte					; day
	movff	RCREG, day

	movff	month,lo		; new month
	dcfsnz	lo,F
	movlw	.31
	dcfsnz	lo,F
	movlw	.28
	dcfsnz	lo,F
	movlw	.31
	dcfsnz	lo,F
	movlw	.30
	dcfsnz	lo,F
	movlw	.31
	dcfsnz	lo,F
	movlw	.30
	dcfsnz	lo,F
	movlw	.31
	dcfsnz	lo,F
	movlw	.31
	dcfsnz	lo,F
	movlw	.30
	dcfsnz	lo,F
	movlw	.31
	dcfsnz	lo,F
	movlw	.30
	dcfsnz	lo,F
	movlw	.31
	cpfsgt	day						; day ok?
	bra	sync_clock1				; OK
	movlw	.1						; not OK, set to 1st
	movwf	day

sync_clock1:
	call	rs232_get_byte					; year
	movff	RCREG, year

	movlw	d'100'
	cpfslt	year
	clrf	year

	bcf		LED_blue					; LEDusb OFF
	bcf		PIR1,RCIF					; clear flag
	bsf		oneminupdate				; set flag, so new time and date will be updated in surfacemode at once
	bsf		PIE1,RCIE					; enable interrupt for UART
	goto	surfloop_loop					; return to surface loop