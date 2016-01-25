
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


; menu "Set Time"
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 5/19/06
; last updated: 080904
; known bugs:
; ToDo: 


menu_settime:
	call	DISP_ClearScreen
	call	menu_pre_loop_common		; Clear some menu flags, timeout and switches

	bcf		set_minutes
	bcf		menubit4
	bcf		set_year
	bcf		set_day
	bcf		set_month
	clrf	menupos2

    call	DISP_divemask_color
	DISPLAYTEXT	.29			; Set Time
    call	DISP_standard_color
	DISPLAYTEXT	.22			; Time:
	DISPLAYTEXT	.23			; Date:

	call	set_time_refresh

	DISPLAYTEXT	.24			; Set Hours
	
settime_loop:
	btfsc	switch_right
	call	add_hours_or_minutes_or_date

	btfsc	switch_left
	call	set_time_next_or_exit

	btfsc	menubit4
	bra	set_time_done

	btfsc	onesecupdate
	call	timeout_surfmode

	btfsc	onesecupdate
	call	set_dive_modes

	bcf		onesecupdate	

	btfsc	sleepmode
	bra	exit_settime

	btfsc	divemode
	goto	restart			

	bra	settime_loop

set_time_refresh:
	WIN_LEFT	.32
	WIN_TOP		.65
	lfsr	FSR2,letter
	OUTPUTTEXT  .22                     ; "Hours:" (actual length depends on translation)

	movff	hours,lo
	output_99x
	PUTC	':'
	movff	mins,lo
	output_99x
	STRCAT_PRINT "  "

set_date_refresh:
	WIN_LEFT	.32
	WIN_TOP		.95
	lfsr	FSR2,letter
	OUTPUTTEXT  .23                     ; "Date: " (actual length depends on translation)

	movff	month,convert_value_temp+0
	movff	day,convert_value_temp+1
	movff	year,convert_value_temp+2
	call	DISP_convert_date		; converts into "DD/MM/YY" or "MM/DD/YY" or "YY/MM/DD" in postinc2
	STRCAT_PRINT "  "
    return

set_time_done:				; Check date
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
	cpfsgt	day                         ; day ok?
	bra	set_time_done2                  ; OK
	movlw	.1                          ; not OK, set to 1st
	movwf	day

set_time_done2:
	WIN_LEFT	.0
	WIN_TOP		.215
	movlw	(.160-.0)/7                ; full line length, for various translations.
	movwf	temp1
	call	DISP_display_clear_common_y1
	
	movlw	d'5'
	movwf	wait_temp
	clrf	secs
	bcf		onesecupdate
	
set_time_done_loop	
	btfss	onesecupdate
	bra		set_time_done_loop
	bcf		onesecupdate

	call	set_date_refresh

	WIN_LEFT	.32
	WIN_TOP		.65
	lfsr	FSR2,letter
	OUTPUTTEXT  .22                     ; "Hours:" (actual length depends on translation)

	movff	hours,lo
	output_99x
	PUTC	':'
	movff	mins,lo
	output_99x
	PUTC	':'
	movff	secs,lo
	output_99x
	STRCAT_PRINT " "

	decfsz	wait_temp,F
	bra	set_time_done_loop
exit_settime:				;Exit
	movlw	d'1'
	movwf	menupos
	goto	more_menu2


set_time_next_or_exit:
	btfsc	set_year
	bsf		menubit4		; quit settime
	incf	menupos2,F
	movff	menupos2,menupos3

	dcfsnz	menupos3,F
	bsf		set_minutes
	dcfsnz	menupos3,F
	bsf		set_month
	dcfsnz	menupos3,F
	bsf		set_day	
	dcfsnz	menupos3,F
	bsf		set_year
	
	WIN_LEFT	.0
	WIN_TOP		.215
    call    DISP_standard_color    
	lfsr	FSR2,letter
	OUTPUTTEXT	.94			    ; Set

	movff	menupos2,menupos3
	decfsz	menupos3,F
	bra	    set_time_next_or_exit2
	OUTPUTTEXT	.90				; Minutes
	bra	    set_time_next_or_exit5
set_time_next_or_exit2:	
	decfsz	menupos3,F
	bra	    set_time_next_or_exit3
	OUTPUTTEXT	.91				; Month
	bra	    set_time_next_or_exit5
set_time_next_or_exit3:	
	decfsz	menupos3,F
	bra	    set_time_next_or_exit4
	OUTPUTTEXT	.92				; Day
	bra	    set_time_next_or_exit5
set_time_next_or_exit4:	
	OUTPUTTEXT	.93				; Year

set_time_next_or_exit5:	
	call	word_processor
	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!
	return
	
add_hours_or_minutes_or_date:
	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!
	clrf	secs

	btfsc	set_year
	bra		add_year

	btfsc	set_day
	bra		add_day

	btfsc	set_month
	bra		add_month

	btfsc	set_minutes
	bra		add_minutes
; Increase hours
add_hours:
	incf	hours,F
	movlw	d'23'
	cpfsgt	hours
	bra	set_time_refresh_x
	clrf	hours
	bra	set_time_refresh_x
add_minutes:
	incf	mins,F
	movlw	d'59'
	cpfsgt	mins
	bra	set_time_refresh_x
	clrf	mins
	bra	set_time_refresh_x
add_day:
	incf	day,F
	movlw	d'31'
	cpfsgt	day
	bra	set_time_refresh_x
	movlw	d'1'
	movwf	day
	bra	set_time_refresh_x
add_month:
	incf	month,F
	movlw	d'12'
	cpfsgt	month
	bra	set_time_refresh_x
	movlw	d'1'
	movwf	month
	bra	set_time_refresh_x
add_year:
	incf	year,F
	movlw	d'20'					; calendar until 2020
	cpfsgt	year
	bra	set_time_refresh_x
	movlw	d'10'
	movwf	year					; Set Year to 2010
	
set_time_refresh_x:
	call	set_time_refresh
	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!
	return
