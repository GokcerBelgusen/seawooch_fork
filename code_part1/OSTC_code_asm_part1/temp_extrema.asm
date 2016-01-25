
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


; takes care of the temperature extrema routine
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 05/15/08
; last updated: 05/15/08
; known bugs:
; ToDo:

check_temp_extrema:			; called once every minute from Sleeploop, Surfloop and Diveloop
    clrf    EEADRH
	read_int_eeprom d'54'			; get lowest temperature so far
	movff	EEDATA,sub_b+0
	read_int_eeprom d'55'
	movff	EEDATA,sub_b+1
	SAFE_2BYTE_COPY	temperature,sub_a
	call	sub16					; sub_c = sub_a - sub_b
	btfss	neg_flag				; new lowest temperature ?
	bra		check_temp_extrema_high	
	; Yes, store new value together with the date
	movff	sub_a+0,EEDATA
	write_int_eeprom	d'54'
	movff	sub_a+1,EEDATA
	write_int_eeprom	d'55'
	movff	month,EEDATA
	write_int_eeprom	d'56'
	movff	day,EEDATA
	write_int_eeprom	d'57'
	movff	year,EEDATA
	write_int_eeprom	d'58'

	; Now check high extrema
check_temp_extrema_high:
	read_int_eeprom d'59'			; get highest temperature so far
	movff	EEDATA,sub_b+0
	read_int_eeprom d'60'
	movff	EEDATA,sub_b+1
	SAFE_2BYTE_COPY	temperature,sub_a
	call	sub16					; sub_c = sub_a - sub_b
	btfsc	neg_flag				; new highest temperature ?
	return							; no, quit!

	; Yes, store new value together with the date
	movff	sub_a+0,EEDATA
	write_int_eeprom	d'59'
	movff	sub_a+1,EEDATA
	write_int_eeprom	d'60'
	movff	month,EEDATA
	write_int_eeprom	d'61'
	movff	day,EEDATA
	write_int_eeprom	d'62'
	movff	year,EEDATA
	write_int_eeprom	d'63'
	return