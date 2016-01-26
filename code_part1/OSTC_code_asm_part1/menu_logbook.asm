
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


; Menu "Logbook"
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 060107
; last updated: 081026
; known bugs: 
; ToDo: 

;=============================================================================
; Temp data, local to this module, moved to ACCES0 area.
;
    CBLOCK 0x010                ; Keep space for aa_wordprocessor's temp.
        count_temperature       ; Current sample count for temperature divisor
        count_deco              ; Current sample count for deco (ceiling) divisor
        logbook0_ptr:2          ; Loogbook pointer inside EEPROM Bank0
        logbook1_ptr:2          ; Loogbook pointer inside EEPROM Bank1
        logbook_cur_depth:2     ; Current depth, for drawing profile.
        logbook_cur_tp:2        ; Current temperature, for drawing profile.
        logbook_last_tp         ; Y of the last item in Tp° curve.
        logbook_min_tp:2        ; Min temperature, for drawing profile.
        logbook_max_tp:2        ; Maximum temperature, for drawing profile.
        logbook_ceiling         ; Current ceiling, for drawing profile.
		divenumber              ; Dive number
    ENDC

;=============================================================================
; searches external EEPROM for dive headers and displays them in a list
; a detailed view with all informations and profile can be selected
; does not require a FAT, will work with other profile intervals as ten seconds, too

menu_logbook:
	bcf			return_from_profileview				; clear some flags
menu_logbook1:
	bcf			logbook_header_drawn
	call		DISP_ClearScreen				; Clear screen
	bcf			all_dives_shown					; clear some flags
	bcf			logbook_profile_view
	bcf			logbook_page_not_empty
	clrf		menupos3						; Here: used rows on current logbook-page	
	clrf		menupos2						; Here: # of current displayed page
	clrf		divenumber						; # of dive in list during search


menu_logbook1a:
    call	DISP_divemask_color
	DISPLAYTEXT	.12							;" Wait.."
    call	DISP_standard_color
	call 		I2CReset					; Reset I2C Bus
	call		get_free_EEPROM_location	; search from "here" backwards through the external memory

	movff		eeprom_address+0,logbook1_ptr+0
	movff		eeprom_address+1,logbook1_ptr+1	; Store Pointer to 0xFE (From 0xFD, 0xFD, 0xFE sequence) for faster display

menu_logbook1a_no_get_free:				; Without repeated search for dive
	clrf		divemins+0					; Here: used as temp variables
	clrf		divemins+1
	movlw		d'5'
	movwf		menupos					; Here: stores current position on display (5-x)

;-----------------------------------------------------------------------------	
; search external EEPROM backwards from eeprom_address
; for 0xFA, 0xFA (store 1st. 0xFA position for next search)
; read header data and display it
; wait for user to confirm/exit
; recopy data to search from here

menu_logbook1b:
    call	DISP_divemask_color
	DISPLAYTEXT	.12						;" Wait.."
    call	DISP_standard_color

    ;---- fast loop: check every other byte ----------------------------------
menu_logbook2:
	movlw		d'2'
	addwf		divemins+0,F
	movlw		d'0'
	addwfc		divemins+1,F			; increase 16Bit value, twice

    incf        divemins+1,W            ; = 0xFF.. ?
    bnz         menu_logbook2a          ; No.
    incf        divemins+0,W            ; = 0x..FF ?
    bz          menu_logbook_reset      ; Yes: FFFF --> loop.

menu_logbook2a:
    movf        divemins+1,W            ; = 0x00.. ?
    bnz         menu_logbook2b          ; No.
    movf        divemins+0,W            ; = 0x..00 ?
    bz          menu_logbook_reset      ; yes, restart (if not empty)

menu_logbook2b:
	decf_eeprom_address	d'2'			; -2 to eeprom address.

	call		I2CREAD					; reads one byte (Slow! Better use Blockread!)

	movlw		0xFA                    ; That was a FA ?
	cpfseq		SSPBUF
    bra         menu_logbook2           ; No: continue the fast loop...
    
    ;---- Slow check : was it before or after that one ? ---------------------

 	incf_eeprom_address	d'1'			; Been one step too far ?
	call		I2CREAD					; reads one byte (Slow! Better use Blockread!)
	movlw		0xFA                    ; That was a FA ?
	xorwf		SSPBUF,W
	bz          menu_loop_tooFar        ; Got both of them...
	
    infsnz      divemins+0,F            ; Advance to the next byte.
	incf        divemins+1,F
	decf_eeprom_address	d'2'			; One step back, two steps forward.
	call		I2CREAD					; reads one byte (Slow! Better use Blockread!)
	movlw		0xFA                    ; It was the second FA ?
	xorwf		SSPBUF,W
	bz          test_FA_DONE
    bra         menu_logbook2           ; No: continue the fast loop...
   
menu_loop_tooFar;
 	decf_eeprom_address	d'1'			; So stays pointing at the second one.

test_FA_DONE:							; Found 0xFA 0xFA!
	movff		eeprom_address+0,eeprom_header_address+0	; store current address into temp register
	movff		eeprom_address+1,eeprom_header_address+1	; we must continue search here later
	incf		divenumber,F            ; new header found, increase divenumber
	bra			menu_logbook4           ; Done with searching, display the header!

menu_logbook3b:
	btfss		logbook_page_not_empty		; Was there at least one dive?
#IFDEF CCR_CTRL
    ; Reorder Menu structure for CCR
	bra 		exit_logbook_menu			; Not a single header was found, leave logbook.
#ELSE
	goto		menu						; Not a single header was found, leave logbook.
#ENDIF
	bra			menu_logbook_display_loop2	; rcall of get_free_eeprom_location not required here (faster)

menu_logbook_reset:	
	movf		divenumber,W
	btfsc		STATUS,Z					; Was there at least one dive?
	bra			menu_logbook3b				; No, Nothing to do

	bsf			all_dives_shown				; Yes
	bra			menu_logbook_display_loop2	; rcall of get_free_eeprom_location not required here (faster)


menu_logbook4:
	; Adjust eeprom_address to set pointer on first headerbyte
	incf_eeprom_address	d'2'            ; Macro, that adds 8Bit to eeprom_address:2

	btfss		logbook_profile_view			; Display profile (search routine is used in profileview, too)
	bra			menu_logbook_display_loop		; No, display overwiev list

	movf		divesecs,W					; divenumber that is searched
	cpfseq		divenumber					; current divenumber
	goto		next_logbook				; No match, continue search
	bra			display_profile2			; Match: Show header and profile
	

menu_logbook_display_loop:
	btfsc		all_dives_shown				; All dives displayed?
	bra			menu_logbook_display_loop2		; Yes, but display first page again.

	call 		display_listdive			; display short header for list on current list position

	movlw		d'5'
	cpfseq		menupos					; first dive on list (top place)?
	bra			menu_logbook_display_loop1		; no, so skip saving of address


	movff		divenumber,mintemp+0			; store all registered required to rebuilt the current logbookpage after the detail/profile view
	movff		eeprom_header_address+0,decodata+0	; several registers are used as temp registers here
	movff		eeprom_header_address+1,decodata+1		
	movff		divemins+0,max_pressure+0			
	movff		divemins+1,max_pressure+1			

	movlw		d'3'
	addwf		decodata+0,F
	movlw 		d'0'
	addwfc		decodata+1,F				; Re-Adjust pointer again
	movlw		d'3'						; So first row will be the same again after detail/profile view
	subwf		max_pressure+0,F
	movlw		d'0'
	subwfb		max_pressure+1,F

menu_logbook_display_loop1:
	decfsz		menupos,F					; List full?
	goto		next_logbook				; no, search another dive for our current logbook page

menu_logbook_display_loop2:
	btfss		logbook_page_not_empty		; Was there one dive at all?
	bra			menu_logbook				; Yes, so reload the first page

	call	DISP_topline_box_clear	; Clears Bar at the top
    call	DISP_divemask_color
	DISPLAYTEXT	.26							; "Logbook"
    call    DISP_standard_color
	DISPLAYTEXT .11							; Displays "Exit" in the last row on the current page

	call		menu_pre_loop_common		; Clear some menu flags, timeout and switches

	movlw		d'1'						; Set cursor to position 1...
	btfsc		return_from_profileview		; .. unless we are returning from a detail/profile view
	movf		mintemp+1,W					; load last cursor position again
	movwf		menupos						; and set menupos byte
	bcf			return_from_profileview		; Do this only once while the page is loaded again!

	bcf			logbook_page_not_empty			; Obviously the current page is NOT empty
	call		DISP_logbook_cursor

menu_logbook_loop:
	call		check_switches_logbook
	
	btfsc		menubit3					; SET/MENU?
	goto		next_logbook3				; adjust cursor or create new page

	btfsc		menubit2					; ENTER?
	bra			display_profile_or_exit		; view details/profile or exit logbook

	btfsc		onesecupdate
	call		menu_check_dive_and_timeout	; "Goto restart" or sets sleepmode flag

	bcf			onesecupdate				; one second update 

	btfsc		sleepmode					; Timeout?
#IFDEF CCR_CTRL
    ; Reorder Menu structure for CCR
	bra 		exit_logbook_menu			; Not a single header was found, leave logbook.
#ELSE
	goto		menu						; Yes
#ENDIF

	goto		menu_logbook_loop			; Wait for something to do

display_profile_or_exit:
	bcf			menubit2					; debounce
	movlw		d'6'						; exit?
	cpfseq		menupos
	bra			display_profile				; No, show details/profile
#IFDEF CCR_CTRL
    ; Reorder Menu structure for CCR
exit_logbook_menu:
	movlw		d'2'						; New Return position
	movwf		menupos
	goto		menu2                       ; New return Menu
#ELSE						; Not a single header was found, leave logbook.
	goto		menu
#ENDIF

display_profile:
    bcf         is_bailout
	movff		menupos,mintemp+1				; store current cursor position
	bsf			return_from_profileview			; tweak search routine to exit after found

	movf		menupos2,W						; Number of page
	mullw		d'5'				
	movf		PRODL,W						
	addwf		menupos,W						; page*5+menupos=
	movwf		divesecs						; # of dive to search

	call		DISP_ClearScreen				; search for dive
	bsf			logbook_profile_view			; set flag for search routine

	clrf		divenumber						; search from scratch

	movff		logbook1_ptr+0,eeprom_address+0
	movff		logbook1_ptr+1,eeprom_address+1	; Restore Pointer to 0xFE (From 0xFD, 0xFD, 0xFE sequence) for faster display

	bra			menu_logbook1a_no_get_free		; Start Search for Dive (Without get_free_EEPROM_location)

display_profile2:
	bcf			logbook_profile_view			; clear flag for search routine

	clrf		average_divesecs+0
	clrf		average_divesecs+1				; holds amount of read samples

	call		DISP_display_wait_clear
	call		DISP_standard_color
	WIN_TOP		.0
	WIN_LEFT	.0
	STRCPY      "#"

	GETCUSTOM15	.28							; Logbook Offset -> lo, hi
	tstfsz		lo							; lo=0?
	bra			display_profile_offset1		; No, adjust offset	
	tstfsz		hi							; hi=0?
	bra			display_profile_offset1		; No, adjust offset
	bra			display_profile_offset2		; lo=0 and hi=0 -> skip Offset routine

display_profile_offset1:
	movlw		d'1'
	addwf		lo,F
	movlw		d'0'
	addwfc		hi,F						; hi:lo = hi:lo + 1
	movff		lo,sub_a+0
	movff		hi,sub_a+1
	movff		divesecs,sub_b+0
	clrf		sub_b+1
	call		sub16						;  sub_c = sub_a - sub_b
	movff		sub_c+0,lo
	movff		sub_c+1,hi
	bsf			leftbind
	output_16dp	d'10'						; # of dive with offset
	bra			display_profile_offset3		; Skip normal routine
	
display_profile_offset2:
	movff		divesecs,lo				; 
	output_99x							; # of dive

display_profile_offset3:
	PUTC		' '
	call		I2CREAD2				; Read Profile version
	movff		SSPBUF,lo				; store in lo

	bsf			logbook_format_0x21		; Set flag for new 0x21 Format
	movlw		0x21
	cpfseq		lo						; Skip if 0x21
	bcf			logbook_format_0x21		; Clear flag for new 0x21 Format

	call		I2CREAD2				; read month
	movff		SSPBUF,lo				; store in lo

; Offset to SamplingRate
	incf_eeprom_address	d'32'				; Macro, that adds 8Bit to eeprom_address:2
	call		I2CREAD						; Read Sampling rate
	movff		SSPBUF,samplesecs_value		; Copy sampling rate
	decf_eeprom_address	d'32'				; Macro, that subtracts 8Bit from eeprom_address:2

	movff		lo,convert_value_temp+0		; Month (in lo, see above)
	call		I2CREAD2					; Day
	movff		SSPBUF,convert_value_temp+1
	call		I2CREAD2					; Year
	movff		SSPBUF,convert_value_temp+2
	call		DISP_convert_date			; converts into "DD/MM/YY" or "MM/DD/YY" or "YY/MM/DD" in postinc2

	PUTC		' '

	btfss	logbook_format_0x21			; Dive made with new 0x21 format?
	bra		display_end_of_divetime

	PUTC		0x93						; "Start of dive" icon
	call		I2CREAD2					; hour
	movff		SSPBUF,lo
	call		I2CREAD2					; minute
	movf		lo,W
	mullw		.60
	movff		SSPBUF,WREG
	addwf		PRODL,F
	movlw		.0
	addwfc		PRODH,F					; PRODH:PRODL has end-of-dive time in minutes

	incf_eeprom_address	d'39'			; Skip Bytes in EEPROM
	call		I2CREAD2				; Total sample time in seconds
	movff		SSPBUF,lo
	call		I2CREAD2				; Total sample time in seconds
	movff		SSPBUF,hi
	decf_eeprom_address	d'41'			; Macro, that subtracts 8Bit from eeprom_address:2
	call		convert_time			; converts hi:lo in seconds to mins (hi) and seconds (lo)
	clrf		sub_b+1
	movff		hi,sub_b+0
	movff		PRODL,sub_a+0
	movff		PRODH,sub_a+1
	call		subU16					; sub_c = sub_a - sub_b (with UNSIGNED values)
    
    bcf         premenu                 ; Clear temporary flag

    btfss       neg_flag
    bra         display_start_dive_normal       ; Not during midnight...

    ; subtract result from 1440 (amount minutes/day)
    bsf         premenu                ; Set temporary flag
    movlw       LOW     .1440
    movwf       sub_a+0
    movlw       HIGH    .1440
    movwf       sub_a+1
    movff       sub_c+0,sub_b+0
    movff       sub_c+1,sub_b+1
    call        subU16					; sub_c = sub_a - sub_b (with UNSIGNED values)

display_start_dive_normal:
	; sub_c:2 holds entry time in minutes
	movff		sub_c+0,lo
	movff		sub_c+1,hi
	call		convert_time			; converts hi:lo in minutes to hours (hi) and minutes (lo)	
	movff		lo,PRODL				; temp
	movff		hi,lo
	output_99x							; hour
	PUTC		':'
	movff		PRODL,lo			
	output_99x							; minute

    btfss       premenu                 ; premenu is still 1 if dive was during midnight...
	bra			logbook_divetime_common

    ; ...show a ",-1" behind the entry time to indicate that...
    PUTC        ","
    PUTC        "-"
    PUTC        "1"
    bcf         premenu
	bra			logbook_divetime_common		; Skip end-of-divetime
	
display_end_of_divetime:
	PUTC		0x94						; "End of dive" icon
	call		I2CREAD2					; hour
	movff		SSPBUF,lo
	output_99x			
	PUTC		':'
	call		I2CREAD2					; Minute
	movff		SSPBUF,lo
	output_99x			

logbook_divetime_common:
    clrf        WREG
    movff       WREG,letter+.23         ; limit this line to 23 chars max (2C hardware)
	call		word_processor			; Display 1st row of details

	WIN_TOP		.25
	WIN_LEFT	.05
	lfsr		FSR2,letter
	call		I2CREAD2				; read max depth
	movff		SSPBUF,lo				
	call		I2CREAD2				; read max depth
	movff		SSPBUF,hi
	movff		lo,xA+0					; calculate y-scale for profile display
	movff		hi,xA+1
	movlw		d'163'					; 163pixel height available
	movwf		xB+0
	clrf		xB+1
	call		div16x16				; does xA/xB=xC
	movff		xC+0,sim_pressure+0		; holds LOW byte of y-scale   (mbar/pixel!)
	movff		xC+1,sim_pressure+1		; holds HIGH byte of y-scale  (mbar/pixel!)
	incf		sim_pressure+0,F		; increase one, because there may be a remainder
	movlw		d'0'
	addwfc		sim_pressure+1,F
	
	movlw		LOW		d'164000'		; 164pixel*1000 height
	movwf		xC+0
	movlw		HIGH	(d'164000' & h'FFFF') ; 164pixel*1000 height
	movwf		xC+1
	movlw		UPPER	d'164000'		; 164pixel*1000 height
	movwf		xC+2
	clrf		xC+3

	movff		lo,xB+0					; Max. Depth in mbar
	movff		hi,xB+1					; Max. Depth in mbar
	call		div32x16				; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder

	movff		xC+0,last_temperature+0	; 
	movff		xC+1,last_temperature+1	; = Pixels/10m (For scale, draw any xx rows a scale-line)

	movf		last_temperature+0,W
	iorwf		last_temperature+1,W		; last_temperature:2 = Null?
	bnz			display_profile_offset4		; No, continue
	incf		last_temperature+1,F		; Yes, make last_temperature+1>1 to make "display_profile2e" working

display_profile_offset4:
	bsf			leftbind
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc		WREG,0					; Enabled ?
	bra			display_profile_offset5	; NO
	call		convert_meters_to_feet2	; converts meters to feet, don't check CF#61
	bsf			ignore_digit4			; do not display feet fractions
	output_16							; max. depth
	bcf			ignore_digit4
	STRCAT      TXT_FOOT3
	bra			display_profile_offset6

display_profile_offset5:
#ENDIF
	output_16dp	d'3'					; max. depth
	STRCAT      TXT_METER2
#IFDEF IMPERIAL
display_profile_offset6:
#ENDIF
	call		I2CREAD2				; divetime in minutes	
	movff		SSPBUF,lo
	call		I2CREAD2	
	movff		SSPBUF,hi				; divetime in minutes

	movff		lo,xA+0					; calculate x-scale for profile display
	movff		hi,xA+1					; calculate total diveseconds first
	movlw		d'60'					; 60seconds are one minute...
	movwf		xB+0
	clrf		xB+1
	call		mult16x16				; result is in xC:2 !

	bsf			leftbind
	PUTC		0x95					; "duration of dive" icon
	output_16							; divetime minutes

	movlw		LOW		d'600'
	movwf		xA+0
	movlw		HIGH	d'600'
	movwf		xA+1					; A vertical line every 600 seconds
	movff		samplesecs_value,xB+0		; Copy sampling rate
	clrf		xB+1
	call		div16x16				; xA/xB=xC with xA as remainder
	movff		xC+0,average_depth_hold_total+0
	movff		xC+1,average_depth_hold_total+1
	;average_depth_hold_total:2 holds interval of samples for vertical 10min line

; Restore divetime in minutes:
	btfss	logbook_format_0x21			; Dive made with new 0x21 format?
	bra		display_profile_old_xscale	; No
; Yes, get real sample time
	incf_eeprom_address	d'35'			; Skip Bytes in EEPROM
	call		I2CREAD2				; Total sample time in seconds
	movff		SSPBUF,xC+0
	call		I2CREAD2				; Total sample time in seconds
	movff		SSPBUF,xC+1
	decf_eeprom_address	d'37'			; Macro, that subtracts 8Bit from eeprom_address:2
	PUTC		':'
	call		I2CREAD2				; read divetime seconds
	movff		SSPBUF,lo
	bra			display_profile_xscale		; continue below

display_profile_old_xscale:
	movff		lo,xA+0					; calculate x-scale for profile display
	movff		hi,xA+1					; calculate total diveseconds first
	movlw		d'60'					; 60seconds are one minute...
	movwf		xB+0
	clrf		xB+1
	call		mult16x16				; result is in xC:2 !
	PUTC		':'
	call		I2CREAD2				; read divetime seconds
	movff		SSPBUF,lo
	movf		lo,W					; add seconds to total seconds
	addwf		xC+0
	movlw		d'0'
	addwfc		xC+1					; xC:2 now holds total dive seconds!

display_profile_xscale:
	movff		xC+0,xA+0				; now calculate x-scale value
	movff		xC+1,xA+1
	movlw		d'154'					; 154pix width available
	movwf		xB+0
	clrf		xB+1
	call		div16x16				; xA/xB=xC
	movff		xC+0,xA+0	
	movff		xC+1,xA+1
	movf		samplesecs_value,W		; devide through sample interval!
	movwf		xB+0
	clrf		xB+1
	call		div16x16				; xA/xB=xC
	movff		xC+0,profile_temp+0		; store value (use any #xC sample, skip xC-1) into temp registers
	movff		xC+1,profile_temp+1		; store value (use any #xC sample, skip xC-1) into temp registers
	incf		profile_temp+0,F		; increase one, because there may be a remainder
	movlw		d'0'
	addwfc		profile_temp+1,F


	bsf			leftbind
	output_99x							; divetime seconds
	STRCAT_PRINT    " "
	
	WIN_LEFT    .05 + 7*.15
	lfsr        FSR2,letter

	call		I2CREAD2	
	movff		SSPBUF,lo
	call		I2CREAD2	
	movff		SSPBUF,hi				; Read min. Temperature
    movff       lo,logbook_min_tp+0     ; Backup min Tp° too.
	movff       hi,logbook_min_tp+1
	movlw       color_orange            ; Use same color as tp° curve
	call        DISP_set_color

#IFDEF IMPERIAL
	call		convert_celsius_to_fahrenheit
#ENDIF
	call		DISP_convert_signed_temperature	; converts lo:hi into signed-short and adds '-' to POSTINC2 if required
	movlw		d'3'
	movwf		ignore_digits
	bsf			leftbind
	output_16dp	d'2'					; temperature
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0                      ; Enabled ?
	bra			display_profile_temp_c	; NO
	STRCAT_PRINT "°F"
	bra			display_profile_pressure

display_profile_temp_c:
#ENDIF
	STRCAT_PRINT "°C"                   ; Display 2nd row of details
#IFDEF IMPERIAL
display_profile_pressure:
#ENDIF
    call        DISP_standard_color     ; Back to normal
    
	WIN_TOP		.50
	WIN_LEFT	.05
	lfsr		FSR2,letter

	call		I2CREAD2				; read Air pressure
	movff		SSPBUF,lo
	call		I2CREAD2				; read Air pressure
	movff		SSPBUF,hi

	bsf			leftbind
	output_16							; Air pressure before dive
	STRCAT      TXT_MBAR5
	OUTPUTTEXT  .014                    ; Desat
	PUTC        ' '

	call		I2CREAD2				; read Desaturation time
	movff		SSPBUF,lo				
	call		I2CREAD2				; read Desaturation time
	movff		SSPBUF,hi		
	call		convert_time				; converts hi:lo in minutes to hours (hi) and minutes (lo)
	bsf			leftbind
	movf		lo,W
	movff		hi,lo
	movwf		hi							; exchange lo and hi...
	output_8								; Hours
	PUTC		':'
	movff		hi,lo						; Minutes
	output_99x
	bcf			leftbind
	call		word_processor				; display 3rd page of details

	movff		eeprom_address+0,average_depth_hold+0
	movff		eeprom_address+1,average_depth_hold+1			; Pointer to Gaslist (For Page 2)

	incf_eeprom_address	d'12'				; Skip 12 Bytes in EEPROM (faster) (Gaslist)
	call		I2CREAD2					; Read start gas (1-5)
	movff		SSPBUF,average_depth_hold_total+3	; keep copy to restore color
    rcall       profile_display_color       ; Back to normal profile color.
	incf_eeprom_address	d'5'				; Skip 5 Bytes in EEPROM (faster) (Battery, firmware)

	call		I2CREAD2					; Read Tp° divisor
	movf		SSPBUF,W
	andlw       0x0F                        ; Clear extra bits.
	movwf       divisor_temperature	        ; Store divisor
	movwf       count_temperature           ; Store to tp° counter too.

	call		I2CREAD2					; Read divisor
	movf        SSPBUF,W
	andlw       0x0F
	movwf       divisor_deco      			; Store divisor
	movwf		count_deco                  ; Store as temp, too

	call		I2CREAD2					; Read divisor
	movff		SSPBUF,divisor_gf			; Store divisor
	call		I2CREAD2					; Read divisor
	movff		SSPBUF,divisor_ppo2			; Store divisor
	call		I2CREAD2					; Read divisor
	movff		SSPBUF,divisor_deco_debug	; Store divisor
	call		I2CREAD2					; Read divisor
	movff		SSPBUF,divisor_cns			; Store divisor
	incf_eeprom_address	d'2'				; Skip 2Bytes in EEPROM (faster)
	; 2 bytes salinity, GF
	btfss	logbook_format_0x21				; 10byte extra?
	bra		display_profile2d				; No
	incf_eeprom_address	d'10'				; Skip another 10 byte from the header for 0x21 format
	; Average Depth, spare bytes

display_profile2d:
	; Start Profile display
; Write 0m X-Line..
	movlw		color_grey	
	call		DISP_set_color				; Make this configurable?

	movlw		d'75'
	movff		WREG,win_top
	movlw		d'5'
	movff		WREG,win_leftx2				; Left border (0-159)
	movlw		d'1'
	movff		WREG,win_height				
	movlw		d'155'
	movff		WREG,win_width				; Right border (0-159)
display_profile2e:
	call		DISP_box					; Inputs:  win_top, win_leftx2, win_height, win_width, win_color1, win_color2
	movff		win_top,WREG				; Get row
	addwf		last_temperature+0,W		; Add line interval distance to win_top
	tstfsz		last_temperature+1			; >255?
	movlw		d'255'						; Yes, make win_top>239 -> Abort here
	btfsc		STATUS,C					; A Cary from the addwf above?
	movlw		d'255'						; Yes, make win_top>239 -> Abort here
	movff		WREG,win_top				; Result in win_top again
	movff		win_top,lo					; Get win_top in Bank1...
	movlw		d'239'						; Limit
	cpfsgt		lo							; >239?
	bra			display_profile2e			; No, draw another line

; Write 0min Y-Line..
	movlw		color_grey	
	call		DISP_set_color				; Make this configurable?

	movlw		d'75'
	movff		WREG,win_top
	movlw		d'4'
	movff		WREG,win_leftx2				; Left border (0-159)
	movlw		d'164'
	movff		WREG,win_height				
	movlw		d'1'
	movff		WREG,win_width				; "Window" Width
	call		DISP_box					; Inputs:  win_top, win_leftx2, win_height, win_width, win_color1, win_color2

; Draw frame around profile
	movlw		color_blue	
	WIN_FRAME_COLOR	.75, .239, .4, .159	;top, bottom, left, right with color in WREG

	call		I2CREAD2					; skip 0xFB		(Header-end)
	clrf		timeout_counter2			; here: used as counter for depth readings
	call		I2CREAD2					; skip 0xFB		(Header-end)
	movlw		d'158'
	movwf		ignore_digits				; here: used as counter for x-pixels
	bcf			second_FD					; clear flag
	movlw		d'5'
	movwf		timeout_counter3			; here: used as colum x2 (Start at Colum 5)
	movlw		d'75'						; Zero-m row
	movwf		apnoe_mins					; here: used for fill between rows
    movwf       logbook_last_tp             ; Initialise for Tp° curve too.
	incf		timeout_counter3,W			; Init Column

    movlw       LOW(-.100)                  ; Initialize max tp° to -10.0 °C.
    movwf       logbook_max_tp+0
    movlw       HIGH 0xFFFF & (-.100)
    movwf       logbook_max_tp+1
    
    setf        logbook_cur_tp+0            ; Initialize Tp°, before the first recorded point.
    setf        logbook_cur_tp+1
    clrf        logbook_last_tp             ; Also reset previous Y for Tp°
    clrf        logbook_ceiling             ; Ceiling = 0, correct value for no ceiling.

    INIT_PIXEL_WRITE timeout_counter3       ; pixel x2			(Also sets standard Color!)

profile_display_loop:
	movff		profile_temp+0,profile_temp2+0
	movff		profile_temp+1,profile_temp2+1		; 16Bit x-scaler
	incf		profile_temp2+1,F					
	tstfsz		profile_temp2+0						; Must not be Zero
	bra			profile_display_loop2				; Not Zero!
	incf		profile_temp2+0,F					; Zero, Increase!

profile_display_loop2:
	rcall		profile_view_get_depth		; reads depth, temp and profile data

	btfsc		second_FD					; end-of profile reached?
	bra			profile_display_loop_done	; Yes, skip all remaining pixels

    ;---- Draw Ceiling curve, if any ---------------------------------------------
    movf        divisor_deco,W
    bz          profile_display_skip_deco

    movf        logbook_ceiling,W           ; Any deco ceiling ?
    bz          profile_display_skip_deco

	mullw       .100                        ; Yes: convert to mbar
	movff       PRODL,sub_a+0
	movff       PRODH,sub_a+1
	movff       logbook_cur_depth+0,sub_b+0    ; Compare with UNSIGNED current depth (16bits)
	movff       logbook_cur_depth+1,sub_b+1
	call        subU16                      ; set (or not) neg_flag

    movlw       color_dark_green            ; Dark green if Ok,
    btfss       neg_flag
    movlw       color_dark_red              ; Or dark red if ceiling overflown.
    call        DISP_set_color
    
	movff       PRODL,xA+0
	movff       PRODH,xA+1
	movff		sim_pressure+0,xB+0			; devide pressure in mbar/quant for row offsett
	movff		sim_pressure+1,xB+1
	call		div16x16					; xA/xB=xC

	movlw		d'76'                       ; Starts right after the top blue line.
	movff		WREG,win_top
	movff		timeout_counter3,win_leftx2 ; Left border (0-159)
	movff		xC+0,win_height				
	call		half_vertical_line			; Inputs:  win_top, win_leftx2, win_height, win_color1, win_color2

; Horizontal bar: jaggy line, so don't keep it.
;   movlw		d'75'
;   addwf		xC+0,F						; add 75 pixel offset to result
;   PIXEL_WRITE timeout_counter3,xC+0       ; Set col(0..159) x row (0..239), put a current color pixel.

profile_display_skip_deco:

    ;---- Draw Tp° curve, if any ---------------------------------------------
    movf        divisor_temperature,W
    bz          profile_display_skip_temp

	movf        logbook_cur_tp+0,W          ; Did we had already a valid Tp°C record ?
	andwf       logbook_cur_tp+1,W
	incf        WREG
	bz          profile_display_skip_temp   ; No: just skip drawing.

    movlw       LOW((.153*.256)/.370)         ; fixed tp° scale: (-2 .. +35°C * scale256 )/153pix
 	movwf		xB+0
    movlw       HIGH((.153*.256)/.370)
 	movwf		xB+1

	movf        logbook_cur_tp+0,W          ; Current Tp° - (-2.0°C) == Tp° + 20.
	addlw       LOW(.20)                    ; Low byte.
	movwf       xA+0
    movf		logbook_cur_tp+1,W
    btfsc       STATUS,C                    ; Propagate carry, if any
    incf        WREG
    movwf       xA+1
    call		mult16x16					; xA*xB=xC

    ; scale: divide by 256, ie. take just high byte.
    movf        xC+1,W
    sublw       .75+.153      				; Upside-down: Y = .75 + (.153 - result)
    movwf       xC+0

	; Check limits
	movlw		d'75'
	movwf		xC+1
	cpfsgt		xC+0
	movff		xC+1,xC+0

    movlw       color_orange
    call        DISP_set_color

    movf        logbook_last_tp,W           ; do we have a valid previous value ?
    bz          profile_display_temp_1      ; No: skip the vertical line.
    movwf       xC+1
	call		profile_display_fill		; In this column between this row (xC+0) and the last row (xC+1)
profile_display_temp_1:	
    movff       xC+0,logbook_last_tp

    PIXEL_WRITE timeout_counter3,xC+0       ; Set col(0..159) x row (0..239), put a current color pixel.

profile_display_skip_temp:

    ;---- Draw depth curve ---------------------------------------------------
	movff		sim_pressure+0,xB+0			; devide pressure in mbar/quant for row offsett
	movff		sim_pressure+1,xB+1
	movff		logbook_cur_depth+0,xA+0
	movff		logbook_cur_depth+1,xA+1
	call		div16x16					; xA/xB=xC
	movlw		d'75'
	addwf		xC+0,F						; add 75 pixel offset to result
	
	btfsc		STATUS,C                    ; Ignore potential profile errors
	movff		apnoe_mins,xC+0

    rcall       profile_display_color       ; Back to normal profile color.

    movff       apnoe_mins,xC+1
	call		profile_display_fill		; In this column between this row (xC+0) and the last row (xC+1)
	movff		xC+0,apnoe_mins				; Store last row for fill routine
    PIXEL_WRITE timeout_counter3,xC+0       ; Set col(0..159) x row (0..239), put a std color pixel.

	incf		timeout_counter3,F          ; Next column
    ;---- Draw Marker square , if any ----------------------------------------
    btfss       log_marker_found            ; Any marker to draw?
    bra         profile_display_skip_marker ; No
 
    ; 2x2 square
    incf        apnoe_mins,W
    movff       WREG,win_top
    movlw       .4
    movff       WREG,win_height
    movlw       .2
    movff       WREG,win_width
    decf        timeout_counter3,W
    movff       WREG,win_leftx2

    movlw       color_orange
    call        DISP_set_color
    call        DISP_box                    ; Draw 2x2 Box
    bcf         log_marker_found            ; Clear flag

profile_display_skip_marker:

    ;---- Draw CNS curve, if any ---------------------------------------------
    movf        divisor_cns,W
    bz          profile_display_skip_cns
    ;
    ; TODO HERE 
    ;
profile_display_skip_cns:

    ;---- Draw GF curve, if any ----------------------------------------------
    movf        divisor_gf,W
    bz          profile_display_skip_gf
    ;
    ; TODO HERE 
    ;
profile_display_skip_gf:

    ;---- All curves done.
    
profile_display_skip_loop1:					; skips readings!
	dcfsnz		profile_temp2+0,F
	bra			profile_display_loop3		; check 16bit....

	rcall		profile_view_get_depth		; reads depth, temp and profile data

	btfsc		second_FD					; end-of profile reached?
	bra			profile_display_loop_done	; Yes, skip all remaining pixels

	bra			profile_display_skip_loop1

profile_display_loop3:
	decfsz		profile_temp2+1,F			; 16 bit x-scaler test
	bra			profile_display_skip_loop1	; skips readings!

	decfsz		ignore_digits,F				; counts drawn x-pixels to zero
	bra			profile_display_loop		; Not ready yet
; Done.
profile_display_loop_done:
    btfss   is_bailout                       ; Bailout during the dive?
    bra     profile_display_loop_done_nobail ; No
    ; Yes, show "Bailout"
   	movlw   color_pink
	call    DISP_set_color
	WIN_TOP		.210
	WIN_LEFT	.105
	WIN_FONT 	FT_SMALL
	lfsr	FSR2,letter
    OUTPUTTEXT d'137'                      ; Bailout
    call	word_processor
profile_display_loop_done_nobail:
	call		DISP_standard_color			; Restore color
	call		menu_pre_loop_common		; Clear some menu flags, timeout and switches

display_profile_loop:
	call		check_switches_logbook
	btfsc		menubit2					; SET/MENU?
	bra			exit_profileview			; back to list
	btfsc		menubit3					; ENTER?
	bra			profileview_page2			; Switch to Page2 of profile view

	btfsc		onesecupdate
	call		menu_check_dive_and_timeout	; "Goto restart" or sets sleepmode flag
	bcf			onesecupdate				; one second update

	btfsc		sleepmode					; Timeout?
	bra			exit_profileview			; back to list
	bra			display_profile_loop		; wait for something to do

;=============================================================================
profile_display_color:
    movff       average_depth_hold_total+3,active_gas ; Restore gas color.
	dcfsnz		active_gas,F
	movlw		color_white					; Color for Gas 1
	dcfsnz		active_gas,F
	movlw		color_green					; Color for Gas 2
	dcfsnz		active_gas,F
	movlw		color_red					; Color for Gas 3
	dcfsnz		active_gas,F
	movlw		color_yellow				; Color for Gas 4
	dcfsnz		active_gas,F
	movlw		color_violet				; Color for Gas 5
	dcfsnz		active_gas,F
	movlw		color_pink                  ; Color for Gas 6
	goto		DISP_set_color				; Set Color...

;=============================================================================
profileview_page2:
    WIN_BOX_BLACK   .0, .74, .0, .159		;top, bottom, left, right

	movff		average_depth_hold+0,eeprom_address+0
	movff		average_depth_hold+1,eeprom_address+1			; Pointer to Gaslist

	movlw		color_white					; Color for Gas 1
	call		DISP_set_color				; Set Color...
	bsf			leftbind
	WIN_TOP		.0
	WIN_LEFT	.0
	STRCPY      TXT_G1_3
	call		I2CREAD2					; Gas1 current O2
	movff		SSPBUF,lo
	output_99x
	PUTC		'/'
	call		I2CREAD2					; Gas1 current HE
	movff		SSPBUF,lo
	output_8
	call		word_processor				; Display Gas information

	movlw		color_green					; Color for Gas 2
	call		DISP_set_color				; Set Color...
	WIN_TOP		.25
	STRCPY      TXT_G2_3
	call		I2CREAD2					; Gas2 current O2
	movff		SSPBUF,lo
	output_8
	PUTC		'/'
	call		I2CREAD2					; Gas2 current HE
	movff		SSPBUF,lo
	output_8
	call		word_processor				; Display Gas information

	movlw		color_red					; Color for Gas 3
	call		DISP_set_color				; Set Color...
	WIN_TOP		.50
	STRCPY      TXT_G3_3
	call		I2CREAD2					; Gas3 current O2
	movff		SSPBUF,lo
	output_8
	PUTC		'/'
	call		I2CREAD2					; Gas3 current HE
	movff		SSPBUF,lo
	output_8
	call		word_processor				; Display Gas information

	movlw		color_yellow				; Color for Gas 4
	call		DISP_set_color				; Set Color...
	WIN_TOP		.0
	WIN_LEFT	.60
	STRCPY      TXT_G4_3
	call		I2CREAD2					; Gas4 current O2
	movff		SSPBUF,lo
	output_8
	PUTC		'/'
	call		I2CREAD2					; Gas4 current HE
	movff		SSPBUF,lo
	output_8
	call		word_processor				; Display Gas information

	movlw		color_violet				; Color for Gas 5
	call		DISP_set_color				; Set Color...
	WIN_TOP		.25
	STRCPY      TXT_G5_3
	call		I2CREAD2					; Gas5 current O2
	movff		SSPBUF,lo
	output_8
	PUTC		'/'
	call		I2CREAD2					; Gas5 current HE
	movff		SSPBUF,lo
	output_8
	call		word_processor				; Display Gas information

	movlw		color_cyan					; Color for Gas 6
	call		DISP_set_color				; Set Color...
	WIN_TOP		.50
	STRCPY      TXT_G6_3
	call		I2CREAD2					; Gas6 current O2
	movff		SSPBUF,lo
	output_8
	PUTC		'/'
	call		I2CREAD2					; Gas6 current HE
	movff		SSPBUF,lo
	output_8
	call		word_processor				; Display Gas information

	call		DISP_standard_color
	WIN_TOP		.0
	WIN_LEFT	.120
	STRCPY      TXT_1ST4
	call		I2CREAD2					; Start Gas
	movff		SSPBUF,lo
	output_8
	call		word_processor				; Display Gas information


	bcf			show_cns_in_logbook			; clear flag
	WIN_TOP		.25
	STRCPY      "V"
	call		I2CREAD2					; Firmware x
	movff		SSPBUF,lo
	movff		SSPBUF,hi
	output_8
	PUTC		'.'
	call		I2CREAD2					; Firmware y
	movff		SSPBUF,lo
	movlw		.83							; Check firmware y > 83
	cpfslt		lo							; <83?
	bsf			show_cns_in_logbook			; No, set flag
	movlw		.2							; Check firmware x > 1
	cpfslt		hi							; <2?
	bsf			show_cns_in_logbook			; No, set flag
	output_99x
	call		word_processor				; Display Gas information
	bcf			leftbind					; Clear flag

	WIN_TOP		.50
	lfsr		FSR2,letter	
	call		I2CREAD2					; Battery lo
	movff		SSPBUF,lo
	call		I2CREAD2					; Battery hi
	movff		SSPBUF,hi
	movlw	d'1'
	movwf	ignore_digits
	bsf		ignore_digit5		; do not display mV
	bsf		leftbind
	output_16dp	d'2'			; e.g. 3.45V
	bcf		leftbind
	STRCAT_PRINT  TXT_VOLT1


;	call		I2CREAD2					; Skip Sampling rate

	call		menu_pre_loop_common		; Clear some menu flags, timeout and switches
display_profile2_loop:
	call		check_switches_logbook
	btfsc		menubit2					; SET/MENU?
	bra			exit_profileview			; back to list
	btfsc		menubit3					; ENTER?
	bra			profileview_page3			; Switch to Page3 of profile view
	btfsc		onesecupdate
	call		menu_check_dive_and_timeout	; "Goto restart" or sets sleepmode flag
	bcf			onesecupdate				; one second update
	btfsc		sleepmode					; Timeout?
	bra			exit_profileview			; back to list
	bra			display_profile2_loop		; wait for something to do

profileview_page3:
    WIN_BOX_BLACK   .0, .74, .0, .159		;top, bottom, left, right

	call		DISP_standard_color

	movff		average_depth_hold+0,eeprom_address+0
	movff		average_depth_hold+1,eeprom_address+1			; Pointer to Gaslist

	incf_eeprom_address	d'24'				; Point to "Salinity"
	bsf			leftbind
	WIN_TOP		.0
	WIN_LEFT	.0
	call		I2CREAD2					; read Salinity
	lfsr	FSR2,letter
	movff	SSPBUF,lo
	clrf	hi
	output_16dp	d'3'
	STRCAT_PRINT TXT_KGL4

	call		I2CREAD2					; Read CNS%

	btfss	show_cns_in_logbook				; Show CNS?
	bra		logbook_skip_cns				; No

	movff		SSPBUF,lo
	WIN_TOP		.25
	STRCPY      TXT_CNS4
	output_8
	STRCAT_PRINT "%"						; Display CNS %

logbook_skip_cns:
	btfss	logbook_format_0x21
	bra		skip_new_format_0x21_info		; Do not show remaining info from dive

; Show all new 0x21 data
; Show average depth
	WIN_TOP		.50
	call		I2CREAD2					; Read average depth 
	movff		SSPBUF,lo
	call		I2CREAD2					; Read average depth 
	movff		SSPBUF,hi
	STRCPY      TXT_AVR4
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc		WREG,0                      ; Enabled ?
	bra			logbook_show_average_depth  ; NO
	call		convert_meters_to_feet2		; converts meters to feet, don't check CF#61
	bsf			ignore_digit4				; do not display feet fractions
	output_16
	bcf			ignore_digit4
	STRCAT_PRINT TXT_FOOT3
	bra			logbook_show_average_depth1

logbook_show_average_depth:
#ENDIF
	output_16dp	d'3'			; Average depth 
	STRCAT_PRINT TXT_METER1

#IFDEF IMPERIAL
logbook_show_average_depth1:
#ENDIF
	incf_eeprom_address	d'4'				; Skip total dive time and GF factors
	call		I2CREAD						; Read deco modell
	decf_eeprom_address	d'2'				; back to GF factos

	WIN_TOP		.0
	WIN_LEFT	.75

	movff		SSPBUF,lo
	movlw		d'3'
	cpfsgt	lo						; GF model used for this dive?
	bra		logbook_show_sat				; No

	
; Show GF settings
	call		I2CREAD2					; Read GF_lo
	movff		SSPBUF,lo
	call		I2CREAD2					; Read GF_hi
	movff		SSPBUF,hi
	STRCPY      TXT_GF3
	output_8								; GF_lo
	PUTC		'/'
	movff		hi,lo						; copy GF_hi
	output_8								; GF_hi
	call		word_processor
	bra			logbook_deco_model			; Skip Sat

logbook_show_sat:
	call		I2CREAD2					; Read Saturation x 
	movff		SSPBUF,hi
	call		I2CREAD2					; Read Desaturation x
	movff		SSPBUF,lo
	STRCPY      TXT_SAT4
	output_8								; Sat x
	STRCAT      "%/"
	movff		hi,lo						; copy Desat x
	output_8								; Desat x
	STRCAT_PRINT "%"

logbook_deco_model:
; Show deco model
	WIN_TOP		.25
	call		I2CREAD2					; Read deco modell
	movff		SSPBUF,lo
	lfsr		FSR2,letter
	incf		lo,F						; +1
	dcfsnz		lo,F						; ZH-L16 OC?
	movlw		d'101'						; Textnumber
	dcfsnz		lo,F						; Gauge?
	movlw		d'102'						; Textnumber
	dcfsnz		lo,F						; ZH-L16 CC?
	movlw		d'104'						; Textnumber
	dcfsnz		lo,F						; Apnoe?
	movlw		d'138'						; Textnumber
	dcfsnz		lo,F						; L16-GF OC?
	movlw		d'152'						; Textnumber
	dcfsnz		lo,F						; L16-GF CC?
	movlw		d'236'						; Textnumber
	dcfsnz		lo,F						; pSCR-GF?
	movlw		d'226'						; Textnumber
	call		displaytext0_low			; Outputs to POSTINC2
	call		word_processor

skip_new_format_0x21_info:
	bcf			menubit2
	bcf			menubit3
	bcf			switch_right
	bcf			switch_left
	clrf		timeout_counter2
display_profile3_loop:
	call		check_switches_logbook
	btfsc		menubit2					; SET/MENU?
	bra			exit_profileview			; back to list
	btfsc		menubit3					; ENTER?
	bra			exit_profileview			; back to list
	btfsc		onesecupdate
	call		menu_check_dive_and_timeout	; "Goto restart" or sets sleepmode flag
	bcf			onesecupdate				; one second update
	btfsc		sleepmode					; Timeout?
	bra			exit_profileview			; back to list
	bra			display_profile3_loop		; wait for something to do
	
;=============================================================================
; Draw a vertical line between xC+1 and xC+0, at current X position.
;
; Note: should keep xC+0
; Note: ascending or descending !
;
profile_display_fill:
    ; First, check if xC+0>apnoe_mins or xC+0<aponoe_mins
	movf	xC+0,W
	cpfseq	xC+1				    ; xC+0 = apone_mins?
	bra		profile_display_fill2	; No!
	return

profile_display_fill2:	
    ; Make sure to init X position.
    movf    timeout_counter3,W
    mullw   2
    decf    PRODL,F
    movlw   0
    subwfb  PRODH,F
    call    pixel_write_col320

	movf	xC+0,W
	cpfsgt	xC+1				    ; apnoe_mins>xC+0?
	bra		profile_display_fill_up	; Yes!

profile_display_fill_down2:			; Loop	
	decf		xC+1,F

    HALF_PIXEL_WRITE    xC+1        ; Updates just row (0..239)

	movf		xC+0,W
	cpfseq		xC+1				; Loop until xC+1=xC+0
	bra			profile_display_fill_down2
	return							; apnoe_mins and xC+0 are untouched

profile_display_fill_up:			; Fill upwards from xC+0 to apone_mins!
	incf		xC+1,F

    HALF_PIXEL_WRITE    xC+1        ; Updates just row (0..239)

	movf		xC+0,W
	cpfseq		xC+1				; Loop until xC+1=apnoe_mins
	bra			profile_display_fill_up
	return							; apnoe_mins and xC+0 are untouched

;=============================================================================

profile_view_get_depth:
	incf		average_divesecs+0,F
	movlw		d'0'
	addwfc		average_divesecs+1,F		; Count read pixels

	movf		average_divesecs+0,W
	cpfseq		average_depth_hold_total+0
	bra			profile_view_get_depth_no_line		; no need to draw a 10min line, continue
	movf		average_divesecs+1,W
	cpfseq		average_depth_hold_total+1
	bra			profile_view_get_depth_no_line		; no need to draw a 10min line, continue
; draw a new 10min line here...
	clrf		average_divesecs+0
	clrf		average_divesecs+1					; clear counting registers for next line

	movlw		color_grey	
	call		DISP_set_color						; Make this configurable?
	movlw		d'76'
	movff		WREG,win_top
	incf		timeout_counter3,W	; draw one line to right to make sure it's the background of the profile
	movff		WREG,win_leftx2		; Left border (0-159)
	movlw		d'163'
	movff		WREG,win_height				
	movlw		d'1'
	movff		WREG,win_width				; "Window" Width
	call		DISP_box					; Inputs:  win_top, win_leftx2, win_height, win_width, win_color1, win_color2

profile_view_get_depth_no_line:
	call		I2CREAD2					; read first depth
	movff		SSPBUF,logbook_cur_depth+0  ; low value
	call		I2CREAD2					; read first depth
	movff		SSPBUF,logbook_cur_depth+1  ; high value
	call		I2CREAD2					; read Profile Flag Byte
	movff		SSPBUF,timeout_counter2		; store Profile Flag Byte

	bcf			event_occured				; clear flag
	btfsc		timeout_counter2,7
	bsf			event_occured				; We also have an Event byte!
	bcf			timeout_counter2,7			; Clear Event Byte Flag (If any)
	; timeout_counter2 now holds the number of additional bytes to ignore (0-127)
	movlw		0xFD						; end of profile bytes?
	cpfseq		logbook_cur_depth+0
	bra			profile_view_get_depth_new1	; no 0xFD
	movlw		0xFD						; end of profile bytes?
	cpfseq		logbook_cur_depth+1
	bra			profile_view_get_depth_new1	; no 0xFD
	bsf			second_FD					; End found! Set Flag! Skip remaining pixels!
	return

profile_view_get_depth_new1:
	btfsc		event_occured				; Was there an event attached to this sample?
	rcall		profile_view_get_depth_events	; Yes, get information about this event(s)
    
    ;---- Read Tp°, if any AND divisor reached AND bytes available -----------
    movf        divisor_temperature,W       ; Is Tp° divisor null ?
    bz          profile_view_get_depth_no_tp; Yes: no Tp° curve.
    decf        count_temperature,F         ; Decrement tp° counter
    bnz         profile_view_get_depth_no_tp; No temperature this time
    
    call		I2CREAD2					; Tp° low
	movff		SSPBUF,logbook_cur_tp+0
    call		I2CREAD2					; Tp° high
	movff		SSPBUF,logbook_cur_tp+1
	decf        timeout_counter2,F
	decf        timeout_counter2,F
	movff       divisor_temperature,count_temperature   ; Restart counter.
    
    ; Compute Tp° max on the fly...
    movff       logbook_cur_tp+0,sub_a+0    ; Compare cur_tp > max_tp ?
    movff       logbook_cur_tp+1,sub_a+1
    movff       logbook_max_tp+0,sub_b+0
    movff       logbook_max_tp+1,sub_b+1
    call        sub16                       ; SIGNED sub_a - sub_b
    btfsc       neg_flag
    bra         profile_view_get_depth_no_tp
    
    movff       logbook_cur_tp+0,logbook_max_tp+0
    movff       logbook_cur_tp+1,logbook_max_tp+1
    
    ;---- Read deco, if any AND divisor=0 AND bytes available ----------------
profile_view_get_depth_no_tp:
    movf        divisor_deco,W
    bz          profile_view_get_depth_no_deco
    decf        count_deco,F
    bnz         profile_view_get_depth_no_deco
    
    call		I2CREAD2
	movff		SSPBUF,logbook_ceiling
	decf        timeout_counter2,F
	movff       divisor_deco,count_deco     ; Restart counter.
    call		I2CREAD2                    ; Skip stop length
	decf        timeout_counter2,F

    ;---- Read GF, if any AND divisor=0 AND bytes available ------------------
profile_view_get_depth_no_deco:
    
    movf        timeout_counter2,W          ; No more extra bytes ?
    btfsc       STATUS,Z
    return                                  ; No: done.
    
    ; Then skip remaining bytes...
	movf		timeout_counter2,W			; number of additional bytes to ignore (0-127)
	call		incf_eeprom_address0		; increases bytes in eeprom_address:2 with 0x8000 bank switching
	return

profile_view_get_depth_events:
	call		I2CREAD2					; Read Event byte
	movff		SSPBUF,EventByte			; store EventByte
	decf		timeout_counter2,F			; reduce counter

; Check Event flags in the EventByte
	btfsc		EventByte,7                 ; Bailout?
	rcall		logbook_event2				; Yes!
 	btfsc		EventByte,4					; Manual Gas Changed?
 	rcall		logbook_event1				; Yes!
	btfsc		EventByte,6                 ; Setpoint Change?
	rcall		logbook_event3				; Yes!
 	btfsc		EventByte,5					; Stored Gas Changed?
    rcall   	logbook_event4				; Yes!
 	return									; No, return

logbook_event4: ; Stored Gas changed!
	call		I2CREAD2					; Read Gas#
	movff		SSPBUF,average_depth_hold_total+3
    rcall       profile_display_color       ; Back to normal profile color.
	decf		timeout_counter2,F			; reduce counter
	return

logbook_event1:
    movlw       6                           ; Just color backup to 6
    movwf       average_depth_hold_total+3
    rcall       profile_display_color       ; Back to normal profile color.
	call		I2CREAD2					; Read O2
    decf		timeout_counter2,F			; reduce counter
	call		I2CREAD2					; Read He
    decf		timeout_counter2,F			; reduce counter
    ; Any Alarm?
    bcf         EventByte,4                 ; Clear bits already tested
    bcf         EventByte,5
    bcf         EventByte,6
    movlw       .6                          ; manual marker?
    cpfseq      EventByte
    return	   ; No, return
    bsf         log_marker_found            ; Manual marker! Draw small orange rectancle here
	return

logbook_event2: ; Bailout
    bsf         is_bailout                  ; Set flag
    movff       average_depth_hold_total+3,total_divetime_seconds+0 ; Backup last gas color in case we return to CCR
    movlw       6                           ; Use Gas6 color
    movwf       average_depth_hold_total+3
    rcall       profile_display_color       ; Back to normal profile color.
	call		I2CREAD2					; Read O2
    decf		timeout_counter2,F			; reduce counter
	call		I2CREAD2					; Read He
    decf		timeout_counter2,F			; reduce counter
	return

logbook_event3: ; Setpoint change
    btfss       is_bailout                  ; Are we in bailout?
    return      ; No, return
    ; We were in bailout before, restore profile color
    movff       total_divetime_seconds+0,average_depth_hold_total+3 ; Restore color
    rcall       profile_display_color       ; Back to normal profile color.
	call		I2CREAD2					; Read Setpoint
    decf		timeout_counter2,F			; reduce counter
    return

exit_profileview:
	bcf			sleepmode
	clrf		timeout_counter2				; restore all registers to build same page again
	movff		decodata+0,eeprom_address+0
	movff		decodata+1,eeprom_address+1		
	movff		max_pressure+0,divemins+0
	movff		max_pressure+1,divemins+1
	movff		mintemp+0, divenumber
	decf		divenumber,F
	bcf			all_dives_shown
	clrf		menupos3					; here: used row on current page
	movlw		d'5'
	movwf		menupos						; here: active row on current page
	call		DISP_ClearScreen			; clear details/profile
	goto		menu_logbook1b					; start search

next_logbook2:
	btfsc		all_dives_shown				; all shown
	goto		menu_logbook1				; all reset

	clrf		menupos3	
	movlw		d'5'
	movwf		menupos					; 
	incf		menupos2,F					; start new screen
	call		DISP_ClearScreen
	
next_logbook:
	movff		eeprom_header_address+0,eeprom_address+0
	movff		eeprom_header_address+1,eeprom_address+1	; continue search here
	goto		menu_logbook1b

check_switches_logbook:
	btfsc		switch_right			
	bsf			menubit3
	btfsc		switch_left
	bsf			menubit2					; Enter

	btfsc	    uart_dump_screen            ; Dumps screen contains ?
	call	    dump_screen     			; Yes!

	return

next_logbook3:
	incf		menupos,F
	movlw		d'7'
	cpfseq		menupos					; =7?
	bra			next_logbook3a				; No
	bra			next_logbook2				; yes, new page please

next_logbook3a:
	incf		menupos3,W					; 
	cpfseq		menupos
	bra			next_logbook3b
	movlw		d'6'
	movwf		menupos					; Jump directly to exit if page is not full

next_logbook3b:
	clrf		timeout_counter2
	call		DISP_logbook_cursor

	bcf			switch_right
	bcf			menubit3					; clear flag
	goto		menu_logbook_loop

display_listdive:
	bsf			logbook_page_not_empty		; Page not empty
	incf		menupos3,F					

	btfsc		logbook_header_drawn		; "Logbook already displayed?
	bra			display_listdive1a
	call        DISP_topline_box_clear      ; Clears Bar at the top
    call        DISP_divemask_color
	DISPLAYTEXT	.26							; "Logbook"
    call        DISP_standard_color
	bsf			logbook_header_drawn
	
display_listdive1a:	
	WIN_LEFT	.20
	
	movf		menupos2,W
	mullw		d'5'
	movf		PRODL,W
	subwf		divenumber,W				; current row on page

	mullw		d'30'						; x30
	movf		PRODL,W						; is pixel-row for entry
	addlw		d'5'						; +5 Pixel, so list entries are at rows 35,65,95,125,155,185
	movff		WREG,win_top

	lfsr		FSR2,letter
	movff		divenumber,lo
	output_99x								; # of dive
	PUTC		' '
	call		I2CREAD3					; logbook_profile_version (1st. byte of Header after the 0xFA, 0xFA)  (Block read start)
	movff		SSPBUF,lo
	movlw		d'13'
	cpfsgt		lo							; Skip if lo>13
	bra			display_listdive2			; use old (Pre 0x20) format

	bsf			logbook_format_0x21		; Set flag for new 0x21 Format
	movlw		0x21
	cpfseq		lo						; Skip if 0x21
	bcf			logbook_format_0x21		; Clear flag for new 0x21 Format

	call		I2CREAD4					; Skip Profile version (Block read)
	movff		SSPBUF,lo					; in new format, read month

display_listdive2:
	movff		lo,convert_value_temp+0		; Month (in lo, see above)
	call		I2CREAD4					; Day (Block read)
	movff		SSPBUF,convert_value_temp+1
	call		I2CREAD4					; Year (Block read)
	movff		SSPBUF,convert_value_temp+2
	call		DISP_convert_date_short		; converts into "DD/MM" or "MM/DD" or "MM/DD" in s


	incf_eeprom_address	d'2'				; Skip Bytes in EEPROM (faster)
;	call		I2CREAD2					; hours (Skip)
;	call		I2CREAD2					; minutes (skip)

	PUTC		' '
	call		I2CREAD3					; Depth (Block read start)
	movff		SSPBUF,lo
	call		I2CREAD4					; Block read
	movff		SSPBUF,hi

	bsf			leftbind
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc		WREG,0						; Enabled ?
	bra			display_listdive3			; NO
	call		convert_meters_to_feet2		; converts meters to feet, don't check CF#61
	bsf			ignore_digit4				; do not display feet fractions
	output_16
	bcf			ignore_digit4
	STRCAT		TXT_FOOT3
	bra			display_listdive4

display_listdive3:
#ENDIF
	bsf			ignore_digit5				; Do not display 1cm figure
	output_16dp	d'3'						; max. depth
	STRCAT      TXT_METER2
#IFDEF IMPERIAL
display_listdive4:
#ENDIF
	call		I2CREAD4					; Block read
	movff		SSPBUF,lo					; read divetime in minutes
	call		I2CREAD4					; Block read
	movff		SSPBUF,hi					; read divetime in minutes
	bsf			leftbind
	output_16								; Divetime minutes
	STRCAT_PRINT "'"                    	; Display header-row in list
	incf_eeprom_address	d'37'				; 12 Bytes read from header, skip 37 Bytes in EEPROM (Remaining Header)
	btfss	logbook_format_0x21				; 10byte extra?
	return									; No, Done.
	incf_eeprom_address	d'10'				; Skip another 10 byte from the header for 0x21 format
	return

logbook_convert_64k:						; Converts <1.91 logbook (32kB) to 64kB variant
	call	DISPLAY_boot
	movlw	color_red
    call	DISP_set_color			; Set to Red
	DISPLAYTEXTH	d'303'			; Please wait!
	clrf	EEADR
	movlw	d'1'
	movwf	EEADRH
	movlw	0xAA
	movwf	EEDATA		
	call	write_eeprom			; write 0xAA to indicate the logbook is already converted
	clrf	EEADRH					; Restore EEADRH
; convert logbook:
; Step 1: Copy 32k from 0xFE + 1 with bank switching to bank1
; Step 2: Copy 32k from bank1 to bank0
; Step 3: delete bank1
	call	get_free_EEPROM_location    ; Searches 0xFD, 0xFD, 0xFE and sets Pointer to 0xFE
	rcall	incf_eeprom_bank0	        ; eeprom_address:2 now at 0xFE+1
; Do Step 1:
	;logbook1_ptr+0 and logbook1_ptr+1 hold address in bank1
	;logbook0_ptr+0 and logbook0_ptr+1 hold address in bank0
	movlw	HIGH	0x8000
	movwf	logbook1_ptr+1 
	movlw	LOW		0x8000
	movwf	logbook1_ptr+0			    ; load address for bank1
	movff	eeprom_address+0,logbook0_ptr+0 
	movff	eeprom_address+1,logbook0_ptr+1	; load address for bank0
	movlw	0x80
	movwf	uart2_temp
logbook_convert2:
	clrf	uart1_temp				    ; counter for copy operation
logbook_convert3:
	; read source
	movff	logbook0_ptr+0,eeprom_address+0
	movff	logbook0_ptr+1,eeprom_address+1
	call	I2CREAD
	movff	SSPBUF,lo				; hold read value
	rcall	incf_eeprom_bank0		; eeprom_address:2 +1 with bank switching
	movff	eeprom_address+0,logbook0_ptr+0
	movff	eeprom_address+1,logbook0_ptr+1	; write source address
	; write target
	movff	logbook1_ptr+0,eeprom_address+0
	movff	logbook1_ptr+1,eeprom_address+1
	movf	lo,W
	call	I2CWRITE				    ; writes WREG into EEPROM@eeprom_address
	movlw	d'1'
	addwf	logbook1_ptr+0,F
	movlw	d'0'
	addwfc	logbook1_ptr+1,F            ; increase target address
	decfsz	uart1_temp,F	
	bra		logbook_convert3
	btg		LED_blue
	decfsz	uart2_temp,F			    ; 32kByte done?
	bra		logbook_convert2		    ; No, continue
; Step 1 done.
	bcf		LED_blue
; Do Step 2:
	movlw	HIGH	0x0000
	movwf	logbook1_ptr+1 
	movlw	LOW		0x0000
	movwf	logbook1_ptr+0              ; load address for bank0
	movlw	HIGH	0x8000
	movwf	logbook0_ptr+1 
	movlw	LOW		0x8000
	movwf	logbook0_ptr+0		        ; load address for bank1
	movlw	0x80
	movwf	uart2_temp
logbook_convert4:
	clrf	uart1_temp				    ; counter for copy operation
logbook_convert5:
	; read source
	movff	logbook0_ptr+0,eeprom_address+0
	movff	logbook0_ptr+1,eeprom_address+1
	call	I2CREAD
	movff	SSPBUF,lo				; hold read value
	incf_eeprom_address	d'1'	
	movff	eeprom_address+0,logbook0_ptr+0
	movff	eeprom_address+1,logbook0_ptr+1    ; write source address
	; write target
	movff	logbook1_ptr+0,eeprom_address+0
	movff	logbook1_ptr+1,eeprom_address+1
	movf	lo,W
	call	I2CWRITE				    ; writes WREG into EEPROM@eeprom_address
	incf_eeprom_address	d'1'
	movff	eeprom_address+0,logbook1_ptr+0
	movff	eeprom_address+1,logbook1_ptr+1; write target address
	decfsz	uart1_temp,F	
	bra		logbook_convert5
	btg		LED_red
	decfsz	uart2_temp,F			    ; 32kByte done?
	bra		logbook_convert4		    ; No, continue
; Step 2 done.
	bcf		LED_red
; Do Step 3:
	movlw	HIGH	0x8000
	movwf	logbook0_ptr+1 
	movlw	LOW		0x8000
	movwf	logbook0_ptr+0              ; load address for bank1
	movlw	0x80
	movwf	uart2_temp
logbook_convert6:
	clrf	uart1_temp				    ; counter for copy operation
logbook_convert7:
	; write target
	movff	logbook0_ptr+0,eeprom_address+0
	movff	logbook0_ptr+1,eeprom_address+1
	movlw	0xFF
	call	I2CWRITE				; writes WREG into EEPROM@eeprom_address
	incf_eeprom_address	d'1'
	movff	eeprom_address+0,logbook0_ptr+0
	movff	eeprom_address+1,logbook0_ptr+1	; write target address
	decfsz	uart1_temp,F	
	bra		logbook_convert7
	btg		LED_red
	btg		LED_blue
	decfsz	uart2_temp,F			; 32kByte done?
	bra		logbook_convert6		; No, continue
; Step 3 done.
	bcf		LED_red
	bcf		LED_blue
	return

incf_eeprom_bank0:
	movlw		d'1'					; increase address
	addwf		eeprom_address+0,F
	movlw		d'0'
	addwfc		eeprom_address+1,F
	btfss		eeprom_address+1,7		; at address 8000?
	return								; no, skip
	clrf		eeprom_address+0		; Clear eeprom address
	clrf		eeprom_address+1
	return