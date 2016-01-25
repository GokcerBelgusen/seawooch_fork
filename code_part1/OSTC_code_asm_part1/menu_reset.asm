
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


; Menu "Reset all"
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 10/30/05
; last updated: 10/12/08 by JD Gascuel at free.fr
; known bugs:
; ToDo: 

; routines to reset external EEPROM (currently inactvated!)
; routines to reset custom function, gases and decompression values
; does not reset clock

;=============================================================================
; CF default values
;

; Macro to check values, and construct PROM CF default table.
; If in types mode, set flags into hi. If not, clear it.
CF_DEFAULT	macro	type, default, min, max
    noexpand
CFn set 1+CFn
    if (type) == CF_INT15
    	if HIGH (default) > .127
    		error CF#v(CFn) "15bit default too big: ", default
    	endif
    	if (min)>0 && (max>min)
    	    error CF#v(CFn) "15bit defaults cannot have both MIN & MAX flags"
    	endif
    	if HIGH(min) > .127
    	    error CF#v(CFn) "15bit MIN value too big: ", min
    	endif
    	if HIGH(max) > .127
    	    error CF#v(CFn) "15bit MAX value too big: ", max
    	endif
    
        ifdef NO_CF_TYPES
    		DB  LOW (default), HIGH(default) + 0x80
    	else
    		DB  LOW (default), HIGH(default) + 0x80
    		if (max) > (min)
        		DB  LOW(max), HIGH(max) + 0x80
        	else
        		DB  LOW(min), HIGH(min)
        	endif
    	endif
    else
        ; Basic sanity check for 8bit values:
    	if HIGH(default) > 0
    		error CF#v(CFn) "8bit default too big: ", default
    	endif
        if type & CF_NEG
        	if HIGH(-min) != 0
        		error CF#v(CFn) "8bit negativ min too big: ", min
        	endif
        else
        	if HIGH(min) != 0
        		error CF#v(CFn) "8bit min too big: ", min
        	endif
        endif
    	if HIGH(max) != 0
    		error CF#v(CFn) "8bit max too big: ", max
    	endif
    	if ((type)==CF_BOOL) && ( (default)>1 )
    		error CF#v(CFn) "BOOL default too big: ", default
    	endif
    	if ((type)==CF_BOOL) && ( (min)>0 || (max)>0 )
    		error CF#v(CFn) "BOOL cannot have min/max"
    	endif
    
        ifdef NO_CF_TYPES
    	    DB  LOW(default), 0
    	else
            local typeFlags
typeFlags   set type
            if (min)!=0
typeFlags       set type + CF_MIN
            endif
            if (max)>(min)
typeFlags       set typeFlags + CF_MAX
            endif
    	    DB  LOW(default), (typeFlags), LOW(min), LOW(max)
        endif
    endif
    expand
    endm

; Starting at CF0
CFn set     -1

; resets all customfunctions to the following default values
cf_default_table0:
    ;---- BANK0 custom function defaults -------------------------------------
    ;                          DEFAULT   MIN     MAX
	CF_DEFAULT    CF_CENTI,	    d'100', d'50',  d'250'  ; CF00 dive_threshold	        100cm
	CF_DEFAULT    CF_CENTI,	    d'75',  d'20',  d'100'  ; CF01 surf_threshold        	75cm
	CF_DEFAULT    CF_INT15,	    d'240', d'0',   d'600'  ; CF02 diveloop_timeout      	240s
#IFDEF CCR_CTRL
    ; Extended Timeout
	CF_DEFAULT    CF_SEC,	    d'255', d'30',  d'255'  ; CF03 surfloop_timeout	        4:15
#ELSE
	CF_DEFAULT    CF_SEC,	    d'120', d'30',  d'240'  ; CF03 surfloop_timeout	        120s
#ENDIF
	CF_DEFAULT    CF_SEC,	    d'5',   d'1',   d'30'   ; CF04 premenu_timeout	        5s

	CF_DEFAULT    CF_INT8, 	    d'7',   d'3',   d'18'   ; CF05 minimum_velocity		    7min/min
	CF_DEFAULT    CF_INT15,	    d'1160',d'950', 0    	; pressure_offset_divemode	1160mbar
	CF_DEFAULT    CF_INT15,	    d'1080',d'1080', 0   	; max_surfpressure		    1080mbar
	CF_DEFAULT    CF_PERCENT,	d'20',  d'1',  d'99'   	; min_gradient_factor		20%
	CF_DEFAULT    CF_PERCENT,	d'20',  d'1',  d'22'	; oxygen_threshold			22%

	CF_DEFAULT    CF_SEC,	    d'45',  d'5',   d'60'   ; CF10 dive_menu_timeout		    45s
	CF_DEFAULT    CF_PERCENT,   d'110', d'110', d'200' 	; saturation_multiplier		x1.10
	CF_DEFAULT    CF_PERCENT,   d'90',  d'50',  d'90'   ; desaturation_multiplier	x0.90
	CF_DEFAULT    CF_PERCENT,	d'60',  d'60',  d'100'	; nofly_time_ratio			60%
	CF_DEFAULT    CF_PERCENT,	d'100', d'50',  d'100'  ; gradient_factor_alarm1	100%

	CF_DEFAULT    CF_PERCENT,	d'10',  d'0',  	d'100'  ; CF15 cns_display_surface			10%
	CF_DEFAULT    CF_DECI,	    d'10',  d'0',  	d'20'	; CF16 deco_distance_for_sim		1m
	CF_DEFAULT    CF_CENTI,     d'019', d'16', 	d'021'	; ppo2_warning_low			0.19 bar
	CF_DEFAULT    CF_CENTI,     d'160', d'0', 	d'160'  ; ppo2_warning_high			1.60 bar
	CF_DEFAULT    CF_CENTI,     d'140', d'0', 	d'150'	; ppo2_display_high			1.40 bar
    
	CF_DEFAULT    CF_INT8,	    d'10',  d'1',   d'120'  ; CF20 sampling_rate				10s
	CF_DEFAULT    CF_INT8,	    d'6',   d'0',   d'15'   ; sampling_divisor_temp		/6
	CF_DEFAULT    CF_INT8,	    d'6',   d'0',   d'15'   ; sampling_divisor_deco		/6
	CF_DEFAULT    CF_INT8,	    d'6',   d'0',   d'15'   ; sampling_divisor_gf		/6
	CF_DEFAULT    CF_INT8,	    d'0',   d'0',   d'15'   ; sampling_divisor_ppo2		never

	CF_DEFAULT    CF_INT8,	    d'0',   d'0',   d'15'   ; CF25 sampling_divisor_deco2	never
	CF_DEFAULT    CF_INT8,	    d'12',  d'0',   d'15'   ; sampling_divisor_cns		/12
	CF_DEFAULT    CF_PERCENT,	d'20',  d'5',   d'75'   ; cns_display_high			20%
	CF_DEFAULT    CF_INT15,	    d'0',   d'0',   0 		; logbook_offset			No Offset, but 15Bit value
	CF_DEFAULT    CF_INT8,	    d'3',   d'2',   d'6'	; last_deco_depth			3m

	CF_DEFAULT    CF_SEC,	    d'10',  d'1',   d'15'   ; CF30 timeout_apnoe_mode		10min
	CF_DEFAULT    CF_BOOL,	    d'0',   0,      0       ; CF31 show_voltage_value		=1 Show value instead of symbol, =0 Show Symbol

    ;---- BANK1 custom function defaults -------------------------------------
cf_default_table1:
    ;                          DEFAULT   MIN     MAX
	CF_DEFAULT    CF_PERCENT,   d'30',  d'5',  	d'90'   ; CF32 GF_low_default			30%
	CF_DEFAULT    CF_PERCENT,   d'85', 	d'30',  d'95'   ; CF33 GF_high_default			85%
	CF_DEFAULT    CF_COLOR,     d'199', 0,      0 		; CF34 color_battery_surface	Color Battery sign: Deep blue
	CF_DEFAULT    CF_COLOR,     d'255', 0,      0 		; CF35 color_standard1			Color Standard: White
	CF_DEFAULT    CF_COLOR,     d'62',  0,      0 		; CF36 color_divemask			Color Divemask: Light green
    
	CF_DEFAULT    CF_COLOR,     d'224', 0,      0 		; CF37 color_warnings			Color Warnings: Red
	CF_DEFAULT    CF_BOOL,	    d'0',   0,      0       ; CF38 show_seconds_divemode		=1 Show the seconds in Divemode
	CF_DEFAULT    CF_BOOL,     	0,      0,      0 		; CF39 Adjust SetPoint if Diluent ppO2 > SetPoint
	CF_DEFAULT    CF_BOOL,	    d'1',   0,      0       ; CF40 warn_ceiling_divemode		=1 Warn ceiling violation in divemode
#IFNDEF SEAWOOCH
	CF_DEFAULT    CF_BOOL,      d'1',   0,      0 		; CF41 unused
#ELSE
	CF_DEFAULT    CF_BOOL,      d'1',   0,      0 		; CF41 Metric Units
#ENDIF
	CF_DEFAULT    CF_BOOL,	    d'1',   0,      0       ; CF42 blink_gas_divemode 		=1 blink better gas
	CF_DEFAULT    CF_INT15,     d'13000', 0,   d'13000' ; CF43 color_warn_depth_mbar		Warn depths
	CF_DEFAULT    CF_PERCENT,	d'101', d'50',  d'101'	; CF44 color_warn_cns_percent    Warn-%
	CF_DEFAULT    CF_PERCENT,	d'101', d'50',  d'101'  ; CF45 color_warn_gf_percent		Warn-%
	CF_DEFAULT    CF_CENTI,     d'161', d'100', d'161'  ; CF46 color_warn_ppo2_cbar		ppO2 warn

	CF_DEFAULT    CF_INT8,	    d'15',  d'7',   d'20'	; CF47 color_warn_celocity_mmin	warn at xx m/min
	CF_DEFAULT    CF_SEC+CF_NEG,d'0',  -d'120' ,d'120'  ; CF48 time_correction_value_default	Adds to Seconds on Midnight
	CF_DEFAULT    CF_BOOL,      d'0',   0,      0 		; CF49 Show Altimeter in surface mode
	CF_DEFAULT    CF_BOOL,     	d'0',   0,      0       ; CF50 Show Log-Marker
	CF_DEFAULT    CF_BOOL,	    d'1',   0,      0 		; CF51 Show Stopwatch
	                
	CF_DEFAULT    CF_BOOL,     	d'0',   0,      0 		; CF52 Show Tissue Graph in Divemode
	CF_DEFAULT    CF_BOOL,	    d'0',   0,      0 		; CF53 Show Laeding Tissue in Divemode
	CF_DEFAULT    CF_BOOL,      d'0',   0,      0 		; CF54 Display shallowest stop first
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'10'   ; CF55 Gas switch additional delay
	CF_DEFAULT    CF_INT8,      d'20',  d'5',   d'50'   ; CF56 Bottom gas usage (SAC l/min)

	CF_DEFAULT    CF_INT8,      d'20',  d'5',   d'50'   ; CF57 Ascent/deco gas usage (SAC l/min)
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'10'   ; CF58 TTS for extra time at current depth [min]
	CF_DEFAULT    CF_INT15,     d'0',   d'0',   d'7000' ; CF59 Cave conso warning [l]
#IFDEF SEAWOOCH
	CF_DEFAULT    CF_BOOL,     	0,   	0,      0 		; CF60 Show Graphical ascend speed indicator
#ELSE
	CF_DEFAULT    CF_BOOL,     	1,   	0,      0 		; CF60 Show Graphical ascend speed indicator
#ENDIF
	CF_DEFAULT    CF_BOOL,      0,      0,      0 		; CF61 Show pSCR ppO2
	                
	CF_DEFAULT    CF_PERCENT,   .4,     .0,     .100	; CF62 pSCR O2 Drop
	CF_DEFAULT    CF_INT8,      .10,    .0,     .100 	; CF63 pSCR counterlung ratio
    
	;---- BANK2 custom function defaults -------------------------------------
cf_default_table2:
	CF_DEFAULT    CF_COLOR,     d'103',  0,      0 		; CF64 color_inactive	Color inactive: grey
	CF_DEFAULT    CF_BOOL,     	0,   	0,      0 		; CF65 Show safety stop
	CF_DEFAULT    CF_BOOL,      0,   	0,      0 		; CF66 Show GF in NDL (If GF > CF08)
	CF_DEFAULT    CF_PERCENT,   d'30',  d'5',  	d'90'   ; CF67 aGF_low_default			30%
	CF_DEFAULT    CF_PERCENT,   d'90', 	d'30',  d'95'   ; CF68 aGF_high_default			90%

	CF_DEFAULT    CF_BOOL,     	0,   	0,      0 		; CF69 Allow GF change (Between GF and aGF)
	CF_DEFAULT    CF_SEC,       d'180', d'10',  d'250'  ; CF70 Safety Stop Duration [s]
	CF_DEFAULT    CF_DECI,      d'51',  d'30',  d'65'   ; CF71 Safety Stop Start Depth [dm]
	CF_DEFAULT    CF_DECI,      d'29',  d'25',  d'50'   ; CF72 Safety Stop End Depth [dm]
	CF_DEFAULT    CF_DECI,      d'101', d'75',  d'201'  ; CF73 Safety Stop Reset Depth [dm]

	CF_DEFAULT    CF_INT15,     d'1800',d'0',   d'3600' ; CF74 Battery Timeout [min]
#IFDEF CCR_CTRL
    ; New CFs for eCTRL
	CF_DEFAULT    CF_BOOL,	    d'1',   0,      0       ; CF75 eCTRL Enabled =1 Use averaged ppO2 from eCTRL, =0 Autonomous
	CF_DEFAULT    CF_PERCENT,   d'100', d'21',  d'100'  ; CF76 eCTRL O2 persentage in Calibration gas 100 == Oxy
	CF_DEFAULT    CF_BOOL,	    d'0',   0,      0       ; CF77 eCTRL Calibration with correction on Ambien pressure =1 - correction, =0 - without correction
	CF_DEFAULT    CF_INT8,      d'42',	d'0',   d'255'  ; CF78 eCTRL Proportional coefficient
	CF_DEFAULT    CF_BOOL,      d'0',   0,      0       ; CF79 eCTRL Solenoid Advanced Algorithm select
	CF_DEFAULT    CF_INT15,     d'180', d'0',   d'600'  ; CF80 eCTRL Scrubber Life in Minutes
	CF_DEFAULT    CF_BOOL,      d'0',   d'0',   d'0'    ; CF81 eCTRL HUD Signalling type
#ELSE
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF75 unused
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF76 unused
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF77 unused
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF78 unused

	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF79 unused
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF80 unused
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF81 unused
#ENDIF
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF82 unused
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF83 unused

	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF84 unused
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF85 unused
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF86 unused
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF87 unused
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF88 unused

	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF89 unused
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF90 unused
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF91 unused
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF92 unused
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF93 unused
	
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF94 unused	
	CF_DEFAULT    CF_INT8,      d'0',   d'0',   d'0'    ; CF95 unused	
cf_default_table3:
;=============================================================================

menu_reset:
	movlw	d'1'
	movwf	menupos

	call	DISP_ClearScreen
	call	DISP_reset_menu_mask

menu_reset2:
	call	menu_pre_loop_common		; Clear some menu flags, timeout and switches
	call	DISP_reset_menu_mask
	call	DISP_menu_cursor
menu_reset_loop:
	call	check_switches_menu
	btfsc	menubit2
	bra		do_menu_reset					; call submenu
	btfss	menubit
	bra		menu							; exit setup menu and return to main menu
	btfsc	onesecupdate
	call	timeout_surfmode
	btfsc	onesecupdate
	call	set_dive_modes
	btfsc	onesecupdate
	call	test_charger				; check if charger IC is active
	btfsc	onesecupdate
	call	get_battery_voltage			; get battery voltage
	bcf		onesecupdate				; End of one second tasks
	btfsc	sleepmode
	goto	menu
	btfsc	divemode
	goto	restart						; exit menu, restart and enter divemode
	bra		menu_reset_loop
do_menu_reset:						; calls submenu
	dcfsnz	menupos,F
	bra		do_menu_reset_exit				; Cancel, exit
	dcfsnz	menupos,F
	bra		do_menu_reset_all				; Reset all settings
	dcfsnz	menupos,F
	bra		do_menu_reset_logbook			; Reset Logbook Memory!
	dcfsnz	menupos,F
	bra		do_menu_reset_reboot			; Reboot OSTC
	dcfsnz	menupos,F
	bra		do_menu_reset_decodata			; Reset Decodata
do_menu_reset_exit:
#IFDEF CCR_CTRL
    ; Reorder Menu structure for CCR
	movlw	d'2'                            ; New Return Position and Menu
	movwf	menupos
	goto	more_menu2
#ELSE
	movlw	d'3'
	movwf	menupos
	bra		menu2							; exit...
#ENDIF


do_menu_reset_reboot:
	call	DISP_confirmbox				; Returns WREG=0 for Cancel (Or Timeout) and WREG=1 for OK!
	movwf	menupos						; Used as temp
	tstfsz	menupos
	bra		do_menu_reset_reboot2		; Delete now!
	bra		do_menu_reset_exit			; Cancel!

do_menu_reset_reboot2:
	call	DISP_DisplayOff					; Power-down DISPLAY 
	movlw	b'00000000'						; Bit6: PPL Disable
	movwf	OSCTUNE
	movlw	b'01111110'						; 8MHz
	movwf	OSCCON
	reset
	goto	0x00000							; restart to 0x00000

do_menu_reset_logbook:
	call	DISP_confirmbox				; Returns WREG=0 for Cancel (Or Timeout) and WREG=1 for OK!
	movwf	menupos						; Used as temp
	tstfsz	menupos
	bra		do_menu_reset_logbook2		; Delete Logbook now!
	bra		do_menu_reset_exit			; Cancel!

do_menu_reset_logbook2:
	call	DISP_ClearScreen
    setf    win_color1                  ; Make sure to display in white color.
    setf    win_color2
	DISPLAYTEXT	.25					    ; "Reset..."
	call	reset_external_eeprom		; delete profile memory
	bra		do_menu_reset_exit

do_menu_reset_decodata:
	call	DISP_confirmbox				; Returns WREG=0 for Cancel (Or Timeout) and WREG=1 for OK!
	movwf	menupos						; Used as temp
	tstfsz	menupos
	bra		do_menu_reset_decodata2		; Reset Deco Data now!
	bra		do_menu_reset_exit			; Cancel!

do_menu_reset_decodata2:
; reset deco data
	call	DISP_ClearScreen
	DISPLAYTEXT	.25					    ; "Reset..."

    SAFE_2BYTE_COPY amb_pressure,int_I_pres_respiration	; copy surface air pressure to deco routine
	call		deco_clear_tissue			; Reset Decodata
	call		deco_calc_desaturation_time	; calculate desaturation time
	call		deco_clear_CNS_fraction			; clear CNS
	movlb		b'00000001'						; select ram bank 1
  	clrf		nofly_time+0        	      	; Reset NoFly
  	clrf		nofly_time+1            	  	; Reset NoFly
	bcf			nofly_active                	; Clear flag
	goto		restart							; done. quit to surfmode

do_menu_reset_all:
	call	DISP_confirmbox				; Returns WREG=0 for Cancel (Or Timeout) and WREG=1 for OK!
	movwf	menupos						; Used as temp
	tstfsz	menupos
	bra		do_menu_reset_all2			; Reset all now!
	bra		do_menu_reset_exit			; Cancel!

do_menu_reset_all2:
	call	DISP_ClearScreen
	DISPLAYTEXT	.25					    ; "Reset..."

reset_start:
; reset deco data
    SAFE_2BYTE_COPY amb_pressure,int_I_pres_respiration	; copy surface air pressure to deco routine
	call		deco_clear_tissue			; Reset Decodata
	call		deco_calc_desaturation_time	; calculate desaturation time
	call		deco_clear_CNS_fraction			; clear CNS
	movlb		b'00000001'						; select ram bank 1
  	clrf		nofly_time+0    	          	; Reset NoFly
  	clrf		nofly_time+1	              	; Reset NoFly
	bcf			nofly_active                	; Clear flag

; reset gases
	rcall	reset_gases
	rcall   reset_all_cf
	goto	restart					; all reset, quit to surfmode

reset_all_cf:
	movlw	d'1'
	movwf	EEDATA
	write_int_eeprom	d'33'		; reset start gas
#IFDEF CCR_CTRL
	movlw	d'5'                    ; Default is L16-GF CC
#ELSE
	movlw	d'4'                    ; Default is L16-GF OC
#ENDIF
	movwf	EEDATA
	write_int_eeprom	d'34'		; reset deco model
	clrf	EEDATA
	write_int_eeprom	d'35'		; unused in Mk.2

	clrf	EEDATA
	write_int_eeprom	d'39'		; Disable Debugbode
	clrf	EEDATA
	write_int_eeprom	d'90'		; Disable Brightness offset? (Dim=1, Normal = 0)

	movlw	d'1'
	movwf	EEDATA
	write_int_eeprom	d'91'		; Reset Date format to DD.MM.YY

	movlw	d'100'
	movwf	EEDATA
	write_int_eeprom	d'26'		; Salinity default: 1.00 kg/l

	movlw	b'00011111'	
	movwf	EEDATA
	write_int_eeprom	d'27'		; reset active gas flags

	clrf	EEDATA
	write_int_eeprom	d'28'		; reset change depth gas #1
	clrf	EEDATA
	write_int_eeprom	d'29'		; reset change depth gas #2
	clrf	EEDATA
	write_int_eeprom	d'30'		; reset change depth gas #3
	clrf	EEDATA
	write_int_eeprom	d'31'		; reset change depth gas #4
	clrf	EEDATA
	write_int_eeprom	d'32'		; reset change depth gas #5

	movlw	d'80'
	movwf	EEDATA
	write_int_eeprom	d'36'		; reset mix1 to ppO2=0.80bar
	movlw	d'100'
	movwf	EEDATA
	write_int_eeprom	d'37'		; reset mix2 to ppO2=1.00bar
	movlw	d'120'
	movwf	EEDATA
	write_int_eeprom	d'38'		; reset mix3 to ppO2=1.20bar
    ; Change depths to zero
    clrf    EEDATA
    write_int_eeprom	d'117'
    write_int_eeprom	d'118'
    write_int_eeprom	d'119'
    ; SP Mode (Manual)
    write_int_eeprom	d'116'

	clrf	nofly_time+0			; Clear nofly time
	clrf	nofly_time+1			; Clear nofly time

reset_all_cf_bank0:	
    clrf    EEADRH					; EEPROM BANK 0
	movlw	d'127'					; address of low byte of first custom function
	movwf	EEADR

    movlw   LOW cf_default_table0    ; Load PROM pointer.
    movwf   TBLPTRL,A
    movlw   HIGH cf_default_table0
    movwf   TBLPTRH,A
    movlw   UPPER cf_default_table0
    movwf   TBLPTRU,A

cf_bank0_loop:
	; Did we already read 32 (decimal) words or double-words (with types) ?
	movf    TBLPTRL,W
	sublw   LOW (cf_default_table1)
	bz      reset_all_cf_bank1
	rcall	reset_customfunction	; saves default and current value
	bra     cf_bank0_loop
	
reset_all_cf_bank1:
	movlw	d'1'
	movwf	EEADRH					; EEPROM BANK 1
	movlw	d'127'					; address of low byte of first custom function
	movwf	EEADR

    movlw   LOW cf_default_table1    ; Load PROM pointer.
    movwf   TBLPTRL,A
    movlw   HIGH cf_default_table1
    movwf   TBLPTRH,A
    movlw   UPPER cf_default_table1
    movwf   TBLPTRU,A

	
cf_bank1_loop:
	; Did we already read another 32 (decimal) words or double-words ?
	movf    TBLPTRL,W
	sublw   LOW (cf_default_table2)
	bz      reset_all_cf_bank2
	rcall	reset_customfunction	; saves default and current value
	bra     cf_bank1_loop

reset_all_cf_bank2:
	movlw	d'2'
	movwf	EEADRH					; EEPROM BANK 2
	movlw	d'127'					; address of low byte of first custom function
	movwf	EEADR

    movlw   LOW cf_default_table2    ; Load PROM pointer.
    movwf   TBLPTRL,A
    movlw   HIGH cf_default_table2
    movwf   TBLPTRH,A
    movlw   UPPER cf_default_table2
    movwf   TBLPTRU,A
	
cf_bank2_loop:
	; Did we already read another 32 (decimal) words or double-words ?
	movf    TBLPTRL,W
	sublw   LOW (cf_default_table3)
	bz      cf_bank2_end
	rcall	reset_customfunction	; saves default and current value
	bra     cf_bank2_loop

cf_bank2_end:
	clrf	EEADRH					; EEPROM BANK 0
    return

reset_gases:
	clrf	EEADRH					; EEPROM BANK 0

	movlw	d'3'					; address of first gas-1
	movwf	EEADR
	clrf	hi						; He part (default for all gases and diluents: 0%)
    movlw   .21
    movwf   lo                      ; O2 part (default for all gases and diluents: 21%)
	rcall	reset_gas               ; saves current value for gas #1
	rcall	reset_gas               ; saves default value for gas #1
	rcall	reset_gas               ; saves current value for gas #2
	rcall	reset_gas               ; saves default value for gas #2
	rcall	reset_gas               ; saves current value for gas #3
	rcall	reset_gas               ; saves default value for gas #3
	rcall	reset_gas               ; saves current value for gas #4
	rcall	reset_gas               ; saves default value for gas #4
	rcall	reset_gas               ; saves current value for gas #5
	rcall	reset_gas               ; saves default value for gas #5
	rcall	reset_gas               ; saves current value for gas #6

	movlw	d'94'					; address of first diluent-1
	movwf	EEADR
	rcall	reset_gas               ; saves current value for diluent #1
	rcall	reset_gas               ; saves default value for diluent #1
	rcall	reset_gas               ; saves current value for diluent #2
	rcall	reset_gas               ; saves default value for diluent #2
	rcall	reset_gas               ; saves current value for diluent #3
	rcall	reset_gas               ; saves default value for diluent #3
	rcall	reset_gas               ; saves current value for diluent #4
	rcall	reset_gas               ; saves default value for diluent #4
	rcall	reset_gas               ; saves current value for diluent #5
	rcall	reset_gas               ; saves default value for diluent #5

    movlw   .1
    movwf   EEDATA
    write_int_eeprom    .33         ; First Gas (1-5)
    write_int_eeprom    .115        ; First Diluent (1-5)
	return

; Write WREG:lo twice, w/o any type clearing, pre-incrementing EEADR
reset_gas:
	incf	EEADR,F
	movff	lo, EEDATA				; O2 value
	call	write_eeprom
	incf	EEADR,F
	movff	hi, EEDATA				; He value
	call    write_eeprom
	return

reset_customfunction:
    tblrd*+
    movff   TABLAT, lo              ; Low byte in lo,
    tblrd*+
    movff   TABLAT, hi              ; High byte in hi

  ifndef NO_CF_TYPES
    tblrd*+                         ; Skip advanced min/max values.
    tblrd*+
    btfss   hi,7                    ; In EEPROM, just clear all types,
    clrf    hi                      ; to keep external program compat (jdivelog etc.)
    bcf     hi,7
  endif

    ; Manage the default/value tuple
    rcall   reset_eeprom_value      ; First pair, untouched.
    bcf     hi,7                    ; Just clear type bit.
    bra    reset_eeprom_value       ; Second pair, cleared

; Write the two bytes lo:hi into EEPROM
reset_eeprom_value:
	incf	EEADR,F
	movff	lo, EEDATA				; Lowbyte Default value

	movlw	d'127'					; Work-around to prevent writing at EEPROM 0x00 to 0x04 
	cpfslt	EEADR					; EEADR > 127?
	call	write_eeprom			; Yes, write!

	incf	EEADR,F
	movff	hi, EEDATA				; Highbyte default value

	movlw	d'127'					; Work-around to prevent writing at EEPROM 0x00 to 0x04 
	cpfslt	EEADR					; EEADR > 127?
	call    write_eeprom			; Yes, write!
	return

reset_external_eeprom:				; deletes complete external eeprom!
	clrf	eeprom_address+0
	clrf	eeprom_address+1

	movlw	d'4'
#IFDEF SEAWOOCH
	btfsc		seawooch_EE512
	movlw	d'2'
#ENDIF
	movwf	temp3
reset_eeprom02:
	clrf	temp4
reset_eeprom01:
	movlw	d'64'
#IFDEF SEAWOOCH
	btfsc		seawooch_EE512
	movlw	d'128'
#ENDIF
	movwf	temp2
	bcf		eeprom_blockwrite		; Blockwrite start
reset_eeprom1:
	setf	ext_ee_temp1			; byte for Blockwrite....
	movf	ext_ee_temp1,W			; So, 1st. Byte of block is fine, too
	call	write_external_eeprom_block
#IFDEF EE512
	decfsz	temp2,F		; 128 Byte done
#ELSE
	decfsz	temp2,F		; 64 Byte done
#ENDIF
	bra		reset_eeprom1
	bsf		SSPCON2,PEN					; Stop condition
	call	WaitMSSP	
	WAITMS	d'7'
	decfsz	temp4,F
	bra		reset_eeprom01				; do this 256 times
	decfsz	temp3,F
	bra		reset_eeprom02				; and this all 4 times -> 1024 *64Bytes = 64KB

	bcf		eeprom_blockwrite			; clear blockwrite flag

	clrf	eeprom_address+0
	clrf	eeprom_address+1

	movlw	0xFD						; With these three bytes the OSTC will find the free area in the EEPROM faster
	call	write_external_eeprom
	movlw	0xFD
	call	write_external_eeprom
	movlw	0xFE						
	call	write_external_eeprom	

	clrf	eeprom_address+0
	clrf	eeprom_address+1
	return
	
