
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


; Mainroutines for divemode
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 10/01/05
; last updated: 15/05/08
; known bugs:
; ToDo: 

diveloop:
	call	diveloop_boot			; Boot tasks for all modes

; start timer3 for deco routine CPU usesage monitor
	movlw	b'00000011'
	movwf	T3CON			; Timer3 with 32768Hz clock running

#IFDEF CCR_CTRL
    btfss	FLAG_const_ppO2_mode        ; Check if in eCTRL mode
    bra     diveloop_boot_0             ; skip if not in eCTRL mode
    btfss   ext_ppO2_enable             ; Check for eCTRL mode
    bra     diveloop_boot_0             ; skip if not in eCTRL mode
	call	enable_rs232_115k_ISR		; enables UART FOREWER
diveloop_boot_0:
#ENDIF

; Startup Tasks for all modes
	ostc_debug	'R'		; Sends debug-information to screen if debugmode active
	call	DISP_ClearScreen			; clean up DISPLAY
	call	DISP_divemode_mask			; Display mask
	call	DISP_temp_divemode			; Displays temperature
   	call	DISP_active_gas_divemode	; Display gas

; Reload last customview
	read_int_eeprom	d'94'				; Read last selected customview dive mode into EEDATA
	movff	EEDATA,menupos3				; Copy to menupos3
	call	customview_toggle2			; Show customview 

	btfsc	FLAG_apnoe_mode
	bsf		realdive					; Set Realdive flag in Apnoe mode

	btfsc	gauge_mode
	bra		diveloop_loop				; Skip in gauge mode
	btfsc	FLAG_apnoe_mode
	bra		diveloop_loop				; Skip in apnoe mode

; Startup Tasks for deco modes
	call	DISP_display_ndl_mask		; display "no stop" if not in gauge or apnoe mode
    GETCUSTOM8  d'55'
    movwf   apnoe_surface_mins
    clrf    apnoe_surface_secs

	btfss	FLAG_const_ppO2_mode		; only in const_ppO2_mode
	bra		diveloop_loop				; OC modes, skip
	bsf		setpoint_changed			; Add a setpoint change to the first depth sample in CC mode
	bsf		event_occured				; set global event flag

diveloop_loop:		; The diveloop starts here
	btfss	onesecupdate					; tasks any new second
	bra		diveloop_loop3

    bcf     onesecupdate					; one seconds update, clear flag here in case it's set again in ISR before all tasks are done.

	btfsc	gauge_mode						; Only in gauge mode
	bra		diveloop_loop1a					; One Second Tasks in Gauge mode
	btfsc	FLAG_apnoe_mode					; Only in apnoe mode
	bra		diveloop_loop1b					; One Second Tasks in Apnoe mode

; Update divetime 
	call	customview_second				; Do every-second tasks for the custom view area
	GETCUSTOM8	d'38'		; Show seconds (=1?)
	movwf	lo
	movlw	d'1'
	cpfseq	lo					; =1?
	bra		diveloop_loop1y		; No, minutes only
	bsf		update_divetime		; Set Update flag
diveloop_loop1y:
	btfss	update_divetime				; display new divetime?
	bra		diveloop_loop1z				; No
	btfsc	premenu						; Is the divemode menu active?
	bra		diveloop_loop1z				; Yes
	call	DISP_divemins				; Display (new) divetime!
diveloop_loop1z:
	bcf		update_divetime				; clear flag

    call    DISP_show_gas_change_countdown  ; Show the gas change countdown (If required)

	btfsc	FLAG_const_ppO2_mode			; only in const_ppO2_mode
	bra		diveloop_loop1c					; One Second Tasks in const_ppO2 mode

; Tasks only for OC modes
	btfsc	show_safety_stop				; Show the safety stop?
	call	DISP_show_safety_stop			; Yes, show/delete if done.
	call	check_ppO2						; check ppO2 and displays warning if required
	call	calc_deko_divemode				; calculate decompression and display result (any two seconds)
	bra		diveloop_loop1x					; Common Tasks

; Tasks only for Gauge mode
diveloop_loop1a:
	btfss	premenu							; Is the divemode menu active?
	call	DISP_divemins					; display (new) divetime!
	call	customview_second				; Do every-second tasks for the custom view area
	call	timeout_divemode				; dive finished? This routine sets the required flags

	btfsc	twosecupdate					; two seconds after the last call
	bra		diveloop_loop1a2				; Common Tasks

	bsf		twosecupdate					; Routines used in the "other second"
	call	calc_average_depth				; calculate average depth
	call	calc_velocity					; calculate vertical velocity and display if > threshold (every two seconds)
	
	bra		diveloop_loop1x					; Common Tasks

diveloop_loop1a2:
	bcf		twosecupdate		
	bra		diveloop_loop1x					; Common Tasks

; Tasks only for Apnoe mode
diveloop_loop1b:
	call	divemode_apnoe_tasks			; 1 sec. Apnoe tasks
	bra		diveloop_loop1x					; Common Tasks

; Tasks only for ppO2 mode
diveloop_loop1c:

#IFDEF CCR_CTRL
    btfss   ext_ppO2_enable                 ; Check for eCTRL mode
	bra		diveloop_loop1d                 ; no - Do not search for sensor in CC mode

    call    DISP_divemode_02sensor_all      ; Show ppO2 for 3 sensors
    
    call    send_message                    ; Send message to eCTRL

	btfsc	is_bailout						; Are we in Bailout mode?
    bra     diveloop_loop1d                 ; Don't use ppO2 from eCTRL.

	btfss	ext_ppO2_valid					; external ppO2 valid?
    bra     diveloop_loop1c1
	btfsc	ext_ppo2_abandon    			; Use O2 Sensor?
    bra     diveloop_loop1c1

	movff	eCTRL_ppo2_av, char_I_const_ppO2	; Yes, copy ACTUAL ppO2 from eCTRL to use for deco calculations
	movff	eCTRL_ppo2_av, ppO2_setpoint_store
    bra     diveloop_loop1d

diveloop_loop1c1:
    movff   eCTRL_SP_DC, char_I_const_ppO2	; No, use commanded SP of the DC to use for deco calculations
	movff	eCTRL_SP_DC, ppO2_setpoint_store

diveloop_loop1d:
#ENDIF

	btfsc	show_safety_stop				; Show the safety stop?
	call	DISP_show_safety_stop			; Yes, show/delete if done.
    call    check_dive_autosp               ; Check for Auto-SP
	call	DISP_const_ppO2_value			; display const ppO2 setting in [bar]
	call	calc_deko_divemode				; calculate decompression and display result (any two seconds)
	btfsc	is_bailout						; Are we in Bailout mode?
	call	check_ppO2_bail					; Yes, display ppO2 (If required)

	bra		diveloop_loop1x					; Common Tasks
	
; Common Tasks for all modes
diveloop_loop1x:
;	call	customview_second				; Do every-second tasks for the custom view area
;
;	GETCUSTOM8	d'38'		; Show seconds (=1?)
;	movwf	lo
;	movlw	d'1'
;	cpfseq	lo					; =1?
;	bra		diveloop_loop1y		; No, minutes only
;	bsf		update_divetime		; Set Update flag
;diveloop_loop1y:
;	btfss	update_divetime				; display new divetime?
;	bra		diveloop_loop1z				; No
;	btfsc	premenu						; Is the divemode menu active?
;	bra		diveloop_loop1z				; Yes
;	call	DISP_divemins				; Display (new) divetime!
;
;diveloop_loop1z:
;	bcf		update_divetime				; clear flag

	btfsc	ppO2_show_value					; show ppO2?
	call	check_ppO2						; check ppO2 and displays warning if required
	call	timeout_divemode				; dive finished? This routine sets the required flags
	btfsc	low_battery_state				; If battery is low, then...
	call	update_batt_voltage_divemode	; Display Battery Warning Text
	btfsc	premenu							; is Menu? displayed?
	call	timeout_premenu_divemode		; No, so check for timeout premenu
	btfsc	menubit							; is the Dive mode menu displayed?
	call	timeout_divemenu				; Yes, so check for timeout divemenu
	call	set_leds_divemode				; Sets warnings, if required. Also Sets buzzer
	btfsc	enter_error_sleep				; Enter Fatal Error Routine?
	goto	fatal_error_sleep				; Yes (In Sleepmode.asm!)

diveloop_loop3:
	btfss	menubit							; Divemode menu active?
	call	test_switches_divemode			; No, Check switches normal

	btfsc	menubit							; Divemode menu active?
	call	test_switches_divemode_menu		; Yes, check switches divemode menu

	btfss	divemode						; Dive finished?
	goto	end_dive						; Dive finished!

	btfsc	pressure_refresh				; new pressure available?
	call	update_divemode1				; Yes, display new depth
	bcf		pressure_refresh				; until new pressure is available

	btfsc	oneminupdate					; one minute tasks
	call	update_divemode60				; Update clock, etc.

	btfsc	store_sample					; store new sample?
	call	store_dive_data					; Store profile data

	btfsc	toggle_customview				; Next view?
	call	customview_toggle				; Yes, show next customview (and delete this flag)

	btfsc	simulatormode_active			; Is Simualtor mode active ?
	bra		diveloop_loop4                  ; YES: don't sleep
#IFDEF CCR_CTRL
    ; Disable SLEEP in divemode in eCTRL mode - use more energy, but prevent message loosing
    btfsc   ext_ppO2_enable                 ; Check for eCTRL mode
	bra		diveloop_loop4                  ; YES: don't sleep

#ENDIF
	btfsc	menubit							; Sleep only with inactive menu...
	bra		diveloop_loop5

    bcf     T0CON,TMR0ON                    ; Stop Timer0 before sleep
    nop
	sleep
	nop
	bra		diveloop_loop					; Loop the divemode

diveloop_loop4:                             ; And test screen dumps too!
	btfsc	uart_dump_screen                ; Asked to dump screen contains ?
	call	dump_screen     			    ; Yes!

diveloop_loop5:
	bra		diveloop_loop					; Loop the divemode

timeout_premenu_divemode:
	incf	timeout_counter3,F              ; Yes...

	GETCUSTOM8	d'4'                        ; loads premenu_timeout into WREG
	cpfsgt	timeout_counter3                ; ... longer then premenu_timeout
	return                                  ; No!

#IFDEF BOTTOM
; Special keyswitches logic in Gauge mode
	btfss	gauge_mode					
	bra		timeout_premenu_divemode1

	bcf		premenu                        ; 
	bcf		res_type                       ; 
	call	DISP_pre_gauge_clear                 ; 
	call	DISP_divemode_mask				; And redraw mask (Redraw missing "T" from "Tauchzeit" in german text version)
	return
timeout_premenu_divemode1:
#ENDIF

	bcf		premenu                         ; Yes, so clear "Menu?" and clear pre_menu bit
	call	DISP_menu_clear                 ; Remove "Menu?"
	call	DISP_divemode_mask				; And redraw mask (Redraw missing "T" from "Tauchzeit" in german text version)
	call	DISP_divemins					; Redraw divetime
	return

divemode_apnoe_tasks:                       ; 1 sec. Apnoe tasks
	call	DISP_divemins					; display (new) divetime!
	call	DISP_display_apnoe_descent		; Show descent timer

	btfsc	divemode2						; Time running?
	bra		divemode_apnoe_tasks2			; New descent, reset data if flag is set

	call	DISP_display_apnoe_surface
	incf	apnoe_surface_secs,F
	movlw	d'60'
	cpfseq	apnoe_surface_secs
	bra		divemode_apnoe_tasks1
	clrf	apnoe_surface_secs
	incf	apnoe_surface_mins,F

divemode_apnoe_tasks1:	
	bcf		FLAG_active_descent				; Clear flag
	btfsc	divemode2						; Time running?
	return									; Yes, return
	
	bsf		FLAG_active_descent				; Set Flag
	return

divemode_apnoe_tasks2:
	btfss	FLAG_active_descent				; Are descending?			
	return									; No, We are at the surface
	call	apnoe_calc_maxdepth				; Yes!
	
divemode_apnoe_tasks3:
	call	DISP_apnoe_clear_surface		; Clear Surface timer
	
	clrf	apnoe_timeout_counter			; Delete timeout
	clrf	apnoe_surface_secs
	clrf	apnoe_surface_mins
	clrf	apnoe_secs
	clrf	apnoe_mins						; Reset Descent time
	clrf	max_pressure+0
	clrf	max_pressure+1					; Reset Max. Depth
	bcf		FLAG_active_descent				; Clear flag
	return

apnoe_calc_maxdepth:
	movff	apnoe_max_pressure+0,sub_a+0
	movff	apnoe_max_pressure+1,sub_a+1
	movff	max_pressure+0,sub_b+0
	movff	max_pressure+1,sub_b+1
	call	sub16				; sub_c = sub_a - sub_b
								; apnoe_max_pressure<max_pressure -> neg_flag=1
								; max_pressure<=apnoe_max_pressure -> neg_flag=0
	btfss	neg_flag	
	return
								;apnoe_max_pressure<max_pressure
	movff	max_pressure+0,apnoe_max_pressure+0
	movff	max_pressure+1,apnoe_max_pressure+1
	return

set_leds_divemode:
	movff	char_O_gradient_factor,lo			; gradient factor absolute (Non-GF model)
	movff	char_I_deco_model,hi
	decfsz	hi,F		; jump over next line if char_I_deco_model == 1
	movff	char_O_relative_gradient_GF,lo		; gradient factor relative (GF model)
	
	GETCUSTOM8	d'14'		; threshold for LED warning into WREG
	cpfslt	lo				; Lower then actual warning?
	rcall	warn_gf1		; No, show warning and set flags
	return					; Yes, return

warn_gf1:
	movlw		d'2'			; Type of alarm (Deco Stop missed/GF violated)
	movwf		AlarmType		; Copy to alarm Register
	bsf			event_occured	; Set Event Flag
	return

calc_deko_divemode:
	btfsc	twosecupdate		; two seconds after the last call
	bra		calc_deko_divemode2		; Yes, calculate and display deco data ("first second")

	bsf		twosecupdate		; No, but next second!
	; Routines used in the "other second"
	call	calc_average_depth	; calculate average depth
	call	calc_velocity		; calculate vertical velocity and display if > threshold (every two seconds)
	call	set_reset_safety_stop	; Set flags for safety stop and/or reset safety stop
	
; Calculate CNS	
    clrf    WREG
    movff   WREG,char_I_step_is_1min    ; Make sure to be in 2sec mode.

	read_int_eeprom d'34'				; Read deco data
	movlw	.6	
	cpfseq	EEDATA						; in pscr-mode?
    rcall   set_actual_ppo2             ; No, set char_I_actual_ppO2

calc_deko_divemode1a:
	call	deco_calc_CNS_fraction		; calculate CNS
	movlb	b'00000001'					; rambank 1 selected

; Check if CNS should be displayed
	movff	char_O_CNS_fraction,lo		; copy into bank1
	GETCUSTOM8	d'27'					; cns_display_high
	subwf	lo,W
	btfsc	STATUS,C
	call	DISP_display_cns			; Show CNS
	call	check_gas_change			; Checks if a better gas should be selected (by user)

;NOTE: always set full gas list, because the user can switch gas,
;      even in NDL mode...

; Copy all gases to char_I_deco_N2_ratio and char_I_deco_He_ratio
divemode_check_decogases:               ; CALLed from Simulator, too
    ASSERT_BANK1    .10001              ; Just make sure everything is correct.

	clrf    EEADRH                      ; Make sure to select eeprom bank 0	

    read_int_eeprom		d'7'			; Read He ratio
    movff	EEDATA,char_I_deco_He_ratio+0	; And copy into hold register
    read_int_eeprom		d'6'			; Read O2 ratio
    movff	char_I_deco_He_ratio+0,WREG ; Get back He -> WREG
    addwf   EEDATA,W                    ; O2 + He -> WREG
    sublw   d'100'                      ; 100 - (O2 + He) -> WREG
    movff	WREG, char_I_deco_N2_ratio+0; Push N2 to C code;

    read_int_eeprom		d'11'			; Read He ratio
    movff	EEDATA,char_I_deco_He_ratio+1	; And copy into hold register
    read_int_eeprom		d'10'			; Read O2 ratio
    movff	char_I_deco_He_ratio+1,WREG ; Get back He -> WREG
    addwf   EEDATA,W                    ; O2 + He -> WREG
    sublw   d'100'                      ; 100 - (O2 + He) -> WREG
    movff	WREG, char_I_deco_N2_ratio+1; Push N2 to C code;

	read_int_eeprom		d'15'			; Read He ratio
	movff	EEDATA,char_I_deco_He_ratio+2	; And copy into hold register
	read_int_eeprom		d'14'			; Read O2 ratio
    movff	char_I_deco_He_ratio+2,WREG ; Get back He -> WREG
    addwf   EEDATA,W                    ; O2 + He -> WREG
    sublw   d'100'                      ; 100 - (O2 + He) -> WREG
    movff	WREG, char_I_deco_N2_ratio+2; Push N2 to C code;

	read_int_eeprom		d'19'			; Read He ratio
	movff	EEDATA,char_I_deco_He_ratio+3	; And copy into hold register
	read_int_eeprom		d'18'			; Read O2 ratio
    movff	char_I_deco_He_ratio+3,WREG ; Get back He -> WREG
    addwf   EEDATA,W                    ; O2 + He -> WREG
    sublw   d'100'                      ; 100 - (O2 + He) -> WREG
    movff	WREG, char_I_deco_N2_ratio+3; Push N2 to C code;

	read_int_eeprom		d'23'			; Read He ratio
	movff	EEDATA,char_I_deco_He_ratio+4; And copy into hold register
	read_int_eeprom		d'22'			; Read O2 ratio
    movff	char_I_deco_He_ratio+4,WREG ; Get back He -> WREG
    addwf   EEDATA,W                    ; O2 + He -> WREG
    sublw   d'100'                      ; 100 - (O2 + He) -> WREG
    movff	WREG, char_I_deco_N2_ratio+4; Push N2 to C code;

; Copy depth of enabled gas into char_I_deco_gas_change
; Note gaslist_active is inited in diveloop_boot, and edited by
; the de-activation menu. So don't reread it from eeprom here.

	read_int_eeprom		d'28'				; read gas_change_depth Gas1
	btfss	gaslist_active,0                ; Apply depth?
	clrf	EEDATA							; No, clear!
	movff	EEDATA,char_I_deco_gas_change+0	; Yes!

	read_int_eeprom		d'29'				; read gas_change_depth Gas2
	btfss	gaslist_active,1    			; Apply depth?
	clrf	EEDATA							; No, clear!
	movff	EEDATA,char_I_deco_gas_change+1	; Yes!

	read_int_eeprom		d'30'				; read gas_change_depth Gas3
	btfss	gaslist_active,2    			; Apply depth?
	clrf	EEDATA							; No, clear!
	movff	EEDATA,char_I_deco_gas_change+2	; Yes!

	read_int_eeprom		d'31'				; read gas_change_depth Gas4
	btfss	gaslist_active,3    			; Apply depth?
	clrf	EEDATA							; No, clear!
	movff	EEDATA,char_I_deco_gas_change+3	; Yes!

	read_int_eeprom		d'32'				; read gas_change_depth Gas5
	btfss	gaslist_active,4    			; Apply depth?
	clrf	EEDATA							; No, clear!
	movff	EEDATA,char_I_deco_gas_change+4	; Yes!
	return

set_reset_safety_stop:						; Set flags for safety stop and/or reset safety stop
 	GETCUSTOM8	d'65'						; Use safety stop
	decfsz		WREG,F						; WREG=1?	
	bra			delete_safety_stop			; No, don't show safety stop

	btfsc	dekostop_active					; Is a deco stop displayed?
	bra		delete_safety_stop				; Yes, don't show safety stop
	; Below "safety_stop_reset"? Set flag and reset count-down timer
    SAFE_2BYTE_COPY rel_pressure, lo
	call	adjust_depth_with_salinity		; computes salinity setting into lo:hi [mbar]
	movff	lo,sub_a+0
	movff	hi,sub_a+1
    GETCUSTOM8  .73                         ; Safety Stop Reset Depth [dm]
    mullw   .10
    movff   PRODL,sub_b+0
    movff   PRODH,sub_b+1
	call	subU16							;  sub_c = sub_a - sub_b
	btfss	neg_flag
	bra		reset_safety_stop				; Below 10m, reset safety stop

	; Above "safety_stop_end"? Clear flag.
    SAFE_2BYTE_COPY rel_pressure, lo
	call	adjust_depth_with_salinity		; computes salinity setting into lo:hi [mbar]
	movff	lo,sub_a+0
	movff	hi,sub_a+1
    GETCUSTOM8  .72                         ; Safety Stop End Depth [dm]
    mullw   .10
    movff   PRODL,sub_b+0
    movff   PRODH,sub_b+1
	call	subU16							;  sub_c = sub_a - sub_b
	btfsc	neg_flag
	bra		delete_safety_stop				; Above 3m, remove safety stop

	; Above "safety_stop_start"? Activate safety stop
    SAFE_2BYTE_COPY rel_pressure, lo
	call	adjust_depth_with_salinity		; computes salinity setting into lo:hi [mbar]
	movff	lo,sub_a+0
	movff	hi,sub_a+1
    GETCUSTOM8  .71                         ; Safety Stop Start Depth [dm]
    mullw   .10
    movff   PRODL,sub_b+0
    movff   PRODH,sub_b+1
	call	subU16							;  sub_c = sub_a - sub_b
	btfsc	neg_flag
	bra		acivate_safety_stop				; Above 5m, activate safety stop

	bra		reset_safety_stop2				; Pause safety stop	

acivate_safety_stop:
	tstfsz	safety_stop_countdown			; Countdown at zero?
	bsf		show_safety_stop				; No, Set flag!
	return

delete_safety_stop:
	clrf	safety_stop_countdown			; reset timer
	bra		reset_safety_stop2				; Remove saftestop from display

reset_safety_stop:
    GETCUSTOM8  .70                         ; Safety Stop Duration [s]
	movwf	safety_stop_countdown			; reset timer
reset_safety_stop2:
	btfss	safety_stop_active				; Safety stop shown
	return									; No, don't delete it
	bcf		show_safety_stop				; Clear flag
	bcf		safety_stop_active				; Clear flag
    call	DISP_clear_decoarea				; Yes, Clear stop
	goto	DISP_display_ndl_mask			; Show NDL again (And return)

;-----------------------------------------------------------------------------
; calculate ppO2 in 0.01bar (e.g. 150 = 1.50 bar ppO2)
set_actual_ppo2:
    SAFE_2BYTE_COPY amb_pressure, xA    ; P_amb in millibar (1000 = 1.00 bar).
	movlw		d'10'
	movwf		xB+0
	clrf		xB+1
	call		div16x16				; xC=p_amb/10 (100 = 1.00 bar).
	movff		xC+0,xA+0
	movff		xC+1,xA+1
	movff		char_I_O2_ratio,xB+0
	clrf		xB+1
	call		mult16x16				; char_I_O2_ratio * (p_amb/10)
	movff		xC+0,xA+0
	movff		xC+1,xA+1
	movlw		d'100'
	movwf		xB+0
	clrf		xB+1
	call		div16x16				; xC=(char_I_O2_ratio * p_amb/10)/100

; Copy ppO2 for CNS calculation
    tstfsz      xC+1                    ; Is ppO2 > 2.55bar ?
    setf        xC+0                    ; yes: bound to 2.55... better than wrap around.

    movff		xC+0, char_I_actual_ppO2	; copy last ppO2 to buffer register
    btfsc		FLAG_const_ppO2_mode		; do in const_ppO2_mode
    movff		char_I_const_ppO2, char_I_actual_ppO2	; copy last ppO2 to buffer register
    return

reset_decompression_gases:				; reset the deco gas while in NDL
	ostc_debug	'F'		; Sends debug-information to screen if debugmode active
  	lfsr    FSR2,char_I_deco_gas_change
  	clrf    POSTINC2                    ; Clear Gas1
  	clrf    POSTINC2
  	clrf    POSTINC2
  	clrf    POSTINC2
  	clrf    POSTINC2                    ; Clear Gas5
	return

calc_deko_divemode2:
	bcf		twosecupdate		

	btfsc	gauge_mode				; ignore decompression calculation in gauge mode
	return
	btfsc	FLAG_apnoe_mode			; ignore decompression calculation in apnoe mode
	return

 	ostc_debug	'B'		; Sends debug-information to screen if debugmode active
    ; Send nes state to screen, if debugmode active	
	movff   char_O_deco_status,WREG ; Status before call
	addlw   '0'                     ; Convert to ascii char
	call    ostc_debug1             ; and send.

	call	divemode_prepare_flags_for_deco
	clrf	WREG
	movff	WREG,char_I_step_is_1min    ; Force 2 second deco mode

	movff	char_I_O2_ratio,lo_temp		; Backup original value for everything
	movff	char_I_N2_ratio,hi_temp		; Backup original value for everything

	read_int_eeprom d'34'				; Read deco data
	movlw		.6	
	cpfseq		EEDATA
	bra			calc_deko_divemode2a
	; in PSCR mode, compute fO2 into char_I_O2_ratio
	call		compute_pscr_ppo2		; pSCR ppO2 into sub_c:2
	movff		sub_c+0,xA+0
	movff		sub_c+1,xA+1
	movlw		LOW		.10
	movwf		xB+0
	movlw		HIGH	.10
	movwf		xB+1
	call		mult16x16		;xA*xB=xC	-> xC:4 = ppO2*10
	SAFE_2BYTE_COPY amb_pressure, xB
	call		div32x16	 	; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder
	; xC+0 has O2 in percent
	movff	xC+0,char_I_O2_ratio

	movff	char_I_He_ratio, wait_temp	; copy into bank1 register
	bsf		STATUS,C					; Borrow bit
	movlw	d'100'						; 100%
	subfwb	wait_temp,W					; minus He
	bsf		STATUS,C					; Borrow bit
	subfwb	xC+0,W						; minus O2
	movff	WREG, char_I_N2_ratio		; = N2!
	
calc_deko_divemode2a:

	clrf	TMR3L
	nop									; See errata DS80284E-page 2
	clrf	TMR3H						; Reset Timer3

	call	deco_calc_hauptroutine		; calc_tissue
	movlb	b'00000001'					; rambank 1 selected
	ostc_debug	'C'		; Sends debug-information to screen if debugmode active

	movff	lo_temp,char_I_O2_ratio		; Restore original value for everything
	movff	hi_temp,char_I_N2_ratio		; Restore original value for everything

	btfss	debug_mode				; Are we in debugmode?
	bra		calc_deko_divemode4		; No...

; Show timer3 reading converted into ms (Error: +2,3%) in custom area
	movff		TMR3L,lo
	movff		TMR3H,hi
	bcf			STATUS,C
	rrcf		hi
	rrcf		lo			;/2
	bcf			STATUS,C
	rrcf		hi
	rrcf		lo			;/4
	bcf			STATUS,C
	rrcf		hi
	rrcf		lo			;/8
	bcf			STATUS,C
	rrcf		hi
	rrcf		lo			;/16
	bcf			STATUS,C
	rrcf		hi
	rrcf		lo			;/32

	WIN_TOP		.216
	WIN_LEFT	.100
	WIN_FONT	FT_SMALL
   	STRCPY  "ms:"
	output_16
	call	word_processor

calc_deko_divemode4:
    movff   char_O_deco_status,WREG     ; Is a compute cycle finished ?
    iorwf   WREG,F
    btfss   STATUS,Z
    return                              ; Return is status <> 0

    ; Check if deco stops are necessary ?
    bcf     decoplan_invalid            ; The decoplan is valid
	movff	char_O_first_deco_depth,wait_temp	; copy ceiling to temp register
	tstfsz	wait_temp							; Ceiling<0m?
	bra		calc_deko_divemode3					; Yes!

	btfsc	dekostop_active             ; Already in nodeco mode ?
	call	DISP_display_ndl_mask       ; Clear deco data, display nostop time
	bcf		dekostop_active             ; clear flag

	clrf	decodata+0                  ; for profile memory
	movff	char_O_nullzeit,decodata+1  ; nostop time
	
	call	DISP_display_ndl            ; display no deco limit

calc_deko_divemode5:
    ; Check if extra cycles are needed to compute @5 variant:
    btfss   tts_extra_time              ; Is @5 displayed ?
	return                              ; No: don't compute it.
	
	decfsz  apnoe_mins                  ; Reached count-down ?
	return                              ; No: don't compute yet.
	
	movlw   .6
	movff   WREG,char_O_deco_status     ; Stole next cycles for @5 variant.
    
    movlw   .2                          ; Restart countdown.
    movwf   apnoe_mins
    return

calc_deko_divemode3:
	btfss	dekostop_active             ; Already in deco mode ?
	call	DISP_display_deko_mask      ; clear nostop time, display decodata
	bsf		dekostop_active             ; Set flag

	movff	char_O_first_deco_depth,decodata+0	; ceiling
	movff	char_O_first_deco_time,decodata+1	; length of first stop in minues

	call	DISP_display_deko           ; display decodata
    bra     calc_deko_divemode5
    
;-----------------------------------------------------------------------------

divemode_prepare_flags_for_deco:
    SAFE_2BYTE_COPY amb_pressure,int_I_pres_respiration ; copy result to deco routine

	GETCUSTOM8	d'11'                           ; Saturation multiplier %
	movff	WREG,char_I_saturation_multiplier
	GETCUSTOM8	d'12'                           ; Desaturation multiplier %
	movff	WREG,char_I_desaturation_multiplier
	GETCUSTOM8	d'16'                           ; Deco distance to decostop in 1/10 meter for simulation
	movff	WREG,char_I_deco_distance
	GETCUSTOM8	d'29'                           ; Depth Last Deco in meter for simulation
	movff	WREG,char_I_depth_last_deco
	movff   divemins+0,int_I_divemins+0         ; Current dive time.
	movff   divemins+1,int_I_divemins+1
	goto	restart_set_modes_and_flags			; Sets decomode (char_I_deco_model) and flags (again)

;-----------------------------------------------------------------------------

store_dive_data:						; CF20 seconds gone
	bcf		store_sample				; update only any CF20 seconds
	bsf		update_divetime				; update divemins every CF20 seconds

    ifndef __DEBUG
    	btfsc	simulatormode_active    ; Are we in simulator mode?
    	return                          ; Yes, discard everything
    endif

	btfsc	header_stored				; Header already stored?
	bra		store_dive_data2			; Yes, store only profile data
	bsf		header_stored				; Store header
	
	movff	eeprom_address+0, eeprom_header_address+0	; store startposition
	movff	eeprom_address+1, eeprom_header_address+1	; store startposition

; shift address for header
; the header will be stored after the dive
	incf_eeprom_address	d'57'				; Macro, that adds 8Bit to eeprom_address:2

store_dive_data2:
    SAFE_2BYTE_COPY rel_pressure, lo
	movf	lo,W				        ; store depth with every sample
	call	write_external_eeprom
	movf	hi,W
	call	write_external_eeprom

;First, find out how many bytes will append to this sample....
	clrf	ProfileFlagByte					; clear number of bytes to append

; Check Extented informations
	decfsz	divisor_temperature,W	; Check divisor
	bra		check_extended1		
	movlw	d'2'				; Information length	
	addwf	ProfileFlagByte,F	; add to ProfileFlagByte
check_extended1:
	decfsz	divisor_deco,W		; Check divisor
	bra		check_extended2		
	movlw	d'2'				; Information length	
	addwf	ProfileFlagByte,F	; add to ProfileFlagByte
check_extended2:
	decfsz	divisor_gf,W		; Check divisor
	bra		check_extended3		
	movlw	d'1'				; Information length	
	addwf	ProfileFlagByte,F	; add to ProfileFlagByte
check_extended3:
	decfsz	divisor_ppo2,W		; Check divisor
	bra		check_extended4		
	movlw	d'3'				; Information length	
	addwf	ProfileFlagByte,F	; add to ProfileFlagByte
check_extended4:
	decfsz	divisor_deco_debug,W; Check divisor
	bra		check_extended5		
	movlw	d'15'				; Information length
	addwf	ProfileFlagByte,F	; add to ProfileFlagByte
check_extended5:
	decfsz	divisor_cns,W		; Check divisor
	bra		check_extended6		
	movlw	d'1'				; Information length	
	addwf	ProfileFlagByte,F	; add to ProfileFlagByte
check_extended6:

; Second, check global event flag
	btfss	event_occured		; Check global event flag
	bra		store_dive_data3	; No Event
	movlw	d'1'
	addwf	ProfileFlagByte,F	; add one byte (The EventByte)

	clrf	EventByte			; reset EventByte

	movf	AlarmType,W			; Type of alarm Bit 0-3
	addwf	EventByte,F			; Copy to EventByte Bit 0-3
	clrf	AlarmType			; Reset AlarmType
	
; Third, check events and add aditional bytes
	btfss	manual_gas_changed	; Check flag	
	bra		check_event1
	movlw	d'2'				; Information length	
	addwf	ProfileFlagByte,F	; add to ProfileFlagByte
	bsf		EventByte,4			; Also set Flag in EventByte!
check_event1:
	btfss	stored_gas_changed	; Check flag	
	bra		check_event2
	movlw	d'1'				; Information length	
	addwf	ProfileFlagByte,F	; add to ProfileFlagByte
	bsf		EventByte,5			; Also set Flag in EventByte!
check_event2:
	btfss	setpoint_changed	; Check flag	
	bra		check_event3
	movlw	d'1'				; Information length	
	addwf	ProfileFlagByte,F	; add to ProfileFlagByte
	bsf		EventByte,6			; Also set Flag in EventByte!
check_event3:
    btfss	store_bailout_event ; Check flag
	bra		check_event4
	movlw	d'2'				; Information length
	addwf	ProfileFlagByte,F	; add to ProfileFlagByte
	bsf		EventByte,7			; Also set Flag in EventByte!
check_event4:
	bsf		ProfileFlagByte,7	; Set EventByte Flag in ProfileFlagByte

store_dive_data3:
	movf	ProfileFlagByte,W	; finally, write ProfileFlagByte!
	call	write_external_eeprom

	btfss	event_occured		; Check global event flag (again)
	bra		store_dive_data4	; No Event

; Store the EventByte + additional bytes now
	movf	EventByte,W		
	call	write_external_eeprom

	btfss	manual_gas_changed		; Check flag	
	bra		store_dive_data3a
	read_int_eeprom	d'24'			; % O2 Gas6
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom	d'25'			; % He Gas6
	movf	EEDATA,W
	call	write_external_eeprom
	bcf		manual_gas_changed		; Clear this event

store_dive_data3a:
	btfss	stored_gas_changed	; Check flag	
	bra		store_dive_data3b			
	movf	active_gas,W		; Store active gas
	call	write_external_eeprom
	bcf		stored_gas_changed	; Clear this event
store_dive_data3b:

store_dive_data4:

; Store extended informations
	decfsz	divisor_temperature,F	; Check divisor
	bra		store_extended1	
	rcall	store_dive_temperature
store_extended1:
	decfsz	divisor_deco,F		; Check divisor
	bra		store_extended2	
	rcall	store_dive_decodata
store_extended2:
	decfsz	divisor_gf,F		; Check divisor
	bra		store_extended3	
	rcall	store_dive_gf
store_extended3:
	decfsz	divisor_ppo2,F		; Check divisor
	bra		store_extended4	
	rcall	store_dive_ppo2
store_extended4:
	decfsz	divisor_deco_debug,F; Check divisor
	bra		store_extended5	
	rcall	store_dive_decodebug
store_extended5:
	decfsz	divisor_cns,F		; Check divisor
	bra		store_extended6	
	rcall	store_dive_cns
store_extended6:

; The next block is required to take care of "store never"
	btfsc	divisor_temperature,7	; Test highest Bit (Register must have been zero before the "decfsz" command!)
	clrf	divisor_temperature		; And clear register again, so it will never reach zero...
	btfsc	divisor_deco,7			; Test highest Bit (Register must have been zero before the "decfsz" command!)
	clrf	divisor_deco			; And clear register again, so it will never reach zero...
	btfsc	divisor_gf,7			; Test highest Bit (Register must have been zero before the "decfsz" command!)
	clrf	divisor_gf				; And clear register again, so it will never reach zero...
	btfsc	divisor_ppo2,7			; Test highest Bit (Register must have been zero before the "decfsz" command!)
	clrf	divisor_ppo2			; And clear register again, so it will never reach zero...
	btfsc	divisor_deco_debug,7	; Test highest Bit (Register must have been zero before the "decfsz" command!)
	clrf	divisor_deco_debug		; And clear register again, so it will never reach zero...
	btfsc	divisor_cns,7			; Test highest Bit (Register must have been zero before the "decfsz" command!)
	clrf	divisor_cns				; And clear register again, so it will never reach zero...

	ostc_debug	'D'		; Sends debug-information to screen if debugmode active

; SetPoint change appended to information due to compatibility reasons
	btfss	setpoint_changed		; Check flag	
	bra		store_dive_data5
	movff	ppO2_setpoint_store,WREG	; SetPoint in cbar
	call	write_external_eeprom
	bcf		setpoint_changed		; Clear this event

store_dive_data5:
    btfss	store_bailout_event     ; Check flag
    bra		store_dive_data6
    movff   char_I_O2_ratio,WREG
    call	write_external_eeprom
    movff   char_I_He_ratio,WREG
    call	write_external_eeprom
    bcf		store_bailout_event		; Clear this event
store_dive_data6:
	bcf		event_occured		; Clear the global event flag
	return						; Done. (Sample with all informations written to EEPROM)
	
store_dive_cns:
	movff	char_O_CNS_fraction,WREG
	call	write_external_eeprom		; Store in EEPROM
	GETCUSTOM8	d'26'
	movwf	divisor_cns			; Reload divisor from CF
	return

store_dive_decodebug:
    movff   char_O_deco_time_for_log+.0,WREG     ; 3m
    call	write_external_eeprom
    movff   char_O_deco_time_for_log+.1,WREG
    call	write_external_eeprom
    movff   char_O_deco_time_for_log+.2,WREG
    call	write_external_eeprom
    movff   char_O_deco_time_for_log+.3,WREG
    call	write_external_eeprom
    movff   char_O_deco_time_for_log+.4,WREG
    call	write_external_eeprom
    movff   char_O_deco_time_for_log+.5,WREG
    call	write_external_eeprom
    movff   char_O_deco_time_for_log+.6,WREG
    call	write_external_eeprom
    movff   char_O_deco_time_for_log+.7,WREG
    call	write_external_eeprom
    movff   char_O_deco_time_for_log+.8,WREG
    call	write_external_eeprom
    movff   char_O_deco_time_for_log+.9,WREG
    call	write_external_eeprom
    movff   char_O_deco_time_for_log+.10,WREG
    call	write_external_eeprom
    movff   char_O_deco_time_for_log+.11,WREG
    call	write_external_eeprom
    movff   char_O_deco_time_for_log+.12,WREG
    call	write_external_eeprom
    movff   char_O_deco_time_for_log+.13,WREG
    call	write_external_eeprom
    movff   char_O_deco_time_for_log+.14,WREG   ; 45m
    call	write_external_eeprom
	GETCUSTOM8	d'25'
	movwf	divisor_deco_debug			; Reload divisor from CF
	return

store_dive_ppo2:
#IFDEF CCR_CTRL
    ; Store actual ppO2 values of the sensors into Logbook.
	movff	eCTRL_ppo2_s1, WREG			; Sensor 1
	call	write_external_eeprom
	movff	eCTRL_ppo2_s2, WREG			; Sensor 2
	call	write_external_eeprom
	movff	eCTRL_ppo2_s3, WREG			; Sensor 3
	call	write_external_eeprom
#ELSE
	movlw	0x00			; Dummy
	call	write_external_eeprom
	movlw	0x00			; Dummy
	call	write_external_eeprom
	movlw	0x00			; Dummy
	call	write_external_eeprom
#ENDIF
	GETCUSTOM8	d'24'
	movwf	divisor_ppo2			; Reload divisor from CF
	return

store_dive_gf:
	movff	char_O_gradient_factor,WREG		; gradient factor absolute
	call	write_external_eeprom
	GETCUSTOM8	d'23'
	movwf	divisor_gf			; Reload divisor from CF
	return

store_dive_decodata:
	movf	decodata+0,W				; =0:no stop dive, if in deco mode: ceiling in m
	call	write_external_eeprom
	movf	decodata+1,W				; no stop time of length of first stop
	call	write_external_eeprom
	GETCUSTOM8	d'22'
	
	btfsc	FLAG_apnoe_mode				; in Apnoe mode?
	movlw	d'0'					; Yes, set to zero
	movwf	divisor_deco			; Reload divisor from CF
	return

store_dive_temperature:
    SAFE_2BYTE_COPY temperature,lo
	movf	lo,W			            ; append temperature to current sample!
	call	write_external_eeprom
	movf	hi,W
	call	write_external_eeprom
	GETCUSTOM8	d'21'
	movwf	divisor_temperature			; Reload divisor from CF
	return

calc_velocity:								; called every two seconds
	btfss	divemode						
	bra		do_not_display_velocity			; display velocity only in divemode

calc_velocity2:
    SAFE_2BYTE_COPY amb_pressure, sub_a
	movff	last_pressure+0,sub_b+0
	movff	last_pressure+1,sub_b+1
	movff	sub_a+0,last_pressure+0	; store old value for velocity
	movff	sub_a+1,last_pressure+1

	call	sub16						; sub_c = amb_pressure - last_pressure

	movff	sub_c+0,xA+0
	movff	sub_c+1,xA+1
	movlw	d'39'						;77 when called every second....
	movwf	xB+0
	clrf	xB+1
	call	mult16x16					; differential pressure in mbar*77...
	movff	xC+0,divA+0
	movff	xC+1,divA+1
	movlw	d'7'
	movwf	divB
	call	div16						; devided by 2^7 equals velocity in m/min

	movlw	d'99'
	cpfsgt	divA						; Limit to 99m/min max.
	bra		calc_velocity3
	movwf	divA						; divA=99

calc_velocity3:
	GETCUSTOM8	d'5'					; threshold for display vertical velocity
	subwf	divA+0,W					; 

	btfss	STATUS,C
	bra		do_not_display_velocity

update_velocity:
	bsf		display_velocity

	GETCUSTOM8	d'60'		; use graphic velocity (=1)?
	movwf	lo
	movlw	d'1'
	cpfseq	lo					; =1?
	bra		update_velocity1	; No

	call	DISP_display_velocity_graphical
	return

update_velocity1:
	call	DISP_display_velocity
	return

do_not_display_velocity:
	btfss	display_velocity			; Velocity was not displayed, do not delete
	return
		
	bcf		display_velocity			; Velocity was displayed, delete velocity now
	call	DISP_display_velocity_clear
	return

check_ppO2:							    ; check current ppO2 and display warning if required
	btfsc		FLAG_const_ppO2_mode    ; ignore in ppO2 mode....
	return

check_ppO2_bail:						; In CC mode but bailout active!
	read_int_eeprom d'34'				; Read deco data
	movlw		.6	
	cpfseq		EEDATA
	bra			check_ppO2_non_pscr
	; in PSCR mode
	call		compute_pscr_ppo2		; pSCR ppO2 into sub_c:2
    movff		sub_c+0,xA+0
    movff		sub_c+1,xA+1
	movlw		d'100'
	movwf		xB+0
	clrf		xB+1
	call		div16x16				; /100
    tstfsz      xC+1                    ; Is ppO2 > 2.55bar ?
    setf        xC+0                    ; yes: bound to 2.55... better than wrap around.
    movff		xC+0,char_I_actual_ppO2	; copy last ppO2 to buffer register (for pSCR CNS)
	clrf		xC+2
	clrf		xC+3
	movff		sub_c+0,xC+0
	movff		sub_c+1,xC+1			; copy for comptibility
	bra			check_ppO2_check

check_ppO2_non_pscr:
    SAFE_2BYTE_COPY amb_pressure, xA
	movlw		d'10'
	movwf		xB+0
	clrf		xB+1
	call		div16x16				; xC=p_amb/10
	movff		xC+0,xA+0
	movff		xC+1,xA+1
	movff		char_I_O2_ratio,xB+0
	clrf		xB+1
	call		mult16x16				; char_I_O2_ratio * p_amb/10 -> xC:4
;	bra			check_ppO2_check
check_ppO2_check:
; Check very high ppO2 manually
	tstfsz		xC+2					; char_I_O2_ratio * p_amb/10 > 65536, ppO2>6,55bar?
	bra			check_ppO2_bail2		; Yes, display Value!

; Check if ppO2 should be displayed
	movff		xC+0,sub_b+0
	movff		xC+1,sub_b+1
	GETCUSTOM8	d'19'					; ppo2_display_high
	mullw		d'100'					; ppo2_display_high*100
	movff		PRODL,sub_a+0
	movff		PRODH,sub_a+1
	call		sub16					
	bcf			ppO2_show_value		; clear flag
	btfsc		neg_flag
	bsf			ppO2_show_value		; set flag if required

;check if we are within our warning thresholds!
	movff		xC+0,sub_b+0
	movff		xC+1,sub_b+1
	GETCUSTOM8	d'18'					; ppo2_warning_high
	mullw		d'100'					; ppo2_warning_high*100
	movff		PRODL,sub_a+0
	movff		PRODH,sub_a+1
	call		sub16					
	btfss		neg_flag
	bra			check_ppO2_0		; Not too high

check_ppO2_bail2:
	bsf			ppO2_show_value		; set flag if required
	movlw		d'5'				; Type of alarm (ppO High Warning)
	movwf		AlarmType			; Copy to alarm Register
	bsf			event_occured		; Set Event Flag

check_ppO2_0:
	movff		xC+0,sub_b+0
	movff		xC+1,sub_b+1
	GETCUSTOM8	d'17'				; ppo2_warning_low
	mullw		d'100'				; ppo2_warning_low*100
	movff		PRODL,sub_a+0
	movff		PRODH,sub_a+1
	call		sub16					
	btfsc		neg_flag
	bra			check_ppO2_1		; Not too low

	bsf			ppO2_show_value		; show ppO2 if below threshold!
	movlw		d'4'				; Type of alarm (ppO Low Warning)
	movwf		AlarmType			; Copy to alarm Register
	bsf			event_occured		; Set Event Flag

check_ppO2_1:
	btfsc		ppO2_show_value		; show value?
	bra			check_ppO2_2		; yes!

	btfss		ppO2_display_active	; is the value displayed?
	bra			check_ppO2_3		; No, so clear not required
	
	call		DISP_show_ppO2_clear; Clear ppO2 value
	bcf			ppO2_display_active	; clear flag
	bra			check_ppO2_3		; done

check_ppO2_2:
	call		DISP_show_ppO2		; Display ppO2 (stored in xC)
	bsf			ppO2_display_active	; Set flag		

check_ppO2_3:
	return		; done

;=============================================================================
; Compare all enabled gas in list, to see if a better one is available.
;
; Output: better_gas_available, better_gas_number
;
check_gas_change:					; Checks if a better gas should be selected (by user)
	bcf		better_gas_available	;=1: A better gas is available and a gas change is advised in divemode

    SAFE_2BYTE_COPY rel_pressure,xA
	movlw	d'100'
	movwf	xB+0
	clrf	xB+1
	call	div16x16				; compute depth in full m -> result in xC+0
#IFDEF CCR_CTRL
    ; Save actual Depth into variable
;	movf	xC+0,W					; load depth in m into WREG
    movff   xC+0, eCTRL_DEPTH       ; Copy depth in m into register
#ENDIF

check_gas_change1:					; check gas1 
	btfss	gaslist_active,0		; check active flag
	bra		check_gas_change2		; skip inactive gases!
	movlw	d'1'
	cpfseq	active_gas				; is this gas currently selected?
	bra		check_gas_change1x		; No...
	bra		check_gas_change2		; Yes, skip depth check
check_gas_change1x:	
	read_int_eeprom		d'28'		; read gas_change_depth
	movlw	minimum_change_depth
	cpfsgt	EEDATA					; Change depth>minimum_change_depth?
	bra		check_gas_change2		; No, Change depth not deep enough, skip!
	movf	xC+0,W					; load depth in m into WREG
	cpfsgt	EEDATA					; gas_change_depth < current depth?
	bra		check_gas_change2		; No, check next gas
	movlw	.1
	movwf	better_gas_number		; number (1-5) of the "better gas" in divemode, =0: no better gas available
	movlw	better_gas_window
	subwf	EEDATA,W				; Change depth-better_gas_window
	cpfslt	xC+0					; current depth<Change depth-better_gas_window?
	bsf		better_gas_available	;=1: A better gas is available and a gas change is advised in divemode

check_gas_change2:					; check gas2
	btfss	gaslist_active,1        ; check active flag
	bra		check_gas_change3		; skip inactive gases!
	movlw	d'2'
	cpfseq	active_gas				; is this gas currently selected?
	bra		check_gas_change2x		; No...
	bra		check_gas_change3		; Yes, skip depth check
check_gas_change2x:	
	read_int_eeprom		d'29'		; read gas_change_depth
	movlw	minimum_change_depth
	cpfsgt	EEDATA					; Change depth>minimum_change_depth?
	bra		check_gas_change3		; No, Change depth not deep enough, skip!
	movf	xC+0,W					; load depth in m into WREG
	cpfsgt	EEDATA					; gas_change_depth < current depth?
	bra		check_gas_change3		; No, check next gas
	movlw	.2
	movwf	better_gas_number		; number (1-5) of the "better gas" in divemode, =0: no better gas available
	movlw	better_gas_window
	subwf	EEDATA,W				; Change depth-better_gas_window
	cpfslt	xC+0					; current depth<Change depth-better_gas_window?
	bsf		better_gas_available	;=1: A better gas is available and a gas change is advised in divemode

check_gas_change3:					; check gas3
	btfss	gaslist_active,2        ; check active flag
	bra		check_gas_change4		; skip inactive gases!
	movlw	d'3'
	cpfseq	active_gas				; is this gas currently selected?
	bra		check_gas_change3x		; No...
	bra		check_gas_change4		; Yes, skip depth check
check_gas_change3x:	
	read_int_eeprom		d'30'		; read gas_change_depth
	movlw	minimum_change_depth
	cpfsgt	EEDATA					; Change depth>minimum_change_depth?
	bra		check_gas_change4		; No, Change depth not deep enough, skip!
	movf	xC+0,W					; load depth in m into WREG
	cpfsgt	EEDATA					; gas_change_depth < current depth?
	bra		check_gas_change4		; No, check next gas
	movlw	.3
	movwf	better_gas_number		; number (1-5) of the "better gas" in divemode, =0: no better gas available
	movlw	better_gas_window
	subwf	EEDATA,W				; Change depth-better_gas_window
	cpfslt	xC+0					; current depth<Change depth-better_gas_window?
	bsf		better_gas_available	;=1: A better gas is available and a gas change is advised in divemode

check_gas_change4:					; check gas4
	btfss	gaslist_active,3        ; check active flag
	bra		check_gas_change5		; skip inactive gases!
	movlw	d'4'
	cpfseq	active_gas				; is this gas currently selected?
	bra		check_gas_change4x		; No...
	bra		check_gas_change5		; Yes, skip depth check
check_gas_change4x:	
	read_int_eeprom		d'31'		; read gas_change_depth
	movlw	minimum_change_depth
	cpfsgt	EEDATA					; Change depth>minimum_change_depth?
	bra		check_gas_change5		; No, Change depth not deep enough, skip!
	movf	xC+0,W					; load depth in m into WREG
	cpfsgt	EEDATA					; gas_change_depth < current depth?
	bra		check_gas_change5		; No, check next gas
	movlw	.4
	movwf	better_gas_number		; number (1-5) of the "better gas" in divemode, =0: no better gas available
	movlw	better_gas_window
	subwf	EEDATA,W				; Change depth-better_gas_window
	cpfslt	xC+0					; current depth<Change depth-better_gas_window?
	bsf		better_gas_available	;=1: A better gas is available and a gas change is advised in divemode

check_gas_change5:					; check gas5
	btfss	gaslist_active,4        ; check active flag
	bra		check_gas_change6		; skip inactive gases!
	movlw	d'5'
	cpfseq	active_gas				; is this gas currently selected?
	bra		check_gas_change5x		; No...
	bra		check_gas_change6		; Yes, skip depth check
check_gas_change5x:	
	read_int_eeprom		d'32'		; read gas_change_depth
	movlw	minimum_change_depth
	cpfsgt	EEDATA					; Change depth>minimum_change_depth?
	bra		check_gas_change6		; No, Change depth not deep enough, skip!
	movf	xC+0,W					; load depth in m into WREG
	cpfsgt	EEDATA					; gas_change_depth < current depth?
	bra		check_gas_change6		; No, check next gas
	movlw	.5
	movwf	better_gas_number		; number (1-5) of the "better gas" in divemode, =0: no better gas available
	movlw	better_gas_window
	subwf	EEDATA,W				; Change depth-better_gas_window
	cpfslt	xC+0					; current depth<Change depth-better_gas_window?
	bsf		better_gas_available	;=1: A better gas is available and a gas change is advised in divemode

check_gas_change6:
	btfss	better_gas_available	; Is a better gas available
	clrf	better_gas_number		; No, clear better gas register

	btfsc	is_bailout				;=1: CC mode, but bailout active!
	bra		check_gas_change7		; In bailout, blink better gas (if required)

	btfsc	FLAG_const_ppO2_mode	; in ppO2 mode?
	bcf		better_gas_available	; Yes, do not blink better gas

check_gas_change7:
	call	DISP_active_gas_divemode; Display gas, if required (and with "*" if required...)
	return

;=============================================================================

calculate_noflytime:
	; calculate nofly time
	movff	int_O_desaturation_time+0,xA+0
	movff	int_O_desaturation_time+1,xA+1

    btfsc   xA+1,7                  ; Is desat time negatif ?
    bra     calculate_noflytime_3   ; Then surely not valid !

	tstfsz	xA+0			; Desat=0?
	bra		calculate_noflytime2
	tstfsz	xA+1			; Desat=0?
	bra		calculate_noflytime2

calculate_noflytime_3:	
	; Desaturation time = zero
	clrf	nofly_time+0			; Clear nofly time
	clrf	nofly_time+1			; Clear nofly time
	bcf		nofly_active			; Clear flag
	return

calculate_noflytime2:	
	movff	xA+0,int_I_temp+0
	movff	xA+1,int_I_temp+1
	GETCUSTOM8 	.13					; nofly_time_ratio
	movff	WREG,char_I_temp
	ostc_debug	'K'		; Sends debug-information to screen if debugmode active
	call	deco_calc_percentage
	movlb	b'00000001'				; select ram bank 1
	ostc_debug	'L'		; Sends debug-information to screen if debugmode active
	movff	int_I_temp+0,xA+0
	movff	int_I_temp+1,xA+1
	tstfsz	xA+0			; Desat=0?
	bra		calculate_noflytime_2_final
	tstfsz	xA+1			; Desat=0?
	bra		calculate_noflytime_2_final
	bra     calculate_noflytime_3

calculate_noflytime_2_final:
	movff	xA+0,nofly_time+0
	movff	xA+1,nofly_time+1
	bsf		nofly_active			; Set flag
	return

end_dive:

#IFDEF CCR_CTRL
    ; Don't disable UART in the end of dive if in CCR mode
    btfss   ext_ppO2_enable     ; Check for eCTRL mode
    call    disable_rs232       ; Disable if not in eCTRL mode
#ENDIF

	btfss	realdive					; dive longer then one minute
	goto	end_dive_common				; No, discard everything

; In DEBUG compile, keep all simulated dives in logbook, Desat time, nofly, etc...
    ifndef __DEBUG
    	btfsc	simulatormode_active		; Are we in simulator mode?
    	goto	end_dive_common_sim         ; Yes, discard everything
    endif

	; Dive finished (and longer then one minute or Apnoe timeout occured)

	btfsc	FLAG_apnoe_mode			; Calc max. depth (again) for very short apnoe dives
	call	apnoe_calc_maxdepth

	; calculate desaturation time
	movff	last_surfpressure_30min+0,int_I_pres_surface+0          ; Pass surface to desat routine !
	movff	last_surfpressure_30min+1,int_I_pres_surface+1

	GETCUSTOM8	d'12'                   ; Desaturation multiplier %
	movff	WREG,char_I_desaturation_multiplier

	ostc_debug	'G'		; Sends debug-information to screen if debugmode active
	call	deco_calc_desaturation_time ; calculate desaturation time
	movlb	b'00000001'                 ; select ram bank 1
    call	calc_deko_surfmode2			; work-around for nofly bug
	rcall	calculate_noflytime         ; Calc NoFly time
    movff	int_O_desaturation_time+0,desaturation_time_buffer+0
    movff	int_O_desaturation_time+1,desaturation_time_buffer+1

#IFDEF BOTTOM
	btfss	gauge_mode
	bra	end_dive0

	;Assign 24 h for counting surface interval.
	movlw	0xA0						;
    	movwf	desaturation_time_buffer+0
	movlw	0x05						;
    	movwf	desaturation_time_buffer+1

end_dive0:
#ENDIF
#IFDEF  CCR_CTRL    
    ; Add Dive time in minutes to Scruber life in internal eeprom
    call    calc_scrub_life_plus_divemins
	clrf    EEADRH                      ; Make sure to select eeprom bank 0
	movff   lo, EEDATA
	write_int_eeprom	eCTRL_SCRUB_LO  ; Scruber life Counter Lo
	movff   hi, EEDATA
	write_int_eeprom	eCTRL_SCRUB_HI  ; Scruber life Counter Hi
#ENDIF

	ostc_debug	'H'                     ; Sends debug-information to screen if debugmode active
										; store header and ...
	movlw	0xFD						; .... End-of-Profile Bytes
	call	write_external_eeprom
	movlw	0xFD
	call	write_external_eeprom
	movlw	0xFE						; This positon will be overwritten for the next profile
	call	write_external_eeprom       ; and is required to find the newest dive after a firmware reset

	movff	eeprom_header_address+0, eeprom_address+0	; set header adress
	movff	eeprom_header_address+1, eeprom_address+1	; write header

	movlw	0xFA						; Header start
	call	write_external_eeprom
	movlw	0xFA
	call	write_external_eeprom
	movlw	logbook_profile_version     ; Defined in definitions.asm
	call	write_external_eeprom
	movf	month,W                     ; Date
	call	write_external_eeprom
	movf	day,W
	call	write_external_eeprom
	movf	year,W
	call	write_external_eeprom
	movf	hours,W					; End of dive time
	call	write_external_eeprom
	movf	mins,W
	call	write_external_eeprom

	btfss	FLAG_apnoe_mode				; Store apnoe max or normal max (Which is only max from the last descent)
	bra		end_dive1					; Store normal depth

	movff	apnoe_max_pressure+0,lo
	movff	apnoe_max_pressure+1,hi
	call	adjust_depth_with_salinity			; computes salinity setting into lo:hi [mbar]
	movff	lo,apnoe_max_pressure+0
	movff	hi,apnoe_max_pressure+1
	
	movf	apnoe_max_pressure+0,W		; Max. depth
	call	write_external_eeprom
	movf	apnoe_max_pressure+1,W
	call	write_external_eeprom
	bra		end_dive2					; skip
		
end_dive1:
	movff	max_pressure+0,lo
	movff	max_pressure+1,hi
	call	adjust_depth_with_salinity			; computes salinity setting into lo:hi [mbar]
	movff	lo,max_pressure+0
	movff	hi,max_pressure+1
	
	movf	max_pressure+0,W			; Max. depth
	call	write_external_eeprom
	movf	max_pressure+1,W
	call	write_external_eeprom

end_dive2:
	movf	divemins+0,W				; divetime minutes
	call	write_external_eeprom
	movf	divemins+1,W
	call	write_external_eeprom
	movf	divesecs,W					; divetime seconds
	call	write_external_eeprom
	movf	mintemp+0,W					; minimum temperature
	call	write_external_eeprom
	movf	mintemp+1,W		
	call	write_external_eeprom
	movf	last_surfpressure_30min+0,W		; airpressure before dive
	call	write_external_eeprom
	movf	last_surfpressure_30min+1,W		
	call	write_external_eeprom
	movff	int_O_desaturation_time+0,lo			; 
	movff	int_O_desaturation_time+1,hi
	movf	lo,W						; desaturation time in minutes
	call	write_external_eeprom
	movf	hi,W						; 
	call	write_external_eeprom

	; Gases....
    btfsc   FLAG_const_ppO2_mode
    bra     end_dive3               ; Store diluents instead of gases

	read_int_eeprom 	d'6'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'7'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'10'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'11'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'14'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'15'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'18'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'19'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'22'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'23'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
    bra     end_dive4

end_dive3:
; New in 2.52 Diluents stored seperately in EEPROM Bank0
;Byte96-97:
;Diluent 1 Default (%O2,%He)
;Byte98-99:
;Diluent 1 Current (%O2,%He)
;Byte100-101:
;Diluent 2 Default (%O2,%He)
;Byte102-103:
;Diluent 2 Current (%O2,%He)
;Byte104-105:
;Diluent 3 Default (%O2,%He)
;Byte106-107:
;Diluent 3 Current (%O2,%He)
;Byte108-109:
;Diluent 4 Default (%O2,%He)
;Byte110-111:
;Diluent 4 Current (%O2,%He)
;Byte112-113:
;Diluent 5 Default (%O2,%He)
;Byte114-115:
;Diluent 5 Current (%O2,%He)
;Byte116:
;First Diluent (1-5)
	read_int_eeprom 	d'97'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'98'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'101'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'102'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'105'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'106'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'109'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'110'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'113'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom 	d'114'		; Read byte (stored in EEDATA)
	movf	EEDATA,W
	call	write_external_eeprom

end_dive4:
	read_int_eeprom	d'24'			; % O2 Gas6
	movf	EEDATA,W
	call	write_external_eeprom
	read_int_eeprom	d'25'			; % He Gas6
	movf	EEDATA,W
	call	write_external_eeprom

    movlw   .33                     ; Start gas
    btfsc   FLAG_const_ppO2_mode    ; In CCR mode?
    movlw   .115                    ; Yes, use start Diluent instead
    movwf   EEADR
    call    read_eeprom
	movf	EEDATA,W
	call	write_external_eeprom

	movlw	softwareversion_x			; Firmware version
	call	write_external_eeprom
	movlw	softwareversion_y
	call	write_external_eeprom
	movf	batt_voltage+0,W			; Battery voltage 
	call	write_external_eeprom
	movf	batt_voltage+1,W
	call	write_external_eeprom

	GETCUSTOM8	d'20'					; sampling rate in WREG
	btfsc	FLAG_apnoe_mode				; Apnoe mode?
	movlw	d'1'						; Apnoe sampling rate
	call	write_external_eeprom

	movlw	d'2'		; information size temperature
	movwf	temp1		; copy to bits 0-3
	swapf	temp1,F		; swap nibbels 0-3 with 4-7
	GETCUSTOM8	d'21'	; Divisor temperature
	addwf	temp1,W		; copy to bits 0-3, result in WREG
	call	write_external_eeprom

	movlw	d'2'		; information size deco
	movwf	temp1		; copy to bits 0-3
	swapf	temp1,F		; swap nibbels 0-3 with 4-7
	GETCUSTOM8	d'22'	; Divisor deco
	btfsc	FLAG_apnoe_mode		; in Apnoe mode?
	movlw	d'0'				; Yes, set to zero
	addwf	temp1,W		; copy to bits 0-3, result in WREG
	call	write_external_eeprom

	movlw	d'1'		; information size GF
	movwf	temp1		; copy to bits 0-3
	swapf	temp1,F		; swap nibbels 0-3 with 4-7
	GETCUSTOM8	d'23'	; Divisor GF
	addwf	temp1,W		; copy to bits 0-3, result in WREG
	call	write_external_eeprom

	movlw	d'3'		; information size pp02
	movwf	temp1		; copy to bits 0-3
	swapf	temp1,F		; swap nibbels 0-3 with 4-7
	GETCUSTOM8	d'24'	; Divisor pp02
	addwf	temp1,W		; copy to bits 0-3, result in WREG
	call	write_external_eeprom

	movlw	d'15'		; information size Decodebug
	movwf	temp1		; copy to bits 0-3
	swapf	temp1,F		; swap nibbels 0-3 with 4-7
	GETCUSTOM8	d'25'	; Divisor Decodebug
	addwf	temp1,W		; copy to bits 0-3, result in WREG
	call	write_external_eeprom

	movlw	d'1'		; information size cns
	movwf	temp1		; copy to bits 0-3
	swapf	temp1,F		; swap nibbels 0-3 with 4-7
	GETCUSTOM8	d'26'	; Divisor cns
	addwf	temp1,W		; copy to bits 0-3, result in WREG
	call	write_external_eeprom

	read_int_eeprom	d'26'			; Read Salinity from EEPROM
	movf	EEDATA,W
	call	write_external_eeprom	; Store Salinity to Dive

	movff	char_O_CNS_fraction,WREG	; copy into bank1
	call	write_external_eeprom		; Stores CNS%

	movff	avr_rel_pressure_total+0,WREG	; Average Depth
	call	write_external_eeprom
	movff	avr_rel_pressure_total+1,WREG	; Average Depth
	call	write_external_eeprom

	movff	total_divetime_seconds+0,WREG	; Total dive time (Regardless of CF01)
	call	write_external_eeprom
	movff	total_divetime_seconds+1,WREG	; Total dive time (Regardless of CF01)
	call	write_external_eeprom

	movlw	d'32'							; GF_lo
	movff	char_I_deco_model,lo
	decfsz	lo,F							; jump over next line if char_I_deco_model == 1
	movlw	d'11'							; Saturation Multiplier
	call	getcustom8_1					; Get Custom function #WREG
	call	write_external_eeprom			; write WREG into external memory

	movlw	d'33'							; GF_hi
	movff	char_I_deco_model,lo
	decfsz	lo,F							; jump over next line if char_I_deco_model == 1
	movlw	d'12'							; Desaturation Multiplier
	call	getcustom8_1					; Get Custom function #WREG
	call	write_external_eeprom			; write WREG into external memory

	read_int_eeprom d'34'					; Read deco modell
	movf	EEDATA,W
	call	write_external_eeprom			; write WREG into external memory

	GETCUSTOM8	d'29'                       ; Last decostop [m]
	call	write_external_eeprom			; write WREG into external memory

	movff   gaslist_active,WREG             ; "Gas active" flag register
	call	write_external_eeprom			; write WREG into external memory

    movff   on_time_seconds+0,xC+0
    movff   on_time_seconds+1,xC+1
    movff   on_time_seconds+2,xC+2
    clrf    xC+4
    movlw   LOW     .3600
    movwf   xB+0
    movlw   HIGH    .3600
    movwf   xB+1
    call    div32x16  ; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder
    movf    xC+0,W
	call	write_external_eeprom			; Full hours of on-time since last full charge

	movlw	0xFB						; Header stop
	call	write_external_eeprom
	movlw	0xFB
	call	write_external_eeprom
	
	; Increase total dive counter
	read_int_eeprom 	d'2'		; Read byte (stored in EEDATA)
	movff	EEDATA,temp1			; Low byte
	read_int_eeprom 	d'3'		; Read byte (stored in EEDATA)
	movff	EEDATA,temp2			; high byte
	bcf		STATUS,C
	movlw	d'1'
	addwf	temp1
	movlw	d'0'
	addwfc	temp2				
	movff	temp1,EEDATA
	write_int_eeprom	d'2'			; write byte stored in EEDATA
	movff	temp2,EEDATA
	write_int_eeprom	d'3'			; write byte stored in EEDATA

; Save dive mode custom view
	movff	menupos3,EEDATA				; Copy to EEDATA
	write_int_eeprom	d'94'			; Write last selected customview dive mode into EEPROM

	GETCUSTOM15	.28							; Logbook Offset -> lo, hi
	tstfsz		lo							; lo=0?
	bra		change_logbook_offset1		; No, adjust offset	
	tstfsz		hi						; hi=0?
	bra		change_logbook_offset1		; No, adjust offset
	bra		change_logbook_offset2		; lo=0 and hi=0 -> skip Offset routine
change_logbook_offset1:
	movlw	d'1'
	addwf	lo
	movlw	d'0'
	addwfc	hi
	movlw	d'112'					; CF28 *4 Bytes...
	addlw	0x82
	movwf	EEADR
	movff	lo,EEDATA
	call	write_eeprom			; Lowbyte
	movlw	d'112'					; CF28 *4 Bytes...
	addlw	0x83
	movwf	EEADR
	movff	hi,EEDATA
	call	write_eeprom			; Highbyte 

change_logbook_offset2:
	bcf		LED_blue
	clrf	surface_interval+0
	clrf	surface_interval+1		; Clear surface interval timer

end_dive_common:
	bcf		simulatormode_active		; if we were in simulator mode

; In DEBUG compile, keep all simulated dives in logbook, Desat time, nofly, etc...
    ifndef __DEBUG
		btfsc	restore_deco_data			; Restore decodata?
		call	simulator_restore_tissue_data		; Yes!
	endif

	goto	surfloop_no_display_init	; and return to surfaceloop

end_dive_common_sim:
    tstfsz  surface_interval+0              ; Was interval zero?
    bra     end_dive_common_sim2            ; No
    tstfsz  surface_interval+1              ; Was interval zero?
    bra     end_dive_common_sim2            ; No
    bra     end_dive_common                 ; Yes, done.

end_dive_common_sim2:
    movf    divemins+0,W
    addwf   surface_interval+0,F
    movf    divemins+1,W
	addwfc  surface_interval+1				; Add simulated divetime to surface interval
    bra     end_dive_common

timeout_divemode:
	btfss	realdive					; Dive longer then one minute
	return
	
	btfsc	FLAG_apnoe_mode				; In Apnoe mode?
	bra		timeout_divemode2			; Yes, use CF30 [min] for timeout

    ifndef __DEBUG
    	btfsc	simulatormode_active    ; In Simulator mode?
    	bra		timeout_divemode3       ; Yes, use fixed 5 seconds timeout			
    endif
	
	bcf		divemode
	incf	timeout_counter,F
	movlw	d'0'
	addwfc	timeout_counter2,F			; timeout is 15bits

	GETCUSTOM15	d'2'					; diveloop_timeout -> lo:hi	

	btfss	dekostop_active			; Is a deco stop displayed?
	bra		timeout_divemode1		; No, use normal CF02 timeout

	; Yes, use 10minutes counter	
	movlw	LOW		.600
	movwf	lo
	movlw	HIGH	.600
	movwf	hi

timeout_divemode1:
	movff	lo,sub_a+0
	movff	hi,sub_a+1
	movff	timeout_counter, sub_b+0
	movff	timeout_counter2, sub_b+1
	call	sub16						;  sub_c = sub_a - sub_b
	btfss	neg_flag					; Result negative?
	bsf		divemode					; No, set flag
	return

timeout_divemode2:
	incf	timeout_counter,F			; seconds...
	movlw	d'60'
	cpfseq	timeout_counter				; timeout_counter=60?
	return								; No.

	clrf	timeout_counter
	bcf		divemode
	incf	apnoe_timeout_counter,F
	GETCUSTOM8	d'30'					; apnoe timeout [min]
	cpfseq	apnoe_timeout_counter
	bsf		divemode
	return

timeout_divemode3:
	bcf		divemode
	incf	timeout_counter,F
	movlw	d'5'						; Fixed timeout of 5 seconds
	cpfsgt	timeout_counter
	bsf		divemode
	return

update_divemode1:						; update any second
	call	set_dive_modes				; tests if depth>threshold
	
	btfsc	divemode
	call	set_max_depth				; update max. depth if required

	btfsc	divemode
	call	set_min_temp				; store min. temp if required

	bcf		temp_changed			    ; Display temperature?
    SAFE_2BYTE_COPY temperature,lo
	movf	lo,W
	cpfseq	last_temperature+0
	bsf		temp_changed			    ; Yes
	movf	hi,W
	cpfseq	last_temperature+1
	bsf		temp_changed			    ; Yes
	btfsc	temp_changed	
	call	DISP_temp_divemode		    ; Displays temperature

	bcf		pres_changed			; Display new depth?
    SAFE_2BYTE_COPY amb_pressure, lo
	movf	lo,W
	cpfseq	last_pressure+0
	bsf		pres_changed			; Yes
	movf	hi,W
	cpfseq	last_pressure+1
	bsf		pres_changed			; Yes

	btfsc	simulatormode_active	; always update depth when in simulator mode
	bsf		pres_changed				

	btfsc	pres_changed	
	call	DISP_depth					; Displays new depth
	return

update_divemode60:					; update any minute
	call	get_battery_voltage			; gets battery voltage
	call	set_powersafe				; red LED blinking if battery is low
	call	DISP_max_pressure			; No, use normal max. depth
	call	check_temp_extrema			; check for new temperature extremas
	call	customview_minute			; Do every-minute tasks for the custom view area
	bcf		oneminupdate
	return

set_max_depth:
	movff	max_pressure+0,sub_a+0
	movff	max_pressure+1,sub_a+1
    SAFE_2BYTE_COPY rel_pressure, sub_b
	call	sub16               ; sub_c = sub_a - sub_b
								; max_pressure<rel_pressure -> neg_flag=1
								; rel_pressure<=max_pressure -> neg_flag=0
	btfss	neg_flag	
	return
								;max_pressure<rel_pressure
	movff	sub_b+0,max_pressure+0
	movff	sub_b+1,max_pressure+1
	call	DISP_max_pressure			; No, use normal max. depth
	return

set_min_temp:
	movff	mintemp+0,sub_a+0
	movff	mintemp+1,sub_a+1
    SAFE_2BYTE_COPY temperature,sub_b
	call	sub16				; sub_c = sub_a - sub_b
								; mintemp<T -> neg_flag=1
								; T<=mintemp -> neg_flag=0
	btfsc	neg_flag	
	return
								;mintemp>=T
	movff	sub_b+0,mintemp+0
	movff	sub_b+1,mintemp+1
	return

set_dive_modes:
	btfsc	high_altitude_mode		; In high altitude (Fly) mode?
	bra		set_dive_modes3			; Yes!

	btfss	divemode				; In divemode?
	bra		set_dive_modes0			; No.

	; in Divemode, check threshold from CF01
	GETCUSTOM8	.1					; loads dive_threshold in WREG
	movwf	sub_a+0					; dive_treshold is in cm

    movlw   .20
    cpfsgt  sub_a+0
    movwf   sub_a+0                 ; Make sure to have at least 20cm threshold

	clrf	sub_a+1
	bra		set_dive_modes1			; Done.

set_dive_modes0:
	GETCUSTOM8	.0					; loads dive_threshold in WREG
	movwf	sub_a+0					; dive_treshold is in cm

    movlw   .20
    cpfsgt  sub_a+0
    movwf   sub_a+0                 ; Make sure to have at least 20cm threshold

	clrf	sub_a+1
	bra		set_dive_modes1			; Done.

set_dive_modes1:
    SAFE_2BYTE_COPY rel_pressure, sub_b
	call	sub16					; sub_c = sub_a - sub_b

	btfss	neg_flag	
	bra		set_dive_modes2			; too shallow (rel_pressure<dive_threshold)

	btfsc	realdive				; Dive longer than one minute?
	clrf 	timeout_counter			; Yes, reset timout counter (Low Byte)
	btfsc	realdive				; Dive longer than one minute?
	clrf 	timeout_counter2		; Yes, reset timout counter (High Byte)

set_dive_modes_common:
	bsf		divemode				; (Re-)Set divemode flag
	bsf		divemode2				; displayed divetime is running
	btfsc	timeout_display			; Was the timeout displayed?
	call	DISP_divemode_timeout_clear	; Yes, Clear (once)
	return

set_dive_modes2:
	bcf		divemode2				; Stop time
	btfss	realdive				; dive longer then one minute?
	bcf		divemode				; no -> this was no real dive

	btfss	divemode				; Are we still diving?
	return							; No, return
; Yes, show divemode timeout

	btfsc	menubit						; Divemode menu active?
	return								; Yes, return
	btfsc	FLAG_apnoe_mode				; In Apnoe mode?
	return								; Yes, return
	btfsc	gauge_mode					; In Gauge mode?
	return								; Yes, return


	btfss	dekostop_active			; Is a deco stop displayed?
	call	DISP_divemode_timeout	; No, show the divemode timeout here...
	btfsc	dekostop_active			; Is a deco stop displayed?
	call	DISP_divemode_timeout2	; Yes, show red warning divemode counter
	return

set_dive_modes3:					; High-altitude mode
	btfsc	realdive				; dive longer then one minute?
	bra		set_dive_modes0			; Yes -> this is a real dive -> Use CF00 threshold for ascend

	movlw	HIGH	d'300'			; hard-wired 300cm threshold
	movwf	sub_a+1
	movlw	LOW		d'300'			; hard-wired 300cm threshold
	movwf	sub_a+0
	bra		set_dive_modes1

set_powersafe:
	btfsc	low_battery_state		; battery warning alread active?
	bra		set_powersafe2			; Yes, but is it still required?

    call    update_batt_get_percent_in_lo
	movlw	d'20'					; 20%
	cpfsgt	lo
#IFDEF CCR_CTRL
    bra     set_powersafe1          ; Less - set low bat state

    btfss	ext_ppO2_valid
    bra     set_powersafe_02        ; No new data received - exit

set_powersafe_01:
    ; Check eCRTL Battery state
    movff   eCTRL_battery, lo
    movlw   eCTRL_BAT_LOW           ; Battery threshold value
    cpfsgt  lo
	bra		set_powersafe1
set_powersafe_02:
#ENDIF
	return

set_powersafe1:
	movlw	d'7'					; Type of alarm (Battery Low)
	movwf	AlarmType				; Copy to alarm Register
	bsf		event_occured			; Set Event Flag
	bsf		low_battery_state		; set flag for battery warning
	return							; return

set_powersafe2:
	movlw	d'13'					; 3,584V
	cpfsgt	batt_voltage+1
	bra		set_powersafe1			; Still to low
#IFDEF CCR_CTRL
    btfss	ext_ppO2_valid
    bra     set_powersafe_12        ; No new data received - exit

set_powersafe_11:
    ;Check eCRTL Battery state
    movff   eCTRL_battery, lo
    movlw   eCTRL_BAT_OK_LOW        ; Battery threshold value
    cpfsgt  lo
	bra		set_powersafe1
set_powersafe_12:
#ENDIF
	bcf		low_battery_state		; clear flag for battery warning mode
	return

calc_average_depth:
	btfsc	reset_average_depth		; Reset the Average depth?
	rcall	reset_average1			; Reset the resettable average depth

#IFDEF BOTTOM
;Reset Timer only?
	btfsc	reset_average_timer		; Reset the Avewrage depth Timer only?
	rcall	reset_average2			; Reset the resettable average depth Timer only
;Reset Depth only?
	btfsc	reset_average		; Reset the Avewrage depth Timer only?
	rcall	reset_average3			; Reset the resettable average depth Timer only
#ENDIF

	; 1. Add new 2xdepth to the Sum of depths registers
    SAFE_2BYTE_COPY rel_pressure, b0_lo	; Buffer...

	movf	b0_lo,w
	addwf	average_depth_hold+0,F
	movf	b0_hi,w
	addwfc	average_depth_hold+1,F
	movlw	d'0'
	addwfc	average_depth_hold+2,F
	addwfc	average_depth_hold+3,F ; Will work up to 9999mbar*60*60*24=863913600mbar

	movf	b0_lo,w
	addwf	average_depth_hold+0,F
	movf	b0_hi,w
	addwfc	average_depth_hold+1,F
	movlw	d'0'
	addwfc	average_depth_hold+2,F
	addwfc	average_depth_hold+3,F ; Will work up to 9999mbar*60*60*24=863913600mbar

; Do the same for the _total registers (Non-Resettable)
	movf	b0_lo,w
	addwf	average_depth_hold_total+0,F
	movf	b0_hi,w
	addwfc	average_depth_hold_total+1,F
	movlw	d'0'
	addwfc	average_depth_hold_total+2,F
	addwfc	average_depth_hold_total+3,F ; Will work up to 9999mbar*60*60*24=863913600mbar

	movf	b0_lo,w
	addwf	average_depth_hold_total+0,F
	movf	b0_hi,w
	addwfc	average_depth_hold_total+1,F
	movlw	d'0'
	addwfc	average_depth_hold_total+2,F
	addwfc	average_depth_hold_total+3,F ; Will work up to 9999mbar*60*60*24=863913600mbar

	; 2. Compute Average Depth on base of average_divesecs:2
	movff	average_divesecs+0,xB+0
	movff	average_divesecs+1,xB+1		; Copy
	movff	average_depth_hold+0,xC+0
	movff	average_depth_hold+1,xC+1
	movff	average_depth_hold+2,xC+2
	movff	average_depth_hold+3,xC+3

	call	div32x16 	; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder
	movff	xC+0,avr_rel_pressure+0
	movff	xC+1,avr_rel_pressure+1

	; Compute Total Average Depth on base of divemins:2 and divesecs
	movff	divemins+0,xA+0
	movff	divemins+1,xA+1
	movlw	d'60'
	movwf	xB+0
	clrf	xB+1
	call	mult16x16				; xC:4=xA:2*xB:2
	movf	divesecs,W
	addwf	xC+0,F
	movlw	d'0'
	addwfc	xC+1,F					; xC:2 holds total dive seconds
	movlw	d'3'					; 2+1
	btfss	divesecs,0				; divesecs even?
	movlw	d'2'					; Yes, do not add +1
	addwf	xC+0,F
	movlw	d'0'
	addwfc	xC+1,F
	; Ignore xC+2 and xC+3. Total Average will only work up to divetime=1092:16
	movff	xC+0,xB+0
	movff	xC+1,xB+1		; Copy
	movff	average_depth_hold_total+0,xC+0
	movff	average_depth_hold_total+1,xC+1
	movff	average_depth_hold_total+2,xC+2
	movff	average_depth_hold_total+3,xC+3

	call	div32x16 	; xC:4 / xB:2 = xC+3:xC+2 with xC+1:xC+0 as remainder
	movff	xC+0,avr_rel_pressure_total+0
	movff	xC+1,avr_rel_pressure_total+1

	return
	
reset_average1:
#IFDEF BOTTOM
;Clear Resettable timer
	movlw	d'2'
	movwf	average_divesecs_timer+0	;Clear Timer
	clrf	average_divesecs_timer+1
;	btfsc	reset_average		; Reset the Avewrage depth?
;	bsf		show_timer					; Enable Timer Show in Gauge mode (after first reset)
reset_average3:
	bcf		reset_average			; Clear flag
#ENDIF
	clrf	average_depth_hold+0
	clrf	average_depth_hold+1
	clrf	average_depth_hold+2
	clrf	average_depth_hold+3		; Clear average depth register
	movlw	d'2'
	movwf	average_divesecs+0
	clrf	average_divesecs+1
	bcf		reset_average_depth			; Clear flag
	return

#IFDEF BOTTOM
;Clear Resettable timer only
reset_average2:
	movlw	d'2'
	movwf	average_divesecs_timer+0	;Clear Timer
	clrf	average_divesecs_timer+1
	bsf		show_timer					; Enable Timer Show in Gauge mode (after first reset)
	bcf		reset_average_timer			; Clear flag
	return
#ENDIF

;=============================================================================
; Check for Auto-SP
;
check_dive_autosp:               ; Check for Auto-SP
    read_int_eeprom .116
    tstfsz      EEDATA           ; =0: Manual
    bra     check_dive_autosp2
    return                       ; Skip check
check_dive_autosp2:
    SAFE_2BYTE_COPY rel_pressure,xA
	movlw	d'100'
	movwf	xB+0
	clrf	xB+1
	call	div16x16				; compute depth in full m -> result in xC+0
    ; Check SP1
    btfsc   sp1_switched            ;=1: This setpoint has been autoselected already
    bra     check_dive_autosp3      ; Skip check
    read_int_eeprom .117            ; Get depth in m
    tstfsz  EEDATA                  ; =?
    bra     $+4                     ; No, continue
    bra     check_dive_autosp3      ; Skip check
    decf    EEDATA,W                ; -1 -> WREG
    cpfsgt  xC+0                    ; Compare with depth
    bra     check_dive_autosp3      ; lower depth, do not switch
    ; auto switch to SP1
    read_int_eeprom .36             ; Get Setpoint
#IFDEF CCR_CTRL
    rcall    set_new_setpoint_from_eedata
#ELSE
	movff	EEDATA, char_I_const_ppO2	; Use SetPoint
	movff	EEDATA, ppO2_setpoint_store	; Store also in this byte...
	bsf		setpoint_changed
	bsf		event_occured			; set global event flag
#ENDIF
    bsf     sp1_switched            ; Set flag
check_dive_autosp3:
    ; Check SP2
    btfsc   sp2_switched            ;=1: This setpoint has been autoselected already
    bra     check_dive_autosp4      ; Skip check
    read_int_eeprom .118            ; Get depth in m
    tstfsz  EEDATA                  ; =?
    bra     $+4                     ; No, continue
    bra     check_dive_autosp4      ; Skip check
    decf    EEDATA,W                ; -1 -> WREG
    cpfsgt  xC+0                    ; Compare with depth
    bra     check_dive_autosp4      ; lower depth, do not switch
    ; auto switch to SP2
    read_int_eeprom .37             ; Get Setpoint
#IFDEF CCR_CTRL
    rcall    set_new_setpoint_from_eedata
#ELSE
	movff	EEDATA, char_I_const_ppO2	; Use SetPoint
	movff	EEDATA, ppO2_setpoint_store	; Store also in this byte...
	bsf		setpoint_changed
	bsf		event_occured			; set global event flag
#ENDIF
    bsf     sp2_switched            ; Set flag
check_dive_autosp4:
    ; Check SP3
    btfsc   sp3_switched            ;=1: This setpoint has been autoselected already
    bra     check_dive_autosp5      ; Skip check
    read_int_eeprom .119            ; Get depth in m
    tstfsz  EEDATA                  ; =?
    bra     $+4                     ; No, continue
    bra     check_dive_autosp5      ; Skip check
    decf    EEDATA,W                ; -1 -> WREG
    cpfsgt  xC+0                    ; Compare with depth
    bra     check_dive_autosp5      ; lower depth, do not switch
    ; auto switch to SP3
    read_int_eeprom .38             ; Get Setpoint
#IFDEF CCR_CTRL
    rcall    set_new_setpoint_from_eedata
#ELSE
	movff	EEDATA, char_I_const_ppO2	; Use SetPoint
	movff	EEDATA, ppO2_setpoint_store	; Store also in this byte...
	bsf		setpoint_changed
	bsf		event_occured			; set global event flag
#ENDIF
    bsf     sp3_switched            ; Set flag
check_dive_autosp5:
    return                          ; Done.

;=============================================================================
; Setup everything to enter divemode.
;
diveloop_boot:	
	ostc_debug	'Q'		; Sends debug-information to screen if debugmode active
	clrf	max_pressure+0				; clear some variables
	clrf	max_pressure+1
	clrf	avr_rel_pressure+0
	clrf	avr_rel_pressure+1

	movlw	d'1'
	movwf	apnoe_max_pressure+0
	clrf	apnoe_max_pressure+1
	clrf	apnoe_surface_mins			
	clrf	apnoe_mins
	clrf	apnoe_secs
	clrf	divemins+0
	clrf	divemins+1
	clrf 	total_divetime_seconds+0
	clrf 	total_divetime_seconds+1
	bsf		divemode2					; displayed divetime is running (Divetime starts HERE)

	clrf    EEADRH                      ; Make sure to select eeprom bank 0
	call	DISP_brightness_low
	read_int_eeprom	d'90'				; Brightness offset? (Dim>0, Normal = 0)
	movlw	d'0'
	cpfsgt	EEDATA
	call	DISP_brightness_full

    bcf     use_aGF
	bcf		timeout_display
	bcf		menu3_active
	clrf	divesecs
	clrf	samplesecs
	clrf	better_gas_number			; number (1-5) of the "better gas" in divemode, =0: no better gas available
	clrf	apnoe_timeout_counter		; timeout in minutes
	clrf 	timeout_counter				; takes care of the timeout (Low byte)
	clrf 	timeout_counter2			; takes care of the timeout (High byte)
	clrf	AlarmType					; Clear all alarms
	bcf		event_occured				; clear flag
	bcf		setpoint_changed			; clear flag
    bcf     store_bailout_event         ; clear flag
	rcall	reset_average1				; Reset the resettable average depth
	clrf	average_depth_hold_total+0
	clrf	average_depth_hold_total+1
	clrf	average_depth_hold_total+2
	clrf	average_depth_hold_total+3	; Clear Non-Resettable Average
#IFNDEF IMPERIAL
	bcf		depth_greater_100m			; clear flag
#ELSE
	bcf		depth_greater_100			; clear flag
#ENDIF
#IFDEF BOTTOM
	bcf		show_timer					; clear flag
#ENDIF
;	setf	last_diluent				; to be displayed after first calculation (range: 0 to 100 [%])
	bcf		dekostop_active
    bcf     decoplan_invalid
	bcf		is_bailout					;=1: CC mode, but bailout active!		
	bcf		better_gas_available        ;=1: A better gas is available and a gas change is advised in divemode
    bcf     tts_extra_time              ;=1: Compute TTS if extra time spent at current depth
	bcf		show_safety_stop			;=1: Show the safety stop
	clrf	safety_stop_countdown		; Clear count-down
    bcf     gaschange_cnt_active        ; Do not show the countdown on start
    ; clear auto-SP flags
    bcf     sp1_switched
    bcf     sp2_switched
    bcf     sp3_switched

	call	get_free_EEPROM_location	; get last position in external EEPROM, may be up to 2 secs!

    btfsc   simulatormode_active
    bra     diveloop_boot_1
    ; Normal mode = Surface pressure is the pressure 30mn before dive.
	movff	last_surfpressure_30min+0,int_I_pres_surface+0	; LOW copy surfacepressure to deco routine
	movff	last_surfpressure_30min+1,int_I_pres_surface+1	; HIGH copy surfacepressure to deco routine
    bra     diveloop_boot_2

diveloop_boot_1:
    ; Simulator mode: Surface pressure is 1bar.
    movlw   LOW .1000
	movff	WREG,int_I_pres_surface+0   ; LOW copy surfacepressure to deco routine
    movlw   HIGH .1000
	movff	WREG,int_I_pres_surface+1   ; HIGH copy surfacepressure to deco routine

diveloop_boot_2:
	SAFE_2BYTE_COPY	temperature,mintemp ; Reset Min-Temp registers

; Init profile recording parameters	
	GETCUSTOM8	d'20'                   ; sample rate
	movwf	samplesecs_value            ; to avoid EEPROM access in the ISR
	GETCUSTOM8	d'21'
	movwf	divisor_temperature         ; load divisors for profile storage
	GETCUSTOM8	d'22'
	btfsc	FLAG_apnoe_mode				; in Apnoe mode?
	movlw	d'0'					; Yes, set to zero
	movwf	divisor_deco				
	GETCUSTOM8	d'23'
	movwf	divisor_gf
	GETCUSTOM8	d'24'
	movwf	divisor_ppo2
	GETCUSTOM8	d'25'
	movwf	divisor_deco_debug
	GETCUSTOM8	d'26'
	movwf	divisor_cns

	btfss	FLAG_apnoe_mode		; In Apnoe mode?
	bra		divemode1
; Overwrite some parameters in Apnoe mode....
	movlw	d'1'
	movwf	samplesecs_value	; to avoid EEPROM access in the ISR

divemode1:
	read_int_eeprom	d'36'				; Read SP 1
	btfsc	FLAG_const_ppO2_mode
	movff	EEDATA,char_I_const_ppO2    ; Set ppO2 setpoint if in ppO2 mode
	movff	EEDATA,ppO2_setpoint_store  ; Store also in this byte...
    bsf     sp1_switched                ; Set flag so we won't auto-switch to it again
#IFDEF CCR_CTRLchar_I_const_ppO2
    ; Send newly setted SP
    btfss   ext_ppO2_enable             ; Check for eCTRL mode
    bra     divemode1a
	movff	EEDATA, eCTRL_SP_DC         ; Store also in this byte...
    movlw   eCTRL_CMD_ACT
    movff   WREG, eCTRL_CMD             ; Send SP requestn with next message
divemode1a:
#ENDIF

	bcf		LED_blue
	bcf		low_battery_state			; clear flag for battery warning mode
	bcf		header_stored				
	bcf		premenu
	bcf		realdive
	bsf		update_divetime				; set flag

#IFNDEF CCR_CTRL
	btfss	simulatormode_active		; do not disable in simulator mode!					
	call	disable_rs232				; Disable RS232
#ENDIF

; Read Start Gas and configure char_I_He_ratio, char_I_O2_ratio and char_I_N2_ratio
set_first_gas:
	btfsc	FLAG_const_ppO2_mode
    bra     set_first_gas_ccr           ; In CCR mode
	read_int_eeprom 	d'33'			; Read byte (stored in EEDATA)
	movff	EEDATA,active_gas			; Read start gas (1-5)
    movff   EEDATA,char_I_current_gas

	decf	active_gas,W				; Gas 0-4
	mullw	d'4'
	movf	PRODL,W			
	addlw	d'7'						; = address for He ratio
	movwf	EEADR
	call	read_eeprom					; Read He ratio
	movff	EEDATA,char_I_He_ratio		; And copy into hold register

	decf	EEADR,F
	call	read_eeprom					; Read O2 ratio
	movff	EEDATA, char_I_O2_ratio		; O2 ratio

	movff	char_I_He_ratio, wait_temp	; copy into bank1 register
	bsf		STATUS,C					; Borrow bit
	movlw	d'100'						; 100%
	subfwb	wait_temp,W					; minus He
	bsf		STATUS,C					; Borrow bit
	subfwb	EEDATA,W					; minus O2
	movff	WREG, char_I_N2_ratio		; = N2!

; Configure gaslist_active flag register
	read_int_eeprom	d'27'
	movff	EEDATA, gaslist_active
	return                              ; Done with "boot"

set_first_gas_ccr:                      ; Set Diluent
    call    get_first_diluent           ; Read first diluent into lo(O2) and hi(He)
	movff	hi,char_I_He_ratio          ; And copy into hold register
	movff	lo, char_I_O2_ratio         ; O2 ratio
	movff	char_I_He_ratio, wait_temp	; copy into bank1 register
	bsf		STATUS,C					; Borrow bit
	movlw	d'100'						; 100%
	subfwb	wait_temp,W					; minus He
	bsf		STATUS,C					; Borrow bit
	subfwb	lo,W        				; minus O2
	movff	WREG, char_I_N2_ratio		; = N2!
    read_int_eeprom d'115'              ; Read First Diluent (1-5)
    movff   EEDATA,active_diluent
    return                              ; Done with "boot"

#IFDEF CCR_CTRL
set_new_setpoint_from_eedata:
    btfss   ext_ppO2_enable             ; Check for eCTRL mode
    bra     set_new_setpoint_exit
	movff	EEDATA, eCTRL_SP_DC         ; Store also in this byte...
    movlw   eCTRL_CMD_ACT
    movff   WREG, eCTRL_CMD             ; Send SP requestn with next message
    bra 	set_new_setpoint_exit2
set_new_setpoint_exit:
	movff	EEDATA,char_I_const_ppO2    ; Set ppO2 setpoint if in ppO2 mode
	movff	EEDATA,ppO2_setpoint_store  ; Store also in this byte...
set_new_setpoint_exit2:
	bsf		setpoint_changed
	bsf		event_occured			; set global event flag
    return
#ENDIF
