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


; Underwater Menu (Set Gas, Decoplan, etc.)
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; written: 11/11/05
; last updated: 090305
; known bugs:
; ToDo:


test_switches_divemode:				; checks switches in divemode
	btfsc	uart_dump_screen                ; Asked to dump screen contains ?
	call	dump_screen     			    ; Yes!

	btfsc	switch_left
	bra		test_switches_divemode2

	btfss	switch_right
	return

	call	wait_switches			; Waits until switches are released, resets flag if button stays pressed!
	
	bcf		select_bailoutgas		; Clear Flag for Bailout list

#IFDEF BOTTOM
; Special keyswitches logic in Gauge mode: Left Key (Select)
	btfss	gauge_mode					
	bra		test_switches_divemode_s0	; no

	btfsc	premenu					; Already in premenu?
	btg		res_type					; Alternate Flag

	bsf		premenu					; Set Flag for premenu
	clrf	timeout_counter3		; timeout_divemenu

	WIN_INVERT		.1

	btfsc	res_type					; Check what to show
	bra		test_switches_divemode_s1

	DISPLAYTEXTH	.385			;Reset Timer only?
	bra		test_switches_divemode_s1a

test_switches_divemode_s1:
	DISPLAYTEXTH	.386			;Reset Average Depth + Timer?
test_switches_divemode_s1a:
	WIN_INVERT		.0
	return

test_switches_divemode_s0:
#ENDIF

	btfsc	premenu					; Pre-Menu? already shown?
	bra		test_switches_divemode0	; Yes, check if we should jump to menu Entry3

test_switches_divemode_a:

	bsf		premenu					; Set Flag for premenu
	bcf		menubit
	clrf	timeout_counter3		; timeout_divemenu
		
	btfsc	FLAG_apnoe_mode			; In Apnoe mode?
	bra		test_switches_divemode1	; Yes!

	call		DISP_divemask_color	; Set Color for Divemode mask
	WIN_INVERT		.1
	DISPLAYTEXT	.4			;Menu?
	WIN_INVERT		.0
    call	DISP_standard_color
	return

test_switches_divemode0:
	btfss	menu3_active				; Something to do at Menupos=3?
	bra		test_switches_divemode_a	; No
; Yes! So show menu and jump to this position
	movlw	d'3'
	movwf	menupos
	bra		test_switches_divemode2b	; Show menu with cursor at menupos=3

test_switches_divemode1:
	DISPLAYTEXT	.141			;Quit?
	return

test_switches_divemode2:
	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!

	btfsc	premenu
	bra		test_switches_divemode2_2
	btfsc	menubit
	bra		test_switches_divemode2_2	; Not in Premenu or Menu...

	bsf		toggle_customview			; Toggle customview (Cleared in customview.asm)

	btfsc	standalone_simulator		; Standalone Simualtor active?
	bra		divemode_menu_simulator		; Yes, Show simulator menu!
	return

test_switches_divemode2_2:
#IFDEF BOTTOM
; Special keyswitches logic in Gauge mode: Right Key (Confirmation)
	btfss	gauge_mode					
	bra		test_switches_divemode_s2	; no

	btfsc	res_type					; Pre-Left? already shown?
	bra		test_switches_divemode_s3	; no

	bsf		reset_average_timer			; Average Depth Timer ONLY will be resetted in divemode.asm
	bra		test_switches_divemode_s4	; no

test_switches_divemode_s3:
	bsf		reset_average			; Average Depth will be resetted in divemode.asm
test_switches_divemode_s4:
	bcf		premenu                        ; 
	bcf		res_type                       ; 
	call	DISP_pre_gauge_clear                 ; 
	call	DISP_divemode_mask				; And redraw mask (Redraw missing "T" from "Tauchzeit" in german text version)
	return

test_switches_divemode_s2:
#ENDIF

	btfss	FLAG_apnoe_mode		; In Apnoe mode?
	bra		test_switches_divemode2a; No!
	
	; Yes, so quit Apnoe mode at once...
	bcf		divemode			; Clear Divemode flag...
	bcf		premenu				; clear premenu flag
	return

test_switches_divemode2a:
	movlw	d'1'
	movwf	menupos					; reset cursor in divemode menu
test_switches_divemode2b:
	bsf		menubit					; Enter Divemode-Menu!
	bcf		premenu					; clear premenu flag
	call	DISP_clear_divemode_menu		; Clear dive mode menu area
	call	DISP_divemode_menu_mask_first	; Write Divemode menu1 mask
	bcf		display_set_simulator			; Clear Simulator-Menu flag
	call	DISP_divemenu_cursor	; show cursor
	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!
	return

test_switches_divemode_menu:
	btfsc	uart_dump_screen                ; Asked to dump screen contains ?
	call	dump_screen     			    ; Yes!

	btfsc	switch_left
	bra		test_switches_divemode_menu3
	btfss	switch_right
	return

	bcf		switch_right				; Left button pressed

	btfsc 	display_see_deco			; Is the Decoplan displayed?
	bra		divemenu_see_decoplan2		; Yes, exit menu on left button press

	clrf	timeout_counter3			; timout_divemenu!
	incf	menupos,F

; Following routine configures the number of menu entries for the different modes
	movlw	d'6'						; number of available gases+1, ; number of menu options+1
	btfsc	display_set_gas				; Are we in the "Gaslist" menu?
	movlw	d'7'						; Yes, Number of entries for this menu+1 = 7
	btfsc	display_set_setpoint		; In SetPoint Menu?
;TODO check original value
#IFDEF CCR_CTRL     ;Menu itemps number
	movlw	d'7'						; Number of entries for this menu+1 = 7
	btfsc	display_set_more			; Are we in the "More" menu?
	movlw	d'6'						; Yes, Number of entries for this menu+1 = 6
#ELSE
	movlw	d'6'						; Number of entries for this menu+1 = 6
#ENDIF
	btfsc	display_set_active			; De/Activate gases underwater menu is visible?
	movlw	d'7'						; Number of entries for this menu+1 = 7
	btfsc	display_set_xgas			; Are we in the Gas6 menu?
	movlw	d'7'						; Number of entries for this menu+1 = 7
	btfsc	display_set_simulator		; Are we in the simulator menu?
	movlw	d'7'						; Number of entries for this menu+1 = 7
    btfsc   display_set_diluent         ; Are we in the "Diluent" list?
    movlw	d'6'						; Number of entries for this menu+1 = 6
	cpfseq	menupos						; =limit?
	bra		test_switches_divemode_menu1; No!
	movlw	d'1'						; Yes, reset to position 1!
	movwf	menupos

test_switches_divemode_menu1:
; Finally, check if menuposition 3 should be skipped (No customview with function displayed)

	btfsc	display_set_gas					; Are we in the "Gaslist", "SetPoint" or De/Activate gases menu?
	bra		test_switches_divemode_menu1a	; Skip test for sub menus
	btfsc	display_set_xgas				; Are we in the "Gaslist", "SetPoint" or De/Activate gases menu?
	bra		test_switches_divemode_menu1a	; Skip test for sub menus
	btfsc	display_set_active				; Are we in the "Gaslist", "SetPoint" or De/Activate gases menu?
	bra		test_switches_divemode_menu1a	; Skip test for sub menus
    btfsc	display_set_diluent				; Are we in the "Gaslist", "SetPoint" or De/Activate gases menu?
    bra		test_switches_divemode_menu1a	; Skip test for sub menus
#IFDEF CCR_CTRL
    ; Additional Menu Item
    btfsc	display_set_more				; Are we in the "SetPoint/More" menu?
    bra		test_switches_divemode_menu1a	; Skip test for sub menus
#ENDIF

	movlw	d'3'
	cpfseq	menupos							; At position 3?
	bra		test_switches_divemode_menu1a	; No
	btfss	menu3_active					; Menu position 3 has functionality?
	incf	menupos,F						; No, +1, skip to menuos=4

test_switches_divemode_menu1a:
	call	DISP_divemenu_cursor		; update cursor
	btfsc	display_set_gas				; In Gaslist or Setpoint list menu?
	call	DISP_show_change_depth		; Yes, show change depth for gas #menupos
	return

test_switches_divemode_menu3:
	call	wait_switches			; Waits until switches are released, resets flag if button stays pressed!
	bsf		menubit					; Enter Divemode-Menu!
	bcf		premenu					; clear premenu flag
	clrf	timeout_counter3

	btfsc	display_set_gas				; Are we in the "Gaslist" or "SetPoint" menu?
	bra		divemenu_set_gas2			; Yes, so set gas and exit menu

	btfsc 	display_see_deco			; Is the Decoplan displayed?
	bra		divemenu_see_decoplan2		; Yes, exit menu on right button press

	btfsc	display_set_xgas			; Are we in the "Set Gas" menu?
	bra		divemenu_set_xgas2			; Yes, so configure gas or set menu and exit menu

	btfsc	display_set_active			; Are we in the "De/Activate gases menu?" menu?
	bra		divemenu_de_activate2		; Yes, so toggle active flag

	btfsc	display_set_simulator		; Are we in the Divemode Simulator menu?
	goto	divemode_menu_simulator2	; Yes, so adjust depth or set and exit

    btfsc	display_set_diluent         ; Are we in the "Diluent" List?
    goto	divemode_set_diluent2       ; Yes, so choose diluent and exit

#IFDEF CCR_CTRL
    ;Action for MORE Menu
    btfsc	display_set_more            ; Are we in the "More" List?
    goto	divemode_set_more2          ; Yes, so choose option and exit
#ENDIF

; Options for Menu 1
	dcfsnz	menupos,F
	bra		divemenu_set_gas			; Set gas sub-menu
	dcfsnz	menupos,F
	bra		divemenu_see_decoplan		; display the full decoplan
	dcfsnz	menupos,F
	bra		divemode_menu3				; Customview-function
	dcfsnz	menupos,F
	bra		divemode_toggle_brightness	; Toggle DISPLAY-Brightness
	dcfsnz	menupos,F
	bra		timeout_divemenu2           ; Quit divemode menu
	bra		timeout_divemenu2			; Quit divemode menu

divemode_menu3:
	dcfsnz	menupos3,W                  ; copy
	bra		toggle_stopwatch			; Toggle Stopwatch/Average register
	dcfsnz	WREG,F
	bra		set_marker					; Set Marker
	dcfsnz	WREG,F
	bra		divemode_menu3_nothing		; clock...
	dcfsnz	WREG,F
	bra		divemode_menu3_nothing		; leading tissue...
	dcfsnz	WREG,F
	bra		divemode_menu3_nothing	    ; Average depth (not resetable)
	dcfsnz	WREG,F
	bra		divemode_menu3_nothing	    ; Graphs...
	dcfsnz	WREG,F
	bra		divemode_menu3_nothing      ; END/EAD in divemode
	dcfsnz	WREG,F
	bra		divemode_menu3_nothing      ; Future TTS...
	dcfsnz	WREG,F
	bra		toggle_stopwatch            ; Cave bailout prediction.
	dcfsnz	WREG,F
	bra		divemode_menu3_nothing      ; pSCR info
	dcfsnz	WREG,F
	bra		toggle_gradient_factors     ; Toggle gradient factors

divemode_menu3_nothing:
	bra		timeout_divemenu2			; Quit divemode menu

toggle_gradient_factors:
    btg     use_aGF                     ; Toggle GF selector bit
    bsf     decoplan_invalid            ; The decoplan needs to updated
    clrf    WREG
    movff   WREG,char_O_deco_status     ; Restart decoplan computation mH
    btfss   use_aGF
    bra     toggle_gradient_factors2    ; Use aGf
    ; Use normal GF
	; Load GF values into RAM
	GETCUSTOM8	d'32'                   ; GF low
	movff	EEDATA,char_I_GF_Low_percentage
	GETCUSTOM8	d'33'                   ; GF high
	movff	EEDATA,char_I_GF_High_percentage
	bra		timeout_divemenu2			; quit menu!
toggle_gradient_factors2:               ; Use aGf
	; Load GF values into RAM
	GETCUSTOM8	d'67'                   ; aGF low
	movff	EEDATA,char_I_GF_Low_percentage
	GETCUSTOM8	d'68'                   ; aGF high
	movff	EEDATA,char_I_GF_High_percentage
	bra     timeout_divemenu2			; quit menu!

set_marker:
	movlw	d'6'                        ; Type of Alarm  (Manual Marker)
	movwf	AlarmType                   ; Copy to Alarm Register
	bsf		event_occured               ; Set Event Flag

    ; save snapshot of depth and time
    SAFE_2BYTE_COPY rel_pressure,marker_depth
    SAFE_2BYTE_COPY divemins,marker_time
    movff   divesecs,marker_time+2

    bra		timeout_divemenu2			; quit menu!

toggle_stopwatch:
	bsf		reset_average_depth			; Average Depth will be resetted in divemode.asm
	bra		timeout_divemenu2			; quit menu!

divemode_toggle_brightness:
	read_int_eeprom	d'90'				; Brightness offset? (Dim>0, Normal = 0)
	tstfsz	EEDATA						; Was dimmed?
	bra		divemode_toggle_brightness1	; Yes...

	call	DISP_brightness_low
	movlw	d'1'
	movwf	EEDATA						; Copy to EEDATA
	write_int_eeprom	d'90'			; Brightness offset? (Dim=1, Normal = 0)
	bra		divemode_toggle_brightness3

divemode_toggle_brightness1:
	call	DISP_brightness_full
	movlw	d'0'
	movwf	EEDATA						; Copy to EEDATA
	write_int_eeprom	d'90'			; Brightness offset? (Dim=1, Normal = 0)

divemode_toggle_brightness3:
; Now, redraw all outputs (All modes)
	call	DISP_active_gas_divemode	; Display gas, if required
	call	DISP_temp_divemode			; Displays temperature
	call	DISP_depth					; Displays new depth...
	call	DISP_max_pressure			; ...and max. depth

	bra		timeout_divemenu2			; quit menu!

divemenu_de_activate:
	bsf		display_set_active			; Set display flag
	bcf		display_set_xgas			; Clear Flag
	call	DISP_clear_divemode_menu	; Clear Menu

	call	DISP_de_activelist			; show (de)active gaslist

	movlw	d'1'
	movwf	menupos						; reset cursor
	call	DISP_divemenu_cursor		; update cursor
	return

divemenu_de_activate2:					; Toggle active flag
	dcfsnz	menupos,F
	bra		divemenu_de_activate2_exit	; Exit, Quit, Abort
	dcfsnz	menupos,F
	bra		divemenu_de_activate2_g1	; Toggle Gas1
	dcfsnz	menupos,F
	bra		divemenu_de_activate2_g2	; Toggle Gas2
	dcfsnz	menupos,F
	bra		divemenu_de_activate2_g3	; Toggle Gas3
	dcfsnz	menupos,F
	bra		divemenu_de_activate2_g4	; Toggle Gas4
	dcfsnz	menupos,F
	bra		divemenu_de_activate2_g5	; Toggle Gas5
	return	; should never be here

divemenu_de_activate2_exit:
	bra		timeout_divemenu2			; quit underwater menu!

divemenu_de_activate2_g1:
	btg		gaslist_active,0    		; Toggle flag
	movlw	d'2'
	movwf	menupos						; reset cursor
	call	DISP_de_activelist			; show (de)active gaslist
	return

divemenu_de_activate2_g2:
	btg		gaslist_active,1    		; Toggle flag
	movlw	d'3'
	movwf	menupos						; reset cursor
	call	DISP_de_activelist			; show (de)active gaslist
	return

divemenu_de_activate2_g3:
	btg		gaslist_active,2    		; Toggle flag
	movlw	d'4'
	movwf	menupos						; reset cursor
	call	DISP_de_activelist			; show (de)active gaslist
	return

divemenu_de_activate2_g4:
	btg		gaslist_active,3    		; Toggle flag
	movlw	d'5'
	movwf	menupos						; reset cursor
	call	DISP_de_activelist			; show (de)active gaslist
	return

divemenu_de_activate2_g5:
	btg		gaslist_active,4    		; Toggle flag
	movlw	d'6'
	movwf	menupos						; reset cursor
	call	DISP_de_activelist			; show (de)active gaslist
	return

divemode_set_xgas:						; Set the extra gas...
	bsf		display_set_xgas			; Set Flag
	bcf		display_set_gas				; Clear Flag
	call	DISP_clear_divemode_menu	; Clear Menu

	movff	char_I_O2_ratio, EEDATA		; Reset Gas6 to current gas
	write_int_eeprom	d'24'
	movff	char_I_He_ratio, EEDATA
	write_int_eeprom	d'25'

	call	DISP_divemode_set_xgas		; Show mask

	movlw	d'1'
	movwf	menupos						; reset cursor
	call	DISP_divemenu_cursor		; update cursor
	return

divemode_menu_simulator:
	bsf		menubit					; Enter Divemode-Menu!
	bcf		premenu					; clear premenu flag
	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!
	bsf		display_set_simulator	; Set Flag
	bsf		menu3_active			; So "+1" is accessible at all times
	call	DISP_clear_divemode_menu	; Clear Menu
	call	DISP_divemode_simulator_mask; Show mask
	movlw	d'1'
	movwf	menupos						; reset cursor
	call	DISP_divemenu_cursor		; update cursor
	return

divemode_menu_simulator2:
	dcfsnz	menupos,F
	bra		timeout_divemenu2			; close underwater menu!
#IFDEF IMPERIAL
;	GETCUSTOM8  d'61'				; Check CF#61 Conversion to imperial units
	GETCUSTOM8  d'41'				; Check CF#41 Conversion to imperial units
	btfsc   WREG,0                      ; Enabled ?
	bra divemode_menu_simulator3        ; NO
	dcfsnz	menupos,F
	bra		divemode_menu_simulator_p1ft	; Adjust +1ft
	dcfsnz	menupos,F
	bra		divemode_menu_simulator_m1ft	; Adjust -1ft
	dcfsnz	menupos,F
	bra		divemode_menu_simulator_p10ft	; Adjust +10ft
	dcfsnz	menupos,F
	bra		divemode_menu_simulator_m10ft	; Adjust -10ft
	bra divemode_menu_simulator4

divemode_menu_simulator3:
#ENDIF
	dcfsnz	menupos,F
	bra		divemode_menu_simulator_p1	; Adjust +1m
	dcfsnz	menupos,F
	bra		divemode_menu_simulator_m1	; Adjust -1m
	dcfsnz	menupos,F
	bra		divemode_menu_simulator_p10	; Adjust +10m
	dcfsnz	menupos,F
	bra		divemode_menu_simulator_m10	; Adjust -10m
#IFDEF IMPERIAL
divemode_menu_simulator4:
#ENDIF
	dcfsnz	menupos,F
	bra		divemode_menu_simulator_quit; Adjust to zero m
	bra		timeout_divemenu2			; quit underwater menu!

divemode_menu_simulator_common:
	call	DISP_divemode_simulator_mask		; Redraw Simualtor mask


#IFDEF SEAWOOCH
	; Check limits (245m and 0m)
	movlw	LOW	d'25500'            ; Compare to 25.5bar=25500mbar (245m).
	subwf   sim_pressure+0,W
	movlw	HIGH	d'25500'
	subwfb  sim_pressure+1,W
	bnc     divemode_menu_simulator_common2 ; No-carry = borrow = not deeper

	; Too deep, limit to 255m
	movlw	LOW	d'25500'
	movwf	sim_pressure+0
	movlw	HIGH	d'25500'
	movwf	sim_pressure+1
#ELSE
	; Check limits (130m and 0m)
	movlw	LOW		d'14000'            ; Compare to 14bar=14000mbar (130m).
	subwf   sim_pressure+0,W
	movlw	HIGH	d'14000'
	subwfb  sim_pressure+1,W
	bnc     divemode_menu_simulator_common2 ; No-carry = borrow = not deeper

	; Too deep, limit to 130m
	movlw	LOW		d'14000'
	movwf	sim_pressure+0
	movlw	HIGH	d'14000'
	movwf	sim_pressure+1
#ENDIF
	return
divemode_menu_simulator_common2:
	movlw	LOW		d'1000'             ; Compare to 1bar == 0m == 1000 mbar.
	subwf   sim_pressure+0,W
	movlw	HIGH	d'1000'
	subwfb  sim_pressure+1,W
	btfsc   STATUS,C                    ; No-carry = borrow = not deeper.
	return                              ; Deeper than 0m == Ok.

	; Too shallow, limit to 1m
	movlw	LOW		d'1000'
	movwf	sim_pressure+0
	movlw	HIGH	d'1000'
	movwf	sim_pressure+1
	return

divemode_menu_simulator_m10:
	movlw	LOW		d'1000'
	subwf	sim_pressure+0,F
	movlw	HIGH	d'1000'
	subwfb	sim_pressure+1,F
	movlw	d'5'
	movwf	menupos						; reset cursor
#IFDEF IMPERIAL
divemode_menu_simulator_m10ft:
	movlw	LOW		d'305'
	subwf	sim_pressure+0,F
	movlw	HIGH	d'305'
	subwfb	sim_pressure+1,F
	movlw	d'5'
	movwf	menupos						; reset cursor
	bra		divemode_menu_simulator_common

divemode_menu_simulator_p10ft:
	movlw	LOW		d'305'
	addwf	sim_pressure+0,F
	movlw	HIGH	d'305'
	addwfc	sim_pressure+1,F

	movlw	d'4'
	movwf	menupos						; reset cursor
	bra		divemode_menu_simulator_common
#ENDIF
	bra		divemode_menu_simulator_common

divemode_menu_simulator_p10:
	movlw	LOW		d'1000'
	addwf	sim_pressure+0,F
	movlw	HIGH	d'1000'
	addwfc	sim_pressure+1,F

	movlw	d'4'
	movwf	menupos						; reset cursor
	bra		divemode_menu_simulator_common

divemode_menu_simulator_quit:
	movlw	LOW		d'1000'
	movwf	sim_pressure+0
	movlw	HIGH	d'1000'
	movwf	sim_pressure+1
	bra		timeout_divemenu2			; quit menu

divemode_menu_simulator_p1:
	movlw	d'100'
	addwf	sim_pressure+0,F
	movlw	d'0'
	addwfc	sim_pressure+1,F
	movlw	d'2'
	movwf	menupos						; reset cursor
	bra		divemode_menu_simulator_common

divemode_menu_simulator_m1:
	movlw	d'100'
	subwf	sim_pressure+0,F
	movlw	d'0'
	subwfb	sim_pressure+1,F
	movlw	d'3'
	movwf	menupos						; reset cursor
	bra		divemode_menu_simulator_common
#IFDEF IMPERIAL
divemode_menu_simulator_p1ft:
	movlw	d'30'
	addwf	sim_pressure+0,F
	movlw	d'0'
	addwfc	sim_pressure+1,F
	movlw	d'2'
	movwf	menupos						; reset cursor
	bra		divemode_menu_simulator_common

divemode_menu_simulator_m1ft:
	movlw	d'30'
	subwf	sim_pressure+0,F
	movlw	d'0'
	subwfb	sim_pressure+1,F
	movlw	d'3'
	movwf	menupos						; reset cursor
	bra		divemode_menu_simulator_common
#ENDIF

divemenu_see_decoplan:
	bsf		display_see_deco			; set flag
	call	DISP_clear_divemode_menu	; Clear Menu
	
	bcf		last_ceiling_gf_shown		; Clear flag
    clrf    decoplan_page               ; Starts on page 0
    bra     divemenu_see_decoplan2_1

divemenu_see_decoplan2:
	incf	decoplan_page,F
	btfsc	last_ceiling_gf_shown		; last ceiling shown?
	bra		divemenu_see_decoplan2_0	; All done, clear and return

divemenu_see_decoplan2_1:
	clrf	timeout_counter3			; Clear timeout Divemode menu
	call	DISP_decoplan   			; Display the new screen
	return

divemenu_see_decoplan2_0:
	bcf		display_see_deco			; clear flag
	bra		timeout_divemenu2			; quit menu!

divemenu_set_xgas2:
	dcfsnz	menupos,F
	bra		divemenu_set_xgas2_exit		; Use the gas6 configured and exit
	dcfsnz	menupos,F
	bra		divemenu_set_xgas2_o2plus	; Adjust O2+
	dcfsnz	menupos,F
	bra		divemenu_set_xgas2_o2minus	; Adjust O2-
	dcfsnz	menupos,F
	bra		divemenu_set_xgas2_heplus	; Adjust He+
	dcfsnz	menupos,F
	bra		divemenu_set_xgas2_heminus	; Adjust He-
	dcfsnz	menupos,F
	bra		divemenu_de_activate		; Goto (De)active gases underwater list
	return	; should never be here

divemenu_set_xgas2_heminus:
	read_int_eeprom		d'25'			; He value
	movff	EEDATA,lo
	decf	lo,F						; decrease He
	movlw	d'255'
	cpfseq	lo
	bra		divemenu_set_xgas2_heminus2
	incf	lo,F						; limit to min=0
divemenu_set_xgas2_heminus2:
	movff	lo, EEDATA
	write_int_eeprom	d'25'			; He Value

	call	DISP_divemode_set_xgas		; Redraw menu
	movlw	d'5'
	movwf	menupos						; reset cursor
	return

divemenu_set_xgas2_heplus:
	read_int_eeprom		d'25'			; He value
	movff	EEDATA,lo
	incf	lo,F						; increase He
	movlw	d'101'
	cpfseq	lo
	bra		divemenu_set_xgas2_heplus2
	movlw	d'4'						; O2 Limit
	movwf	lo
divemenu_set_xgas2_heplus2:				; test if O2+He>100...
	read_int_eeprom		d'24'			; O2 value
	movf	EEDATA,W
	addwf	lo,W						; add He value
	movwf	hi							; store in temp
	movlw	d'101'
	cpfseq	hi							; O2 and He > 100?
	bra		divemenu_set_xgas2_heplus3	; No!
	decf	lo,F						; reduce He again = unchanged after operation
divemenu_set_xgas2_heplus3:				; save current value
	movff	lo, EEDATA
	write_int_eeprom	d'25'			; He Value

	call	DISP_divemode_set_xgas		; Redraw menu
	movlw	d'4'
	movwf	menupos						; reset cursor
	return

divemenu_set_xgas2_o2minus:
	read_int_eeprom		d'24'			; O2 value
	movff	EEDATA,lo
	decf	lo,F						; decrease O2
	movlw	d'0'
	cpfseq	lo
	bra		divemenu_set_xgas2_o2minus2
    read_int_eeprom		d'25'			; Read He ratio
    movf    EEDATA,W                    ; into WREG
    sublw   .100                        ; 100% total...
    movwf   lo                          ; Set to Max. value
divemenu_set_xgas2_o2minus2:
	movff	lo, EEDATA
	write_int_eeprom	d'24'			; O2 Value

	call	DISP_divemode_set_xgas		; Redraw menu
	movlw	d'3'
	movwf	menupos						; reset cursor
	return

divemenu_set_xgas2_o2plus:
	read_int_eeprom		d'24'			; O2 value
	movff	EEDATA,lo
	incf	lo,F						; increase O2
	movlw	d'101'
	cpfseq	lo
	bra		divemenu_set_xgas2_o2plus2
	movlw	d'5'						; O2 limit
	movwf	lo
divemenu_set_xgas2_o2plus2:				; test if O2+He>100...
	read_int_eeprom		d'25'			; He value
	movf	EEDATA,W
	addwf	lo,W						; add O2 value
	movwf	hi							; store in temp
	movlw	d'100'
	cpfsgt	hi							; O2 and He > 100?
	bra		divemenu_set_xgas2_o2plus3	; No!
	decf	lo,F						; reduce O2 again = unchanged after operation
divemenu_set_xgas2_o2plus3:				; save current value
	movff	lo, EEDATA
	write_int_eeprom	d'24'			; O2 Value

	call	DISP_divemode_set_xgas		; Redraw menu
	movlw	d'2'
	movwf	menupos						; reset cursor
	return

divemenu_set_xgas2_exit:
	read_int_eeprom		d'25'			; Read He ratio
	movff	EEDATA,char_I_He_ratio		; And copy into hold register

	read_int_eeprom		d'24'			; Read O2 ratio
	movff	EEDATA, char_I_O2_ratio		; O2 ratio
	movff	char_I_He_ratio, wait_temp	; copy into bank1 register
	bsf		STATUS,C					; 
	movlw	d'100'						; 100%
	subfwb	wait_temp,W					; minus He
	subfwb	EEDATA,F					; minus O2
	movff	EEDATA, char_I_N2_ratio		; = N2!
	bsf		manual_gas_changed			; set event flag
	bsf		event_occured				; set global event flag
    bsf     decoplan_invalid            ; The decoplan needs to updated
    clrf    WREG
    movff   WREG,char_O_deco_status     ; Restart decoplan computation mH
    bsf		is_bailout					;=1: CC mode, but bailout active!
    bsf     store_bailout_event
	clrf	lo							; clear Setpoint, DISP_const_ppO2_value now displayes "Bail"
	movff	lo,char_I_const_ppO2
#IFDEF CCR_CTRL     
    ; Set add variable and send command to eCTRL
	movff	lo, eCTRL_SP_DC             ; Store also in this byte...
    btfss   ext_ppO2_enable             ; Check for eCTRL mode
    bra     divemenu_set_xgas2_exit1    ; skip if not in eCTRL mode
    movlw   eCTRL_CMD_ACT
    movff   WREG, eCTRL_CMD             ; Send ACT (SP) request with next message
divemenu_set_xgas2_exit1:
#ENDIF
    movlw   6
    movff   WREG,char_I_current_gas     ; Current gas is Gas6 (manual setting).
	bra		timeout_divemenu2			; quit underwater menu!

divemenu_set_gas:
	btfsc	FLAG_const_ppO2_mode		; in ppO2 mode?
	bra		divemenu_set_setpoint		; Yes, display SetPoint list

divemenu_set_gas_2:
	bsf		display_set_gas				; set flag	
	call	DISP_clear_divemode_menu	; Clear Menu
	call	DISP_gas_list				; Display all 5 gases

	movlw	d'1'						; Reset cursor
	btfsc	better_gas_available		;=1: A better gas is available and a gas change is advised in divemode
	movf	better_gas_number,W			; better gas 1-5
	movwf	menupos						; reset cursor
	call	DISP_divemenu_cursor		; update cursor
	call	DISP_show_change_depth		; And show the first change depth
	return

divemenu_set_setpoint:
	bsf		display_set_setpoint		; set flag	
	bsf		display_set_gas				; set flag	

	call	DISP_clear_divemode_menu	; Clear Menu
	call	DISP_splist_start			; Display SetPoints
	DISPLAYTEXT d'137'  				; Bailout (as a sub-menu)
    DISPLAYTEXT d'232'                  ; Diluent (as a sub-menu)
#IFDEF CCR_CTRL
    DISPLAYTEXTH	.393                ; More
#ENDIF

	movlw	d'1'
	movwf	menupos						; reset cursor
	call	DISP_divemenu_cursor		; update cursor
	return


divemenu_set_gas2:
	btfsc	select_bailoutgas			; Are we in the Bailout list?
	bra		divemenu_set_gas2a			; Yes, choose gas

	btfss	FLAG_const_ppO2_mode		; are we in ppO2 mode?
	bra		divemenu_set_gas2a			; no, choose gas
	; Yes, so select SP 1-3
	
divemenu_set_gas1:

#IFDEF CCR_CTRL
divemenu_set_gas1aa:
	movlw	d'1'				
	cpfseq	menupos						; At the "Bailout" position?		
	bra		divemenu_set_gas1b			; No, select SetPoint 1-3 or Diluent

    bsf		select_bailoutgas			; Set Flag
	bcf		display_set_setpoint		; Clear Flag
    btfsc   is_bailout                  ; Already in bailout?
	bra		divemenu_set_gas_2			; Yes.
    
    ; Setup first gas as better gas
    bsf     better_gas_available
    read_int_eeprom .33                 ; 1-5
    movff   EEDATA,better_gas_number
	bra		divemenu_set_gas_2			; Configure the extra gas / Select Bailout

divemenu_set_gas1b:
	bcf		is_bailout					; =1: CC mode, but bailout active!
	call	DISP_show_ppO2_clear		; Clear ppO2 value
	movlw	d'5'
	cpfseq	menupos						; At the "Diluent" position?
	bra		divemenu_set_gas1c			; No, select SetPoint 1-3
    ; Choose Diluent from list
    bcf		display_set_setpoint		; Clear Flag
    bcf     display_set_gas             ; Clear Flag
    bsf     display_set_diluent         ; Set Flag
	call	DISP_clear_divemode_menu	; Clear Menu
	call	DISP_diluent_list			; Display all 5 diluents
	movlw	d'1'						; Reset cursor
	movwf	menupos						; reset cursor
	call	DISP_divemenu_cursor		; update cursor
	return
#ENDIF

divemode_set_diluent2:                  ; Choose diluent #menupos
    movff  menupos,active_diluent       ; 1-5
    bra    divemenu_set_gas1d           ; Continue here...

#IFDEF CCR_CTRL
divemode_set_more2:                     ; Choose more action

    ; action by right button in Setup/More menu
#IFDEF CCR_CTRL_DBG_P
	dcfsnz	menupos,F
	bra		divemode_set_more2_inc_p    ; Inc P
	dcfsnz	menupos,F
	bra		divemode_set_more2_dec_p    ; Dec P
#ELSE
	dcfsnz	menupos,F
	bra		divemode_set_more2_inc_SP4  ; Inc SP3
	dcfsnz	menupos,F
	bra		divemode_set_more2_sel_SP4  ; Dec SP3
#ENDIF

	dcfsnz	menupos,F
	bra		divemode_set_more2_ppO2_src
	dcfsnz	menupos,F
	bra		divemode_set_more2_sol_ctrl
	bra		divemode_set_more2_exit

#IFDEF CCR_CTRL_DBG_P
divemode_set_more2_inc_p:
    movff   eCTRL_P,lo
    incf    lo, f
    movff   lo, eCTRL_P
    call    DISP_more_p_plus
    call    DISP_more_p_minus
	movlw	d'1'
	movwf	menupos
    return

divemode_set_more2_dec_p:
    movff   eCTRL_P,lo
    decf    lo, f
    movff   lo, eCTRL_P
    call    DISP_more_p_plus
    call    DISP_more_p_minus
	movlw	d'2'
	movwf	menupos
    return
#ELSE
divemode_set_more2_inc_SP4:
    movff   eCTRL_SP3_adj,lo
    movlw   d'10'                       ; Increase on 0.1
    addwf   lo,F
    movlw   eCTRL_ADJ_MIN               ; min. allowable
    cpfsgt  lo
    movff   WREG, lo
    movlw   eCTRL_ADJ_MAX               ; Compare with max allowed
    cpfsgt  lo
    bra     divemode_set_more2aa        ; Continue here...
    movlw   eCTRL_ADJ_MIN               ; turn over to begin from lowest allowable
    movff   WREG, lo
divemode_set_more2aa:
    movff   lo, eCTRL_SP3_adj           ; Load with new value
    call    DISP_more_SP4

	movlw	d'1'
	movwf	menupos
    return

divemode_set_more2_sel_SP4:
	movff	eCTRL_SP3_adj, char_I_const_ppO2	; Use SetPoint
	movff	eCTRL_SP3_adj, ppO2_setpoint_store	; Store also in this byte...
	movff	eCTRL_SP3_adj, eCTRL_SP_DC            ; Store also in this byte...
    bra     divemode_set_more2_exit
#ENDIF

divemode_set_more2_ppO2_src:
    btg     ext_ppo2_abandon
    call    DISP_more_ppo2
	movlw	d'3'
	movwf	menupos
    return
divemode_set_more2_sol_ctrl:
    btg     CTRL_sol_off_mode
    call    DISP_more_sol
	movlw	d'4'
	movwf	menupos
    movlw   eCTRL_CMD_ACT
    movff   WREG, eCTRL_CMD               ; Send ACT (SP) request with next message
    return
divemode_set_more2_exit:
	movlw	d'1'
	movwf	menupos
	bra		timeout_divemenu2			; quit menu!
#ENDIF

divemenu_set_gas1c:
#IFDEF CCR_CTRL
	movlw	d'6'
	cpfseq	menupos						; At the "More" position?
	bra		divemenu_set_gas1c2			; No, select SetPoint 1-3
    ; Choose Diluent from list
    bcf		display_set_setpoint		; Clear Flag
    bcf     display_set_gas             ; Clear Flag
    bsf     display_set_more            ; Set Flag
	call	DISP_clear_divemode_menu	; Clear Menu
	call	DISP_more_list	  		    ; Display More menu
	movlw	d'1'						; Reset cursor
	movwf	menupos						; reset cursor
	call	DISP_divemenu_cursor		; update cursor
	return

divemenu_set_gas1c2:
#ENDIF
	decf	menupos,F					; Adjust 1-3 to 0-2...
	movlw	d'35'						; offset in memory
	addwf	menupos,W					; add SP number 0-2
	movwf	EEADR
	call	read_eeprom					; Read SetPoint
#IFDEF CCR_CTRL
    movff   EEDATA,eCTRL_TMP
divemenu_set_gas1c3:
	movff	eCTRL_TMP, char_I_const_ppO2	; Use SetPoint
	movff	eCTRL_TMP, ppO2_setpoint_store	; Store also in this byte...
	movff	eCTRL_TMP, eCTRL_SP_DC          ; Store also in this byte...
    btfss   ext_ppO2_enable                 ; Check for eCTRL mode
    bra     divemenu_set_gas1c4             ; skip if not in eCTRL mode
    movlw   eCTRL_CMD_ACT
    movff   WREG, eCTRL_CMD                 ; Send ACT (SP) request with next message
divemenu_set_gas1c4:
#ELSE
	movff	EEDATA, char_I_const_ppO2	; Use SetPoint
	movff	EEDATA, ppO2_setpoint_store	; Store also in this byte...
#ENDIF
	bsf		setpoint_changed
	bsf		event_occured				; set global event flag

divemenu_set_gas1d:                     ; (Re-)Set Diluent
    decf   active_diluent,W             ; 0-4 -> WREG mH
    mullw   .4
    movf    PRODL,W
    addlw   d'98'
    movwf   EEADR
    call	read_eeprom					; Read He
	movff	EEDATA,char_I_He_ratio		; And copy into hold register
    decf   active_diluent,W             ; 0-4 -> WREG
    mullw   .4
    movf    PRODL,W
    addlw   d'97'
    movwf   EEADR
    call	read_eeprom					; Read O2
	movff	EEDATA, char_I_O2_ratio		; O2 ratio
	movff	char_I_He_ratio, wait_temp	; copy into bank1 register
	bsf		STATUS,C					; Borrow bit
	movlw	d'100'						; 100%
	subfwb	wait_temp,W					; minus He
	bsf		STATUS,C					; Borrow bit
	subfwb	EEDATA,W					; minus O2
	movff	WREG, char_I_N2_ratio		; = N2!
	bra		timeout_divemenu2			; quit menu!

divemenu_set_gas1a:
	bcf		display_set_setpoint		; Clear Display Flag
	bsf		stored_gas_changed			; set event flag
	bsf		event_occured				; set global event flag
    bsf     decoplan_invalid            ; The decoplan needs to updated
    btfsc   better_gas_available        ; If a gas change was planned...
    bsf     gaschange_cnt_active        ; Show the countdown
    clrf    WREG
    movff   WREG,char_O_deco_status     ; Restart decoplan computation mH
	bra		timeout_divemenu2			; quit menu!

divemenu_set_gas2a:
	movlw	d'6'				
	cpfseq	menupos						; At the "Gas 6.." position?		
	bra		divemenu_set_gas2b			; No, select Gas1-5 (Stored in Menupos)
	bra		divemode_set_xgas			; Configure the extra gas

divemenu_set_gas2b:
    btfss   FLAG_const_ppO2_mode        ; In CCR mode ?
    bra     divemenu_set_gas2c          ; No
	bsf		is_bailout					;=1: CC mode, but bailout active!
    bsf     store_bailout_event
	bsf		event_occured				; set global event flag
divemenu_set_gas2c:
	clrf	lo							; clear Setpoint, DISP_const_ppO2_value now displayes "Bail"
	movff	lo,char_I_const_ppO2		
#IFDEF CCR_CTRL
	movff	lo, eCTRL_SP_DC             ; Store also in this byte...
    btfss   ext_ppO2_enable             ; Check for eCTRL mode
    bra     divemenu_set_gas2d          ; skip if not in eCTRL mode
    movlw   eCTRL_CMD_ACT
    movff   WREG, eCTRL_CMD             ; Send ACT (SP) request with next message
divemenu_set_gas2d:
#ENDIF

	bcf		display_set_gas				; clear flag
	movff	menupos,active_gas			; copy into active gas register
	decf	menupos,W					; # of selected gas (0-4)
	mullw	d'4'						; times 4...
	movf	PRODL,W						;
	addlw	d'7'						; +7 = address for He ratio
	movwf	EEADR
	call	read_eeprom					; Read He ratio
	movff	EEDATA,char_I_He_ratio		; And copy into hold register

	decf	menupos,W					; # of selected gas (0-4)
	mullw	d'4'						; times 4...
	movf	PRODL,W						;
	addlw	d'6'						; +6 = address for O2 ratio
	movwf	EEADR
	call	read_eeprom					; Read O2 ratio
	movff	EEDATA, char_I_O2_ratio		; O2 ratio
	movff	char_I_He_ratio, wait_temp	; copy into bank1 register
	bsf		STATUS,C					; 
	movlw	d'100'						; 100%
	subfwb	wait_temp,W					; minus He
	subfwb	EEDATA,F					; minus O2
	movff	EEDATA, char_I_N2_ratio		; = N2!
	bsf		stored_gas_changed			; set event flag
	bsf		event_occured				; set global event flag
    btfsc   better_gas_available        ; If a gas change was planned...
    bsf     gaschange_cnt_active        ; Show the countdown
    bsf     decoplan_invalid            ; The decoplan needs to updated
    clrf    WREG
    movff   WREG,char_O_deco_status     ; Restart decoplan computation mH

    movff   menupos,char_I_current_gas  ; Inform deco code too.
	bra		timeout_divemenu2			; quit menu!
	
timeout_divemenu:
	btfss	menubit					; is the Dive mode menu displayed?
	return							; No

	btfsc	display_set_simulator	; Is the Simulator Mask active?
	bra		timeout_divemenu6		; Yes, update Simulator mask
	
	btfss	display_see_deco		; Is the decoplan active?
	bra		timeout_divemenu1		; No, skip updating the decoplan
	bra		timeout_divemenu3	    ; Yes...
	
timeout_divemenu1:	
	incf	timeout_counter3,F		; increase timeout_counter3
	GETCUSTOM8	d'10'				; loads timeout_divemenu into WREG
	cpfsgt	timeout_counter3		; ... longer then timeout_divemenu
	return							; No!

timeout_divemenu2:					; quit divemode menu
; Restore some outputs
	clrf	decoplan_page           ; Page 0-1 of deco list
	call	DISP_clear_divemode_menu; Clear dive mode menu

	btfsc	FLAG_apnoe_mode				; Ignore in Apnoe mode
	bra		timeout_divemenu2b			; skip!
	btfsc	gauge_mode					; Ignore in Gauge mode
	bra		timeout_divemenu2b			; skip!

	bcf		menubit
	btfsc	dekostop_active
	call	DISP_display_deko_mask	; clear nostop time, display decodata
	btfss	dekostop_active
	call	DISP_display_ndl_mask	;  Clear deco data, display nostop time

    btfsc   decoplan_invalid        ; The decoplan needs to updated
    bra     timeout_divemenu2a      ; Yes, skip update

	btfsc	dekostop_active
	call	DISP_display_deko		; Update deco display at once
	btfss	dekostop_active
	call	DISP_display_ndl		; Update NDL display at once

timeout_divemenu2a:
	btfsc	safety_stop_active
	bcf		safety_stop_active		; Clear flag to rebuild the safety stop

timeout_divemenu2b:
	bcf		menubit
	bcf		premenu					; Yes, clear flags and menu, display dive time and mask again
	call	DISP_active_gas_divemode; Display gas, if required
	call	DISP_divemode_mask		; Display mask
	call	DISP_divemins			; Display (new) divetime!
	call	customview_mask			; Redraw current customview mask
	clrf	timeout_counter3		; Also clear timeout
	bcf		display_see_deco		; clear all display flags
	bcf		display_set_gas			
	bcf		display_set_xgas
	bcf		display_set_setpoint
#IFDEF CCR_CTRL
	bcf		display_set_more        ; Clear flag
#ENDIF
	bcf		display_set_simulator
	bcf		display_set_active
    bcf     display_set_diluent
	bcf		menu3_active
	call	wait_switches		; Waits until switches are released, resets flag if button stays pressed!
	return

; Re-Draw current page of decoplan (may have more stops)
timeout_divemenu3:
    movff   char_O_deco_status,WREG     ; Get last computation state (BANK safe)
    iorwf   WREG                        ; Is it zero ?
    btfsc   STATUS,Z
	call	DISP_decoplan               ; Yes: new data available.
	bra		timeout_divemenu1			; Check timeout

timeout_divemenu6:
	; Update Simulator Mask
	bsf		menu3_active				; So "+1" is accessible at all times
	call	DISP_divemode_simulator_mask; Show mask
	call	DISP_divemenu_cursor		; update cursor
	bra		timeout_divemenu1			; Check timeout

