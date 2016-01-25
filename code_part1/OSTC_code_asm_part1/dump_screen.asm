;=============================================================================
;
;    File dump_screen.asm
;
;    Dump screen contains to the serial interface.
;
;    This program is free software: you can redistribute it and/or modify
;    it under the terms of the GNU General Public License as published by
;    the Free Software Foundation, either version 3 of the License, or
;    (at your option) any later version.
;
;    This program is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;    GNU General Public License for more details.
;
;    You should have received a copy of the GNU General Public License
;    along with this program.  If not, see <http://www.gnu.org/licenses/>.
;
;    Copyright (c) 2011, JD Gascuel.
;=============================================================================
; HISTORY
;  2011-05-08 : [jDG] Creation.
;
; BUGS :
;  * ...
;=============================================================================
    CBLOCK 0x000                        
        	ds_line                 ; Current line (0..239).
        	ds_column               ; Current columnx2 (0..159)
        	ds_pixel:2              ; Current pixel color.
        	ds_count                ; Repetition count.
    ENDC
;=============================================================================

; Manage interface to the OSTC platform:
dump_screen:
	bcf		    uart_dump_screen        ; clear flag!

	movlw	    'l'
	movwf	    TXREG                   ; Send command echo.
	bsf		    no_sensor_int           ; No Sensor Interrupt
	bcf		    PIE1,RCIE               ; no interrupt for UART
	bcf		    PIR1,RCIF               ; clear flag
	bsf		    LED_blue                ; LEDusb ON
	call		rs232_wait_tx           ; wait for UART

    movff       win_flags,WREG          ; BEWARE: bank0 bit-test
    btfss       WREG,1                  ; Display1?
    call        dump_screen_0           ; No, dump screen

	bcf		    no_sensor_int           ; Restore Sensor Interrupt
	bcf			LED_blue                ; Clear led
	bcf			LED_red                 ; Clear led
	bsf			PIE1,RCIE               ; Interrupt for RS232
	return

;=============================================================================
; Dump screen contains to the UART

dump_screen_0:

    ;---- Send DISPLAY box command for the full screen window -------------------
    mullw       0                       ; PRODH:L <- 0

    AA_CMD_WRITE    0x35                ; VerticalStartAddress HIGH:LOW
    AA_DATA_WRITE_PROD                  ; 00:00

    AA_CMD_WRITE    0x36                ; VerticalEndAddress HIGH:LOW
    AA_DATA_WRITE   0x01
    AA_DATA_WRITE   0x3F

    AA_CMD_WRITE    0x37                ; HorizontalAddress START:END
    AA_DATA_WRITE   0x00
    AA_DATA_WRITE   0xEF

    btfss       win_flip_screen         ; OSTC 2N have a flipped screen,
    bra         dump_screen_mk2         ; So we should start 239 instead.
    movlw       LOW(.239)
    movwf       PRODL
    movlw       HIGH(.239)
    movwf       PRODH
dump_screen_mk2:    

    AA_CMD_WRITE    0x20                ; Start Address Horizontal (.0 - .239)
    AA_DATA_WRITE_PROD                  ; 00:00

    mullw       0                       ; Make sure PROD is 0 again.
    AA_CMD_WRITE    0x21                ; Start Address Vertical (.0 - .319)
    AA_DATA_WRITE_PROD                  ; 00:00

    AA_CMD_WRITE    0x22                ; Start reading.
    rcall       DISP_DataRead           ; Dummy pixel to skip.
    rcall       DISP_DataRead           ; Dummy pixel to skip.

   	movlw	    .160                    ; 160x2 columns
	movwf	    ds_column
    rcall       dump_screen_pixel_reset

dump_screen_1:
    btg         LED_red                 ; LEDactivity toggle

    AA_CMD_WRITE    0x22                ; Re-sync data.

    setf        TRISD                   ; PortD as input.

    ; Dump even column
	movlw	    .240                    ; 240 lines, once.
	movwf	    ds_line
dump_screen_2:
    rcall       DISP_DataRead           ; read pixel-high byte
    movwf       PRODH
    rcall       DISP_DataRead           ; read pixel-low byte
    movwf       PRODL
    rcall       dump_screen_pixel

    decfsz	    ds_line,F
    bra		    dump_screen_2
    rcall       dump_screen_pixel_flush

    ; Dump odd column
	movlw	    .240                    ; 240 lines, twice.
	movwf	    ds_line
dump_screen_3:
    rcall       DISP_DataRead           ; read pixel-high byte
    movwf       PRODH
    rcall       DISP_DataRead           ; read pixel-low byte
    movwf       PRODL
    rcall       dump_screen_pixel

    decfsz	    ds_line,F
    bra		    dump_screen_3
    rcall       dump_screen_pixel_flush

    clrf        TRISD                   ; Back to normal (PortD as output)

    decfsz	    ds_column,F
    bra		    dump_screen_1

    AA_CMD_WRITE    0x00                ; NOP, to stop Address Update Counter
    return

;=============================================================================
; Pixel compression
;
; Input: PRODH:L = pixel.
; Output: Compressed stream on output.
; Compressed format:
;       0ccccccc    : BLACK pixel, repeated ccccccc+1 times (1..128).
;       11cccccc    : WHITE pixel, repeated cccccc+1 times (1..64).
;       10cccccc HIGH LOW : color pixel (H:L) repeated ccccc+1 times (1..64).
;
dump_screen_pixel:
    movf        PRODH,W                 ; Compare pixel-high
    xorwf       ds_pixel+1,W
    bnz         dump_screen_pixel_1     ; Different -> dump.

    movf        PRODL,W                 ; Compare pixel-low
    xorwf       ds_pixel+0,W
    bnz         dump_screen_pixel_1     ; Different -> dump.

    incf        ds_count,F              ; Same color: just increment.
    return

dump_screen_pixel_1:                    ; Send (pixel,count) tuple
    movf        ds_count,W              ; Is count zero ?
    bz          dump_screen_pixel_2     ; Yes: skip sending.

    movf        ds_pixel+1,W            ; This is a BLACK pixel ?
    iorwf       ds_pixel+0,W    
    bz          dump_screen_pix_black   ; YES.

    movf        ds_pixel+1,W            ; This is a white pixel ?
    andwf       ds_pixel+0,W
    incf        WREG
    bz          dump_screen_pix_white   ; YES.

    ; No: write the pixel itself...
    movlw       .64                     ; Max color pixel on a single byte.
    cpfsgt      ds_count                ; Skip if count > 64
    movf        ds_count,W              ; W <- min(64,count)
    subwf       ds_count,F              ; ds_count <- ds_count-W
    decf        WREG                    ; Save as 0..63
    iorlw       b'10000000'             ; MARK as a color pixel.

    movwf       TXREG
    call		rs232_wait_tx           ; wait for UART
    movff       ds_pixel+1,TXREG
    call		rs232_wait_tx           ; wait for UART
    movff       ds_pixel+0,TXREG
    call		rs232_wait_tx           ; wait for UART
    bra         dump_screen_pixel_1

dump_screen_pixel_2:
    movff       PRODH,ds_pixel+1        ; Save new pixel color
    movff       PRODL,ds_pixel+0
    movlw       1
    movwf       ds_count                ; And set count=1.
    return

dump_screen_pix_black:
    movlw       .128                    ; Max black pixel on a single byte.
    cpfsgt      ds_count                ; Skip if count > 128
    movf        ds_count,W              ; W <- min(128,count)
    subwf       ds_count,F              ; ds_count <- ds_count-W
    decf        WREG                    ; Save as 0..127
dump_screen_pix_3:
    movwf       TXREG
    call        rs232_wait_tx
    bra         dump_screen_pixel_1     ; More to dump ?

dump_screen_pix_white:
    movlw       .64                     ; Max white pixel on a single byte.
    cpfsgt      ds_count                ; Skip if count > 64
    movf        ds_count,W              ; W <- min(64,count)
    subwf       ds_count,F              ; ds_count <- ds_count-W
    decf        WREG                    ; Save as 0..63
    iorlw       b'11000000'             ; MARK as a compressed white.
    bra         dump_screen_pix_3

dump_screen_pixel_flush:
    clrf        PRODH
    clrf        PRODL
    rcall       dump_screen_pixel_1     ; Send it
dump_screen_pixel_reset:
    clrf        ds_count                ; But clear count.
    return