;=============================================================================
;
;    File strings.inc
;
;    Implementation code various string functions.
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
;    Copyright (c) 2010, JD Gascuel.
;=============================================================================
; HISTORY
;  2010-12-02 : [jDG] Creation...
;
; RATIONALS for STRCAT / STRCPY / STRCAT_PRINT / STRCPY_PRINT:
;  * Gain PROM space: each time a string operation is done, we have a bunch
;    of lines in the style:
;           movlw   '+'
;           movwf   POSTINC2
;           movlw   '/'
;           movwf   POSTINC2
;           movlw   '-'
;           movwf   POSTINC2
;           movlw   ':'
;           movwf   POSTINC2
;           call    word_processor
;    which takes 20 bytes of PROM space (eg. 4 + chars/4). Whereas :
;           STRCAT_PRINT  "+/-:"
;    takes just 10 bytes (4 + chars + 1, even rounded up).
;
;    Note that the other common sequence :
;           OUTPUTTEXT  .113
;    does compile in (6 bytes per call), plus (1 + chars, even rouded up)
;    once it the whole code. Also, it is slower to execute, because it have
;    to roll through the string table.
;
; RATIONALS for a formating variant... not yet done... 
;  * Embeding formatting commands into the string allows even mode compact
;    sequence.
;    Possible commandes:
;           \xF0    output_8            ; arg in hi, and move lo to hi,
;           \xF1    output_9            ; so we can have two 8bits concertion
;           \xF2    output_99           ; in a single sequence...
;           \xF3    output_99x
;           \xF4    output_16           ; arg in hi:lo
;           \xF5    output_16dp 1
;           \xF6    output_16dp 2
;           \xF7    output_16dp 3
;           ....
;    Usage:
;           movff       apnoe_min, hi
;           movff       apnoe_sec, lo
;           FORMAT_PRINT "Time: \xF3:\xF3"
;
;  * Cons: code redeability is poor: reader have to remember exactly what
;    each cryptic hexa code is doing...
;    No-macro-syntax:
;           call    format_print_block
;           DB  "Time: ", FORMAT_99x, ':', FORMAT_99x, 0
;    is denitively more secure in that respect...

;=============================================================================
; Copy a short embebed string at start of the string buffer (letter)
; Input:    (nothing)
; Output:   chars are copied into letter, starting at the beginning.
;           FSR2 point to the first unused char (the NULL termination).
;           
; Trashed:  WREG, TBLPTR, TABLAT, PRODL
; Note:     This are all bank-safe call.
;
STRCPY  macro   string
        call    strcpy_block
        DB  string, 0
        endm

;=============================================================================
; A variant of STRCPY that appends chars to the current FSR2 pointer.
; Input/Output/Trashed : see STRCPY.
STRCAT  macro   string
        call    strcat_block
        DB  string, 0
        endm

;=============================================================================
; A variant of STRCAT when there is just on char to output
; Input/Output/Trashed : none.
PUTC    macro   char
        movlw   char
        movwf   POSTINC2
        endm

;=============================================================================
; A variant of STRCPY that send the string to the word processor afterwards.
; Input/Output: see STRCPY.
; Trashed: See STRCPY + word_processor. In particular, switch RAM to Bank1 
;
STRCPY_PRINT    macro string
        call    strcpy_block_print
        DB  string, 0
        endm

; Trashed: See STRCPY + word_processor. In particular, switch RAM to Bank1 
STRCAT_PRINT    macro string
        call    strcat_block_print
        DB  string, 0
        endm

;=============================================================================
; A shortcut for the macros WIN_TOP + WIN_LEFT + WIN_FONT + WIN_INVERT.
; The idea is to replace a 4x6=24 bytes sequence by a more compact 6bytes one.
;
; Trashed: TBLPTR, TABLAT, WREG.
; Note:    This are all bank-safe call.

WIN_SMALL macro x, y
        call    start_small_block
        DB      x, y
        endm

WIN_SMALL_INVERT macro x, y
        call    start_small_invert_block
        DB      x, y
        endm

WIN_MEDIUM   macro   x, y
        call    start_medium_block
        DB      x, y
        endm

WIN_MEDIUM_INVERT   macro   x, y
        call    start_medium_invert_block
        DB      x, y
        endm

WIN_LARGE   macro   x, y
        call    start_large_block
        DB      x, y
        endm

WIN_LARGE_INVERT   macro   x, y
        call    start_large_invert_block
        DB      x, y
        endm

;=============================================================================
; A shortcut for PLED_box and PLED_frame call sequences.
;
; Erase a given screen area.
WIN_BOX_BLACK macro   top, bottom, left, right
        call    box_black_block
        db      top, (bottom)-(top)+1, left, (right)-(left)+1
        endm

; Fill a given screen area with standard color (CF35).
WIN_BOX_STD macro   top, bottom, left, right
        call    box_std_block
        db      top, (bottom)-(top)+1, left, (right)-(left)+1
        endm

; Fill a given screen area with color from WREG (8bits rrrgggbb)
WIN_BOX_COLOR macro   top, bottom, left, right
        call    box_color_block
        db      top, (bottom)-(top)+1, left, (right)-(left)+1
        endm

; Draw a frame in standard color (CF35).
WIN_FRAME_STD macro   top, bottom, left, right
        call    box_frame_std
        db      top, (bottom)-(top)+1, left, (right)-(left)+1
        endm

; Draw a frame with color from WREG (8bits rrrgggbb)
WIN_FRAME_COLOR macro   top, bottom, left, right
        call    box_frame_color
        db      top, (bottom)-(top)+1, left, (right)-(left)+1
        endm


