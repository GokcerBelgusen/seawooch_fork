;=============================================================================
;
;    File color_processor.asm
;
;    Decompress and draw an image.
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
;  2010-12-13 : [jDG] Creation.
;  2010-12-30 : [jDG] Revised to put temp into ACCESSRAM0
;
; RATIONALS: The OSTC have a nice color screen, and a std geek attitude impose
;            to show off ... ;-)
;
; Inputs: TBLPTR points to the image description block.
;         win_top, win_leftx2 the Top/Leftx2 corner here to put the image.
; Ouputs: None.
; Trashed: TBLPTR, TABLAT, FSR2, PROD, aa_width, aa_height
;
; ImageBloc:
;       db  widthx2, height
;       db  nbColors, 0     ; Unused yet... Should be 0 to keep packing happy.
;       dw  color0, color1, color2, color3, ...
;       db  ...packed pixels...
;
; Limitations:
; * nbColors should be <= 15.
; * image width should be even.
; * image left border should be on even position too.
; Compressed format:
; - Upper nibble = color, lower nibble = count-1.
; - All bytes F* accumulates to make count larger than 16.
; Eg. 00          is 1 pixel  color 0
;     07          is 8 pixels color 0
;     70          is 1 pixel  color 7
;     bf          is 16 pixels of color .11
;     F1 F2 F3 04 is 0x1235 pixels of color 0.
;
;Temporary overlay (in bank 0), ACCESS area
    CBLOCK  0x000
        img_colors
        img_width:2                     ; SHOULD be @1, because of DISP_box_write
        img_pixelsL
        img_pixelsH
        img_pixelsU
        img_countL
        img_countH
        colorTable:.30
    ENDC

;-----------------------------------------------------------------------------
color_image:
        movlb   HIGH(win_height)        ; Switch to bank 0.

        ;---- Get image parameters -------------------------------------------
        tblrd*+
        movff   TABLAT,img_width
        tblrd*+
        movff   TABLAT,win_height
        tblrd*+
        movff   TABLAT,img_colors
        tblrd*+
        ;---- Copy color table -----------------------------------------------
        movf    img_colors,W
        lfsr    FSR2,colorTable
get_colors_loop:
        tblrd*+
        movff   TABLAT,POSTINC2
        tblrd*+
        movff   TABLAT,POSTINC2
        decfsz  WREG
        bra     get_colors_loop

        ; Compute width * height * 2 : the number of pixels to write.
        clrf    img_pixelsU
        movf    img_width,W,ACCESS      ; Compute number of pixels to draw
        mulwf   win_height              ; 0 .. 160x240
        bcf     STATUS,C                ; BEWARE: milw does not reset carry flag !
        rlcf    PRODL                   ; x2 --> 0 .. 320x240, might by > 0xFFFF
        rlcf    PRODH
        movff   PRODL, img_pixelsL
        movff   PRODH, img_pixelsH
        rlcf    img_pixelsU             ; Get the upper bit in place.

        clrf    WREG                    ; Decrement count to ease end detection.
        decf    img_pixelsL,F
        subwfb  img_pixelsH,F
        subwfb  img_pixelsU,F

        ;---- Send window command --------------------------------------------
        clrf    img_width+1             ; x2 on width, for the true box size.
        rlcf    img_width+0
        rlcf    img_width+1
        call    DISP_box_write
        AA_CMD_WRITE 0x22

        ;---- Decode pixels --------------------------------------------------
color_image_loop_xy:
        ; Get pixel count
        clrf    img_countL
        clrf    img_countH

        ;---- Decode repetition count
color_image_decode_1:
        tblrd*+                         ; Get one byte

        btfss   TABLAT,7                ; High bit cleared ?
        bra     color_image_decode_2    ; YES: this is a color byte.

        rlcf    TABLAT,F                ; Drop high bit.
        movlw   .7                      ; Move 7 bits
color_image_decode_3:
        rlcf    TABLAT,F                ; Get bit into carry
        rlcf    img_countL,F           ; Push into pixel count
        rlcf    img_countH,F
        decfsz  WREG
        bra     color_image_decode_3    ; and loop foreach 7 bits.

        bra     color_image_decode_1    ; Decode next byte.

color_image_decode_2:
        ;---- Get pixel color into PROD
        movf    TABLAT,W                ; Get color index.
        addwf   WREG                    ; *2
        lfsr    FSR2,colorTable         ; Reinitialize color table.
        movff   WREG,FSR2L              ; LOW(buffer) == 0
        movff   POSTINC2,PRODL
        movff   POSTINC2,PRODH

        ; Substract count-1 from the number of pixel we should do.
        movf    img_countL,W           ; Make a 24bit substraction.
        subwf   img_pixelsL,F
        movf    img_countH,W
        subwfb  img_pixelsH,F
        movlw   0
        subwfb  img_pixelsU,F

color_image_not_over:
        infsnz  img_countL             ; Increment count.
        incf    img_countH

        ; Loop sending pixel color
        incf    img_countH             ; Because we decrement first, should add one here !
color_image_loop_pixel:
        AA_DATA_WRITE_PROD
        decfsz  img_countL
        bra     color_image_loop_pixel
        decfsz  img_countH
        bra     color_image_loop_pixel

        ; And count (on a 24bit counter)
        clrf    WREG                    ; Make a 24bit decrement.
        decf    img_pixelsL
        subwfb  img_pixelsH,F
        subwfb  img_pixelsU,F

        bnn     color_image_loop_xy     ; Not finished ? loop...

        ;---- Closeup --------------------------------------------------------
        AA_CMD_WRITE 0x00
        return
