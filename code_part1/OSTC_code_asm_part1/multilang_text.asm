
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


; hold texts and parameters for the texts
; written by: Matthias Heinrichs, info@heinrichsweikamp.com
; History:
; 2007/10/13: written
; 2008/05/24: Last update Matthias
; 2011/02/02: moving texts to english_text.asm to prepare multilingual.
; known bugs:
; ToDo: 
#IFDEF SEAWOOCH
;texts code_pack    0x15800
texts code_pack    0x16000
#ELSE
texts code_pack    0x15000
#ENDIF

TCODE_1 macro   x, y, text
    ; Compile-time checking on provided parameters:
    If x<0 || x>.159
        Error "Bad X coordinate ", #v(tcode_idx), x
    Endif
    If y<0 || y>.239
        Error "Bad Y coordinate ", y
    Endif
        dw      tcode_ptr_#v(tcode_idx)
        db      0+x, 0+y
tcode_idx set tcode_idx+1
        endm

TCODE_2 macro   x, y, text
tcode_ptr_#v(tcode_idx):
        db      text, 0
tcode_idx set tcode_idx+1
        endm

;---- Manage language -------------------------------------------------------
; Compile with ASM macro definition GERMAN=1 to use another
; file...
#ifdef ITALIAN
#define LANGUAGE_FILE "italian_text.asm"
#endif
#ifdef SPANISH
#define LANGUAGE_FILE "spanish_text.asm"
#endif
#ifdef GERMAN
#define LANGUAGE_FILE "german_text.asm"
#endif
#ifdef FRENCH
#define LANGUAGE_FILE "french_text.asm"
#endif
#ifdef RUSSIAN
#define LANGUAGE_FILE "russian_text.asm"
#endif
#ifdef TURKISH
#define LANGUAGE_FILE "turkish_text.asm"
#endif
#ifndef LANGUAGE_FILE
#define LANGUAGE_FILE "english_text.asm"
#endif

;---- PASS 1 : generate description block ------------------------------------
text_pointer:

tcode_idx   set     1
#define     TCODE TCODE_1
#include    LANGUAGE_FILE
#undefine   TCODE

;---- PASS 2 : generate text contens -----------------------------------------
tcode_idx   set     1
#define     TCODE TCODE_2
#include    LANGUAGE_FILE
#undefine   TCODE

            code

