;=============================================================================
;
;    File aa_fonts.asm
;
;    Font-data for the anti-aliased word processor
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
;  2010-11-23 : [jDG] Creation with the original 1.72 fonts repacked.
;  2010-12-01 : [jDG] Adding 3bits antialiased fonts.
;=============================================================================

; Original fonts where byte swapped in PROM memory, but the repacked don't...
; AA_BYTE_SWAP		EQU	1

;---- SMALL font description and data ----------------------------------------
aa_fonts	code_pack
#ifndef RUSSIAN
aa_font28_block:
			DB	'°', 0x7F               ; Remap a few ASCII chars, to avoid
			DB	'ö', 0x80               ; holes in the character table...
			DB	'ä', 0x81
			DB	'ü', 0x82
			DB	'ß', 0x83
			DB	'é', 0x84               ; French accents
			DB	'è', 0x85
			DB	'ê', 0x86
			DB	'ç', 0x87
			DB	'á', 0x88               ; Spanish accents
			DB	'í', 0x89
			DB	'ó', 0x8A
			DB	'ú', 0x8B
			DB	'ñ', 0x8C
			DB	'¡', 0x8D
			DB	'¿', 0x8E
			DB	'¤', 0x8F               ; Unused
			; 90, 91 are the logo.
			DB	0xB7,0x92		        ; Cursor
			DB	0xB8,0x93		        ; Dimmed cursor.
			DB	0				; End of translation table
			DB	aa_font28_firstChar			; To be substracted
			DB	aa_font28_chars				; Max value
			DB	0x8F-aa_font28_firstChar; replace by ¤ when unknown.
			DB	aa_font28_height + 0x80
;
#include	"aa_font28_idx.inc"				; SHOULD FOLLOW !
#include	"aa_font28.inc"
aa_font28_end:
#else
aa_font28_block:
			DB	0xC0, 0x41
			DB	0xC2, 0x42
			DB	0xD1, 0x43
			DB	0xC5, 0x45
			DB	0xCD, 0x48
			DB	0xCA, 0x4B
			DB	0xCC, 0x4D
			DB	0xCE, 0x4F
			DB	0xD0, 0x50
			DB	0xD2, 0x54
			DB	0xD5, 0x58
			DB	0xE0, 0x61
			DB	0xF1, 0x63
			DB	0xE5, 0x65
			DB	0xEE, 0x6F
			DB	0xF0, 0x70
			DB	0xF5, 0x78
			DB	0xF3, 0x79
			DB	0xB7, 0x92
			DB	0xB8, 0x93
			DB	0xBA, 0x7F
			DB	0xC1, 0x80
			DB	0xC3, 0x81
			DB	0xC4, 0x82
			DB	0xC6, 0x83
			DB	0xC7, 0x84
			DB	0xC8, 0x85
			DB	0xC9, 0x86
			DB	0xCB, 0x87
			DB	0xCF, 0x88
			DB	0xD3, 0x89
			DB	0xD4, 0x8A
			DB	0xD6, 0x8B
			DB	0xD7, 0x8C
			DB	0xD8, 0x8D
			DB	0xD9, 0x8E
			DB	0xDA, 0x8F
			DB	0xDB, 0x96
			DB	0xDC, 0x97
			DB	0xDD, 0x98
			DB	0xDE, 0x99
			DB	0xDF, 0x9A
			DB	0xE1, 0x9B
			DB	0xE2, 0x9C
			DB	0xE3, 0x9D
			DB	0xE4, 0x9E
			DB	0xE6, 0x9F
			DB	0xE7, 0xA0
			DB	0xE8, 0xA1
			DB	0xE9, 0xA2
			DB	0xEA, 0xA3
			DB	0xEB, 0xA5
			DB	0xEC, 0xA6
			DB	0xED, 0xA7
			DB	0xEF, 0xA8
			DB	0xF2, 0xA9
			DB	0xF4, 0xAA
			DB	0xF6, 0xAB
			DB	0xF7, 0xAC
			DB	0xF8, 0xAD
			DB	0xF9, 0xAF
			DB	0xFA, 0xB1
			DB	0xFB, 0xB2
			DB	0xFC, 0xB3
			DB	0xFD, 0xB4
			DB	0xFE, 0xB5
			DB	0xFF, 0xB6
			DB	0				; End of translation table
			DB	aa_font28_firstChar			; To be substracted
			DB	aa_font28_chars				; Max value
			DB	0xA4-aa_font28_firstChar	; replace by ¤ when unknown.
			DB	aa_font28_height + 0x80
;
#include	"aa_font28_idx_rus.inc"				; SHOULD FOLLOW !
#include	"aa_font28_rus.inc"
aa_font28_end:
#endif
; Make sure this is coherent...
	if aa_font28_nbbits != 3
		error SMALL fount should be encoded with anti-aliasing...
	endif

;---- MEDIUM font description and data ---------------------------------------
#ifndef RUSSIAN
aa_font48_block:
			DB	0x27, 0x3B					; ' char
			DB	'"', 0x3C
			DB	'm', 0x3D
			DB	' ', 0x3E
			DB	0
			DB	aa_font48_firstChar
			DB	aa_font48_chars
			DB	0x3E-aa_font48_firstChar
			DB	aa_font48_height + 0x80		; AA flag.
;
#include	"aa_font48_idx.inc"
#include	"aa_font48.inc"
aa_font48_end:
#else
aa_font48_block:
			DB	0x27, 0x3B
			DB	0x22, 0x3C
			DB	0xEC, 0x3D
			DB	0x20, 0x3E
			DB	0				; End of translation table
			DB	aa_font48_firstChar			; To be substracted
			DB	aa_font48_chars				; Max value
			DB	0x3E-aa_font48_firstChar
			DB	aa_font48_height + 0x80
;
#include	"aa_font48_idx_rus.inc"				; SHOULD FOLLOW !
#include	"aa_font48_rus.inc"
aa_font48_end:
#endif
; Make sure this is coherent...
	if aa_font48_nbbits != 3
		error MEDIUM fount should be encoded with 3bits anti-aliasing...
	endif

;---- LARGE font description and data ----------------------------------------
aa_font90_block:
			DB	' ', 0x2F
			DB	0
			DB	aa_font90_firstChar
			DB	aa_font90_chars
			DB	0x2F-aa_font90_firstChar
			DB	aa_font90_height + 0x80		; AA flag.
;
#include	"aa_font90_idx.inc"
#include	"aa_font90.inc"
aa_font90_end:
; Make sure this is coherent...
	if aa_font90_nbbits != 3
		error LARGE fount should be encoded with 3bits anti-aliasing...
	endif

;---- HUGE font description and data ----------------------------------------
aa_font120_block:
			DB	' ', 0x2F
			DB	0
			DB	c120_aa_firstChar
			DB	c120_aa_chars
			DB	0x2F-c120_aa_firstChar
			DB	c120_aa_height + 0x80		; AA flag.
;
#include	"c120_aa_idx.inc"
#include	"c120_aa.inc"
aa_font92_end:
; Make sure this is coherent...
	if aa_font90_nbbits != 3
		error HUGE fount should be encoded with 3bits anti-aliasing...
	endif

;=============================================================================