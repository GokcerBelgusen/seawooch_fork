;==========================================================================
; Font index exported lun. mars 17 2014 by FontIO.cpp,v 50eb4d95b392 2014/03/17 12:06:30 jDG $ 
;==========================================================================
c92_aa_chars		EQU	.12
c92_aa_firstChar	EQU	'.'
c92_aa_lastChar	EQU	'9'
;
c92_aa_idx:
			DW	c92_aa_bits + 0x0000	; . dot
			DW	c92_aa_bits + 0x004F	; / slash
			DW	c92_aa_bits + 0x007A	; 0 0
			DW	c92_aa_bits + 0x01C9	; 1 1
			DW	c92_aa_bits + 0x0277	; 2 2
			DW	c92_aa_bits + 0x03F0	; 3 3
			DW	c92_aa_bits + 0x058E	; 4 4
			DW	c92_aa_bits + 0x067F	; 5 5
			DW	c92_aa_bits + 0x07CD	; 6 6
			DW	c92_aa_bits + 0x097B	; 7 7
			DW	c92_aa_bits + 0x0A8B	; 8 8
			DW	c92_aa_bits + 0x0C75	; 9 9
			DW	c92_aa_bits + 0x0E10	; END OF INDEX
