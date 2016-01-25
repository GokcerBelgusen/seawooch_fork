;==========================================================================
; Font index exported lun. mars 17 2014 by FontIO.cpp,v 50eb4d95b392 2014/03/17 12:06:30 jDG $ 
;==========================================================================
c120_aa_chars		EQU	.12
c120_aa_firstChar	EQU	'.'
c120_aa_lastChar	EQU	'9'
;
c120_aa_idx:
			DW	c120_aa_bits + 0x0000	; . dot
			DW	c120_aa_bits + 0x006D	; / slash
			DW	c120_aa_bits + 0x00B8	; 0 0
			DW	c120_aa_bits + 0x0275	; 1 1
			DW	c120_aa_bits + 0x038C	; 2 2
			DW	c120_aa_bits + 0x0565	; 3 3
			DW	c120_aa_bits + 0x0781	; 4 4
			DW	c120_aa_bits + 0x08CD	; 5 5
			DW	c120_aa_bits + 0x0A93	; 6 6
			DW	c120_aa_bits + 0x0CD5	; 7 7
			DW	c120_aa_bits + 0x0E43	; 8 8
			DW	c120_aa_bits + 0x10BC	; 9 9
			DW	c120_aa_bits + 0x12F4	; END OF INDEX
