// *********************************************************
// ** Common definitions for the OSTC decompression code  **
// *********************************************************

//////////////////////////////////////////////////////////////////////////////
// OSTC Mk.2, 2N and 2C - diving computer code
// Copyright (C) 2015 HeinrichsWeikamp GbR
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//////////////////////////////////////////////////////////////////////////////
// history:
// 12/25/10 v110: [jDG] split in three files (deco.c, main.c, definitions.h)

/*
#define	DBG_c_gas	0x0001
#define	DBG_c_ppO2	0x0002
#define	DBG_RUN 	0x0004
#define	DBG_RESTART 0x0008

#define	DBG_CdeSAT 	0x0010
#define	DBG_C_MODE	0x0020
#define	DBG_C_SURF	0x0040
#define	DBG_HEwoHE 	0x0080

// #define	DBG_UNUSED	0x0100
#define	DBG_C_DGAS 	0x0200
#define	DBG_C_DIST	0x0400
#define	DBG_C_LAST	0x0800

#define	DBG_C_GF	0x1000
#define	DBG_ZH16ERR	0x2000
#define	DBG_PHIGH	0x4000
#define	DBG_PLOW	0x8000

#define	DBS_mode	0x0001
#define	DBS_ppO2	0x0002
#define	DBS_HE_sat	0x0004
// #define	DBS_UNUSED  0x0008

#define	DBS_SAT2l	0x0010
#define	DBS_SAT2h	0x0020
#define	DBS_GFLOW2l	0x0040
#define	DBS_GFLOW2h	0x0080

#define	DBS_GFHGH2l	0x0100
#define	DBS_GFHGH2h	0x0200
#define	DBS_GASO22l	0x0400
#define	DBS_GASO22h	0x0800

#define	DBS_DIST2h 	0x1000
#define	DBS_LAST2h 	0x2000
#define	DBS_DECOO2l	0x4000
#define	DBS_DECOO2h	0x8000

#define	DBS2_PRES2h 0x0001
#define	DBS2_PRES2l 0x0002
#define	DBS2_SURF2l	0x0004
#define	DBS2_SURF2h	0x0008

#define DBS2_DESAT2l 0x0010
#define DBS2_DESAT2h 0x0020
#define	DBS2_GFDneg  0x0040

*/

#define	MBAR_REACH_GASCHANGE_AUTO_CHANGE_OFF	150

// *************************
// ** P R O T O T Y P E S **
// *************************

extern void calc_percentage(void);
extern void deco_calc_hauptroutine(void);
extern void deco_clear_tissue(void);
extern void deco_calc_percentage(void);
extern void deco_calc_wo_deco_step_1_min(void);
extern void deco_calc_dive_interval(void);
extern void deco_gradient_array(void);
extern void deco_hash(void);
extern void deco_calc_desaturation_time(void);
extern void deco_calc_CNS_fraction(void);
extern void deco_calc_CNS_decrease_15min(void);
extern void deco_clear_CNS_fraction(void);
extern void deco_push_tissues_to_vault(void);
extern void deco_pull_tissues_from_vault(void);
extern void deco_calc_CNS_planning(void);
extern void deco_gas_volumes(void);
extern void calc_v_ceiling(void);

// ***********************************************
// **         Allow compile on VisualC          **
// ***********************************************

#if defined(WIN32) || defined(UNIX)
    // Some keywords just dont exists on Visual C++:
#   define CROSS_COMPILE
#   define __18CXX
#   define ram
#   define rom
#   define overlay
#   define PARAMETER

    // Avoid warnings about float/double mismatches:
#   ifdef WIN32
#       pragma warning(disable: 4244 4068 4305)
#   endif
#else
#   define PARAMETER static
#   ifdef __DEBUG
#       define assert(predicate) if( !(predicate) ) assert_failed(__LINE__)
#   else
#       define assert(predicate)
#   endif
#endif

//////////////////////////////////////////////////////////////////////////////
