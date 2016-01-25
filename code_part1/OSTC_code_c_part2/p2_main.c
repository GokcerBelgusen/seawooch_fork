// ********************************************************************
// ** main code for simulation / tests without full simulation code  **
// ** This is NOT a part of the OSTC                                 **
// ********************************************************************

//////////////////////////////////////////////////////////////////////////////
// OSTC - diving computer code
// Copyright (C) 2008 HeinrichsWeikamp GbR
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
// History:
// 12/25/10 v110: [jDG] split in three files (deco.c, main.c, definitions.h)
// 01/24/11     : [jDG] make it link with oled+wordprocessor display functions

#include <stdlib.h>
#include <stdio.h>

//////////////////////////////////////////////////////////////////////////////
// Compile with:
//      p2_deco.c
//      p2_environment.asm
// To have a linkable application, downloadable on the OSTC.
//////////////////////////////////////////////////////////////////////////////

#define TEST_MAIN 1
#include "p2_definitions.h"
#include "../OSTC_code_c_part2/shared_definitions.h"

//////////////////////////////////////////////////////////////////////////////
//

static void print_stops(void)
{
    //TODO: print decompression stops using aa_wordprocessor...
    PLED_ClearScreen();
    win_top = 0;
    win_leftx2 = 0;
    win_font = 0;
    win_invert = 0;
    PLED_standard_color();

    sprintf(letter, "TTS: %d", char_O_ascenttime);
    aa_wordprocessor();
    
    //TODO: Wait click (to continue)
}

//////////////////////////////////////////////////////////////////////////////
#pragma code main = 0x9000
void main(void)
{
    static unsigned int i;
    static unsigned int debug_temp;

    char_I_deco_model = 0;
    deco_clear_CNS_fraction();

    char_I_N2_ratio = 79; //38;
    char_I_He_ratio = 0; //50;
    char_I_deco_distance = 10; // 10 = 1 meter
    char_I_depth_last_deco = 3;	// values below 3 (meter) are ignored
    
    char_I_const_ppO2 = 0;
    char_I_deco_ppO2_change = 0; // [dm] 10 = 1 meter
    char_I_deco_ppO2 = 0;
    
    char_I_deco_gas_change1 = 20; // [m] 1 = 1 meter
    char_I_deco_N2_ratio1 = 50;
    char_I_deco_He_ratio1 = 0;
    
    char_I_deco_gas_change2 = 6; // [m] 1 = 1 meter
    char_I_deco_N2_ratio2 = 0;
    char_I_deco_He_ratio2 = 0;
    
    char_I_deco_gas_change3 = 0; // [m] 1 = 1 meter
    char_I_deco_gas_change4 = 0; // [m] 1 = 1 meter
    char_I_deco_gas_change5 = 0; // [m] 1 = 1 meter

    //char_I_actual_ppO2;					// 0x507
    char_I_GF_High_percentage = 100;			// 0x514	new in v.102
    char_I_GF_Low_percentage  = 100;			// 0x515	new in v.102
    
    char_I_saturation_multiplier = 110;
    char_I_desaturation_multiplier = 90;
    
    //---- Starts at zero meter ----------------------------------------------
    int_I_pres_respiration = 1000;//980;
    int_I_pres_surface = 1000;//980;
    deco_clear_tissue();

    //---- Calculate 29min at 45m --------------------------------------------
    char_I_step_is_1min = 1;
    int_I_pres_respiration = 4500 + int_I_pres_surface;
    
    for (i=0;i<29;i++)
        deco_calc_hauptroutine();

    // Wait for one full computation
    char_I_step_is_1min = 0;
    char_O_deco_status = 255;
    while (char_O_deco_status)
    	deco_calc_hauptroutine();
    print_stops();

    // And a second one
    while (char_O_deco_status)
    	deco_calc_hauptroutine();
    print_stops();

    //---- 3000 seconds at 90m -----------------------------------------------
    int_I_pres_respiration = 10000;
    for (i=0;i<1500;i++)
        deco_calc_hauptroutine();

    while(char_O_deco_status != 0)
        deco_calc_hauptroutine();
    print_stops();

    //---- 300 seconds at 29m ------------------------------------------------
    int_I_pres_respiration = 3000;
    for (i=0;i<150;i++)
    	deco_calc_hauptroutine();

    while(char_O_deco_status != 0)
        deco_calc_hauptroutine();
    print_stops();
}
