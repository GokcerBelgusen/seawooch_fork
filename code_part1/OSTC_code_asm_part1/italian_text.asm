;=============================================================================
; OSTC Mk.2, 2N and 2C - diving computer code
; Copyright (C) 2015 HeinrichsWeikamp GbR
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
; Hold texts and screen position to display the texts.
; History:
; 2007/10/13 : Initial version by Matthias Heinrichs, info@heinrichsweikamp.com
; 2008/05/24 : MW.
; 2011/02/02 : Jean-Do Gascuel : split into different files for multi-lingual support
; 2011/08/12 : Sergei V. Rozinov: Complete translation patch
;
; known bugs:
; ToDo: 
;=============================================================================
;
; Instructions for translating:
;
; * Strings are accessed according to the order in the file.
;   So don't change ordering !
;
; * Keep the english original version on the right column. So translations
;   can be reviewed and maintened.
;
; * One of the main constraint is to keep texts short, to avoid clobering
;   the OSTC screen. Of course, the technical, precise terms should be used.
;   Generally, there is no hard constraint: you can be one or two chars 
;   shorter or longer.
; 
; * Beware that some strings do have a fixed length. You should then use 
;   exactly the same size.
;
; * Beware that some strings have ponctuation, or trailling space(s). In that
;   case, you should keep EXACTLY the same ponctuation AND TRAILING SPACES.
;
; * Ascii chars: we can support a few specific chars. öäüß for German.
;   éèêç for French. áíóúñ¡¿ for Spanish.
;   If you really, absolutly, need more: ask...
;
; * Do not translate comments (everithing after the ;), because they are
;   used for maintenance.
;
; * The X column is position on screen. Some texts are centered, left
;   padded or right padded. In that case, if you changed the text size,
;   you will have to adjust position. A char is 7 pixels wide.
;
;=============================================================================
; Define's section
;   Definition			     translation                 ; English original
#IFNDEF	TXT_DEFINED                                                           
#DEFINE	TXT_DEFINED                                                           
                                                                              
#DEFINE	TXT_GAS_C		     'G'                         ; 'G'                    
#DEFINE	TXT_GAS1		     "G"                         ; "G"
#DEFINE	TXT_METER_C		     'm'                         ; 'm'                    
#DEFINE	TXT_METER5		     "m    "                     ; "m    "                
#DEFINE	TXT_METER3		     "m  "                       ; "m  "                  
#DEFINE	TXT_METER2		     "m "                        ; "m "                   
#DEFINE	TXT_METER1		     "m"                         ; "m"                    
#DEFINE	TXT_MBAR7		     " mbar  "                   ; " mbar  "              
#DEFINE	TXT_MBAR5		     "mbar "                     ; "mbar "                
#DEFINE	TXT_BAR4		     "bar "                      ; "bar "                 
#DEFINE	TXT_BAR3			 "bar"                       ; "bar"
#DEFINE	TXT_ALT5		     "Alt: "                     ; "Alt: "                
#DEFINE	TXT_KGL4		     "kg/l"                      ; "kg/l"                 
#DEFINE	TXT_VOLT2			 "V "                        ; "V "
#DEFINE	TXT_VOLT1		     "V"                         ; "V"                    
#DEFINE	TXT_STEP5		     "Step:"                     ; "Step:"                
#DEFINE	TXT_CF2			     "CF"                        ; "CF"                   
#DEFINE	TXT_O2_4		     "O2: "                      ; "O2: "                 
#DEFINE	TXT_O2_3		     "O2 "                       ; "O2 "                  
#DEFINE	TXT_AIR4		     "AIR "                      ; "AIR "                 
#DEFINE	TXT_ERR4		     "ERR "                      ; "ERR "                 
#DEFINE	TXT_HE4			     "He: "                      ; "He: "                 
#DEFINE	TXT_NX3			     "NX "                       ; "NX "                  
#DEFINE	TXT_TX3			     "TX "                       ; "TX "                  
#DEFINE	TXT_AT4			     " at "                      ; " at "
#DEFINE	TXT_G1_3		     "G1:"                       ; "G1:"                  
#DEFINE	TXT_G2_3		     "G2:"                       ; "G2:"                  
#DEFINE	TXT_G3_3		     "G3:"                       ; "G3:"                  
#DEFINE	TXT_G4_3		     "G4:"                       ; "G4:"                  
#DEFINE	TXT_G5_3		     "G5:"                       ; "G5:"                  
#DEFINE	TXT_G6_3		     "G6:"                       ; "G6:"                  
#DEFINE	TXT_1ST4		     "1st:"                      ; "1st:"                 
#DEFINE	TXT_CNS4		     "CNS:"                      ; "CNS:"                 
#DEFINE	TXT_CNSGR10		     "CNS > 250%"                ; "CNS > 250%"           
#DEFINE	TXT_AVR4		     "Avr:"                      ; "Avr:"                 
#DEFINE	TXT_GF3			     "GF:"                       ; "GF:"                  
#DEFINE	TXT_SAT4		     "Sat:"                      ; "Sat:"                 
#DEFINE	TXT_0MIN5		     "0min "                     ; "0min "
#DEFINE	TXT_MIN4		     "min "                      ; "min "  
#DEFINE	TXT_BSAT5		     "BSat:"                     ; "BSat:" 
#DEFINE	TXT_BDES5		     "BDes:"                     ; "BDes:" 
#DEFINE	TXT_LAST5		     "Last:"                     ; "Last:" 
#DEFINE	TXT_GFLO6	         "GF_lo:"                    ; "GF_lo:"
#DEFINE	TXT_GFHI6		     "GF_hi:"                    ; "GF_hi:"
#DEFINE	TXT_PPO2_5		     "PpO2:"                     ; "ppO2:" 
#DEFINE	TXT_SP2			     "SP"                        ; "SP"    
#DEFINE	TXT_DIL4		     "Dil:"                      ; "Dil:"  
#DEFINE	TXT_N2_2		     "N2"                        ; "N2"    
#DEFINE	TXT_HE2			     "He"                        ; "He"    
#DEFINE	TXT_TX1				 "T"						 ; "T"
#DEFINE	TXT_TX2				 "x"						 ; "x"
#DEFINE	TXT_NX1				 "N"						 ; "N"
#DEFINE	TXT_NX2				 "x"						 ; "x"
#DEFINE TXT_DIL_C            "D"                         ; "D"
#DEFINE	TXT_DIL5			 "Dil.#"                     ; "Dil.#"
#DEFINE TXT_aGF4             "aGF:"                      ; "aGF:"
#DEFINE TXT_BATT             "B:"                        ; "B:"

#ENDIF                                                                   
;=============================================================================
;   macro     X     Y        "Italiano   "               ; English original
    TCODE    .0,   .0,       "Building MD2 Hash"         ;001 Building MD2 Hash
    TCODE    .0,   .25,      "Aspetta..."                ;002 Please Wait...
    TCODE    .0,   .2,       "HeinrichsWeikamp OSTC2"    ;003 HeinrichsWeikamp OSTC2
    TCODE    .65,  .2,       "Menu?"                     ;004 Menu?
    TCODE    .65,  .2,       "Menu:"                     ;005 Menu:
    TCODE    .20,  .35,      "Logbook"                   ;006 Logbook
    TCODE    .20,  .65,      "Imposta Gas"               ;007 Gas Setup
    TCODE    .20,  .35,      "Imposta Ora"               ;008 Set Time
    TCODE    .20,  .95,      "Reset Menu"                ;009 Reset Menu
    TCODE    .20,  .125,     "Impostazioni"              ;010 Setup
    TCODE    .20,  .185,     "Esci"                      ;011 Exit
    TCODE    .111, .2,       "Aspetta..."                ;012 Wait...
    TCODE    .0,   .24,      "MD2 Hash:"                 ;013 MD2 Hash:
    TCODE    .0,   .0,       "Desat"                     ;014 Desat         (Desaturation count-down)
    TCODE    .50,  .2,       "Connessione"               ;015 Interface		(Connected to USB)
    TCODE    .10,  .30,      "Inizio"                    ;016 Start
    TCODE    .10,  .55,      "Data"                      ;017 Data
    TCODE    .10,  .80,      "Testata"                   ;018 Header
    TCODE    .10,  .105,     "Profilo"                   ;019 Profile
    TCODE    .10,  .130,     "Fatto."                    ;020 Done.
    TCODE    .20,  .35,      "Annulla Reset"             ;021 Cancel Reset
    TCODE    .32,  .65,      "Ora: "                     ;022 Time:
    TCODE    .32,  .95,      "Data:"                     ;023 Date:
    TCODE    .0,   .215,     "Imposta Ore"               ;024 Set Hours
    TCODE    .6,   .0,       "Azzera..."                 ;025 Reset...
    TCODE    .55,  .2,       "Logbook"                   ;026 Logbook
    TCODE    .14,  .2,       "Funzioni Custom I"         ;027 Custom Functions I
    TCODE    .40,  .2,       "Reset Menu"                ;028 Reset Menu
    TCODE    .43,  .2,       "Imposta ora:"              ;029 Set Time:
    TCODE    .100, .50,      "Marker"                    ;030 Marker            (Add a mark in logbook profile)
    TCODE    .100, .25,      "Decoplan"                  ;031 Decoplan
    TCODE    .100, .0,       "ListaGas"                  ;032 Gaslist
    TCODE    .100, .50,      "Azz. Avr"                  ;033 ResetAvr          (Reset average depth)
    TCODE    .100, .100,     "Esci"                      ;034 Exit		        (Exit current menu)
    TCODE    .0,   .0,       "NonVolo"                   ;035 NoFly		        (No-flight count-down)
;
; 32 custom function descriptors I (FIXED LENGTH = 15 chars).
    TCODE    .40,  .35,      "Inizio Imm. [m]"           ;036 Start Dive  [m]	(depth to switch to dive mode)
    TCODE    .40,  .35,      "Fine Imm.   [m]"           ;037 End Dive    [m]	(depth to switch back to surface mode)
    TCODE    .40,  .35,      "End Delay [sec]"           ;038 End Delay [sec]  	(duration dive screen stays after end of dive)
    TCODE    .40,  .35,      "Spegni    [min]"           ;039 Power Off [min]
    TCODE    .40,  .35,      "Pre-menu  [min]"           ;040 Pre-menu  [min]	(Delais to keep surface-mode menus displayed)
    TCODE    .40,  .35,      "velocitá[m/min]"           ;041 velocity[m/min]
    TCODE    .40,  .35,      "Wake-up  [mbar]"           ;042 Wake-up  [mbar]
    TCODE    .40,  .35,      "Max.Surf.[mbar]"           ;043 max.Surf.[mbar]
    TCODE    .40,  .35,      "Mostra GF   [%]"           ;044 GF display  [%]
    TCODE    .40,  .35,      "Min. O2 Dis.[%]"           ;045 min. O2 Dis.[%]
    TCODE    .40,  .35,      "T.out Menu[min]"           ;046 Dive menus[min]
    TCODE    .40,  .35,      "Saturaz. x  [%]"           ;047 Saturate x  [%]
    TCODE    .40,  .35,      "Desaturaz. x[%]"           ;048 Desaturate x[%]
    TCODE    .40,  .35,      "Ratio NoVolo[%]"           ;049 NoFly Ratio [%]	(Grandient factor tolerance for no-flight countdown).
    TCODE    .40,  .35,      "Allarme GF 1[%]"           ;050 GF alarm 1  [%]
    TCODE    .40,  .35,      "CNS Superf. [%]"           ;051 CNSshow surf[%]
    TCODE    .40,  .35,      "Deco Offset [m]"           ;052 Deco Offset [m]
    TCODE    .40,  .35,      "PpO2 Bassa[bar]"           ;053 ppO2 low  [bar]
    TCODE    .40,  .35,      "PpO2 Alta [bar]"           ;054 ppO2 high [bar]
    TCODE    .40,  .35,      "MostraPpO2[bar]"           ;055 ppO2 show [bar]
    TCODE    .40,  .35,      "Campionamento  "           ;056 sampling rate  
    TCODE    .40,  .35,      "Campion. Temp. "           ;057 Divisor Temp   
    TCODE    .40,  .35,      "Campion. Deco  "           ;058 Divisor Decodat
    TCODE    .40,  .35,      "Campion. GF    "           ;059 Divisor GF
    TCODE    .40,  .35,      "Campion. PpO2  "           ;060 Divisor ppO2 
    TCODE    .40,  .35,      "Campion. Debug "           ;061 Divisor Debug  
    TCODE    .40,  .35,      "Campion. CNS   "           ;062 Divisor CNS
    TCODE    .40,  .35,      "CNS Immers. [%]"           ;063 CNSshow dive[%]
    TCODE    .40,  .35,      "Logbook offset "           ;064 Logbook offset 
    TCODE    .40,  .35,      "Ultima Deco [m]"           ;065 Last Deco at[m]
    TCODE    .40,  .35,      "Fine Apnea  [h]"           ;066 End Apnoe   [h]
    TCODE    .40,  .35,      "Mostra Batt. % "           ;067 Show Battery %
; End of function descriptor I
;
;licence:
    TCODE    .0,   .35,      "Questo programma è"        ;068 This program is
    TCODE    .0,   .65,      "distribuito nella"         ;069 distributed in the
    TCODE    .0,   .95,      "speranza che vi sará"      ;070 hope that it will be
    TCODE    .0,   .125,     "utile, ma SENZA"           ;071 useful, but WITHOUT
    TCODE    .0,   .155,     "NESSUNA GARANZIA"          ;072 ANY WARRANTY
    TCODE    .0,   .185,     "anche l'implicita"         ;073 even the implied
    TCODE    .0,   .215,     "garanzia di"               ;074 warranty of
    TCODE    .0,   .35,      "MERCHANTABILITY or"        ;075 MERCHANTABILITY or
    TCODE    .0,   .65,      "FITNESS FOR A"             ;076 FITNESS FOR A
    TCODE    .0,   .95,      "PARTICULAR PURPOSE."       ;077 PARTICULAR PURPOSE.
    TCODE    .0,   .125,     "Vedi anche GNU General"    ;078 See the GNU General
    TCODE    .0,   .155,     "Public License per"        ;079 Public License for
    TCODE    .0,   .185,     "maggiori dettagli:"        ;080 more details:
    TCODE    .0,   .215,     "heinrichsweikamp.com"      ;081 heinrichsweikamp.com
; end of licence
;
    TCODE    .95,   .54,     "TappaDeco"                 ;082 Decostop
    TCODE    .0,    .0,      "m/min"                     ;083 m/min
    TCODE    .108,  .113,    "No Stop"                   ;084 No Stop
    TCODE    .135,  .113,    "TTS"                       ;085 TTS
    TCODE    .114,  .0,      "Minuti"                  ;086 Divetime
    TCODE    .0,    .0,      "Profonditá"                ;087 Depth
    TCODE    .0,    .0,      "Primo Gas?"                ;088 First Gas?
    TCODE    .0,    .0,      "Default:"                  ;089 Default:
    TCODE    .0,    .0,      "Minuti"                    ;090 Minutes
    TCODE    .0,    .0,      "Mese"                      ;091 Month  
    TCODE    .0,    .0,      "Giorno"                    ;092 Day    
    TCODE    .0,    .0,      "Anno"                      ;093 Year   
    TCODE    .0,    .0,      "Imp."                      ;094 Set 
    TCODE    .0,    .0,      "Gas# "                     ;095 Gas# 
    TCODE    .0,    .0,      "SI"                        ;096 Yes
    TCODE    .0,    .0,      "Attuale:"                  ;097 Current:
    TCODE    .40,   .2,      "Setup Menu:"               ;098 Setup Menu:
    TCODE    .20,   .35,     "Funzioni Custom I"         ;099 Custom FunctionsI
    TCODE    .20,   .125,    "TipoDeco:"                 ;100 Decotype:
    TCODE    .85,   .125,    "ZH-L16 OC"                 ;101 ZH-L16 OC
    TCODE    .85,   .125,    "Profondim."                ;102 Gauge    
    TCODE    .85,   .125,    "Profondim."                ;103 Gauge
    TCODE    .85,   .125,    "ZH-L16 CC"                 ;104 ZH-L16 CC
    TCODE    .0,    .0,      "Gas Attivo? "              ;105 Active Gas?
    TCODE    .10,   .2,      "Imposta Gas - lista"	 ;106 Gas Setup - Gaslist
    TCODE    .20,   .95,     "Prof. +/-:"                ;107 Depth +/-:
    TCODE    .20,   .125,    "Cambia:" 		         ;108 Change:
    TCODE    .20,   .155,    "Default:"			 ;109 Default:
    TCODE    .20,   .65,     "CCR SetPoint Menu"         ;110 CCR SetPoint Menu
    TCODE    .20,   .2,      "CCR SetPoint Menu"         ;111 CCR SetPoint Menu
    TCODE    .0,    .0,      "SP#"                       ;112 SP#
    TCODE    .20,   .95,     "Info Batteria"             ;113 Battery Info
    TCODE    .10,   .2,      "Informazioni Batteria"     ;114 Battery Information
    TCODE    .0,    .9,      "Cicli:"                    ;115 Cycles:
    TCODE    .85,   .125,    "Apnea"                     ;116 Apnoe
    TCODE    .0,    .18,     "Ultima Carica:"            ;117 Last Complete:
    TCODE    .0,    .27,     "Minimo Vbatt:"             ;118 Lowest Vbatt:
    TCODE    .0,    .36,     "Minimo il:"                ;119 Lowest at:
    TCODE    .0,    .45,     "Tmin:"                     ;120 Tmin:
    TCODE    .0,    .54,     "Tmax:"                     ;121 Tmax:
    TCODE    .100,  .124,    "Altri"                   	 ;122 More (Gaslist)
    TCODE    .100,  .25,     "O2 +"                      ;123 O2 +
    TCODE    .100,  .50,     "O2 -"                      ;124 O2 -
    TCODE    .100,  .75,     "He +"                      ;125 He +
    TCODE    .100,  .100,    "He -"                      ;126 He -
    TCODE    .100,  .0,      "Esci"                      ;127 Exit
    TCODE    .100,  .25,     "Cancella"                  ;128 Delete
    TCODE    .20,   .65,     "Debug:"                    ;129 Debug:
    TCODE    .65,   .65,     "ON "                       ;130 ON 
    TCODE    .65,   .65,     "OFF"                       ;131 OFF
    TCODE    .100,  .50,     "Canc. tutto"               ;132 Del. all
    TCODE    .10,   .0,      "Reset Inaspettato dal"      ;133 Unexpected reset from
    TCODE    .10,   .25,     "Divemode! Aiutaci,"        ;134 Divemode! Please help
    TCODE    .10,   .50,     "comunicaci le info"     	 ;135 and report the Debug 
    TCODE    .10,   .75,     "del Debug qui sotto!"      ;136 Information below!
    TCODE    .100,  .0,	     "Bailout"                   ;137 Bailout
    TCODE    .85,   .125,    "Apnea"                     ;138 Apnoe    
    TCODE    .105,  .120,    "Discesa"                   ;139 Descent
    TCODE    .105,  .60,     "Superficie"                ;140 Surface
    TCODE    .65,   .2,      "Esci?"                     ;141 Quit?
    TCODE    .20,   .155,    "Altro"                     ;142 More
    TCODE    .42,   .72,     "Conferma:"                 ;143 Confirm:
    TCODE    .60,   .2,      "Menu 2:"                   ;144 Menu 2:
    TCODE    .52,   .96,     "Cancella"                  ;145 Cancel
    TCODE    .52,   .120,    "OK!"                       ;146 OK!
    TCODE    .20,   .35,     "Altro"                     ;147 More
    TCODE    .0,    .0,      ":.........:"               ;148 :.........:
    TCODE    .0,    .8,      "PpO2"                      ;149 ppO2
    TCODE    .2,    .39,     "bar "                      ;150 bar 
    TCODE    .108,  .216,    "Marker?"                   ;151 Marker?
    TCODE    .85,   .125,    "L16-GF OC"                 ;152 L16-GF OC
    TCODE    .20,   .65,     "Funzioni Custom II"        ;153 Custom FunctionsII
;
; 32 custom function descriptors II (FIXED LENGTH = 15 chars).
    TCODE    .40,   .35,     "GF Basso    [%]"           ;154 GF Low      [%]
    TCODE    .40,   .35,     "GF Alto     [%]"           ;155 GF High     [%]
    TCODE    .40,   .35,     "Colore#Batteria"           ;156 Color# Battery 
    TCODE    .40,   .35,     "Colore#Standard"           ;157 Color# Standard
    TCODE    .40,   .35,     "Colore#Divemask"           ;158 Color# Divemask
    TCODE    .40,   .35,     "Colore# Allarmi"           ;159 Color# Warnings
    TCODE    .40,   .35,     "Sec. in Immers."           ;160 Divemode secs. 
    TCODE    .40,   .35,     "Regola SP fisso"           ;161 Adjust fixed SP
    TCODE    .40,   .35,     "All. salto deco"           ;162 Warn Ceiling
    TCODE    .40,   .35,     "unused         "           ;163 unused
    TCODE    .40,   .35,     "Avv.Miglior Gas"           ;164 Blink BetterGas	(Remainder in divemode to switch to a better decompression gas).
    TCODE    .40,   .35,     "All. Prof[mbar]"           ;165 DepthWarn[mbar]
    TCODE    .40,   .35,     "Allarme CNS [%]"           ;166 CNS warning [%]
    TCODE    .40,   .35,     "Allarme GF  [%]"           ;167 GF warning  [%]
    TCODE    .40,   .35,     "All. PpO2 [bar]"           ;168 ppO2 warn [bar]
    TCODE    .40,   .35,     "All. Vel[m/min]"           ;169 Vel.warn[m/min]
    TCODE    .40,   .35,     "Aggiusta orario"           ;170 Time offset/day
    TCODE    .40,   .35,     "Vedi altimetro "           ;171 Show altimeter
    TCODE    .40,   .35,     "vedi Log-Marker"           ;172 Show Log-Marker
    TCODE    .40,   .35,     "Vedi Stopwatch "           ;173 Show Stopwatch
    TCODE    .40,   .35,     "VediGrafTessuti"           ;174 ShowTissueGraph
    TCODE    .40,   .35,     "Vedi Tess.Prim."           ;175 Show Lead.Tiss.
    TCODE    .40,   .35,     "Inver. Tab Deco"           ;176 Shallow stop 1st  (Reverse order of deco plans)
    TCODE    .40,   .35,     "Cambio Gas[min]"           ;177 Gas switch[min]   (Show Countdown after gas change)
    TCODE    .40,   .35,     "Gas Fondo[/min]"           ;178 BottomGas[/min]   (Bottom gas usage, for volume estimation).
    TCODE    .40,   .35,     "Gas Risal[/min]"           ;179 AscentGas[/min]   (Ascent+Deco gas usage)
    TCODE    .40,   .35,     "TTS Futuro[min]"           ;180 Future TTS[min]   (Compute TTS for extra time at current depth)
    TCODE    .40,   .35,     "Allarme Cave[l]"           ;181 Cave Warning[l]   (Consomation warning for cave divers)
    TCODE    .40,   .35,     "GraficoVelocitá"           ;182 Graph. Velocity	(Show a graphical representation of the ascend speed)
    TCODE    .40,   .35,     "Vedi PpO2 pSCR "           ;183 Show pSCR ppO2	(Show the ppO2 for pSCR divers)
    TCODE    .40,   .35,     "pSCR O2 Drop[%]"           ;184 pSCR O2 Drop[%]	(pSCR O2 drop in percent)
    TCODE    .40,   .35,     "Rap C.Polm pSCR"           ;185 pSCR lung ratio	(pSCR counterlung ratio)
; End of function descriptor II
;
    TCODE    .13,   .2,      "Funzioni Custom II"        ;186 Custom Functions II
    TCODE    .20,   .95,     "Vedi Licenza"              ;187 Show License
    TCODE    .0,    .2,      "Risul. simul:"             ;188 Sim. Results:
    TCODE    .90,   .25,     "Superficie"                ;189 Surface
    TCODE    .0,    .0,      "PpO2 +"                    ;190 ppO2 +
    TCODE    .0,    .0,      ""                          ;191 unused
    TCODE    .0,    .0,      "Dil."                      ;192 Dil.			       (Rebreather diluent)

; 32 custom function descriptors III (FIXED LENGTH = 15 chars).
    TCODE    .40,   .35,     "Color# inattivo"           ;193 Color# inactive
    TCODE    .40,   .35,     "Usa tappa sicur"           ;194 Use safety stop
    TCODE    .40,   .35,     "Show GF in NDL "           ;195 Show GF in NDL	(If GF > CF08)
    TCODE    .40,   .35,     "Alt. GF Low [%]"           ;196 Alt. GF Low [%]
    TCODE    .40,   .35,     "Alt. GF High[%]"           ;197 Alt. GF High[%]
    TCODE    .40,   .35,     "Allow GF change"           ;198 Allow GF change
    TCODE    .40,   .35,     "S.StopLength[s]"           ;199 S.StopLength[s] (CF70: Safety Stop Duration [s])
    TCODE    .40,   .35,     "S.StopStart [m]"           ;200 S.StopStart [m] (CF71: Safety Stop Start Depth [m])
    TCODE    .40,   .35,     "S.StopEnd   [m]"           ;201 S.StopEnd   [m] (CF72: Safety Stop End Depth [m])
    TCODE    .40,   .35,     "S.StopReset [m]"           ;202 S.StopReset [m] (CF73: Safety Stop Reset Depth [m])
    TCODE    .40,   .35,     "Tempo Batt[min]"           ;203 Batt. Time [min] (CF74: Battery time-out [min])
    TCODE    .40,   .35,     "inutilizzato   "           ;204 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;205 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;206 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;207 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;208 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;209 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;210 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;211 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;212 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;213 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;214 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;215 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;216 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;217 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;218 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;219 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;220 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;221 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;222 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;223 unused
    TCODE    .40,   .35,     "inutilizzato   "           ;224 unused
;
    TCODE    .7,    .2,      "Funzioni Custom III"       ;225 Custom Functions III
    TCODE    .85,   .125,    "pSCR-GF  "                 ;226 pSCR-GF
    TCODE    .90,   .54,     "TappaSicur"                ;227 SafetyStop
    TCODE    .0,    .0,      "Immersioni totali: "       ;228 Total Dives: 
    TCODE    .20,   .35,     "Diluent Setup"             ;229 Diluent Setup
    TCODE    .20,   .65,     "Setpoint Setup"            ;230 Setpoint Setup
    TCODE    .5,    .2,      "Dil. Setup - Gaslist"	 	 ;231 Dil. Setup - Gaslist
    TCODE    .100,  .100,	 "Diluent"                   ;232 Diluent
    TCODE    .93,   .170,    "  Ceiling"                 ;233 Ceiling (9Chars, right alligned)
    TCODE    .20,   .95,     "SP Mode:  "                ;234 SP Mode: (10 chars)

    TCODE    .10,   .2,      "Deco Cambiato!"    	 ;235 Decomode changed!
    TCODE    .85,   .125,    "L16-GF CC"                 ;236 L16-GF CC
    TCODE    .2,    .12,     "Non trovato"               ;237 Not found
    TCODE    .100,  .0,      "SetPoint"                  ;238 SetPoint
    TCODE    .100,  .0,      "No Deco"                   ;239 No Deco
    TCODE    .90,   .50,     "Intervallo:"               ;240 Interval:
    TCODE    .100,  .75,     "Display"                   ;241 Display
    TCODE    .100,  .0,      "No deco"                   ;242 No deco
    TCODE    .132,  .0,      "beta"                      ;243 beta
    TCODE    .100,  .100,    "unuse"                     ;244 unuse
    TCODE    .20,   .65,     "Resetta CF,Gas & Deco"     ;245 Reset CF,Gas & Deco
    TCODE    .50,   .145,    "Batt!"                     ;246 LowBatt!
    TCODE    .20,   .125,    "Simulatore"                ;247 Simulator
    TCODE    .27,   .2,      "Simulatore OSTC"           ;248 OSTC Simulator
    TCODE    .20,   .65,     "Inizio Immersione"         ;249 Start Dive
    TCODE    .100,  .25,     "+ 1m"                      ;250 + 1m
    TCODE    .100,  .50,     "- 1m"                      ;251 - 1m
    TCODE    .100,  .75,     "+10m"                      ;252 +10m
    TCODE    .100,  .100,    "-10m"                      ;253 -10m
    TCODE    .100,  .0,      "Chiudi"                    ;254 Close
    TCODE    .138,  .170,    "Ora"                       ;255 Time
;
; Text Bank2 (Texts 256-511)
;
    TCODE    .0,    .0,      "x"                         ;256 x
    TCODE    .20,   .35,     "Formato data:"             ;257 Date format:
    TCODE    .40,   .2,      "Setup Menu 2:"             ;258 Setup Menu 2:
    TCODE    .105,  .35,     "MMDDYY"                    ;259 MMDDYY
    TCODE    .105,  .35,     "DDMMYY"                    ;260 DDMMYY
    TCODE    .105,  .35,     "YYMMDD"                    ;261 YYMMDD
    TCODE    .1,    .1,      "OSTC "                     ;262 OSTC 
    TCODE    .65,   .168,    "Bail "                     ;263 Bail 
    TCODE    .7,    .48,     "Aria "                     ;264 Air
    TCODE    .115,  .135,    "Aria "                     ;265 Air

    TCODE    .0,    .0,      "Info pSCR"             	 ;266 pSCR Info (Must be 9Chars!)
    TCODE    .0,    .184,    "Max."                      ;267 Max.
    TCODE    .93,   .170,    "GF Values"                 ;268 GF Values
    TCODE    .100,  .50,     "ToggleGF"               	 ;269 ToggleGF (In Divemode Menu)
    TCODE    .93,   .170,    "Decogas"		             ;270 Decogas

; New CFs Warning
    TCODE    .24,   .2,      "Nuova CF aggiunta!"        ;271 New CF added!
    TCODE    .0,    .35,     "Nuove Funz. custom"        ;272 New CustomFunctions
    TCODE    .0,    .65,     "Aggiunte! Controlla"       ;273 were added! Check
    TCODE    .0,    .95,     "i menu CF I CF II"         ;274 CF I and CF II Menu
    TCODE    .0,    .125,    "per i Dettagli!"           ;275 for Details!
    TCODE    .20,   .125,    "Salinitá: "                ;276 Salinity:
;
    TCODE    .20,   .95,     "Tempo fondo:"              ;277 Bottom Time:
    TCODE    .20,   .125,    "Profonditá Max:"           ;278 Max. Depth:
    TCODE    .20,   .155,    "Calcola Deco"              ;279 Calculate Deco
    TCODE    .20,   .155,    "Luminositá:"          	 ;280 Brightness:
;
    TCODE    .90,   .170,    "Prof.Media"                ;281 Avr.Depth
    TCODE    .90,   .170,    "Ass. Tess."                ;282 Lead Tiss.
    TCODE    .90,   .170,    "Cronometro"                ;283 Stopwatch
    TCODE    .20,   .95,     "Azzera Logbook"            ;284 Reset Logbook
    TCODE    .20,   .125,    "Riavvia OSTC"              ;285 Reboot OSTC
    TCODE    .20,   .155,    "Azzera Decodata"           ;286 Reset Decodata
; Altimeter extension
    TCODE    .20,   .155,    "Altimetro"                 ;287 Altimeter
    TCODE    .20,   .1,      "Imposta Altimetro"         ;288 Set Altimeter
    TCODE    .20,   .35,     "Liv. Mare:"                ;289 Sea ref: 
    TCODE    .0,    .0,      "Abilitato: "               ;290 Enabled:
    TCODE    .20,   .95,     "Default: 1013 mbar"        ;291 Default: 1013 mbar
    TCODE    .20,   .125,    "+1 mbar"                   ;292 +1 mbar
    TCODE    .20,   .155,    "-1 mbar"                   ;293 -1 mbar
    TCODE    .85,   .185,    "Alt: "                     ;294 Alt: 
;
    TCODE    .20,   .95,     "Funzioni CustomIII"        ;295 Custom FunctionsIII
    TCODE    .50,    .2,     "Raw Data:"                 ;296 Raw Data:
; Gas-setup addons:
    TCODE    .0,    .0,      "MOD:"                      ;297 MOD:                  (max operating depth of a gas).
    TCODE    .0,    .0,      "END:"                      ;298 END:                  (equivalent nitrogen depth of a gas).
    TCODE    .0,    .0,      "EAD:"                      ;299 EAD:                  (equivalent air depth of a gas).
    TCODE    .100,  .125,    "More"			 ;300 More                  (Enable/Disable Gas underwater)
    TCODE    .0,    .2,      "OCR Gas Usage:"            ;301 OCR Gas Usage:        (Planned gas consumtion by tank).
; 115k Bootloader support:
    TCODE    .45,   .100,    "Bootloader"		 ;302 Bootloader
    TCODE    .30,   .130,    "Perfavore aspetta!"	 ;303 Please wait!
    TCODE    .50,   .130,    "Aborted!"		         ;304 Aborted
; @5 variant
    TCODE    .0,    .0,      "TTS Futuro"                ;305 Future TTS            (=10 chars. Title for @5 customview).
;
    TCODE    .100,  .125,    "Esci Sim"                  ;306 Quit Sim              (=8char max. Quit Simulator mode)
; Dive interval
    TCODE    .20,   .35,     "Intervallo:"               ;307 Interval:
    TCODE    .0,    .0,      "Adesso"                    ;308 Now                   (7 chars min)
    TCODE    .108,  .112,    "Medio"	 		 ;309 Average
;
    TCODE    .94,    .54,    "Cronometro" 		 ;310 Stopwatch             (BIG Stopwatch in Gauge mode)
; Cave consomation
    TCODE    .0,    .0,      "Cave Bail."                ;311 Cave Bail.            (=10 chars.)
; DISPLAY Brightness settings
    TCODE    .103,  .155,    "Eco "	    	         ;312 Eco 					(Same length as #313!)
    TCODE    .103,  .155,    "High" 	                 ;313 High					(Same length as #312!)

; ZH-L16 mode description
    TCODE    .0,    .35,     "TipoDeco: ZH-L16 OC"       ;314 Decotype: ZH-L16 OC
    TCODE    .0,    .65,     "Per Sub in Circuito" 	 ;315 For Open Circuit
    TCODE    .0,    .95,     "Aperto. Supporta 5"    	 ;316 Divers. Supports 5
    TCODE    .0,    .125,    "Miscele Trimix."       	 ;317 Trimix Gases.
    TCODE    .0,    .155,    "Configura il tuo gas"      ;318 Configure your gas
    TCODE    .0,    .185,    "nel menu ImpostaGas."      ;319 in Gassetup menu.
    TCODE    .0,    .215,    "Controlla CF11 & CF12 !"   ;320 Check CF11 & CF12 !
; Gaugemode description
    TCODE    .0,    .35,     "Tipodeco:Profondimetro"    ;321 Decotype: Gauge    
    TCODE    .0,    .65,     "Il tempo di immersione"    ;322 Divetime will be in
    TCODE    .0,    .95,     "sará Minuti:Secondi."      ;323 Minutes:Seconds.   
    TCODE    .0,    .125,    "OSTC2 non calcolerá"       ;324 OSTC2 will not     
    TCODE    .0,    .155,    "Decompressione, tempo"     ;325 compute Deco, NoFly
    TCODE    .0,    .185,    "di non volo e tempo di"    ;326 time and Desat.
    TCODE    .0,    .215,    "desaturazione, MAI!"    	 ;327 time at all!
; Const.ppO2 description
    TCODE    .0,    .35,     "Tipodeco: ZH-L16 CC"       ;328 Decotype: ZH-L16 CC
    TCODE    .0,    .65,     "Per Rebreather "       	 ;329 For closed
    TCODE    .0,    .95,     "a circuito chiuso"         ;330 circuit rebreathers
    TCODE    .0,    .125,    "Configura i 3"      	 ;331 Configure the 3
    TCODE    .0,    .155,    "SetPoints nel menu"      	 ;332 SetPoints in CCR -
    TCODE    .0,    .185,    "imposta-CCR. 5 bail-"      ;333 Setup menu. 5 bail-
    TCODE    .0,    .215,    "outs sono disponibili."    ;334 outs are available.
; Apnoemode description
    TCODE    .0,    .35,     "Tipodeco: Apnea"      	 ;335 Decotype: Apnoe
    TCODE    .0,    .65,     "OSTC2 mostrerá ogni"	         ;336 OSTC2 will display
    TCODE    .0,    .95,     "discesa separatamente"       ;337 each descent separ-
    TCODE    .0,    .125,    "visualizzando Min:Sec."      ;338 ately in Min:Sec.
    TCODE    .0,    .155,    "Il campionamento sarà"        ;339 Will temporally set
    TCODE    .0,    .185,    "impostato a 1 secondo"     ;340 samplerate to 1 sec
    TCODE    .0,    .215,    "senza calcolo Deco"       ;341 No Deco calculation
; Multi GF OC mode description
    TCODE    .0,    .35,     "TipoDeco: L16-GF OC"        ;342 Decotype: L16-GF OC
    TCODE    .0,    .65,     "Calcolo decompressione"    ;343 Decompression cal-
    TCODE    .0,    .95,     "con i Gradient Factor"     ;344 culations with the
    TCODE    .0,    .125,    "(GF_lo/GF_hi)"             ;345 GF-Method (GF_lo/GF
    TCODE    .0,    .155,    "Controlla CF32 & CF33!"    ;346 _hi). Check CF32 &
    TCODE    .0,    .185,    "Circuito Aperto con"           ;347 CF33! Open Circuit
    TCODE    .0,    .215,    "Deep Stops."	  	 ;348 with Deep Stops.
; Multi GF CC mode description
    TCODE    .0,    .35,     "TipoDeco: L16-GF CC"       ;349 Decotype: L16-GF CC
    TCODE    .0,    .65,     "Calcolo decompressione"  	 ;350 Decompression cal-
    TCODE    .0,    .95,     "con i Gradient Factor"  	 ;351 culations with the
    TCODE    .0,    .125,    "(GF_lo/GF_hi)"              ;352 GF-Method (GF_lo/GF
    TCODE    .0,    .155,    "Controlla CF32 & CF33!"    ;353 _hi). Check CF32 &
    TCODE    .0,    .185,    "Circuito Chiuso con"      ;354 CF33!Closed Circuit
    TCODE    .0,    .215,    "Deep Stops."       	 ;355 with Deep Stops.
; pSCR-GF mode description
    TCODE    .0,    .35,     "TipoDeco: pSCR-GF"     	 ;356 Decotype: pSCR-GF
    TCODE    .0,    .65,     "Per rebreather a circ."     	 ;357 For passive semi-
    TCODE    .0,    .95,     "semichiuso passivo."    	 ;358 closed rebreather.
    TCODE    .0,    .125,    "Controlla CF32 & CF33"     ;359 Check CF32 & CF33
    TCODE    .0,    .155,    "per i Gradient Factor"     ;360 for gradient factors
    TCODE    .0,    .185,    "e CF61-CF63 per la"     	 ;361 and CF61-CF63 for
    TCODE    .0,    .215,    "modalitá pSCR."    	 ;362 pSCR features.
;
; Setpoint Mode
    TCODE    .60,  .95,     "Manual"	    	        ;363 Manual      Same length as #364
    TCODE    .60,  .95,     "Auto  " 	                ;364 Auto        Same length as #363

;=============================================================================
