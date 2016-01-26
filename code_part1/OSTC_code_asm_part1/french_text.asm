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
; 2011/02/02 : Jean-Do Gascuel: split into different files for multi-lingual support
; 2011/02/09 : Pierre Vidalot: French translation.
; 2011/08/12 : Sergei V. Rozinov: Complete translation patch
; 2012/12/12 : Matthieu Foll: Added missing french translation (106,229,230,231)
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
#DEFINE	TXT_GAS1			 "G"                         ; "G"
#DEFINE	TXT_METER_C		     'm'                         ; 'm'                    
#DEFINE	TXT_METER5		     "m    "                     ; "m    "                
#DEFINE	TXT_METER3		     "m  "                       ; "m  "                  
#DEFINE	TXT_METER2		     "m "                        ; "m "                   
#DEFINE	TXT_METER1		     "m"                         ; "m"                    
#IFDEF IMPERIAL
 #DEFINE	TXT_FOOT5		     "ft   "                     ; "ft   "
 #DEFINE	TXT_FOOT4		     "ft  "                      ; "ft  "
 #DEFINE	TXT_FOOT3		     "ft "                       ; "ft "
 #DEFINE	TXT_FOOT2		     "ft"                        ; "ft"
#ENDIF
#DEFINE	TXT_MBAR7		     " mbar  "                   ; " mbar  "              
#DEFINE	TXT_MBAR5		     "mbar "                     ; "mbar "                
#DEFINE	TXT_BAR4		     "bar "                      ; "bar "                 
#DEFINE	TXT_BAR3			 "bar"                       ; "bar"
#DEFINE	TXT_ALT5		     "Alt: "                     ; "Alt: "                
#DEFINE	TXT_KGL4		     "kg/l"                      ; "kg/l"                 
#DEFINE	TXT_VOLT2			 "V "                        ; "V "
#DEFINE	TXT_VOLT1		     "V"                         ; "V"                    
#DEFINE	TXT_STEP5		     "Pas :"                     ; "Step:"                
#DEFINE	TXT_CF2			     "CF"                        ; "CF"                   
#DEFINE	TXT_O2_4		     "O2: "                      ; "O2: "                 
#DEFINE	TXT_O2_3		     "O2 "                       ; "O2 "                  
#DEFINE	TXT_AIR4		     "AIR "                      ; "AIR "                 
#DEFINE	TXT_ERR4		     "ERR "                      ; "ERR "                 
#DEFINE	TXT_HE4			     "He: "                      ; "He: "                 
#DEFINE	TXT_NX3			     "NX "                       ; "NX "                  
#DEFINE	TXT_TX3			     "TX "                       ; "TX "                  
#DEFINE	TXT_AT4			     " a  "                      ; " at "
#DEFINE	TXT_G1_3		     "G1:"                       ; "G1:"                  
#DEFINE	TXT_G2_3		     "G2:"                       ; "G2:"                  
#DEFINE	TXT_G3_3		     "G3:"                       ; "G3:"                  
#DEFINE	TXT_G4_3		     "G4:"                       ; "G4:"                  
#DEFINE	TXT_G5_3		     "G5:"                       ; "G5:"                  
#DEFINE	TXT_G6_3		     "G6:"                       ; "G6:"                  
#DEFINE	TXT_1ST4		     "1er:"                      ; "1st:"                 
#DEFINE	TXT_CNS4		     "SNC:"                      ; "CNS:"                 
#DEFINE	TXT_CNSGR10		     "SNC > 250%"                ; "CNS > 250%"           
#DEFINE	TXT_AVR4		     "Moy:"                      ; "Avr:"                 
#DEFINE	TXT_GF3			     "GF:"                       ; "GF:"                  
#DEFINE	TXT_SAT4		     "Sat:"                      ; "Sat:"                 
#DEFINE	TXT_0MIN5			 "0min "                     ; "0min "
#DEFINE	TXT_MIN4			 "min "                      ; "min "  
#DEFINE	TXT_BSAT5			 "BSat:"                     ; "BSat:" 
#DEFINE	TXT_BDES5			 "BDes:"                     ; "BDes:" 
#DEFINE	TXT_LAST5			 "Last:"                     ; "Last:" 
#DEFINE	TXT_GFLO6			 "GF_lo:"                    ; "GF_lo:"
#DEFINE	TXT_GFHI6			 "GF_hi:"                    ; "GF_hi:"
#DEFINE	TXT_PPO2_5			 "ppO2:"                     ; "ppO2:" 
#DEFINE	TXT_SP2				 "SP"                        ; "SP"    
#DEFINE	TXT_DIL4			 "Dil:"                      ; "Dil:"  
#DEFINE	TXT_N2_2			 "N2"                        ; "N2"    
#DEFINE	TXT_HE2				 "He"                        ; "He"    
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
;   macro     X     Y        "translation"               ; English original
    TCODE    .0,   .0,       "Calcul du hash MD2"        ;001 Building MD2 Hash
    TCODE    .0,   .25,      "Attendez SVP..."           ;002 Please Wait...
    TCODE    .0,   .2,       "HeinrichsWeikamp OSTC2"    ;003 HeinrichsWeikamp OSTC2
    TCODE    .65,  .2,       "Menu?"                     ;004 Menu?
    TCODE    .65,  .2,       "Menu:"                     ;005 Menu:
    TCODE    .20,  .35,      "Carnet de plongées"        ;006 Logbook
    TCODE    .20,  .65,      "Réglage des Gaz"           ;007 Gas Setup
    TCODE    .20,  .35,      "Réglage Heure"             ;008 Set Time
    TCODE    .20,  .95,      "Remises a zéro"            ;009 Reset Menu
    TCODE    .20,  .125,     "Menu Réglages"             ;010 Setup
    TCODE    .20,  .185,     "Sortir"                    ;011 Exit
    TCODE    .84,  .2,       "Attendez..."               ;012 Wait...
    TCODE    .0,   .24,      "Hash MD2:"                 ;013 MD2 Hash:
    TCODE    .0,   .0,       "Désat"                     ;014 Desat         (Desaturation count-down)
    TCODE    .50,  .2,       "Interface"                 ;015 Interface		(Connected to USB)
    TCODE    .10,  .30,      "Démarrer"                  ;016 Start
    TCODE    .10,  .55,      "Données"                   ;017 Data
    TCODE    .10,  .80,      "En-tête"                   ;018 Header
    TCODE    .10,  .105,     "Profil"                    ;019 Profile
    TCODE    .10,  .130,     "Fait."                     ;020 Done.
    TCODE    .20,  .35,      "Annuler RaZ"               ;021 Cancel Reset
    TCODE    .32,  .65,      "Heure:"                    ;022 Time:
    TCODE    .32,  .95,      "Date :"                    ;023 Date:
    TCODE    .0,   .215,     "Réglage Heures"            ;024 Set Hours     (see also 090..094)
    TCODE    .6,   .0,       "Initialisation..."         ;025 Reset...
    TCODE    .17,  .2,       "Carnet de plongées"        ;026 Logbook
    TCODE    .14,  .2,       "Config Fonctions I"        ;027 Custom Functions I
    TCODE    .31,  .2,       "Remises a zéro:"           ;028 Reset Menu
    TCODE    .50,  .2,       "Reg.Heure:"                ;029 Set Time:
    TCODE    .100, .50,      "Repère"                    ;030 SetMarker         (Add a mark in logbook profile)
    TCODE    .100, .25,      "Paliers"                   ;031 Decoplan
    TCODE    .100, .0,       "ListeGaz"                  ;032 Gaslist
    TCODE    .100, .50,      "RazMoyn."                  ;033 ResetAvr          (Reset average depth)
    TCODE    .100, .100,     "Sortir"                    ;034 Exit              (Exit current menu)
    TCODE    .0,   .0,       "Avion"                 	 ;035 NoFly (5Char max.)             (No-flight count-down)
;
; 32 custom function descriptors I (FIXED LENGTH = 15 chars).
    TCODE    .40,  .35,      "Début Plong.[m]"           ;036 Start Dive  [m]	(depth to switch to dive mode)
    TCODE    .40,  .35,      "Fin Plongée [m]"           ;037 End Dive    [m]	(depth to switch back to surface mode)
    TCODE    .40,  .35,      "Délai Fin [sec]"           ;038 End Delay [sec]  	(duration dive screen stays after end of dive)
    TCODE    .40,  .35,      "Eteindre  [min]"           ;039 Power Off [min]
    TCODE    .40,  .35,      "Pré-menu  [min]"           ;040 Pre-menu  [min]	(Delais to keep surface-mode menus displayed)
    TCODE    .40,  .35,      "Vitesse [m/min]"           ;041 velocity[m/min]
    TCODE    .40,  .35,      "Allumer  [mbar]"           ;042 Wake-up  [mbar]
    TCODE    .40,  .35,      "Max.Surf.[mbar]"           ;043 max.Surf.[mbar]
    TCODE    .40,  .35,      "Affichage GF[%]"           ;044 GF display  [%]
    TCODE    .40,  .35,      "Aff. O2 min [%]"           ;045 min. O2 Dis.[%]
    TCODE    .40,  .35,      "MenusPlong[min]"           ;046 Dive menus[min]
    TCODE    .40,  .35,      "Saturat. x  [%]"           ;047 Saturate x  [%]
    TCODE    .40,  .35,      "Desaturat. x[%]"           ;048 Desaturate x[%]
    TCODE    .40,  .35,      "NoFly Ratio [%]"           ;049 NoFly Ratio [%]	(Grandient factor tolerance for no-flight countdown).
    TCODE    .40,  .35,      "Alarme GF   [%]"           ;050 GF alarm 1  [%]
    TCODE    .40,  .35,      "Aff.CNS Surf[%]"           ;051 CNSshow surf[%]
    TCODE    .40,  .35,      "Décal. Déco [m]"           ;052 Deco Offset [m]
    TCODE    .40,  .35,      "ppO2 mini [bar]"           ;053 ppO2 low  [bar]
    TCODE    .40,  .35,      "ppO2 maxi [bar]"           ;054 ppO2 high [bar]
    TCODE    .40,  .35,      "Aff. ppO2 [bar]"           ;055 ppO2 show [bar]
    TCODE    .40,  .35,      "Freq. Mesures  "           ;056 sampling rate  
    TCODE    .40,  .35,      "Diviseur Temp. "           ;057 Divisor Temp   
    TCODE    .40,  .35,      "Divis.Donn.Déco"           ;058 Divisor Decodat
    TCODE    .40,  .35,      "Diviseur GF    "           ;059 Divisor GF
    TCODE    .40,  .35,      "Diviseur ppO2  "           ;060 Divisor ppO2 
    TCODE    .40,  .35,      "Divis. Paliers "           ;061 Divisor Decoplan
    TCODE    .40,  .35,      "Diviseur CNS   "           ;062 Divisor CNS
    TCODE    .40,  .35,      "Aff.CNSPlong[%]"           ;063 CNSshow dive[%]
    TCODE    .40,  .35,      "Décalage Carnet"           ;064 Logbook offset 
    TCODE    .40,  .35,      "Dern. Palier[m]"           ;065 Last Deco at[m]
    TCODE    .40,  .35,      "Fin Apnée   [h]"           ;066 End Apnoe   [h]
    TCODE    .40,  .35,      "Aff. Batt. %   "           ;067 Show Battery %
; End of function descriptor I
;
;licence:
    TCODE    .0,   .35,      "Ce    programme    est"    ;068 This program is
    TCODE    .0,   .65,      "distribué  dans le but"    ;069 distributed in the
    TCODE    .0,   .95,      "d'être   utile,   mais"    ;070 hope that it will be
    TCODE    .0,   .125,     "SANS AUCUNE  GARANTIE;"    ;071 useful, but WITHOUT
    TCODE    .0,   .155,     "sans même  la garantie"    ;072 ANY WARRANTY
    TCODE    .0,   .185,     "tacite    de   QUALITE"    ;073 even the implied
    TCODE    .0,   .215,     "MARCHANDE           ou"    ;074 warranty of
    TCODE    .0,   .35,      "D'ADEQUATION    A   UN"    ;075 MERCHANTABILITY or
    TCODE    .0,   .65,      "USAGE PARTICULIER."        ;076 FITNESS FOR A
    TCODE    .0,   .95,      "Référez-vous    a   la"    ;077 PARTICULAR PURPOSE.
    TCODE    .0,   .125,     "Licence       Publique"    ;078 See the GNU General
    TCODE    .0,   .155,     "Générale GNU pour plus"    ;079 Public License for
    TCODE    .0,   .185,     "de détails sur:"           ;080 more details:
    TCODE    .0,   .215,     "heinrichsweikamp.com"      ;081 heinrichsweikamp.com
; end of licence
;
    TCODE    .118,  .54,     "Palier"                    ;082 Decostop
    TCODE    .0,    .0,      "m/min"                     ;083 m/min
    TCODE    .90,   .113,    "SansPalier"                ;084 No Stop
    TCODE    .139,  .113,    "DTR"                       ;085 TTS
    TCODE    .125,  .0,      "Durée"                     ;086 Divetime
    TCODE    .0,    .0,      "Profondeur"                ;087 Depth
    TCODE    .0,    .0,      "Premier Gaz?"              ;088 First Gas?
    TCODE    .0,    .0,      "Défaut:"                   ;089 Default:
    TCODE    .0,    .0,      "Minutes"                   ;090 Minutes
    TCODE    .0,    .0,      "Mois   "                   ;091 Month  
    TCODE    .0,    .0,      "Jour   "                   ;092 Day    
    TCODE    .0,    .0,      "Année  "                   ;093 Year   
    TCODE    .0,    .0,      "Réglage "                  ;094 Set 
#IFNDEF IMPERIAL
    TCODE    .0,    .0,      "#Gaz "                     ;095 Gas# 
#ELSE
    TCODE    .0,    .0,      "#Gaz"                      ;095 Gas#
#ENDIF
    TCODE    .0,    .0,      "Oui"                       ;096 Yes
    TCODE    .0,    .0,      "Valeur:"                   ;097 Current:
    TCODE    .31,   .2,      "Menu Réglages:"            ;098 Setup Menu:
    TCODE    .20,   .35,     "Config Fonctions I"        ;099 Custom FunctionsI
    TCODE    .20,   .125,    "ModeDéco:"                 ;100 Decotype:
    TCODE    .85,   .125,    "ZH-L16 OC"                 ;101 ZH-L16 OC
    TCODE    .85,   .125,    "Profondi."                 ;102 Gauge    
    TCODE    .85,   .125,    "Profondi."                 ;103 Gauge
    TCODE    .85,   .125,    "ZH-L16 CC"                 ;104 ZH-L16 CC
    TCODE    .0,    .0,      "Gaz Actif ? "              ;105 Active Gas?
    TCODE    .31,   .2,      "Liste des Gaz"            ;106 Gas Setup - Gaslist
    TCODE    .20,   .95,     "Prof. +/-:"  		         ;107 Depth +/-:
    TCODE    .20,   .125,    "Changer:"	                 ;108 Change:
    TCODE    .20,   .155,    "Défaut:"                   ;109 Default:
    TCODE    .20,   .65,     "Menu SetPoint CCR"         ;110 CCR SetPoint Menu
    TCODE    .20,   .2,      "Menu SetPoint CCR"         ;111 CCR SetPoint Menu
    TCODE    .0,    .0,      "#SP"                       ;112 SP#
    TCODE    .20,   .95,     "Info Batterie"             ;113 Battery Info
    TCODE    .6,   .2,       "Informations Batterie"     ;114 Battery Information
    TCODE    .0,    .9,      "Cycles:"                   ;115 Cycles:
    TCODE    .85,   .125,    "Apnée"                     ;116 Apnoe
    TCODE    .0,    .18,     "Dern.Complète:"            ;117 Last Complete:
    TCODE    .0,    .27,     "PlusBas Vbat:"             ;118 Lowest Vbatt:
    TCODE    .0,    .36,     "PlusBas le :"              ;119 Lowest at:
    TCODE    .0,    .45,     "Tmin:"                     ;120 Tmin:
    TCODE    .0,    .54,     "Tmax:"                     ;121 Tmax:
    TCODE    .100,  .124,    "Suite"                   	 ;122 More (Gaslist)
    TCODE    .100,  .25,     "O2 +"                      ;123 O2 +
    TCODE    .100,  .50,     "O2 -"                      ;124 O2 -
    TCODE    .100,  .75,     "He +"                      ;125 He +
    TCODE    .100,  .100,    "He -"                      ;126 He -
    TCODE    .100,  .0,      "Sortie"                    ;127 Exit
    TCODE    .100,  .25,     "Suppr."                    ;128 Delete
    TCODE    .20,   .65,     "Débug:"                    ;129 Debug:
    TCODE    .65,   .65,     "OUI"                       ;130 ON 
    TCODE    .65,   .65,     "NON"                       ;131 OFF
    TCODE    .100,  .50,     "Suppr.tout"                ;132 Del. all
    TCODE    .10,   .0,      "Réinitialisation"          ;133 Unexpected reset from
    TCODE    .10,   .25,     "inattendue! Merci de"      ;134 Divemode! Please help
    TCODE    .10,   .50,     "reporter les données"      ;135 and report the Debug 
    TCODE    .10,   .75,     "d'analyse ci-dessous:"     ;136 Information below!
    TCODE    .100,  .0,      "Bailout"                   ;137 Bailout
    TCODE    .85,   .125,    "Apnée    "                 ;138 Apnoe    
    TCODE    .105,  .120,    "Déscente"                  ;139 Descent
    TCODE    .105,  .60,     "Surface"                   ;140 Surface
    TCODE    .65,   .2,      "Quit?"                     ;141 Quit?
    TCODE    .20,   .155,    "Suite..."                  ;142 More
    TCODE    .42,   .72,     "Confirm:"                  ;143 Confirm:
    TCODE    .55,   .2,      "Menu 2:"                   ;144 Menu 2:
    TCODE    .52,   .96,     "Annul."                    ;145 Cancel
    TCODE    .52,   .120,    "OK!"                       ;146 OK!
    TCODE    .20,   .35,     "Suite..."                  ;147 More
    TCODE    .0,    .0,      ":.........:"               ;148 :.........:
    TCODE    .0,    .8,      "ppO2" 	                 ;149 ppO2
    TCODE    .2,    .39,     "bar "                      ;150 bar 
    TCODE    .108,  .216,    "Repère?"                   ;151 Marker?
    TCODE    .85,   .125,    "L16-GF OC"                 ;152 L16-GF OC
    TCODE    .20,   .65,     "Config Fonctions II"       ;153 Custom FunctionsII
;
; 32 custom function descriptors II (FIXED LENGTH = 15 chars).
    TCODE    .40,   .35,     "GF Bas      [%]"           ;154 GF Low      [%]
    TCODE    .40,   .35,     "GF Haut     [%]"           ;155 GF High     [%]
    TCODE    .40,   .35,     "CouleurBatterie"           ;156 Color# Battery 
    TCODE    .40,   .35,     "CouleurStandard"           ;157 Color# Standard
    TCODE    .40,   .35,     "Couleur Legende"           ;158 Color# Divemask
    TCODE    .40,   .35,     "Couleur Alarmes"           ;159 Color# Warnings
    TCODE    .40,   .35,     "Secs.TempsPlong"           ;160 Divemode secs. 
    TCODE    .40,   .35,     "Ajuster SP fixe"           ;161 Adjust fixed SP
    TCODE    .40,   .35,     "Alarme Plafond "           ;162 Warn Ceiling
#IFNDEF IMPERIAL
    TCODE    .40,   .35,     "unused         "           ;163 unused
#ELSE
    TCODE    .40,   .35,     "Unites metriq. "           ;163 Metric System
#ENDIF
    TCODE    .40,   .35,     "Aff.MeilleurGaz"           ;164 Blink BetterGas	(Remainder in divemode to switch to a better decompression gas).
    TCODE    .40,   .35,     "AlarmProf[mbar]"           ;165 DepthWarn[mbar]
    TCODE    .40,   .35,     "Alarme CNS  [%]"           ;166 CNS warning [%]
    TCODE    .40,   .35,     "Alarme GF   [%]"           ;167 GF warning  [%]
    TCODE    .40,   .35,     "Al. ppO2  [bar]"           ;168 ppO2 warn [bar]
    TCODE    .40,   .35,     "Al.Vites[m/min]"           ;169 Vel.warn[m/min]
    TCODE    .40,   .35,     "Décal Heur/Jour"           ;170 Time offset/day
    TCODE    .40,   .35,     "Aff. Altimètre "           ;171 Show altimeter
    TCODE    .40,   .35,     "Aff. Repère    "           ;172 Show Log-Marker
    TCODE    .40,   .35,     "Aff. Chrono.   "           ;173 Show Stopwatch
    TCODE    .40,   .35,     "Aff.GraphTissus"           ;174 ShowTissueGraph
    TCODE    .40,   .35,     "Aff.Tiss.Direct"           ;175 Show Lead.Tiss.
    TCODE    .40,   .35,     "Ordre paliers  "           ;176 Shallow stop 1st   (Reverse order of deco plans)
    TCODE    .40,   .35,     "Chgt gaz  [min]"           ;177 Gas switch[min]   (Show Countdown after gas change)
    TCODE    .40,   .35,     "ConsoFond[/min]"           ;178 BottomGas[/min]   (Bottom gas usage, for volume estimation).
    TCODE    .40,   .35,     "ConsoDeco[/min]"           ;179 AscentGas[/min]   (Ascent+Deco gas usage)
    TCODE    .40,   .35,     "DTR Réchap[min]"           ;180 Future TTS[min]   (Compute TTS for extra time at current depth)
    TCODE    .40,   .35,     "Alarme Sout.[l]"           ;181 Cave Warning[l]   (Consomation warning for cave divers)
    TCODE    .40,   .35,     "Indic. Vitesse "           ;182 Graph. Velocity	(Show a graphical representation of the ascend speed)
    TCODE    .40,   .35,     "Aff. ppO2 pSCR "           ;183 Show pSCR ppO2	(Show the ppO2 for pSCR divers)
    TCODE    .40,   .35,     "BaisseO2pSCR[%]"           ;184 pSCR O2 Drop[%]	(pSCR O2 drop in percent)
    TCODE    .40,   .35,     "Rapp.Poum. pSCR"           ;185 pSCR lung ratio	(pSCR counterlung ratio)
; End of function descriptor II
;
    TCODE    .13,   .2,      "Config Fonctions II"       ;186 Custom Functions II
    TCODE    .20,   .95,     "Voir la licence"           ;187 Show License
    TCODE    .0,    .2,      "Résultat Sim:"             ;188 Sim. Results:
    TCODE    .90,   .25,     "Surface"                   ;189 Surface
    TCODE    .0,    .0,      "ppO2 +"                    ;190 ppO2 +
    TCODE    .0,    .0,      ""                          ;191 unused
    TCODE    .0,    .0,      "Dil."                      ;192 Dil.			       (Rebreather diluant)

; 32 custom function descriptors III (FIXED LENGTH = 15 chars).
    TCODE    .40,   .35,     "CouleurInactive"           ;193 Color# inactive
    TCODE    .40,   .35,     "Palier sécurité"           ;194 Use safety stop
    TCODE    .40,   .35,     "Aff. GF Nodeco "           ;195 Show GF in NDL	(If GF > CF08)
    TCODE    .40,   .35,     "Alt. GF Bas [%]"           ;196 Alt. GF Low [%]
    TCODE    .40,   .35,     "Alt. GF Haut[%]"           ;197 Alt. GF High[%]
    TCODE    .40,   .35,     "Changement GF  "           ;198 Allow GF change
    TCODE    .40,   .35,     "Durée P.Sécu[s]"  		 ;199 S.StopLength[s] (CF70: Safety Stop Duration [ s])
    TCODE    .40,   .35,     "Début P.Sécu[m]"  		 ;200 S.StopStart [m] (CF71: Safety Stop Start Depth [m])
    TCODE    .40,   .35,     "Fin P.Sécu  [m]"  		 ;201 S.StopEnd   [m] (CF72: Safety Stop End Depth [m])
    TCODE    .40,   .35,     "RaZ P.Sécu  [m]"  		 ;202 S.StopReset [m] (CF73: Safety Stop Reset Depth [m])
    TCODE    .40,   .35,     "AlarmeAccu[min]"           ;203 Batt. Time [min] (CF74: Battery time-out [min])
    TCODE    .40,   .35,     "unused         "           ;204 unused
    TCODE    .40,   .35,     "unused         "           ;205 unused
    TCODE    .40,   .35,     "unused         "           ;206 unused
    TCODE    .40,   .35,     "unused         "           ;207 unused
    TCODE    .40,   .35,     "unused         "           ;208 unused
    TCODE    .40,   .35,     "unused         "           ;209 unused
    TCODE    .40,   .35,     "unused         "           ;210 unused
    TCODE    .40,   .35,     "unused         "           ;211 unused
    TCODE    .40,   .35,     "unused         "           ;212 unused
    TCODE    .40,   .35,     "unused         "           ;213 unused
    TCODE    .40,   .35,     "unused         "           ;214 unused
    TCODE    .40,   .35,     "unused         "           ;215 unused
    TCODE    .40,   .35,     "unused         "           ;216 unused
    TCODE    .40,   .35,     "unused         "           ;217 unused
    TCODE    .40,   .35,     "unused         "           ;218 unused
    TCODE    .40,   .35,     "unused         "           ;219 unused
    TCODE    .40,   .35,     "unused         "           ;220 unused
    TCODE    .40,   .35,     "unused         "           ;221 unused
    TCODE    .40,   .35,     "unused         "           ;222 unused
    TCODE    .40,   .35,     "unused         "           ;223 unused
    TCODE    .40,   .35,     "unused         "           ;224 unused
;
	TCODE    .7,    .2,      "Config Fonctions III"      ;225 Custom Functions III
    TCODE    .85,   .125,    "pSCR-GF  "                 ;226 pSCR-GF
	TCODE    .90,   .54,     "PalierSécu"                ;227 SafetyStop
    TCODE    .0,    .0,      "Plongées totales: "        ;228 Total Dives: 
    TCODE    .20,   .35,     "Réglage Diluant"           ;229 Diluent Setup
    TCODE    .20,   .65,     "Réglage SetPoint"          ;230 Setpoint Setup
    TCODE    .5,    .2,      "Liste des Diluants"	 	 ;231 Dil. Setup - Gaslist
    TCODE    .100,  .100,	 "Diluant"                   ;232 Diluent
    TCODE    .93,   .170,    "  Ceiling"                 ;233 Ceiling (9Chars, right alligned)
    TCODE    .20,   .95,     "SP Mode:  "                ;234 SP Mode: (10 chars)

    TCODE    .10,   .2,      "Mode de Déco changé!"      ;235 Decomode changed!
    TCODE    .85,   .125,    "L16-GF CC"                 ;236 L16-GF CC
    TCODE    .2,    .12,     "Non trouvé"                ;237 Not found
    TCODE    .100,  .0,      "SetPoint"                  ;238 SetPoint
    TCODE    .100,  .0,      "SansDéco"                  ;239 No Deco
    TCODE    .90,   .50,     "Interval:"                 ;240 Interval:
    TCODE    .100,  .75,     "Contrast"                  ;241 Display
    TCODE    .100,  .0,      "SansDéco"                  ;242 No deco
    TCODE    .132,  .0,      "béta"                      ;243 beta
    TCODE    .100,  .100,    "unuse"                     ;244 unuse
    TCODE    .20,   .65,     "RaZ CF,Gaz & Déco"         ;245 Reset CF,Gas & Deco
    TCODE    .58,   .145,    "Batt!"                     ;246 Batt!
    TCODE    .20,   .125,    "Simulateur"                ;247 Simulator
    TCODE    .27,   .2,      "Simulateur OSTC"           ;248 OSTC Simulator
    TCODE    .20,   .65,     "Début Simulation..."       ;249 Start Dive
    TCODE    .100,  .25,     "+ 1m"                      ;250 + 1m
    TCODE    .100,  .50,     "- 1m"                      ;251 - 1m
    TCODE    .100,  .75,     "+10m"                      ;252 +10m
    TCODE    .100,  .100,    "-10m"                      ;253 -10m
    TCODE    .100,  .0,      "Fermer"                    ;254 Close
    TCODE    .125,  .170,    "Heure"                     ;255 Time
;
; Text Bank2 (Texts 256-511)
;
    TCODE    .0,    .0,      "x"                         ;256 x
    TCODE    .20,   .35,     "Format Date:"              ;257 Date format:
    TCODE    .24,   .2,      "Menu Réglages 2:"          ;258 Setup Menu 2:
    TCODE    .105,  .35,     "MMJJAA"                    ;259 MMDDYY
    TCODE    .105,  .35,     "JJMMAA"                    ;260 DDMMYY
    TCODE    .105,  .35,     "AAMMJJ"                    ;261 YYMMDD
    TCODE    .1,    .1,      "OSTC "                     ;262 OSTC 
    TCODE    .65,   .168,    "Bail "                     ;263 Bail 
    TCODE    .7,    .48,     "Air   "                    ;264 Air
    TCODE    .115,  .135,    "Air   "                    ;265 Air

    TCODE    .0,    .0,      "pSCR Info"             	 ;266 pSCR Info (Must be 9Chars!)
	TCODE    .0,    .184,    "Max."                      ;267 Max.
    TCODE    .93,   .170,    "GF Values"                 ;268 GF Values
    TCODE    .100,  .50,     "ToggleGF"               	 ;269 ToggleGF (In Divemode Menu)
    TCODE    .93,   .170,    "Decogas"		             ;270 Decogas

; New CFs Warning
    TCODE    .3,    .2,      "Nouvelles CF ajoutées!"    ;271 New CF added!
    TCODE    .0,    .35,     "Nouv. Config Fonctions"    ;272 New CustomFunctions
    TCODE    .0,    .65,     "ajoutées! Regardez les"    ;273 were added! Check
    TCODE    .0,    .95,     "Menus  CF I, II et III"    ;274 CF I and CF II Menu
    TCODE    .0,    .125,    "pour plus de détails!"     ;275 for Details!
    TCODE    .20,   .125,    "Salinité: "                ;276 Salinity:
;
    TCODE    .20,   .95,     "Temps fond:"               ;277 Bottom Time:
    TCODE    .20,   .125,    "Prof. Max.:"               ;278 Max. Depth:
    TCODE    .20,   .155,    "Calculer la Déco"          ;279 Calculate Deco
    TCODE    .20,   .155,    "Luminosité:"          	 ;280 Brightness:
;
    TCODE    .97,   .170,    "Prof.Moyn"                 ;281 Avr.Depth
    TCODE    .90,   .170,    "TissuDirec"                ;282 Lead Tiss.
    TCODE    .118,   .170,   "Chrono"                    ;283 Stopwatch
    TCODE    .20,   .95,     "RaZ Carnet Plongées"       ;284 Reset Logbook
    TCODE    .20,   .125,    "Redémarrer l'OSTC"         ;285 Reboot OSTC
    TCODE    .20,   .155,    "RaZ Saturation"            ;286 Reset Decodata
; Altimeter extension
    TCODE    .20,   .155,    "Altimètre"                 ;287 Altimeter
    TCODE    .24,   .1,      "Réglage Altimètre"         ;288 Set Altimeter
    TCODE    .20,   .35,     "Référence: "               ;289 Sea ref: 
    TCODE    .0,    .0,      "Marche: "                  ;290 Enabled:
    TCODE    .20,   .95,     "Défaut: 1013 mbar"         ;291 Default: 1013 mbar
    TCODE    .20,   .125,    "+1 mbar"                   ;292 +1 mbar
    TCODE    .20,   .155,    "-1 mbar"                   ;293 -1 mbar
    TCODE    .85,   .185,    "Alt: "                     ;294 Alt: 
;
    TCODE    .20,   .95,     "Config Fonctions III"      ;295 Custom FunctionsIII
	TCODE    .50,    .2,     "Données brutes:"           ;296 Raw Data:
; Gas-setup addons:
    TCODE    .0,    .0,      "PMU:"                      ;297 MOD:                  (max operating depth of a gas).
    TCODE    .0,    .0,      "PEN:"                      ;298 END:                  (equivalent nitrogen depth of a gas).
    TCODE    .0,    .0,      "PEA:"                      ;299 EAD:                  (equivalent air depth of a gas).
	TCODE    .100,  .125,	 "Suite"					 ;300 More               	(Enable/Disable Gas underwater)
	TCODE    .0,    .2,      "OCR Gas Usage:"            ;301 OCR Gas Usage:        (Planned gas consumtion by tank).
; 115k Bootloader support:
	TCODE	 .45,	.100,	 "Bootloader"				 ;302 Bootloader
	TCODE	 .40,	.130,	 "Attendez SVP!"			 ;303 Please wait!
	TCODE	 .50,	.130,	 "Annulée!"					 ;304 Aborted
; @5 variant
    TCODE    .0,    .0,      " Futur DTR"                ;305 Future TTS            (=10 chars. Title for @5 customview).
;
    TCODE    .100,  .125,    "Stop Sim"                  ;306 Quit Sim 				(=8char max. Quit Simulator mode)
; Dive interval
    TCODE    .20,   .35,     "Intervale:"                ;307 Interval:
    TCODE    .0,    .0,      "Non    "                   ;308 Now                   (7 chars min)
#IFNDEF BOTTOM
	TCODE	 .108,	.112,	 "Moyenne"			 		 ;309 Average
;
	TCODE	 .116,	.54,	 "Chrono"		 		 	 ;310 Stopwatch             (BIG Stopwatch in Gauge mode)
#ELSE
	TCODE	 .108,	.124,	 "Moyenne"			 		 ;309 Average
;
	TCODE	 .116,	.60,	 "Chrono"		 		 	 ;310 Stopwatch             (BIG Stopwatch in Gauge mode)
#ENDIF
;
; Cave consomation
    TCODE    .0,    .0,      "Bail. Sout"                ;311 Cave Bail.            (=10 chars.)
; DISPLAY Brightness settings
    TCODE    .103,  .155,    "Eco  "	    	         ;312 Eco 					(Same length as #313!)
    TCODE    .103,  .155,    "Forte" 	                 ;313 High					(Same length as #312!)

; ZH-L16 mode description
    TCODE    .0,    .35,     "TypeDéco: ZH-L16 OC   "    ;314 Decotype: ZH-L16 OC	(22 chars maximum)
    TCODE    .0,    .65,     "Pour les plongeurs  en"    ;315 For Open Circuit
    TCODE    .0,    .95,     "Circuit Ouvert.       "    ;316 Divers. Supports 5
    TCODE    .0,    .125,    "Supporte 5 Gaz Trimix."    ;317 Trimix Gases.
    TCODE    .0,    .155,    "Config des gaz dans le"    ;318 Configure your gas
    TCODE    .0,    .185,    "menu Réglage des Gaz. "    ;319 in Gassetup menu.
    TCODE    .0,    .215,    "Vérifiez CF11 & CF12 !"    ;320 Check CF11 & CF12 !
; Gaugemode description
    TCODE    .0,    .35,     "TypeDéco:Profondimètre"    ;321 Decotype: Gauge    
    TCODE    .0,    .65,     "La durée  est affichée"    ;322 Divetime will be in
    TCODE    .0,    .95,     "en Minutes:Secondes.  "    ;323 Minutes:Seconds.   
    TCODE    .0,    .125,    "L'OSTC2  ne  calculera"    ;324 OSTC2 will not     
    TCODE    .0,    .155,    "pas de Déco,  de durée"    ;325 compute Deco, NoFly
    TCODE    .0,    .185,    "sans avion ni de temps"    ;326 time and Desat.
    TCODE    .0,    .215,    "de désaturation !     "    ;327 time at all!
; Const.ppO2 description
    TCODE    .0,    .35,     "TypeDéco: ZH-L16 CC   "    ;328 Decotype: ZH-L16 CC
    TCODE    .0,    .65,     "Pour les  recycleurs a"    ;329 For Closed
    TCODE    .0,    .95,     "circuit fermé."    		 ;330 Circuit rebreathers
    TCODE    .0,    .125,    "Configurez    les    3"    ;331 Configure the 3
    TCODE    .0,    .155,    "SetPoints   dans    le"    ;332 SetPoints in CCR -
    TCODE    .0,    .185,    "Menu SetPoint CCR.    "    ;333 Setup menu. 5 bail-
    TCODE    .0,    .215,    "5 bailouts disponibles"    ;334 outs are available.
; Apnoemode description
    TCODE    .0,    .35,     "TypeDéco: Apnée       "    ;335 Decotype: Apnoe
    TCODE    .0,    .65,     "L'OSTC2 affichera  les"    ;336 OSTC2 will display
    TCODE    .0,    .95,     "descentes   séparément"    ;337 each descent separ-
    TCODE    .0,    .125,    "en    Minutes:Secondes"    ;338 ately in Min:Sec.
    TCODE    .0,    .155,    "sans calculer de Déco."    ;339 Will temporally set
    TCODE    .0,    .185,    "Les  mesures  se  font"    ;340 samplerate to 1 sec
    TCODE    .0,    .215,    "toutes les secondes.  "    ;341 No Deco calculation
; Multi GF OC mode description
    TCODE    .0,    .35,     "TypeDéco: L16-GF OC   "    ;342 Decotype: L16-GF OC
    TCODE    .0,    .65,     "Calcul  de  Déco  avec"    ;343 Decompression cal-
    TCODE    .0,    .95,     "Facteurs  de  Gradient"    ;344 culations with the
    TCODE    .0,    .125,    "(GF_bas/GF_haut).     "    ;345 GF-Method (GF_lo/GF
    TCODE    .0,    .155,    "Vérifiez CF32 & CF33 !"    ;346 _hi). Check CF32 &
    TCODE    .0,    .185,    "Pour  Circuit   Ouvert"    ;347 CF33! Open Circuit
    TCODE    .0,    .215,    "avec paliers profonds."    ;348 with Deep Stops.
; Multi GF CC mode description
    TCODE    .0,    .35,     "TypeDéco: L16-GF CC   "    ;349 Decotype: L16-GF CC
    TCODE    .0,    .65,     "Calcul  de  Déco  avec"    ;350 Decompression cal-
    TCODE    .0,    .95,     "Facteurs  de  Gradient"    ;351 culations with the
    TCODE    .0,    .125,    "(GF_bas/GF_haut).     "    ;352 GF-Method (GF_lo/GF
    TCODE    .0,    .155,    "Vérifiez CF32 & CF33 !"    ;353 _hi). Check CF32 &
    TCODE    .0,    .185,    "Pour   Circuit   Fermé"    ;354 CF33!Closed Circuit
    TCODE    .0,    .215,    "avec paliers profonds."    ;355 with Deep Stops.
; pSCR-GF mode description
    TCODE    .0,    .35,     "Type déco: pSCR-GF"     	 ;356 Decotype: pSCR-GF
    TCODE    .0,    .65,     "Pour semi passif-"     	 ;357 For passive semi-
    TCODE    .0,    .95,     "circuit fermé."            ;358 closed rebreather.
    TCODE    .0,    .125,    "Vérifiez CF32&CF33"     	 ;359 Check CF32 & CF33
    TCODE    .0,    .155,    "pour facteurs gradient"    ;360 for gradient factors
    TCODE    .0,    .185,    "et CF61-CF63 pour"     	 ;361 and CF61-CF63 for
    TCODE    .0,    .215,    "fonctions pSCR."    		 ;362 pSCR features.
;
; Setpoint Mode
    TCODE    .60,  .95,     "Manual"	    	        ;363 Manual      Same length as #364
    TCODE    .60,  .95,     "Auto  " 	                ;364 Auto        Same length as #363

#IFDEF SEAWOOCH
; room for possible extension in original code
    TCODE    .0,    .0,      ""		                     ;365 unused
    TCODE    .0,    .0,      ""		                     ;366 unused
    TCODE    .0,    .0,      ""		                     ;367 unused
    TCODE    .0,    .0,      ""		                     ;368 unused
    TCODE    .0,    .0,      ""		                     ;369 unused
    TCODE    .0,    .0,      ""		                     ;370 unused
    TCODE    .0,    .0,      ""		                     ;371 unused
    TCODE    .0,    .0,      ""		                     ;372 unused
    TCODE    .0,    .0,      ""		                     ;373 unused
    TCODE    .0,    .0,      ""		                     ;374 unused
    TCODE    .0,    .0,      ""		                     ;375 unused
    TCODE    .0,    .0,      ""		                     ;376 unused
    TCODE    .0,    .0,      ""		                     ;377 unused
    TCODE    .0,    .0,      ""		                     ;378 unused
    TCODE    .0,    .0,      ""		                     ;379 unused
#ENDIF

#IFDEF IMPERIAL
    TCODE    .100,  .25,     "+  1ft"                    ;380 was 363 +1ft
    TCODE    .100,  .50,     "-  1ft"                    ;381 was 364 -1ft
    TCODE    .100,  .75,     "+ 10ft"                    ;382 was 365 +10ft
    TCODE    .100,  .100,    "- 10ft"                    ;383 was 366 -10ft
    TCODE    .0,    .0,      "ft/min"                    ;384 was 367 ft/min
#ENDIF
#IFDEF BOTTOM
    TCODE    .50,  .2,       "RaZ Chro?"                 ;385 was 368 Reset Timer?
    TCODE    .50,  .2,       "RaZ Moyn?"                 ;386 was 369 Reset Average?
#ENDIF
#IFDEF CCR_CTRL
    TCODE    .20,   .95,     "eCTRL Setup"               ;387 CCR Controller Setup
    TCODE    .10,    .2,     "eCTRL Setup"               ;388 CCR Controller Setup
    TCODE    .20,   .95,     "Disable Ext ppO2?"         ;389
    TCODE    .20,   .95,     "Enable Ext ppO2?"          ;390
    TCODE    .20,   .125,    "Calibrate"                 ;391
    TCODE    .20,   .155,    "Calibrate Air ?"           ;392
    TCODE    .20,   .35,     "Not Found..."              ;393
	TCODE    .50,    .23,    "Max."                      ;394 Max. (at top of screen)

#ENDIF
;=============================================================================
