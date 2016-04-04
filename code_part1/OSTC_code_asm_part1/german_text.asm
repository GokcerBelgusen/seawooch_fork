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
; 2011/03/29 : Oliver J. Albrecht: German translation.
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
#DEFINE	TXT_MBAR7		     " mbar  "                   ; " mbar  "              
#DEFINE	TXT_MBAR5		     "mbar "                     ; "mbar "                
#DEFINE	TXT_BAR4		     "bar "                      ; "bar "                 
#DEFINE	TXT_BAR3			 "bar"                       ; "bar"
#DEFINE	TXT_ALT5		     "Alt: "                     ; "Alt: "                
#DEFINE	TXT_KGL4		     "kg/l"                      ; "kg/l"                 
#DEFINE	TXT_VOLT2			 "V "                        ; "V "
#DEFINE	TXT_VOLT1		     "V"                         ; "V"                    
#DEFINE	TXT_STEP5		     "Stufe:"                    ; "Step:"
#DEFINE	TXT_CF2			     "CF"                        ; "CF"                   
#DEFINE	TXT_O2_4		     "O2: "                      ; "O2: "                 
#DEFINE	TXT_O2_3		     "O2 "                       ; "O2 "                  
#DEFINE	TXT_AIR4		     "AIR "                      ; "AIR "                 
#DEFINE	TXT_ERR4		     "ERR "                      ; "ERR "                 
#DEFINE	TXT_HE4			     "He: "                      ; "He: "                 
#DEFINE	TXT_NX3			     "NX "                       ; "NX "                  
#DEFINE	TXT_TX3			     "TX "                       ; "TX "                  
#DEFINE	TXT_AT4			     " in "                      ; " at "
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
    TCODE    .0,   .0,       "Erstelle MD2 Hash"         ;001 Building MD2 Hash
    TCODE    .0,   .25,      "Bitte warten..."           ;002 Please Wait...
    TCODE    .0,   .2,       "HeinrichsWeikamp OSTC2"    ;003 HeinrichsWeikamp OSTC2
    TCODE    .65,  .2,       "Menü?"                     ;004 Menu?
    TCODE    .65,  .2,       "Menü:"                     ;005 Menu:
    TCODE    .20,  .35,      "Logbuch"                   ;006 Logbook
    TCODE    .20,  .65,      "Gas-Einstellungen"         ;007 Gas Setup
    TCODE    .20,  .35,      "Uhrzeit einstellen"        ;008 Set Time
    TCODE    .20,  .95,      "Daten Zurücksetzen"        ;009 Reset Menu
    TCODE    .20,  .125,     "Einstellungen"             ;010 Setup
    TCODE    .20,  .185,     "Ende"                      ;011 Exit
    TCODE    .97, .2,        "Warten..."                 ;012 Wait...
    TCODE    .0,   .24,      "MD2 Hash:"                 ;013 MD2 Hash:
    TCODE    .0,   .0,       "Entsä"                     ;014 Desat         (Desaturation count-down)
    TCODE    .50,  .2,       "Interface"                 ;015 Interface		(Connected to USB)
    TCODE    .10,  .30,      "Start"                     ;016 Start
    TCODE    .10,  .55,      "Daten"                     ;017 Data
    TCODE    .10,  .80,      "Kopfzeile"                 ;018 Header
    TCODE    .10,  .105,     "Profil"                    ;019 Profile
    TCODE    .10,  .130,     "Fertig."                   ;020 Done.
    TCODE    .20,  .35,      "Zurücksetzen abbr."        ;021 Cancel Reset
    TCODE    .32,  .65,      "Uhrzeit:"                  ;022 Time:
    TCODE    .32,  .95,      "Datum:"                    ;023 Date:
    TCODE    .0,   .215,     "Einst.Stunden"             ;024 Set Hours
    TCODE    .6,   .0,       "Zurücksetzen..."           ;025 Reset...
    TCODE    .55,  .2,       "Logbuch"                   ;026 Logbook
    TCODE    .14,  .2,       "Custom Funktionen I"       ;027 Custom Functions I
    TCODE    .14,  .2,       "Daten Zurücksetzen"        ;028 Reset Menu
    TCODE    .14,  .2,       "Uhrzeit einstellen:"       ;029 Set Time:
    TCODE    .100, .50,      "Markern"                   ;030 SetMarker         (Add a mark in logbook profile)
    TCODE    .100, .25,      "Dekoplan"                  ;031 Decoplan
    TCODE    .100, .0,       "Gasliste"                  ;032 Gaslist
    TCODE    .100, .50,      "DTiefeLö"                  ;033 ResetAvr          (Reset average depth)
    TCODE    .100, .100,     "Ende"                      ;034 Exit		        (Exit current menu)
    TCODE    .0,   .0,       "Flugv"                     ;035 NoFly		        (No-flight count-down)
;
; 32 custom function descriptors I (FIXED LENGTH = 15 chars).
    TCODE    .40,  .35,      "TG Start    [m]"           ;036 Start Dive  [m]	(depth to switch to dive mode)
    TCODE    .40,  .35,      "TG Ende     [m]"           ;037 End Dive    [m]	(depth to switch back to surface mode)
    TCODE    .40,  .35,      "TGNachlauf[sek]"           ;038 End Delay [sec]  	(duration dive screen stays after end of dive)
    TCODE    .40,  .35,      "Standby   [min]"           ;039 Power Off [min]
    TCODE    .40,  .35,      "Pre-Menü  [min]"           ;040 Pre-menu  [min]	(Delais to keep surface-mode menus displayed)
    TCODE    .40,  .35,      "Geschw. [m/min]"           ;041 velocity[m/min]
    TCODE    .40,  .35,      "Aufwachen[mbar]"           ;042 Wake-up  [mbar]
    TCODE    .40,  .35,      "max.Ober.[mbar]"           ;043 max.Surf.[mbar]
    TCODE    .40,  .35,      "GF Anzeige  [%]"           ;044 GF display  [%]
    TCODE    .40,  .35,      "min. O2 Dis.[%]"           ;045 min. O2 Dis.[%]
    TCODE    .40,  .35,      "TG Menüs  [min]"           ;046 Dive menus[min]
    TCODE    .40,  .35,      "Sättigung x [%]"           ;047 Saturate x  [%]
    TCODE    .40,  .35,      "Entsätt. x  [%]"           ;048 Desaturate x[%]
    TCODE    .40,  .35,      "Flugv.Ratio [%]"           ;049 NoFly Ratio [%]	(Grandient factor tolerance for no-flight countdown).
    TCODE    .40,  .35,      "GF Alarm 1  [%]"           ;050 GF alarm 1  [%]
    TCODE    .40,  .35,      "CNSAnzOberf.[%]"           ;051 CNSshow surf[%]
    TCODE    .40,  .35,      "Deko Versatz[m]"           ;052 Deco Offset [m]
    TCODE    .40,  .35,      "ppO2 min. [bar]"           ;053 ppO2 low  [bar]
    TCODE    .40,  .35,      "ppO2 max. [bar]"           ;054 ppO2 high [bar]
    TCODE    .40,  .35,      "ppO2 anz. [bar]"           ;055 ppO2 show [bar]
    TCODE    .40,  .35,      "Abtastrate     "           ;056 sampling rate  
    TCODE    .40,  .35,      "Divisor Temp   "           ;057 Divisor Temp   
    TCODE    .40,  .35,      "Divisor Dekodat"           ;058 Divisor Decodat
    TCODE    .40,  .35,      "Divisor GF     "           ;059 Divisor GF
    TCODE    .40,  .35,      "Divisor ppO2   "           ;060 Divisor ppO2 
    TCODE    .40,  .35,      "Divisor Dekopln"           ;061 Divisor Decopln
    TCODE    .40,  .35,      "Divisor CNS    "           ;062 Divisor CNS
    TCODE    .40,  .35,      "CNS-Anz. TG [%]"           ;063 CNSshow dive[%]
    TCODE    .40,  .35,      "Logbuch Versatz"           ;064 Logbook offset 
    TCODE    .40,  .35,      "Letzte Deko [m]"           ;065 Last Deco at[m]
    TCODE    .40,  .35,      "Apnoe Ende  [h]"           ;066 End Apnoe   [h]
    TCODE    .40,  .35,      "Zeige Batterie%"           ;067 Show Battery %
; End of function descriptor I
;
;licence:
    TCODE    .0,   .35,      "Dieses Programm wird"      ;068 This program is
    TCODE    .0,   .65,      "in der Hoffnung bereit"    ;069 distributed in the
	TCODE    .0,   .95,      "gestellt, dass es nütz"    ;070 hope that it will be
    TCODE    .0,   .125,     "lich ist, aber OHNE"       ;071 useful, but WITHOUT
    TCODE    .0,   .155,     "JEDE GEWAEHRLEISTUNG,"     ;072 ANY WARRANTY
    TCODE    .0,   .185,     "auch keine implizierte"    ;073 even the implied
    TCODE    .0,   .215,     "Gebrauchstauglichkeit"     ;074 warranty of
    TCODE    .0,   .35,      "oder Eignung für einen"    ;075 MERCHANTABILITY or
    TCODE    .0,   .65,      "bestimmten Zweck."         ;076 FITNESS FOR A
    TCODE    .0,   .95,      "Weitere Informationen"     ;077 PARTICULAR PURPOSE.
    TCODE    .0,   .125,     "finden Sie in der GNU"     ;078 See the GNU General
    TCODE    .0,   .155,     "General Public License "   ;079 Public License for
    TCODE    .0,   .185,     "unter:"                    ;080 more details:
    TCODE    .0,   .215,     "heinrichsweikamp.com"      ;081 heinrichsweikamp.com
; end of licence
;
    TCODE    .93,  .54,      "Dekostopp"                 ;082 Decostop
    TCODE    .0,    .0,      "m/min"                     ;083 m/min
    TCODE    .100,  .113,    "Nullzeit"                  ;084 No Stop
    TCODE    .135,  .113,    "TTS"                       ;085 TTS
    TCODE    .93,  .0,       "Tauchzeit"                 ;086 Divetime
    TCODE    .0,    .0,      "Tiefe"                     ;087 Depth
    TCODE    .0,    .0,      "Erstes Gas?"               ;088 First Gas?
    TCODE    .0,    .0,      "Standard:"                 ;089 Default:
    TCODE    .0,    .0,      "Minuten"                   ;090 Minutes
    TCODE    .0,    .0,      "Monat  "                   ;091 Month  
    TCODE    .0,    .0,      "Tag    "                   ;092 Day    
    TCODE    .0,    .0,      "Jahr   "                   ;093 Year   
    TCODE    .0,    .0,      "Einst."                    ;094 Set 
    TCODE    .0,    .0,      "Gas# "                     ;095 Gas# 
    TCODE    .0,    .0,      "Ja"                        ;096 Yes
    TCODE    .0,    .0,      "Aktuell: "                 ;097 Current:
    TCODE    .14,   .2,      "Einstellungs-Menü:"        ;098 Setup Menu:
    TCODE    .20,   .35,     "Custom FunktionenI"        ;099 Custom FunctionsI
    TCODE    .20,   .125,    "Dekomod.:"                 ;100 Decotype:
    TCODE    .85,   .125,    "ZH-L16 OC"                 ;101 ZH-L16 OC
    TCODE    .85,   .125,    "Tiefenme."                 ;102 Gauge    
    TCODE    .85,   .125,    "Tiefenme."                 ;103 Gauge
    TCODE    .85,   .125,    "ZH-L16 CC"                 ;104 ZH-L16 CC
    TCODE    .0,    .0,      "Aktives Gas? "             ;105 Active Gas?
    TCODE    .10,   .2,      "Gas-Einst. - Gasliste"     ;106 Gas Setup - Gaslist
    TCODE    .20,   .95,     "Tiefe +/-:"		         ;107 Depth +/-:
    TCODE    .20,   .125,    "Wechsel:"			  		 ;108 Change:
    TCODE    .20,   .155,    "Standard:"                 ;109 Default:
    TCODE    .20,   .65,     "CCR Setup Menü"            ;110 CCR Setup Menu
    TCODE    .28,   .2,      "CCR Setup Menü"            ;111 CCR Setup Menu
    TCODE    .0,    .0,      "SP#"                       ;112 SP#
    TCODE    .20,   .95,     "Batterie Info"             ;113 Battery Info
    TCODE    .10,   .2,      "Batterie Information"      ;114 Battery Information
    TCODE    .0,    .9,      "Zyklen:"                   ;115 Cycles:
    TCODE    .85,   .125,    "Apnoe"                     ;116 Apnoe
    TCODE    .0,    .18,     "Zuletzt voll:"             ;117 Last Complete:
    TCODE    .0,    .27,     "Tiefste Vbatt:"            ;118 Lowest Vbatt:
    TCODE    .0,    .36,     "Datum tief:"               ;119 Lowest at:
    TCODE    .0,    .45,     "Tmin:"                     ;120 Tmin:
    TCODE    .0,    .54,     "Tmax:"                     ;121 Tmax:
    TCODE    .100,  .124,    "Mehr"                    	 ;122 More (Gaslist)
    TCODE    .100,  .25,     "O2 +"                      ;123 O2 +
    TCODE    .100,  .50,     "O2 -"                      ;124 O2 -
    TCODE    .100,  .75,     "He +"                      ;125 He +
    TCODE    .100,  .100,    "He -"                      ;126 He -
    TCODE    .100,  .0,      "Ende"                      ;127 Exit
    TCODE    .100,  .25,     "Löschen"                   ;128 Delete
    TCODE    .20,   .65,     "Debug:"                    ;129 Debug:
    TCODE    .65,   .65,     "AN "                       ;130 ON 
    TCODE    .65,   .65,     "AUS"                       ;131 OFF
    TCODE    .100,  .50,     "alle löschen"              ;132 Del. all
    TCODE    .10,   .0,      "Unerwarteter Reset im"     ;133 Unexpected reset from
    TCODE    .10,   .25,     "TG-Modus! Bitte melden"    ;134 Divemode! Please help
    TCODE    .10,   .50,     "Sie die u.a. Debug "       ;135 and report the Debug 
    TCODE    .10,   .75,     "Informationen!"            ;136 Information below!
    TCODE    .100,  .0,      "Bailout"                   ;137 Bailout
    TCODE    .85,   .125,    "Apnoe    "                 ;138 Apnoe    
    TCODE    .105,  .120,    "Abstieg"                   ;139 Descent
    TCODE    .105,  .60,     "Oberfl."                   ;140 Surface
    TCODE    .65,   .2,      "Beenden?"                  ;141 Quit?
    TCODE    .20,   .155,    "Mehr"                      ;142 More
    TCODE    .42,   .72,     "Sicher?"                   ;143 Confirm:
 	TCODE    .60,   .2,      "Menü 2:"                   ;144 Menu 2:
    TCODE    .52,   .96,     "Zurück"                    ;145 Cancel
    TCODE    .52,   .120,    "OK!"                       ;146 OK!
    TCODE    .20,   .35,     "Mehr"                      ;147 More
    TCODE    .0,    .0,      ":.........:"               ;148 :.........:
    TCODE    .0,    .8,      "ppO2"                      ;149 ppO2
    TCODE    .2,    .39,     "bar "                      ;150 bar 
    TCODE    .108,  .216,    "Marker?"                   ;151 Marker?
    TCODE    .85,   .125,    "L16-GF OC"                 ;152 L16-GF OC
    TCODE    .20,   .65,     "Custom FunktionenII"       ;153 Custom FunctionsII
;
; 32 custom function descriptors II (FIXED LENGTH = 15 chars).
    TCODE    .40,   .35,     "GF Low      [%]"           ;154 GF Low      [%]
    TCODE    .40,   .35,     "GF High     [%]"           ;155 GF High     [%]
    TCODE    .40,   .35,     "Farbe# Batterie"           ;156 Color# Battery 
    TCODE    .40,   .35,     "Farbe# Standard"           ;157 Color# Standard
    TCODE    .40,   .35,     "Farbe# Maske   "           ;158 Color# Divemask
    TCODE    .40,   .35,     "Farbe# Warnung "           ;159 Color# Warnings
    TCODE    .40,   .35,     "Tauchmodus Sek."           ;160 Divemode secs. 
    TCODE    .40,   .35,     "Festen SP ände."           ;161 Adjust fixed SP
    TCODE    .40,   .35,     "Warnung Ceiling"           ;162 Warn Ceiling
    TCODE    .40,   .35,     "unbenutzt      "           ;163 unused
    TCODE    .40,   .35,     "BesseresGasAnz "           ;164 Blink BetterGas	(Remainder in divemode to switch to a better decompression gas).
    TCODE    .40,   .35,     "TiefeWarn[mbar]"           ;165 DepthWarn[mbar]
    TCODE    .40,   .35,     "CNS Warnung [%]"           ;166 CNS warning [%]
    TCODE    .40,   .35,     "GF Warnung  [%]"           ;167 GF warning  [%]
    TCODE    .40,   .35,     "ppO2 Warn [bar]"           ;168 ppO2 warn [bar]
    TCODE    .40,   .35,     "GescWarn[m/min]"           ;169 Vel.warn[m/min]
    TCODE    .40,   .35,     "ZeitVersatz/Tag"           ;170 Time offset/day
    TCODE    .40,   .35,     "Höhenmesser anz"           ;171 Show altimeter
    TCODE    .40,   .35,     "Log-Marker anz."           ;172 Show Log-Marker
    TCODE    .40,   .35,     "Stoppuhr anz.  "           ;173 Show Stopwatch
    TCODE    .40,   .35,     "Gewebegraph anz"           ;174 ShowTissueGraph
    TCODE    .40,   .35,     "Leitgewebe anz "           ;175 Show Lead.Tiss.
    TCODE    .40,   .35,     "Flach.StoppOben"           ;176 Shallow stop 1st  (Reverse order of deco plans)
    TCODE    .40,   .35,     "Gas switch[min]"           ;177 Gas switch[min]   (Show Countdown after gas change)
    TCODE    .40,   .35,     "BottomGas[/min]"           ;178 BottomGas[/min]   (Bottom gas usage, for volume estimation).
    TCODE    .40,   .35,     "Sonst.Gas[/min]"           ;179 AscentGas[/min]   (Ascent+Deco gas usage)
    TCODE    .40,   .35,     "TTS @ Zeit[min]"           ;180 Future TTS[min]   (Compute TTS for extra time at current depth)
    TCODE    .40,   .35,     "Cave Warnung[l]"           ;181 Cave Warning[l]   (Consomation warning for cave divers)
    TCODE    .40,   .35,     "Graph. Geschwi."           ;182 Graph. Velocity	(Show a graphical representation of the ascend speed)
    TCODE    .40,   .35,     "Zeige pSCR ppO2"           ;183 Show pSCR ppO2	(Show the ppO2 for pSCR divers)
    TCODE    .40,   .35,     "pSCR O2 Drop[%]"           ;184 pSCR O2 Drop[%]	(pSCR O2 drop in percent)
    TCODE    .40,   .35,     "pSCR Gegenlunge"           ;185 pSCR lung ratio	(pSCR counterlung ratio)
; End of function descriptor II
;
    TCODE    .13,   .2,      "Custom Funktionen II"      ;186 Custom Functions II
    TCODE    .20,   .95,     "Lizenz anzeigen "          ;187 Show License
    TCODE    .0,    .2,      "Sim. Daten:"               ;188 Sim. Results:
    TCODE    .90,   .25,     "Oberfl."                   ;189 Surface
    TCODE    .0,    .0,      "ppO2 +"                    ;190 ppO2 +
    TCODE    .0,    .0,      ""                          ;191 unused
    TCODE    .0,    .0,      "Dil."                      ;192 Dil.			       (Rebreather diluent)

; 32 custom function descriptors III (FIXED LENGTH = 15 chars).
	TCODE    .40,   .35,     "Farbe# inaktiv "           ;193 Color# inactive
    TCODE    .40,   .35,     "Sicherheitsstop"           ;194 Use safety stop
    TCODE    .40,   .35,     "Zeige GF in NZ "           ;195 Show GF in NDL	(If GF > CF08)
    TCODE    .40,   .35,     "Alt. GF Low [%]"           ;196 Alt. GF Low [%]
    TCODE    .40,   .35,     "Alt. GF High[%]"           ;197 Alt. GF High[%]
    TCODE    .40,   .35,     "GF Wechsel mögl"           ;198 Allow GF change
    TCODE    .40,   .35,     "S.Stop Länge[s]"           ;199 S.StopLength[s] (CF70: Safety Stop Duration [s])
    TCODE    .40,   .35,     "S.Stop Start[m]"           ;200 S.StopStart [m] (CF71: Safety Stop Start Depth [m])
    TCODE    .40,   .35,     "S.Stop Ende [m]"           ;201 S.StopEnd   [m] (CF72: Safety Stop End Depth [m])
    TCODE    .40,   .35,     "S.Stop Reset[m]"           ;202 S.StopReset [m] (CF73: Safety Stop Reset Depth [m])
    TCODE    .40,   .35,     "Batt.Zeit [min]"           ;203 Batt. Time [min] (CF74: Battery time-out [min])
    TCODE    .40,   .35,     "unbenutzt      "           ;204 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;205 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;206 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;207 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;208 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;209 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;210 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;211 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;212 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;213 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;214 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;215 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;216 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;217 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;218 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;219 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;220 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;221 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;222 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;223 unused
    TCODE    .40,   .35,     "unbenutzt      "           ;224 unused
;
    TCODE    .7,    .2,      "Custom Funktionen III"     ;225 Custom Functions III
    TCODE    .85,   .125,    "pSCR-GF  "                 ;226 pSCR-GF
	TCODE    .90,   .54,     "Sicherheit"                ;227 SafetyStop
    TCODE    .0,    .0,      "Anzahl TG: "			     ;228 Total Dives: 
    TCODE    .20,   .35,     "Diluent Setup"             ;229 Diluent Setup
    TCODE    .20,   .65,     "Setpoint Setup"            ;230 Setpoint Setup
    TCODE    .5,    .2,      "Dil. Setup - Gasliste" 	 ;231 Dil. Setup - Gaslist
    TCODE    .100,  .100,	 "Diluent"                   ;232 Diluent
    TCODE    .93,   .170,    "  Ceiling"                 ;233 Ceiling (9Chars, right alligned)
    TCODE    .20,   .95,     "SP Modus: "                ;234 SP Mode: (10 chars)

    TCODE    .10,   .2,      "Dekomodell verändert!"     ;235 Decomode changed!
    TCODE    .85,   .125,    "L16-GF CC"                 ;236 L16-GF CC
    TCODE    .2,    .12,     "Nicht gefunden"            ;237 Not found
    TCODE    .100,  .0,      "SetPoint"                  ;238 SetPoint
    TCODE    .100,  .0,      "k. Deko"                   ;239 No Deco
    TCODE    .90,   .50,     "Intervall:"                ;240 Interval:
    TCODE    .100,  .75,     "Anzeige"                   ;241 Display
    TCODE    .100,  .0,      "Keine Deko"                ;242 No deco
    TCODE    .132,  .0,      "beta"                      ;243 beta
    TCODE    .100,  .100,    "frei"                      ;244 unuse
    TCODE    .20,   .65,     "CF,Gas&Deko zurücks."      ;245 Reset CF,Gas & Deco
    TCODE    .50,   .145,    "Batt!"                     ;246 LowBatt!
    TCODE    .20,   .125,    "Simulator"                 ;247 Simulator
    TCODE    .30,   .2,      "OSTC Simulator"            ;248 OSTC Simulator
    TCODE    .20,   .65,     "TG beginnen"               ;249 Start Dive
    TCODE    .100,  .25,     "+ 1m"                      ;250 + 1m
    TCODE    .100,  .50,     "- 1m"                      ;251 - 1m
    TCODE    .100,  .75,     "+10m"                      ;252 +10m
    TCODE    .100,  .100,    "-10m"                      ;253 -10m
    TCODE    .100,  .0,      "Ende"                      ;254 Close
    TCODE    .128,  .170,    "Zeit"                      ;255 Time
;
; Text Bank2 (Texts 256-511)
;
    TCODE    .0,    .0,      "x"                         ;256 x
    TCODE    .20,   .35,     "Datumsform.:"              ;257 Date format:
    TCODE    .10,   .2,      "Einstellungs-Menü 2:"      ;258 Setup Menu 2:
    TCODE    .105,  .35,     "MMDDYY"                    ;259 MMDDYY
    TCODE    .105,  .35,     "DDMMYY"                    ;260 DDMMYY
    TCODE    .105,  .35,     "YYMMDD"                    ;261 YYMMDD
    TCODE    .1,    .1,      "OSTC "                     ;262 OSTC 
    TCODE    .65,   .168,    "Bail "                     ;263 Bail 
    TCODE    .7,    .48,     "Luft  "                    ;264 Air
    TCODE    .115,  .135,    "Luft  "                    ;265 Air

    TCODE    .0,    .0,      "pSCR Info"             	 ;266 pSCR Info (Must be 9Chars!)
	TCODE    .0,    .184,    "Max."                      ;267 Max.
    TCODE    .93,   .170,    "GF Werte"                  ;268 GF Values
    TCODE    .100,  .50,     "GF Wech."               	 ;269 ToggleGF (In Divemode Menu)
    TCODE    .93,   .170,    "Dekogas"		             ;270 Decogas

; New CFs Warning
    TCODE    .10,   .2,      "Neue CF hinzugefügt!"      ;271 New CF added!
    TCODE    .0,    .35,     "Neue Custom Funktionen"    ;272 New CustomFunctions
    TCODE    .0,    .65,     "wurden hinzugefügt!"       ;273 were added! Check
    TCODE    .0,    .95,     "Prüfe CF I - CF III"       ;274 CF I - CF III Menu
    TCODE    .0,    .125,    "Menü für Details!"         ;275 for Details!
    TCODE    .20,   .125,    "Salzgeh.: "                ;276 Salinity:
;
    TCODE    .20,   .95,     "Grundzeit:"                ;277 Bottom Time:
    TCODE    .20,   .125,    "Max. Tiefe:"               ;278 Max. Depth:
    TCODE    .20,   .155,    "Dekoplan berechn."         ;279 Calculate Deco
    TCODE    .20,   .155,    "Helligkeit:"          	 ;280 Brightness:
;
    TCODE    .107,   .170,   "D-Tiefe"                   ;281 Avr.Depth
    TCODE    .90,   .170,    "Leitgewebe"                ;282 Lead Tiss.
    TCODE    .100,   .170,   "Stoppuhr"                  ;283 Stopwatch
    TCODE    .20,   .95,     "Logbuch zurücks."          ;284 Reset Logbook
    TCODE    .20,   .125,    "OSTC neu starten"          ;285 Reboot OSTC
    TCODE    .20,   .155,    "Dekodaten zurücks."        ;286 Reset Decodata
; Altimeter extension
    TCODE    .20,   .155,    "Höhenmesser"               ;287 Altimeter
    TCODE    .18,   .1,      "Höhenmesser einst."        ;288 Set Altimeter
    TCODE    .20,   .35,     "Höhe NN: "                 ;289 Sea ref: 
    TCODE    .0,    .0,      "Aktiv:   "                 ;290 Enabled:
    TCODE    .20,   .95,     "Normal:  1013 mbar"        ;291 Default: 1013 mbar
    TCODE    .20,   .125,    "+1 mbar"                   ;292 +1 mbar
    TCODE    .20,   .155,    "-1 mbar"                   ;293 -1 mbar
    TCODE    .85,   .185,    "Höhe "                     ;294 Alt: 
;
	TCODE    .20,   .95,     "Custom FunktionenIII"      ;295 Custom FunctionsIII
    TCODE    .50,    .2,     "Rohdaten:"                 ;296 Raw Data:
; Gas-setup addons:
    TCODE    .0,    .0,      "MOD:"                      ;297 MOD:                  (max operating depth of a gas).
    TCODE    .0,    .0,      "END:"                      ;298 END:                  (equivalent nitrogen depth of a gas).
    TCODE    .0,    .0,      "EAD:"                      ;299 EAD:                  (equivalent air depth of a gas).
	TCODE    .100,  .125,	 "Mehr"						 ;300 More               	(Enable/Disable Gas underwater)
    TCODE    .0,    .2,      "OCR Gasverbrauch:"         ;301 OCR Gas Usage:        (Planned gas consumtion by tank).
; 115k Bootloader support:
	TCODE	 .45,	.100,	 "Bootloader"				 ;302 Bootloader
	TCODE	 .35,	.130,	 "Bitte Warten!"			 ;303 Please wait!
	TCODE	 .40,	.130,	 "Abgebrochen!"				 ;304 Aborted
;@5 variant
    TCODE    .0,    .0,      "TTS @+Min."                ;305 Future TTS            (=10 chars. Title for @5 customview).
;
    TCODE    .100,  .125,    "Ende Sim"                  ;306 Quit Sim              (=8char max. Quit Simulator mode)
;Dive interval
    TCODE    .20,   .35,     "Interval:"                 ;307 Interval:
    TCODE    .0,    .0,      "Jetzt  "                   ;308 Now                   (7 chars min)
	TCODE	 .109,	.113,	 "D-Tiefe"	 	 		 	 ;309 Average
	TCODE	 .109,	.54,	 "Stoppuhr"		 		 	 ;310 Stopwatch             (BIG Stopwatch in Gauge mode)
; Cave consomation
    TCODE    .0,    .0,      "Cave Bail."                ;311 Cave Bail.            (=10 chars.)
; DISPLAY Brightness settings
    TCODE    .103,  .155,    "Eco "	    	             ;312 Eco 					(Same length as #313!)
    TCODE    .103,  .155,    "Hoch" 	                 ;313 High					(Same length as #312!)

; ZH-L16 mode description
    TCODE    .0,    .35,     "Dekomodell: ZH-L16 OC"     ;314 Decotype: ZH-L16 OC
    TCODE    .0,    .65,     "Für das Tauchen mit"       ;315 For Open Circuit
    TCODE    .0,    .95,     "offenen Systemen. "        ;316 Divers. Supports 5
    TCODE    .0,    .125,    "Bis zu 5 Trimix Gase."     ;317 Trimix Gases.
    TCODE    .0,    .155,    "Hierzu das Menü Gas-"      ;318 Configure your gas
    TCODE    .0,    .185,    "einstellung verwenden."    ;319 in Gassetup menu.
    TCODE    .0,    .215,    "CF11 & CF12 prüfen!"       ;320 Check CF11 & CF12 !
; Gaugemode description
    TCODE    .0,    .35,     "Dekomodell:Tiefenmesser"   ;321 Decotype: Gauge    
    TCODE    .0,    .65,     "Die Tauchzeit wird in"     ;322 Divetime will be in
    TCODE    .0,    .95,     "Minuten:Sekunden ange-"    ;323 Minutes:Seconds.   
    TCODE    .0,    .125,    "zeigt. Der OSTC2 be-"      ;324 OSTC2 will not     
    TCODE    .0,    .155,    "rechnet keine Daten für"   ;325 compute Deco, NoFly
    TCODE    .0,    .185,    "Deko, Flugverbots- oder"   ;326 time and Desat.
    TCODE    .0,    .215,    "Entsättigungszeiten!"      ;327 time at all!
; Const.ppO2 description
    TCODE    .0,    .35,     "Dekomodell: ZH-L16 CC"     ;328 Decotype: ZH-L16 CC
    TCODE    .0,    .65,     "Für geschlossene"  		 ;329 For Closed
    TCODE    .0,    .95,     "Kreislaufsysteme."         ;330 Circuit rebreathers
    TCODE    .0,    .125,    "Stelle die 3 Setpoints"    ;331 Configure the 3
    TCODE    .0,    .155,    "im CCR Setpoint Menü"      ;332 SetPoints in CCR -
    TCODE    .0,    .185,    "ein. Bis zu 5 Bailout"     ;333 Setup menu. 5 bail-
    TCODE    .0,    .215,    "Gase sind verfügbar."      ;334 outs are available.
; Apnoemode description
    TCODE    .0,    .35,     "Dekomodell: Apnoe    "     ;335 Decotype: Apnoe
    TCODE    .0,    .65,     "Der OSTC2 zeigt jeden"     ;336 OSTC2 will display
    TCODE    .0,    .95,     "Abstieg getrennt in"       ;337 each descent separ-
    TCODE    .0,    .125,    "Min:Sek an. Die Abtast-"   ;338 ately in Min:Sec.
    TCODE    .0,    .155,    "rate wird temporär auf"    ;339 Will temporally set
    TCODE    .0,    .185,    "1 Sekunde eingestellt."    ;340 samplerate to 1 sec
    TCODE    .0,    .215,    "Keine Deko Berechnung"     ;341 No Deco calculation
; Multi GF OC mode description
    TCODE    .0,    .35,     "Dekomodell: L16-GF OC"     ;342 Decotype: L16-GF OC
    TCODE    .0,    .65,     "Berechnung der Deko"       ;343 Decompression cal-
    TCODE    .0,    .95,     "mittels der GF-Methode"    ;344 culations with the
    TCODE    .0,    .125,    "(GF_lo/GF_hi). "           ;345 GF-Method (GF_lo/GF
    TCODE    .0,    .155,    "Prüfe die CF32 & CF33"     ;346 _hi). Check CF32 &
    TCODE    .0,    .185,    "Werte! Für offene Sys-"    ;347 CF33! Open Circuit
    TCODE    .0,    .215,    "teme mit Tiefenstopps."    ;348 with Deep Stops.
; Multi GF CC mode description
    TCODE    .0,    .35,     "Dekomodell: L16-GF CC"     ;349 Decotype: L16-GF CC
    TCODE    .0,    .65,     "Berechnung der Deko"       ;350 Decompression cal-
    TCODE    .0,    .95,     "mittels der GF-Methode"    ;351 culations with the
    TCODE    .0,    .125,    "(GF_lo/GF_hi). Prüfe"      ;352 GF-Method (GF_lo/GF
    TCODE    .0,    .155,    "die CF32 & CF33 Werte!"    ;353 _hi). Check CF32 &
    TCODE    .0,    .185,    "Für geschlossene Sys-"     ;354 CF33!Closed Circuit
    TCODE    .0,    .215,    "teme mit Tiefenstopps."    ;355 with Deep Stops.
; pSCR-GF mode description
    TCODE    .0,    .35,     "Dekomodell: pSCR-GF"     	;356 Decotype: pSCR-GF
    TCODE    .0,    .65,     "Für passive halb-"     	;357 For passive semi-
    TCODE    .0,    .95,     "geschlossene Kreisel."   	;358 closed rebreather.
    TCODE    .0,    .125,    "Prüfe CF32 & CF33"     	;359 Check CF32 & CF33
    TCODE    .0,    .155,    "für Gradienten Faktoren" 	;360 for gradient factors
    TCODE    .0,    .185,    "und CF61-CF63 für"     	;361 and CF61-CF63 for
    TCODE    .0,    .215,    "pSCR Parameter."    		;362 pSCR features.
;
; Setpoint Mode
    TCODE    .90,  .95,     "Manuell"	    	        ;363 Manual      Same length as #364
    TCODE    .90,  .95,     "Auto   " 	                ;364 Auto        Same length as #363


;=============================================================================
