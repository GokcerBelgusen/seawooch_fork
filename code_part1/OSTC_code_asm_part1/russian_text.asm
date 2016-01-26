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
; 2011/07/25 : Sergei V. Rozinov: Russian translation.
; 2011/07/28 : Sergei V. Rozinov: Corrected.
; 2011/08/04 : Sergei V. Rozinov: Adapted for firmware 1.95+
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
; * Ascii chars: we can support a few specific chars. цдьЯ for German.
;   йикз for French. бнуъсЎї for Spanish.
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
                                                                 
#DEFINE	TXT_GAS_C		     'Г'                         ; 'G'         
#DEFINE	TXT_GAS1			 "Г"                         ; "G"
#DEFINE	TXT_METER_C		     'м'                         ; 'm'         
#DEFINE	TXT_METER5		     "м    "                     ; "m    "     
#DEFINE	TXT_METER3		     "м  "                       ; "m  "       
#DEFINE	TXT_METER2		     "м "                        ; "m "        
#DEFINE	TXT_METER1		     "м"                         ; "m"         
#IFDEF IMPERIAL
 #DEFINE	TXT_FOOT5		     "фт   "                     ; "ft   "
 #DEFINE	TXT_FOOT4		     "фт  "                      ; "ft  "
 #DEFINE	TXT_FOOT3		     "фт "                       ; "ft "
 #DEFINE	TXT_FOOT2		     "фт"                        ; "ft"
#ENDIF
#DEFINE	TXT_MBAR7		     " мбар  "                   ; " mbar  "   
#DEFINE	TXT_MBAR5		     "мбар "                     ; "mbar "     
#DEFINE	TXT_BAR4		     "бар "                      ; "bar "      
#DEFINE	TXT_BAR3			 "бар"                       ; "bar"
#DEFINE	TXT_ALT5		     "Альт "                     ; "Alt: "     
#DEFINE	TXT_KGL4		     "кг/л"                      ; "kg/l"      
#DEFINE	TXT_VOLT2			 "V "                        ; "V "
#DEFINE	TXT_VOLT1		     "V"                         ; "V"         
#DEFINE	TXT_STEP5		     "Шаг: "                     ; "Step:"     
#DEFINE	TXT_CF2			     "ДП"                        ; "CF"        
#DEFINE	TXT_O2_4		     "O2: "                      ; "O2: "      
#DEFINE	TXT_O2_3		     "O2 "                       ; "O2 "       
#DEFINE	TXT_AIR4		     "AIR "                      ; "AIR "      
#DEFINE	TXT_ERR4		     "ERR "                      ; "ERR "      
#DEFINE	TXT_HE4			     "He: "                      ; "He: "      
#DEFINE	TXT_NX3			     "NX "                       ; "NX "       
#DEFINE	TXT_TX3			     "TX "                       ; "TX "       
#DEFINE	TXT_AT4			     " на "                      ; " at "
#DEFINE	TXT_G1_3		     "Г1:"                       ; "G1:"       
#DEFINE	TXT_G2_3		     "Г2:"                       ; "G2:"       
#DEFINE	TXT_G3_3		     "Г3:"                       ; "G3:"       
#DEFINE	TXT_G4_3		     "Г4:"                       ; "G4:"       
#DEFINE	TXT_G5_3		     "Г5:"                       ; "G5:"       
#DEFINE	TXT_G6_3		     "Г6:"                       ; "G6:"       
#DEFINE	TXT_1ST4		     "Нач:"                      ; "1st:"      
#DEFINE	TXT_CNS4		     "ЦНС:"                      ; "CNS:"      
#DEFINE	TXT_CNSGR10		     "ЦНС > 250%"                ; "CNS > 250%"
#DEFINE	TXT_AVR4		     "Срд:"                      ; "Avr:"      
#DEFINE	TXT_GF3			     "ГФ:"                       ; "GF:"       
#DEFINE	TXT_SAT4		     "Сат:"                      ; "Sat:"      
#DEFINE	TXT_0MIN5			 "0мин "                     ; "0min "
#DEFINE	TXT_MIN4			 "мин "                      ; "min "  
#DEFINE	TXT_BSAT5			 "КСат:"                     ; "BSat:" 
#DEFINE	TXT_BDES5			 "КДес:"                     ; "BDes:" 
#DEFINE	TXT_LAST5			 "Стоп:"                     ; "Last:" 
#DEFINE	TXT_GFLO6			 "ГФниж:"                    ; "GF_lo:"
#DEFINE	TXT_GFHI6			 "ГФврх:"                    ; "GF_hi:"
#DEFINE	TXT_PPO2_5			 "ppO2:"                     ; "ppO2:" 
#DEFINE	TXT_SP2				 "СП"                        ; "SP"    
#DEFINE	TXT_DIL4			 "Дил:"                      ; "Dil:"  
#DEFINE	TXT_N2_2			 "N2"                        ; "N2"    
#DEFINE	TXT_HE2				 "He"                        ; "He"    
#DEFINE	TXT_TX1				 "T"						 ; "T"
#DEFINE	TXT_TX2				 "x"						 ; "x"
#DEFINE	TXT_NX1				 "N"						 ; "N"
#DEFINE	TXT_NX2				 "x"						 ; "x"
#DEFINE TXT_DIL_C			 "Д"                         ; "D"
#DEFINE	TXT_DIL5			 "Дил.#"					 ; "Dil.#"
#DEFINE TXT_aGF4			 "дГФ:"						 ; "aGF:"
#DEFINE TXT_BATT             "B:"                        ; "B:"

#ENDIF
;=============================================================================
;   macro     X     Y        "translation"               ; English original
    TCODE    .0,   .0,       "Расчет MD2 хэша"           ;001 Building MD2 Hash
    TCODE    .0,   .25,      "Пожалуйста ждите..."       ;002 Please Wait...
    TCODE    .0,   .2,       "HeinrichsWeikamp OSTC2"    ;003 HeinrichsWeikamp OSTC2
    TCODE    .65,  .2,       "Меню?"                     ;004 Menu?
    TCODE    .65,  .2,       "Меню:"                     ;005 Menu:
    TCODE    .20,  .35,      "Журнал"                    ;006 Logbook
    TCODE    .20,  .65,      "Настройка газов"           ;007 Gas Setup
    TCODE    .20,  .35,      "Установка часов"           ;008 Set Time
    TCODE    .20,  .95,      "Меню сброса"               ;009 Reset Menu
    TCODE    .20,  .125,     "Настройка"                 ;010 Setup
    TCODE    .20,  .185,     "Выход"                     ;011 Exit
    TCODE    .104, .2,       "Ждите..."                  ;012 Wait...
    TCODE    .0,   .24,      "MD2 хэш:"                  ;013 MD2 Hash:
    TCODE    .0,   .0,       "Десат"                     ;014 Desat         (Desaturation count-down)
    TCODE    .50,  .2,       "Интерфейс"                 ;015 Interface		(Connected to USB)
    TCODE    .10,  .30,      "Старт"                     ;016 Start
    TCODE    .10,  .55,      "Данные"                    ;017 Data
    TCODE    .10,  .80,      "Заголовок"                 ;018 Header
    TCODE    .10,  .105,     "Профиль"                   ;019 Profile
    TCODE    .10,  .130,     "Готово."                   ;020 Done.
    TCODE    .20,  .35,      "Отменить сброс"            ;021 Cancel Reset
    TCODE    .32,  .65,      "Время:"                    ;022 Time:
    TCODE    .32,  .95,      "Дата :"                    ;023 Date:
    TCODE    .0,   .215,     "Установить Час"            ;024 Set Hours
    TCODE    .6,   .0,       "Сброс..."                  ;025 Reset...
    TCODE    .55,  .2,       "Журнал"                    ;026 Logbook
    TCODE    .20,  .2,       "Доп. Параметры I"          ;027 Custom Functions I
    TCODE    .40,  .2,       "Меню сброса"               ;028 Reset Menu
    TCODE    .15,  .2,       "Установка времени:"        ;029 Set Time:
    TCODE    .100, .50,      "Маркер"                    ;030 SetMarker         (Add a mark in logbook profile)
    TCODE    .100, .25,      "Декоплан"                  ;031 Decoplan
    TCODE    .100, .0,       "Мои газы"                  ;032 Gaslist
    TCODE    .100, .50,      "Сбр.Сред"                  ;033 ResetAvr          (Reset average depth)
    TCODE    .100, .100,     "Выход"                     ;034 Exit		        (Exit current menu)
    TCODE    .0,   .0,       "Нелєт"                     ;035 NoFly		        (No-flight count-down)
;
; 32 custom function descriptors I (FIXED LENGTH = 15 chars).
    TCODE    .40,  .35,      "Начало погр.[м]"           ;036 Start Dive  [m]	(depth to switch to dive mode)
    TCODE    .40,  .35,      "Конец погр. [м]"           ;037 End Dive    [m]	(depth to switch back to surface mode)
    TCODE    .40,  .35,      "Жду после [сек]"           ;038 End Delay [sec]  	(duration dive screen stays after end of dive)
    TCODE    .40,  .35,      "Отключение[мин]"           ;039 Power Off [min]
    TCODE    .40,  .35,      "Пред-меню [мин]"           ;040 Pre-menu  [min]	(Delais to keep surface-mode menus displayed)
    TCODE    .40,  .35,      "Скорость[м/мин]"           ;041 velocity[m/min]
    TCODE    .40,  .35,      "Автовключ[мбар]"           ;042 Wake-up  [mbar]
    TCODE    .40,  .35,      "max. Верх[мбар]"           ;043 max.Surf.[mbar]
    TCODE    .40,  .35,      "Показать ГФ [%]"           ;044 GF display  [%]
    TCODE    .40,  .35,      "мин.O2 показ[%]"           ;045 min. O2 Dis.[%]
    TCODE    .40,  .35,      "Меню погр.[мин]"           ;046 Dive menus[min]
    TCODE    .40,  .35,      "Насыщение x [%]"           ;047 Saturate x  [%]
    TCODE    .40,  .35,      "Рассыщение x[%]"           ;048 Desaturate x[%]
    TCODE    .40,  .35,      "Нелєт фактор[%]"           ;049 NoFly Ratio [%]	(Grandient factor tolerance for no-flight countdown).
    TCODE    .40,  .35,      "ГФ тревога 1[%]"           ;050 GF alarm 1  [%]
    TCODE    .40,  .35,      "ЦНС наверху [%]"           ;051 CNSshow surf[%]
    TCODE    .40,  .35,      "Деко ниже   [м]"           ;052 Deco Offset [m]
    TCODE    .40,  .35,      "ppO2 низк [бар]"           ;053 ppO2 low  [bar]
    TCODE    .40,  .35,      "ppO2 высок[бар]"           ;054 ppO2 high [bar]
    TCODE    .40,  .35,      "ppO2 показ[бар]"           ;055 ppO2 show [bar]
    TCODE    .40,  .35,      "Интервал данных"           ;056 sampling rate
    TCODE    .40,  .35,      "Делитель темпер"           ;057 Divisor Temp
    TCODE    .40,  .35,      "Делитель деко  "           ;058 Divisor Decodat
    TCODE    .40,  .35,      "Делитель ГФ    "           ;059 Divisor GF
    TCODE    .40,  .35,      "Делитель ppO2  "           ;060 Divisor ppO2
    TCODE    .40,  .35,      "Делитель дебаг "           ;061 Divisor Debug
    TCODE    .40,  .35,      "Делитель ЦНС   "           ;062 Divisor CNS
    TCODE    .40,  .35,      "ЦНС показать[%]"           ;063 CNSshow dive[%]
    TCODE    .40,  .35,      "Номер в журнале"           ;064 Logbook offset
    TCODE    .40,  .35,      "Крайняя деко[м]"           ;065 Last Deco at[m]
    TCODE    .40,  .35,      "Конец Апноэ [ч]"           ;066 End Apnoe   [h]
    TCODE    .40,  .35,      "Показ напр.бат."           ;067 Show Battery %
; End of function descriptor I
;
;licence:
    TCODE    .0,   .35,      "Эта программа свободно"    ;068 This program is
    TCODE    .0,   .65,      "распространяется в"        ;069 distributed in the
    TCODE    .0,   .95,      "надежде, что она будет"    ;070 hope that it will be
    TCODE    .0,   .125,     "полезной, но БЕЗО"         ;071 useful, but WITHOUT
    TCODE    .0,   .155,     "ВСЯКИХ ГАРАНТИЙ; также"    ;072 ANY WARRANTY
    TCODE    .0,   .185,     "без подразумеваемых"       ;073 even the implied
    TCODE    .0,   .215,     "гарантий КОММЕРЧЕСКОЙ"     ;074 warranty of
    TCODE    .0,   .35,      "ЦЕННОСТИ или"              ;075 MERCHANTABILITY or
    TCODE    .0,   .65,      "ПРИГОДНОСТИ ДЛЯ"           ;076 FITNESS FOR A
    TCODE    .0,   .95,      "КОНКРЕТНОЙ ЦЕЛИ."          ;077 PARTICULAR PURPOSE.
    TCODE    .0,   .125,     "Смотри GNU General"        ;078 See the GNU General
    TCODE    .0,   .155,     "Public License для"        ;079 Public License for
    TCODE    .0,   .185,     "полной информации:"        ;080 more details:
    TCODE    .0,   .215,     "heinrichsweikamp.com"      ;081 heinrichsweikamp.com
; end of licence
;
    TCODE    .102,  .54,     "Декостоп"                  ;082 Decostop
    TCODE    .0,    .0,      "м/мин"                     ;083 m/min
    TCODE    .102,  .113,    "Без деко"                  ;084 No Stop
    TCODE    .135,  .113,    "TTS"                       ;085 TTS
    TCODE    .121,  .0,      "В®емя"                     ;086 Divetime
    TCODE    .0,    .0,      "Глубина"                   ;087 Depth
    TCODE    .0,    .0,      "Первый Газ?"               ;088 First Gas?
    TCODE    .0,    .0,      "Умолчание:"                ;089 Default:
    TCODE    .0,    .0,      "Минуты"                    ;090 Minutes
    TCODE    .0,    .0,      "Месяц "                    ;091 Month
    TCODE    .0,    .0,      "День  "                    ;092 Day
    TCODE    .0,    .0,      "Год   "                    ;093 Year
    TCODE    .0,    .0,      "Установить "               ;094 Set
#IFNDEF IMPERIAL
    TCODE    .0,    .0,      "Газ# "                     ;095 Gas#
#ELSE
    TCODE    .0,    .0,      "Газ#"                      ;095 Gas#
#ENDIF
    TCODE    .0,    .0,      "Да"                        ;096 Yes
    TCODE    .0,    .0,      "Действует:"                ;097 Current:
    TCODE    .23,   .2,      "Меню настройки:"           ;098 Setup Menu:
    TCODE    .20,   .35,     "Доп. Параметры I"          ;099 Custom FunctionsI
    TCODE    .20,   .125,    "Алгоритм:"                 ;100 Decotype:
    TCODE    .85,   .125,    "ZH-L16 OC"                 ;101 ZH-L16 OC
    TCODE    .85,   .125,    "Таймер   "                 ;102 Gauge
    TCODE    .85,   .125,    "Таймер"                    ;103 Gauge
    TCODE    .85,   .125,    "ZH-L16 CC"                 ;104 ZH-L16 CC
    TCODE    .0,    .0,      "Активный Газ? "            ;105 Active Gas?
    TCODE    .10,   .2,      "Настройка газов"	         ;106 Gas Setup - Gaslist
    TCODE    .20,   .95,     "Глуб. +/-:"                ;107 Depth +/-:
    TCODE    .20,   .125,    "Изменить:" 	             ;108 Change:
	TCODE	 .20,	.155,	 "Умолчание:"			  	 ;109 Default:
    TCODE    .20,   .65,     "Сетпоинты CCR"             ;110 CCR SetPoint Menu
    TCODE    .20,   .2,      "Меню сетпоинтов CCR"       ;111 CCR SetPoint Menu
    TCODE    .0,    .0,      "СП#"                       ;112 SP#
    TCODE    .20,   .95,     "Состояние батареи"         ;113 Battery Info
    TCODE    .17,   .2,      "Информация батареи"        ;114 Battery Information
    TCODE    .0,    .9,      "Циклов:"                   ;115 Cycles:
    TCODE    .85,   .125,    "Апноэ"                     ;116 Apnoe
    TCODE    .0,    .18,     "Посл. зарядка:"            ;117 Last Complete:
    TCODE    .0,    .27,     "Минимум Vбат:"             ;118 Lowest Vbatt:
    TCODE    .0,    .36,     "Минимум дата:"             ;119 Lowest at:
    TCODE    .0,    .45,     "Tmin:"                     ;120 Tmin:
    TCODE    .0,    .54,     "Tmax:"                     ;121 Tmax:
    TCODE    .100,  .124,    "Далее"		          	 ;122 More (Gaslist)
    TCODE    .100,  .25,     "O2 +"                      ;123 O2 +
    TCODE    .100,  .50,     "O2 -"                      ;124 O2 -
    TCODE    .100,  .75,     "He +"                      ;125 He +
    TCODE    .100,  .100,    "He -"                      ;126 He -
    TCODE    .100,  .0,      "Выход"                     ;127 Exit
    TCODE    .100,  .25,     "Удалить"                   ;128 Delete
    TCODE    .20,   .65,     "Дебаг:"                    ;129 Debug:
    TCODE    .65,   .65,     "Вкл "                      ;130 ON
    TCODE    .65,   .65,     "Выкл"                      ;131 OFF
    TCODE    .100,  .50,     "Удал. все"                 ;132 Del. all
    TCODE    .10,   .0,      "Неожиданный сброс из "     ;133 Unexpected reset from
    TCODE    .10,   .25,     "режима погружения!   "     ;134 Divemode! Please help
    TCODE    .10,   .50,     "Сообщите об ошибке,  "     ;135 and report the Debug
    TCODE    .10,   .75,     "отправьте отчет ниже!"     ;136 Information below!
    TCODE    .100,  .0,      "На запас"                  ;137 Bailout
    TCODE    .85,   .125,    "Апноэ     "                ;138 Apnoe
    TCODE    .112,  .120,    "В воде"                    ;139 Descent
    TCODE    .105,  .60,     "Наверху"                   ;140 Surface
    TCODE    .65,   .2,      "Откл?"                     ;141 Quit?
    TCODE    .20,   .155,    "Далее"                     ;142 More
    TCODE    .42,   .72,     "Уверены?"                  ;143 Confirm:
    TCODE    .60,   .2,      "Меню 2:"                   ;144 Menu 2:
    TCODE    .52,   .96,     "Отмена"                    ;145 Cancel
    TCODE    .52,   .120,    "OK!"                       ;146 OK!
    TCODE    .20,   .35,     "Далее"            	     ;147 More
    TCODE    .0,    .0,      ":.........:"               ;148 :.........:
    TCODE    .0,    .8,      "ppO2"                      ;149 ppO2
    TCODE    .2,    .39,     "бар "                      ;150 bar
    TCODE    .108,  .216,    "Маркер?"                   ;151 Marker?
    TCODE    .85,   .125,    "L16-GF OC"                 ;152 L16-GF OC
    TCODE    .20,   .65,     "Доп. Параметры II"	     ;153 Custom FunctionsII
;
; 32 custom function descriptors II (FIXED LENGTH = 15 chars).
    TCODE    .40,   .35,     "ГФ ниж. гран[%]"           ;154 GF Low      [%]
    TCODE    .40,   .35,     "ГФ верх.гран[%]"           ;155 GF High     [%]
    TCODE    .40,   .35,     "Цвет#   батареи"           ;156 Color# Battery
    TCODE    .40,   .35,     "Цвет#  стардарт"           ;157 Color# Standard
    TCODE    .40,   .35,     "Цвет# под водой"           ;158 Color# Divemask
    TCODE    .40,   .35,     "Цвет# предупреж"           ;159 Color# Warnings
    TCODE    .40,   .35,     "Время погр.сек."           ;160 Divemode secs.
    TCODE    .40,   .35,     "Поправ. фикс.SP"           ;161 Adjust fixed SP
    TCODE    .40,   .35,     "Предупреж. стоп"           ;162 Warn Ceiling
#IFNDEF IMPERIAL
    TCODE    .40,   .35,     "не используется"           ;163 unused
#ELSE
    TCODE    .40,   .35,     "Метрическая сис"           ;163 Metic System
#ENDIF
    TCODE    .40,   .35,     "Напом. лучш.газ"           ;164 Blink BetterGas	(Remainder in divemode to switch to a better decompression gas).
	TCODE    .40,   .35,     "Трев.глуб[мбар]"           ;165 DepthWarn[mbar]
    TCODE    .40,   .35,     "ЦНС предупр.[%]"           ;166 CNS warning [%]
    TCODE    .40,   .35,     "ГФ предупр. [%]"           ;167 GF warning  [%]
    TCODE    .40,   .35,     "ppO2 пред.[бар]"           ;168 ppO2 warn [bar]
    TCODE    .40,   .35,     "Скор.пр.[м/мин]"           ;169 Vel.warn[m/min]
    TCODE    .40,   .35,     "Коррекция часов"           ;170 Time offset/day
    TCODE    .40,   .35,     "Показ альтиметр"           ;171 Show altimeter
    TCODE    .40,   .35,     "Показать маркер"           ;172 Show Log-Marker
    TCODE    .40,   .35,     "Показать таймер"           ;173 Show Stopwatch
    TCODE    .40,   .35,     "Показ граф. ткн"           ;174 ShowTissueGraph
    TCODE    .40,   .35,     "Показ глав. ткн"           ;175 Show Lead.Tiss.
    TCODE    .40,   .35,     "Мелк.ост.вверху"           ;176 Shallow stop 1st  (Reverse order of deco plans)
    TCODE    .40,   .35,     "Перекл.газ[мин]"           ;177 Gas switch[min]   (Show Countdown after gas change)
    TCODE    .40,   .35,     "Донн.расх[/мин]"           ;178 BottomGas[/min]   (Bottom gas usage, for volume estimation).
    TCODE    .40,   .35,     "Подъ.расх[/мин]"           ;179 AscentGas[/min]   (Ascent+Deco gas usage)
    TCODE    .40,   .35,     "Будущ. TTS[мин]"           ;180 Future TTS[min]   (@5 variant: compute TTS for extra time at current depth)
    TCODE    .40,   .35,     "Пещер. Пред.[л]"           ;181 Cave Warning[l]   (Consomation warning for cave divers)
    TCODE    .40,   .35,     "График скорости"           ;182 (Show a graphical representation of the ascend speed)
    TCODE    .40,   .35,     "Показ pSCR ppO2"           ;183 Show pSCR ppO2	(Show the ppO2 for pSCR divers)
    TCODE    .40,   .35,     "pSCR O2 Сниж[%]"           ;184 pSCR O2 Drop[%]	(pSCR O2 drop in percent)
    TCODE    .40,   .35,     "pSCR отн. мешка"           ;185 pSCR lung ratio	(pSCR counterlung ratio)
; End of function descriptor II
;
    TCODE    .20,   .2,      "Доп. Параметры II"         ;186 Custom Functions II
    TCODE    .20,   .95,     "Показать лицензию"         ;187 Show License
    TCODE    .0,    .2,      "Результаты:"               ;188 Sim. Results:
    TCODE    .90,   .25,     "Надводный"                 ;189 Surface
    TCODE    .0,    .0,      "ppO2 +"                    ;190 ppO2 +
    TCODE    .0,    .0,      ""                          ;191 unused
    TCODE    .0,    .0,      "Дил."                      ;192 Dil.			       (Rebreather diluent)

; 32 custom function descriptors III (FIXED LENGTH = 15 chars).
    TCODE    .40,   .35,     "Цвет#  неактивн"           ;193 Color# inactive
    TCODE    .40,   .35,     "Остан. безопасн"           ;194 Use safety stop
    TCODE    .40,   .35,     "Показ ГФ в NDL "           ;195 Show GF in NDL	(If GF > CF08)
    TCODE    .40,   .35,     "Доп. ГФ Ниж [%]"           ;196 Alt. GF Low [%]
    TCODE    .40,   .35,     "Доп. ГФ Верх[%]"           ;197 Alt. GF High[%]
    TCODE    .40,   .35,     "Разреш. Доп. ГФ"           ;198 Allow GF change
    TCODE    .40,   .35,     "Ост.Без.Длит[s]"           ;199 S.StopLength[s] (CF70: Safety Stop Duration [s])
    TCODE    .40,   .35,     "Ост.Без. Нач[m]"           ;200 S.StopStart [m] (CF71: Safety Stop Start Depth [m])
    TCODE    .40,   .35,     "Ост.Без. Кнц[m]"           ;201 S.StopEnd   [m] (CF72: Safety Stop End Depth [m])
    TCODE    .40,   .35,     "Ост.Без. Сбр[m]"           ;202 S.StopReset [m] (CF73: Safety Stop Reset Depth [m])
    TCODE    .40,   .35,     "Batt.Time [min]"           ;203 Batt. Time [min] (CF74: Battery time-out [min])
    TCODE    .40,   .35,     "не используется"           ;204 unused
    TCODE    .40,   .35,     "не используется"           ;205 unused
    TCODE    .40,   .35,     "не используется"           ;206 unused
    TCODE    .40,   .35,     "не используется"           ;207 unused
    TCODE    .40,   .35,     "не используется"           ;208 unused
    TCODE    .40,   .35,     "не используется"           ;209 unused
    TCODE    .40,   .35,     "не используется"           ;210 unused
    TCODE    .40,   .35,     "не используется"           ;211 unused
    TCODE    .40,   .35,     "не используется"           ;212 unused
    TCODE    .40,   .35,     "не используется"           ;213 unused
    TCODE    .40,   .35,     "не используется"           ;214 unused
    TCODE    .40,   .35,     "не используется"           ;215 unused
    TCODE    .40,   .35,     "не используется"           ;216 unused
    TCODE    .40,   .35,     "не используется"           ;217 unused
    TCODE    .40,   .35,     "не используется"           ;218 unused
    TCODE    .40,   .35,     "не используется"           ;219 unused
    TCODE    .40,   .35,     "не используется"           ;220 unused
    TCODE    .40,   .35,     "не используется"           ;221 unused
    TCODE    .40,   .35,     "не используется"           ;222 unused
    TCODE    .40,   .35,     "не используется"           ;223 unused
    TCODE    .40,   .35,     "не используется"           ;224 unused
;
	TCODE    .12,   .2,      "Доп. Параметры III"        ;225 Custom Functions III
    TCODE    .85,   .125,    "pSCR-GF  "                 ;226 pSCR-GF
	TCODE    .90,   .54,     "Ост.Безоп."                ;227 SafetyStop
    TCODE    .0,    .0,      "Всего погружений: "        ;228 Total Dives: 
    TCODE    .20,   .35,     "Настройка дилюентов"       ;229 Diluent Setup
    TCODE    .20,   .65,     "Настройка сетпоинтов"      ;230 Setpoint Setup
    TCODE    .3,    .2,      "Список газов-дилюентов"	 ;231 Dil. Setup - Gaslist
    TCODE    .100,  .100,	 "Дилюент"                   ;232 Diluent
    TCODE    .93,   .170,    "  Потолок"                 ;233 Ceiling (9Chars, right alligned)
    TCODE    .0,    .0,      ""     	                 ;234 unused

    TCODE    .10,   .2,      "Алгоритм изменен!  "       ;235 Decomode changed!
    TCODE    .85,   .125,    "L16-GF CC"                 ;236 L16-GF CC
    TCODE    .2,    .12,     "Не найден"                 ;237 Not found
    TCODE    .100,  .0,      "Сетпоинт"                  ;238 SetPoint
    TCODE    .100,  .0,      "Нет деко"                  ;239 No Deco
    TCODE    .90,   .50,     "Интервал:"                 ;240 Interval:
    TCODE    .100,  .75,     "Яркость"                   ;241 Display
    TCODE    .100,  .0,      "Нет деко"                  ;242 No deco
    TCODE    .132,  .0,      "beta"                      ;243 beta
    TCODE    .100,  .100,    "unuse"                     ;244 unuse
    TCODE    .20,   .65,     "Сброс ДП,газ и деко"       ;245 Reset CF,Gas & Deco
    TCODE    .58,   .145,    "Батарея!"                  ;246 LowBatt!
    TCODE    .20,   .125,    "Планировщик"               ;247 Simulator
    TCODE    .30,   .2,      "OSTC Планировщик"          ;248 OSTC Simulator
    TCODE    .20,   .65,     "Начать имитацию"           ;249 Start Dive
    TCODE    .100,  .25,     "+ 1м"                      ;250 + 1m
    TCODE    .100,  .50,     "- 1м"                      ;251 - 1m
    TCODE    .100,  .75,     "+10м"                      ;252 +10m
    TCODE    .100,  .100,    "-10м"                      ;253 -10m
    TCODE    .100,  .0,      "Закрыть"                   ;254 Close
    TCODE    .131,  .170,    "Часы"                      ;255 Time
;
; Text Bank2 (Texts 256-511)
;
    TCODE    .0,    .0,      "x"                         ;256 x
    TCODE    .20,   .35,     "Формат даты:"              ;257 Date format:
    TCODE    .23,   .2,      "Меню настройки 2:"         ;258 Setup Menu 2:
    TCODE    .105,  .35,     "MMDDYY"                    ;259 MMDDYY
    TCODE    .105,  .35,     "DDMMYY"                    ;260 DDMMYY
    TCODE    .105,  .35,     "YYMMDD"                    ;261 YYMMDD
    TCODE    .1,    .1,      "OSTC "                     ;262 OSTC
    TCODE    .65,   .168,    "Запас"                     ;263 Bail
    TCODE    .7,    .48,     "Возд."                     ;264 Air
    TCODE    .115,  .135,    "Возд."                     ;265 Air

    TCODE    .0,    .0,      "pSCR Инфо"             	 ;266 pSCR Info (Must be 9Chars!)
    TCODE    .0,    .184,    "Макс."                     ;267 Max.
    TCODE    .93,   .170,    "Па®ам. ГФ"                 ;268 GF Values
    TCODE    .100,  .50,     "Смена ГФ"               	 ;269 ToggleGF (In Divemode Menu)
    TCODE    .93,   .170,    "ДекоГаз"		             ;270 Decogas

; New CFs Warning
    TCODE    .24,   .2,      "Добавлены ДП!"             ;271 New CF added!
    TCODE    .0,    .35,     "Новые Доп. Параметры"      ;272 New CustomFunctions
    TCODE    .0,    .65,     "добавлены! Проверьте"      ;273 were added! Check
    TCODE    .0,    .95,     "Меню ДП I, II и III"       ;274 CF I and CF II Menu
    TCODE    .0,    .125,    "для информации!"           ;275 for Details!
    TCODE    .20,   .125,    "Соленость: "               ;276 Salinity:
;
    TCODE    .20,   .95,     "Время на дне :"            ;277 Bottom Time:
    TCODE    .20,   .125,    "Макс. глубина:"            ;278 Max. Depth:
    TCODE    .20,   .155,    "Вычислить деко"            ;279 Calculate Deco
    TCODE    .20,   .155,    "Яркость:"			       	 ;280 Brightness:
;
    TCODE    .107,  .170,    "С®едняя"                   ;281 Avr.Depth
    TCODE    .90,   .170,    "Глав ткань"                ;282 Lead Tiss.
    TCODE    .114,  .170,    "Тайме®"                    ;283 Stopwatch
    TCODE    .20,   .95,     "Сброс журнала"             ;284 Reset Logbook
    TCODE    .20,   .125,    "Перезагрузка OSTC"         ;285 Reboot OSTC
    TCODE    .20,   .155,    "Сброс данных деко"         ;286 Reset Decodata
; Altimeter extension
    TCODE    .20,   .155,    "Альтиметр"                 ;287 Altimeter
    TCODE    .10,   .1,      "Настройка альтиметра"      ;288 Set Altimeter
    TCODE    .20,   .35,     "Уров.моря: "               ;289 Sea ref:
    TCODE    .0,    .0,      "Включен? : "               ;290 Enabled:
    TCODE    .20,   .95,     "Умолчание: 1013 мбар"      ;291 Default: 1013 mbar
    TCODE    .20,   .125,    "+1 мбар"                   ;292 +1 mbar
    TCODE    .20,   .155,    "-1 мбар"                   ;293 -1 mbar
    TCODE    .78,   .185,    "Альт: "                    ;294 Alt:
;
    TCODE    .20,   .95,     "Доп. Параметры III"	     ;295 Custom FunctionsIII
	TCODE    .50,    .2,     "Дамп:"                     ;296 Raw Data:
; Gas-setup addons:
    TCODE    .0,    .0,      "MOD:"                      ;297 MOD:                  (max operating depth of a gas).
    TCODE    .0,    .0,      "END:"                      ;298 END:                  (equivalent nitrogen depth of a gas).
    TCODE    .0,    .0,      "EAD:"                      ;299 EAD:                  (equivalent air depth of a gas).
	TCODE    .100,  .125,	 "Далее"					 ;300 More               	(Enable/Disable Gas underwater)
	TCODE    .0,    .2,      "Расход OCR газов:"         ;301 OCR Gas Usage:        (Planned gas consumtion by tank).
; 115k Bootloader support:
	TCODE	 .45,	.100,	 "Загрузчик"				 ;302 Bootloader
	TCODE	 .19,	.130,	 "Пожалуйста ждите!"    	 ;303 Please wait!
	TCODE	 .50,	.130,	 "Прервано!"				 ;304 Aborted
; @5 variant
    TCODE    .0,    .0,      "Будущ. TTS"                ;305 Future TTS            (=10 chars. Title for @5 customview).
    TCODE    .100,  .125,    "Вых. Сим"                  ;306 Quit Sim              (=8char max. Quit Simulator mode)
; Dive interval
    TCODE    .20,   .35,     "Интервал:"                 ;307 Interval:
    TCODE    .0,    .0,      "Сейчас "                   ;308 Now                   (7 chars min)
#IFNDEF BOTTOM
	TCODE	 .108,	.112,	 "С®едняя"			 		 ;309 Average
;
	TCODE	 .115,	.54,	 "Тайме®"			 		 ;310 Stopwatch             (BIG Stopwatch in Gauge mode)
#ELSE
	TCODE	 .108,	.124,	 "С®едняя"			 		 ;309 Average
;
	TCODE	 .115,	.60,	 "Тайме®"			 		 ;310 Stopwatch             (BIG Stopwatch in Gauge mode)
#ENDIF
; Cave consomation
    TCODE    .0,    .0,      "Пещер.Зап."                ;311 Cave Bail.            (=10 chars.)
; DISPLAY Brightness settings
    TCODE    .103,  .155,    "Норм"	    	             ;312 Eco 					(Same length as #313!)
    TCODE    .103,  .155,    "Ярко" 	                 ;313 High					(Same length as #312!)

; ZH-L16 mode description
    TCODE    .0,    .35,     "Алгоритм: ZH-L16 OC"       ;314 Decotype: ZH-L16 OC
    TCODE    .0,    .65,     "Для открытой схемы "       ;315 For Open Circuit
    TCODE    .0,    .95,     "дыхания. Доступно  "       ;316 Divers. Supports 5
    TCODE    .0,    .125,    "до 5 Тримикс-смесей"       ;317 Trimix Gases.
    TCODE    .0,    .155,    "Задайте свои газы  "       ;318 Configure your gas
    TCODE    .0,    .185,    "в меню настройки.  "       ;319 in Gassetup menu.
    TCODE    .0,    .215,    "Уточн. ДП11 & ДП12!"       ;320 Check CF11 & CF12 !
; Gaugemode description
    TCODE    .0,    .35,     "Алгоритм: Таймер   "       ;321 Decotype: Gauge
    TCODE    .0,    .65,     "Время под водой в  "       ;322 Divetime will be in
    TCODE    .0,    .95,     "виде Минуты:Секунды"       ;323 Minutes:Seconds.
    TCODE    .0,    .125,    "OSTC2 не вычисляет "       ;324 OSTC2 will not
    TCODE    .0,    .155,    "декомпрессию,      "       ;325 compute Deco, NoFly
    TCODE    .0,    .185,    "нелєтное время и   "       ;326 time and Desat.
    TCODE    .0,    .215,    "время рассыщения!  "       ;327 time at all!
; Const.ppO2 description
    TCODE    .0,    .35,     "Алгоритм: ZH-L16 CC"       ;328 Decotype: ZH-L16 CC
    TCODE    .0,    .65,     "Для закрытой       " 		 ;329 For Closed
    TCODE    .0,    .95,     "схемы дыхания.     "       ;330 Circuit rebreathers
    TCODE    .0,    .125,    "Задайте 3 Сетпоинта"       ;331 Configure the 3
    TCODE    .0,    .155,    "в меню настройки   "       ;332 SetPoints in CCR -
    TCODE    .0,    .185,    "CCR. Доступно до 5 "       ;333 Setup menu. 5 bail-
    TCODE    .0,    .215,    "запасных смесей.   "       ;334 outs are available.
; Apnoemode description
    TCODE    .0,    .35,     "Алгоритм: Апноэ    "       ;335 Decotype: Apnoe
    TCODE    .0,    .65,     "OSTC2 показывает   "       ;336 OSTC2 will display
    TCODE    .0,    .95,     "каждое погружение  "       ;337 each descent separ-
    TCODE    .0,    .125,    "отдельно в Мин:Сек."       ;338 ately in Min:Sec.
    TCODE    .0,    .155,    "Временно выставляет"       ;339 Will temporally set
    TCODE    .0,    .185,    "период данных 1 сек"       ;340 samplerate to 1 sec
    TCODE    .0,    .215,    "Не вычисляет деко! "       ;341 No Deco calculation
; Multi GF OC mode description
    TCODE    .0,    .35,     "Алгоритм: L16-GF OC"       ;342 Decotype: L16-GF OC
    TCODE    .0,    .65,     "Расчет декомпрессии"       ;343 Decompression cal-
    TCODE    .0,    .95,     "с методом градиент-"       ;344 culations with the
    TCODE    .0,    .125,    "фактора (ГФниж/ГФ  "       ;345 GF-Method (GF_lo/GF
    TCODE    .0,    .155,    "врх). Уточн. ДП32 &"       ;346 _hi). Check CF32 &
    TCODE    .0,    .185,    "ДП33!Открытый цикл,"       ;347 CF33! Open Circuit
    TCODE    .0,    .215,    "глубокие остановки."       ;348 with Deep Stops.
; Multi GF CC mode description
    TCODE    .0,    .35,     "Алгоритм: L16-GF CC"       ;349 Decotype: L16-GF CC
    TCODE    .0,    .65,     "Расчет декомпрессии"       ;350 Decompression cal-
    TCODE    .0,    .95,     "с методом градиент-"       ;351 culations with the
    TCODE    .0,    .125,    "фактора (ГФниж/ГФ  "       ;352 GF-Method (GF_lo/GF
    TCODE    .0,    .155,    "врх). Уточн. ДП32 &"       ;353 _hi). Check CF32 &
    TCODE    .0,    .185,    "ДП33!Закрытый цикл,"       ;354 CF33!Closed Circuit
    TCODE    .0,    .215,    "глубокие остановки."       ;355 with Deep Stops.
; pSCR-GF mode description
    TCODE    .0,    .35,     "Алгоритм: pSCR-GF  "     	;356 Decotype: pSCR-GF
    TCODE    .0,    .65,     "Для пассивного полу-"    	;357 For passive semi-
    TCODE    .0,    .95,     "закрытого ребризера."    	;358 closed rebreather.
    TCODE    .0,    .125,    "Уточн. ДП32 & ДП33 "     	;359 Check CF32 & CF33
    TCODE    .0,    .155,    "для град.-факторов "    	;360 for gradient factors
    TCODE    .0,    .185,    "и ДП61-ДП63 для    "     	;361 and CF61-CF63 for
    TCODE    .0,    .215,    "параметров pSCR.   " 		;362 pSCR features.

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
  #IFDEF V_CEILING
    TCODE    .93,   .170,    "В.Потолок"                 ;370 Variable ceiling display
  #ELSE
    TCODE    .0,    .0,      ""                          ;370 unused
  #ENDIF
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

IFDEF IMPERIAL
; Simulator menu with imperial units (was 314 - 320: need to add 49)
    TCODE    .100,  .25,     "+  1фт"                    ;380 was 363 +1ft
    TCODE    .100,  .50,     "-  1фт"                    ;381 was 364 -1ft
    TCODE    .100,  .75,     "+ 10фт"                    ;382 was 365 +10ft
    TCODE    .100,  .100,    "- 10фт"                    ;383 was 366 -10ft
    TCODE    .0,    .0,      "фт/мин"                    ;384 was 367 ft/min
#ENDIF
#IFDEF BOTTOM
    TCODE    .50,  .2,       "Сбр.Тайм?"                 ;385 was 368 Reset Timer?
    TCODE    .50,  .2,       "Сбр.СрГл?"                 ;386 was 369 Reset Average?
#ENDIF
#IFDEF CCR_CTRL
    TCODE    .20,   .95,     "Настройка eCTRL"           ;387 CCR Controller Setup
    TCODE    .10,    .2,     "Настройка eCTRL"           ;388 CCR Controller Setup
    TCODE    .20,   .95,     "Отключ. Вн.ppO2?"          ;389
    TCODE    .20,   .95,     "Использ. Вн.ppO2?"         ;390
    TCODE    .20,   .125,    "Калибровка"                ;391
    TCODE    .20,   .155,    "Калибровка Возд.?"         ;392
    TCODE    .20,   .35,     "Нету связи..."             ;393
    TCODE    .50,    .23,    "Макс."                     ;394 Max. (at top of screen)

#ENDIF

;=============================================================================