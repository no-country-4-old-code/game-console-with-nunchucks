.include "m88def.inc"
/* Dieses Modul steuert den VGA Monitor an und managt den Spielverlauf ( Bewegungen, Ball pos , evt. Rst etc ) -> Ausnahme: Punktestand !! -> C-Modul korntrolliert den
und senden gegebendfalls einen Interrupt!
Dazu kommuniziert es mit dem Controller-Modul über die UART Schnittstelle ( Rs232 8N1 , Baudrate= 221145/(16*(43))*10^3 = 32,1 KBps )
Das Controller-Modul schickt diesem Modul ( VGA-Modul ) die Bewegung ( bzw. Änderung der Position ) und evt. Rst.
Aufbau Protokoll ( C-Modul -> VGA-Modul ) : [ 7:6 | 5:3 | 2:0  ]  == [ 11=rst | BewegungPlayer2 | BewegungPlayer1 ]

Das VGA-Modul schickt dem C-Modul die Änderung der Spielstände und evt. Rsten der Spielstände ( Anzeigen übernimmt das C-Modul über 7-Segment Anzeige ).
Aufbau Protokoll ( VGA-Modul -> C-Modul ) : [ 7:2 | 1 | 0 ] == [ reserviert ( zu belegen mit "101010" ) | ÄnderungSpielstand2 | ÄnderungSpielstand1 ]

*/
//zum Teil finden sich kleinere Anpassungen meist in Form von Magic Numbers (das bedeutet nicht zwangsläufig das alle MagicNumbers auch anpassungen sind. Meist für Maskierung )


// Allgemein 
.equ CLK=221145 //in khz [derbst übertaktet]

.equ sram_anfang=0x0100 //0x0060 ist frühste Addresse -> vorher sind ja noch Register[0x00-0x1F] und I/O Ports [0x20-0x5f] und beim Atmega88 ext- Ports [0x60-0xFF]
.equ replay_zeile=4 //jeden DatenBlock 4 mal wiederholen (=> 4*4 Fragmente)
.equ bytes_per_line=40 //40Byte pro 640Pixel bzw für 160Fragmente je 2 Bit

//Bytes der Images
.equ BYTES_complete_line = bytes_per_line
.equ BYTES_spielerbalken = 7
.equ BYTES_spielball = 4
.equ BYTES_Register=1

//SRAM	
 //Spieldaten für main
.equ databyte_controller=sram_anfang // Protokollaufbau : 8 Nutzbits // [0:2] mov_player1 ; [3:5] mov_player2 ; [6:7] cmd
.equ cmds_for_controller=sram_anfang+BYTES_REGISTER  // beinhaltet die Änderung der Spielstände ( wird an C_Modul geschickt )
.equ pos_Byte_player1=cmds_for_controller+BYTES_REGISTER 
.equ pos_Byte_player2=pos_Byte_player1+BYTES_Register
.equ counter_Bit_player1=pos_Byte_player2+BYTES_Register
.equ counter_Bit_player2=counter_Bit_player1+BYTES_Register
.equ pos_Byte_Ball_x=counter_Bit_player2+BYTES_Register  // an welches Byte das IMG gezeichnet werden soll
.equ counter_Bit_Ball_x=pos_Byte_Ball_x+BYTES_Register // da nur Byteweiseadressiert werden kann, werden IMG geshiftet -> init mit 4 , shift_links=-1,shift_rechts=+1 , {==0 || ==8} -> reset und pos_Byte_Ball_x -- oder ++
.equ pos_Ball_y_low=counter_Bit_Ball_x+BYTES_Register
.equ pos_Ball_y_high=pos_Ball_y_low+BYTES_Register
.equ v_Ball_x_count=pos_Ball_y_high+BYTES_Register
.equ v_Ball_y_count=v_Ball_x_count+BYTES_Register
.equ v_Ball_x_max=v_Ball_y_count+BYTES_Register // Das MSB gibt jeweils die Richtung an [ MSB==1 -> links ; MSB==0 -> rechts ]
.equ v_Ball_y_max=v_Ball_x_max+BYTES_Register  // MSB==1 up ; MSB==0 down
.equ last_input_ply1=v_Ball_y_max+BYTES_Register // zum erstellen von Pseudo-Zufallszahlen
.equ last_input_ply2=last_input_ply1+BYTES_Register //zum erstellen von Pseudo-Zufallszahlen
.equ clocks_since_reset=last_input_ply2+BYTES_Register //zum erstellen von Pseudo-Zufallszahlen
 //Images
.equ sram_IMG_spielerbalken1=clocks_since_reset+BYTES_Register
.equ sram_IMG_spielerbalken2=sram_IMG_spielerbalken1+BYTES_spielerbalken
.equ sram_IMG_spielball_part_1=sram_IMG_spielerbalken2+BYTES_spielerbalken
.equ sram_IMG_spielball_part_2=sram_IMG_spielball_part_1+BYTES_spielball
.equ sram_IMG_spielball_part_3=sram_IMG_spielball_part_2+BYTES_spielball
// Worklinies
.equ sram_linke_seitenbegrenzung=sram_IMG_spielball_part_3+BYTES_spielball
.equ sram_mittellinie=sram_linke_seitenbegrenzung+BYTES_complete_line
.equ sram_spielerbalken_player1=sram_mittellinie+BYTES_complete_line
.equ sram_spielerbalken_player2=sram_spielerbalken_player1+BYTES_complete_line
.equ sram_spielball_part_1=sram_spielerbalken_player2+BYTES_complete_line
.equ sram_spielball_part_2=sram_spielball_part_1+BYTES_complete_line
.equ sram_spielball_part_3=sram_spielball_part_2+BYTES_complete_line

// Anordnung ( wo fängt was an )
.equ start_spielerbalken1_vertikal= 54 // ( Spielerbalken1 wird in ZeileH=0 gezeichnet (sichtbar wirds in ZeileH=0 erst ab ZeileL=47 )
.equ spielerbalken1_high= 3
.equ spielerbalken1_length=BYTES_spielerbalken+1
.equ start_spielerbalken2_vertikal= 9  // ( Spielerbalken2 wird in ZeileH=2 gezeichnet ( sichtbar wird in ZeileH=2 im Bereich ZeileL=0 bis ZeileL=17 )
.equ spielerbalken2_high= 3
.equ spielerbalken2_length=BYTES_spielerbalken+1

.equ offset_null_bits_spielerbalken=10 // Bit_paare
.equ offset_null_bits_spielball=7 // // Bit_paare // hier nur Mittelwert
.equ linke_seitenbegrenzung=5  // in x Richtung -> nur für main relevant
.equ rechte_seitenbegrenzung=35
// bei 480 Bildpunkten liegt die Mitte bei 240 (oha) , da es bei 47 anfängt ,bei 267.. somit bei H=1 , L=12
.equ start_mittellinie= 23 // (die Mittelline wird in ZeileH=1 gezeichnet [ sichtbar komplettes Spektrum von ZeileL {0:255} ] )
.equ breite_mittellinie= 1
.equ Spielball_high= 9 // Bewegt sich über alle Zeilen ( == Bytes die aus Memory Map gelesen werden ) 

//Startpositionen
.equ startpos_spielball_vertikal_low=23
.equ startpos_spielball_vertikal_high=1
.equ startpos_spielball_horizontal=18 // Byteweise Addressierung

.equ startpos_spielerbalken1_horizontal=19
.equ startpos_spielerbalken2_horizontal=19




//Inittailiserungswerte
.equ init_counter_bit=4
.equ init_v_Ball_y_max=0x81
.equ init_v_Ball_y_count=0x01
.equ init_cmds_for_controller=0b10101000

.equ PLAYER_1=1
.equ PLAYER_2=2


//Protokoll c2vga ( bei movment Spielerbalken verwendet und rst-Prüfung )
//  ( 000 = keine Bewegung ; 101 bis 111 schnell bis weniger schnell rechts ; 001 bis 011 schnell bis weniger schnell links )
// bezogen auf die 3 Move Bits des jeweiligen Players
.equ cmd_c2vga_reset=0b11000000

//Stackpointer Positionen
.equ stack_ptr_main_low =LOW(RAMEND-4) // -> - Rücksprungadresse ( 16Bit) und tmp,tmp2, --> 0x45F-0x004 == 0x45B ( high ändert sich nicht!)
.equ stack_ptr_display_vga=stack_ptr_main_low-4 // falls das der Fall ist ,wird der stack_ptr wieder auf "stack_ptr_main_low" gesetzt !
// aus Zeitgründen setzen wir den Stackpointer [bzw SPL] im timer_interrupt einfach IMMER auf "stack_ptr_main_low" nachdem wir XH,XL,tmp,tmp2 gepusht haben!

//UART 
.equ BAUD_RATE_HIGH=0  // da beide mit USRR ==12 arbeiten  -> Baudrate == 22,1145 Mhz /(16*(12+1) == 106,319 Kbps (reicht zeitlich für hin und rück )
.equ BAUD_RATE_LOW=42
.equ UART_CMD_START=42

   // Port C
.equ SYNC_VGA_DDR=DDRD
.equ SYNC_VGA_PORT=PORTD 
.equ RXD=0
.equ TXD=1
.equ H_SYNC=2
.equ V_SYNC=3

// Port D
.equ FARBE_VGA_DDR=DDRB // alle Pins des PORTS sind output! (nehmen leider alle in Beschlag,da maskieren zu lange dauern würde)
.equ FARBE_VGA_PORT=PORTB 
.equ BLACK_WHITE=3
.equ GREEN=2


// Definitionen

.def new_data=R0
.def null=R1
.def eins=R2 //weil bei inc kein carry gesetzt wird !
.def reload=R4
.def Spielball_pos_Start_low=R5
.def Spielball_pos_Start_high=R6
.def memory_map_low_byteadresse=R7
.def memory_map_high_byteadresse=R8
.def stackpointer_main_low=R9

.def replay_count=R10 //zählt wiederholung jeder Zeilen ( 4 mal damit 4*4 Fragmente entstehen)
.def current_player=R11 // für refresehn der Spielerbalken gedacht
.def SREG_SAVE=R15 //R15 würde zu lange dauern ,brauchen wir nicht
.def tmp=R16
.def tmp2=R17
.def ZeileL=R18 //da wir mehr als 255 Zeilen haben ,benötigen wir ein 16Bit Register
.def ZeileH=R19
.def out_of_main_flag=R20
.def tmp_main=R21

.cseg

 //los gehts
 .org 0000
 //INTERRUPT TABELLE
  rjmp init //1 ersterer Interupvektor (reset)
  reti //2 Interrupt für Pin INT0
  reti //3  Interrup für Pin INT1
  reti //4 PcINT0
  reti //5 PcINT1
  reti //6 PcINT2
  reti //7 WDT
  reti //8  Timer 2 stimm mit Vergleichswert überein
  reti //9  Timer 2 stimm mit anderem Vergleichswert überein
  reti //10  Timer 2 ist übergelaufen
  reti //11  Timer 1 -> Event
  reti //12  Timer 1 übereinstimmen mit A
  reti //13  Timer 1 übereinstimmung mit B
  reti //14  Timmer 1 übergelaufen
  nop
  nop
  out TCNT0,reload//9  Timmer 0 übergelaufen --> sofort wieder counter starten
  rjmp timer0_int  // --> erst dann springen [ hier steht eig. der SPI-Interrupt (können uns aber eh keine weiteren Interrupts erlauben!)
 /* reti // 17 Timer OVF
  reti // 18 SPI ,STC */
  reti //19  USART , Rx complete
  reti //20  USART , DataReg empty
  reti //21  USART , Tx complete
  reti //22  A/D C , Conversion complete
  reti //23  EEPROM ready
  reti //24 Analog Comperator
  reti //25 TWI (two wired Interface)
  reti //26 SPM_rdy ,store programm Memory ready

 // CODE
 
 init:
 //Einrichten des Stackpointers
 ldi R16,HIGH(RAMEND)
 out SPH,R16
 ldi R16,LOW(RAMEND)
 out SPL,R16
 //Stack_ptr initalisiert
 // Fake Rücksprungadresse draufpacken ( erst LOW ,dann HIGH )
 ldi tmp,LOW(main)
 push tmp
 ldi tmp,HIGH(main)
 push tmp
 ldi tmp,0x00
 push tmp
 push tmp
 // Stackpointer steht jetzt an "stack_ptr_main" bzw SPL ist 0x5B
/* //Test -> gucken ob er zu main springt 
 pop tmp
 pop tmp
 ret 
 */

 


 //PORTS konfigurieren
 in tmp2,SYNC_VGA_PORT
 ldi tmp,(1<<H_SYNC) | (1<<V_SYNC)
 or tmp2,tmp
 out SYNC_VGA_PORT,tmp2 // SYNC sind low aktiv
 in tmp2,SYNC_VGA_DDR
 or tmp2,tmp
 out SYNC_VGA_DDR,tmp2
 
 SER tmp //tmp=0xFF
 out FARBE_VGA_DDR,tmp
 CLR tmp
 out FARBE_VGA_PORT,tmp

 rcall init_usart 

 //TIMER0
 ldi tmp,2

 out TCCR0B,tmp  // Prescaler 8
 ldi tmp,1 
 ldi YL,LOW(TIMSK0)
 ldi YH,HIGH(TIMSK0)
 st  Y,tmp// Bit 0 = 1 ;; enabled OVERFLOW INTERRUPT für TIMER0
 ldi tmp,(255-87)
 mov reload,tmp //TimerInterrupt alle 87 Schritte (bei timerclk=CLK/8 ergibt sich eine InterruptFrequenz von (Clk/8)/87== ~31,5-32,0)
 out TCNT0,reload //ergibt 698 Pixel,die wir nutzen können ( 9 + 37[HSYNC] + 12 + 640 )


 //init der Register
 clr ZeileL
 clr ZeileH
 clr out_of_main_flag
 ldi tmp,replay_zeile
 mov replay_count,tmp
 clr null
 ldi tmp,1
 mov eins,tmp
 ldi tmp,stack_ptr_main_low
 mov stackpointer_main_low,tmp
 


 //Init der SRAM Daten für main
 rcall init_sram_data	
 


 
 // in SRAM kopieren
 rcall copy_spirites_to_sram
 
 // Worklines initalisieren
 rcall init_worklines
 
 // Memory Map Adresse in Register schreiben (byteweise Adressierung !! -> also Flash_adresse*2)
 ldi tmp,LOW(Spielball_memory_map)
 mov memory_map_low_byteadresse,tmp
 ldi tmp,HIGH(Spielball_memory_map)
 mov memory_map_high_byteadresse,tmp
 ldi tmp,2
 mul memory_map_low_byteadresse,tmp // Ergebnis in R0:R1
 mov memory_map_low_byteadresse,R0
 mov tmp2,R1
 mul memory_map_high_byteadresse,tmp
 mov memory_map_high_byteadresse,R0 // R1 wird leer sein
 add memory_map_high_byteadresse,tmp2 

 	

 // default Wert für X
 ldi XH,HIGH(sram_linke_seitenbegrenzung)
 ldi XL,LOW(sram_linke_seitenbegrenzung)


 //Interrupt global enablen
 sei

 wait_for_first_interrupt: //bevor wir irgendwas Zeichnen synchroniseren wir erstmal
	rjmp wait_for_first_interrupt

 display_vga_data:
					// Byte |A,B|C,D|E,F|G,H| -> ausgabe jetzt in PB 3 und PB2
	// Byte          |A,B|
	
	nop //+1
	out FARBE_VGA_PORT,tmp//+1			__Bits |G,H| raus
	ld new_data,X+ //+2
	swap new_data      // _new_data={|E,F|G,H|A,B|C,D|}     
	out FARBE_VGA_PORT,new_data //+1	__ Bits |A,B| raus
	mov tmp,new_data //+1				__tmp={|E,F|G,H|A,B|C,D|}
	lsl tmp //+1 
	lsl tmp //+1						__ tmp={|G,H|A,B|C,D|00|}
	out FARBE_VGA_PORT,tmp//+1		__ Bits |C,D| raus
	swap new_data //+1					__ new_data= {|A,B|C,D|E,F|G,H|}
	swap tmp//+1						__ tmp= {|C,D|00|G,H|A,B|}
	nop 
	out FARBE_VGA_PORT,new_data //+1	__ Bits |E,F| raus
	rjmp display_vga_data //+2


 //Interrupt-service-routinen


 timer0_int:
	out FARBE_VGA_PORT,null //blank-> Farbe während der Schwarzschulter bestimmt Spannung für Dunkel // 9 takte Front Porch
	push tmp  // auf Stack retten (tmp ,tmp2) 
	push tmp2	
	cbi SYNC_VGA_PORT,H_SYNC //HSYNC beginnt
	
	in SREG_SAVE,SREG // Statusregister retten :)

	add ZeileL,eins //absoluten dargestellten Zeilen
	adc ZeileH,null //Bei CarryBit hier noch einen druff

	sei //interrupts enablen (da wir uns ja eig. grade in einem befinden ,sind diese normalerweise maskiert bis reti)
	
	cpi ZeileH,0
	breq poll_VSYNC //wenn ZeileH=0 ,überprüfe ZeileL bzgl Vsync (kommt mit 14 wieder raus)
	rjmp poll_ENDE  // anderenfalls überprüfe ob ZeileH=2 und ZeileL=13 -> ZeilenReset (kommt mit 15 wieder raus)



poll_VSYNC: //6
	cpi ZeileL,11 //1 bis 10 FrontPorch 
	brcs VSYNC_FRONT //wenn ZeileL<10
	cpi ZeileL,13 //11 bis 12 liegt VSYNC an 
	brcs VSYNC_ENABLED
	cpi ZeileL,46 //13 bis 45 V-BackPorch
	brcs VSYNC_BACK
	nop
	rjmp BACK_from_cmp //15  // Wenn nicht in VSYNC werden Ptr XH,XL benutzt

poll_ENDE: //7 // Gucken ob ENDE erreicht -> ZeileH,ZeileL einstellen
	cpi ZeileH,2 //8 Takte seit HSYNC an
	brne poll_ENDE_skip1//9
	cpi ZeileL,12
	brne poll_ENDE_skip2 //11
	ldi ZeileH,0
	ldi ZeileL,0
	rjmp BACK_from_cmp //15
poll_ENDE_skip1:
	nop //11
	nop
	nop //13
poll_ENDE_skip2:
	rjmp BACK_from_cmp //15
skip_set_stackpointer:
	rjmp back_skip_set_stackpointer

VSYNC_FRONT: // wenn reinkommt schon 9 Takte!
	//cbi SYNC_VGA_PORT,V_SYNC //35 VSYNC enabled (low)
	cpi out_of_main_flag,1
	breq skip_set_stackpointer
	out SPL,stackpointer_main_low //+2 // nur bei Übergang "vga_display" zu "main" nötig [ deshalb in VSYNC_ENABLED und VSYNC_BACK egal ]
	nop
back_skip_set_stackpointer:	
	nop
	ldi tmp2,2 
	rcall NOP_BLOCK //10+24 = 34
	ldi tmp2,10
	rcall NOP_BLOCK // HSync verlängern
	sbi SYNC_VGA_PORT,H_SYNC //37 == HSYNC endet
	out SREG,SREG_SAVE
	pop tmp2
	pop tmp
	reti 

VSYNC_ENABLED:// wenn reinkommt schon 11 Takte!
	cbi SYNC_VGA_PORT,V_SYNC //35 VSYNC enabled (low)
	nop
	nop
	ldi tmp2,2 
	rcall NOP_BLOCK //12+21 = 33
	ldi tmp2,10
	rcall NOP_BLOCK // HSync verlängern
	sbi SYNC_VGA_PORT,H_SYNC //37 == HSYNC endet
	out SREG,SREG_SAVE
	pop tmp2
	pop tmp
	reti

VSYNC_BACK:// wenn reinkommt schon 13 Takte!	
	//init für neuen Durchgang
	sbi SYNC_VGA_PORT,V_SYNC // VSYNC disabled (high)
	ldi tmp,replay_zeile //14
	mov replay_count,tmp 
	
	//ldi XH,HIGH(sram_linke_seitenbegrenzung)
	//ldi XL,LOW(sram_linke_seitenbegrenzung)
	ldi out_of_main_flag,0

	ldi tmp2,1 //18
	rcall NOP_BLOCK //18 + 15 = 33
	ldi tmp2,10
	rcall NOP_BLOCK // HSync verlängern
	sbi SYNC_VGA_PORT,H_SYNC //37== HSYNC disabled (high)
	out SREG,SREG_SAVE
	pop tmp2
	pop tmp
	reti

BACK_from_cmp:	// poll_VSYNC und poll_ENDE kommen hier wieder raus
	
	/* Die sichtbaren Zeilen liegen zwischen 46 und 530  d.h ZeileH kann 0,1,2 einnehmen. */
	out SPL,stackpointer_main_low // um StackOverflow zu vermeiden !!
	nop
	ldi tmp2,1 // +12
	rcall NOP_BLOCK // HSync verlängern
	// ab hier zählen

//=========== Zeichnen untere Priorität ( Priorität durch Reihenfolge bestimmbar )

drawing_low_priority_select: // [ Festlegen ob BALL oder Seitenbegrenzung gemalt werden soll ( erstmal als Defaultwerte) ]
	
	mov ZL,ZeileL // 1
	mov ZH,ZeileH
	sub ZL,Spielball_pos_Start_low // minus die Spielballposition
	sbc ZH,null
	cp ZH,Spielball_pos_Start_high // subtrahieren von carryflag   __4
	brne draw_seitenbegrenzung_1 //Wenn ZH nicht null  (mit 7 rein )
	cpi ZL,Spielball_high // ZL muss unter Höhe sein [normiert auf Spielball_pos_Start]
	brcc draw_seitenbegrenzung_2 // mit 8 rein
	//YL enthält Spielballbitnummer bzw Bytenummer (nicht optimal ,aber wir gönnen uns den Speicherplatz mal)
	add ZL,memory_map_low_byteadresse
	adc ZH,memory_map_high_byteadresse
	mov ZH,memory_map_high_byteadresse // nicht schön ,weil unflexibel (muss bei Bedarf angepasst werden , falls memory_map einen ZH Wechsel erfordert)
	lpm tmp,Z
	cpi tmp,1 // 11
	breq draw_spielball_part_1
	brcc draw_spielball_part_2
	rjmp draw_spielball_part_3
//=================== Zeichnen mit niedriger Priorität
draw_spielball_part_2:
	ldi XH,HIGH(sram_spielball_part_2) // 15
	ldi XL,LOW(sram_spielball_part_2) // 16
	nop
	rjmp drawing_high_priority_select // mit 19 raus
draw_spielball_part_1:
	ldi XH,HIGH(sram_spielball_part_1) // 14
	ldi XL,LOW(sram_spielball_part_1) // 15
	nop
	nop
	rjmp drawing_high_priority_select // mit 19 raus
draw_spielball_part_3:
	ldi XH,HIGH(sram_spielball_part_3) // 16
	ldi XL,LOW(sram_spielball_part_3) // 17
	rjmp drawing_high_priority_select //mit 19 raus

draw_seitenbegrenzung_1: // 7
	nop
	nop
draw_seitenbegrenzung_2: //9
	ldi XH,HIGH(sram_linke_seitenbegrenzung) // 11
	ldi XL,LOW(sram_linke_seitenbegrenzung) // 12
	nop
	nop //12
	nop //13
	nop //14
	nop //15
	nop //16
	nop //17
	nop
	nop
	nop
	rjmp drawing_high_priority_select //mit 19 raus

//============
//================ Zeichnen mit hoher Priorität (überzeichnet evt. die Zeichen von niedriger Priorität )
drawing_high_priority_select: // mit 19 rein [ Wenn wir hier nichts finden ,wird das aus vorherigem gemalt ]
//19
// neuer Block (hier Zählweise wieder ab 0 )

	cpi ZeileH,1 // +1
	breq ZeileH_1 // false : +1 , true +2 
	brcs ZeileH_0
ZeileH_2: // 3 ( ZeileL hat hier Werte von 0 bis 17 )
	nop
	cpi ZeileL,start_spielerbalken2_vertikal //+1
	brcc draw_spielerbalken2 // false : +1 , true +2
	nop
	nop
	nop
	nop
back_fail_draw_spielerbalken2:
	nop
	nop
	nop //12
back_draw_spielerbalken2:
	nop //13 //fertig
	rjmp finish_draw

ZeileH_1: // 3 ( ZeileL hat hier Werte von 0 bis 255 )
	cpi ZeileL,start_mittellinie //+1
	brcc draw_mittellinie // false : +1 , true +2
	//falls kleiner als "start_mittellinie" geht es hier weiter	// mit 5
	nop
	nop
	nop
	nop // 9
back_fail_draw_mittellinie:
	nop
	nop
	nop //12
back_draw_mittellinie:
	nop //13 // fertig
	rjmp finish_draw

ZeileH_0: // 4 ( ZeileL hat hier Werte von 46 bis 255 )
	cpi ZeileL,start_spielerbalken1_vertikal //+1
	brcc draw_spielerbalken1 // false : +1 , true +2
	nop 
	nop
	nop //new one
	nop
	nop //10
back_fail_draw_spielerbalken1:
	nop
	nop
	nop //13
back_draw_spielerbalken1:
	rjmp finish_draw
//============ Zeichnen mit hoher Priorität
draw_mittellinie: // 6
	cpi ZeileL,start_mittellinie+breite_mittellinie //+1
	brcc back_fail_draw_mittellinie // false : +1 , true +2 (geht mit 9 raus)
	ldi XH,HIGH(sram_mittellinie) //+1
	ldi XL,LOW(sram_mittellinie) //+1 //10
	rjmp back_draw_mittellinie //12

draw_spielerbalken2: // 6
	cpi ZeileL,start_spielerbalken2_vertikal+spielerbalken2_high
	brcc back_fail_draw_spielerbalken2 // wenn Zeile größer ist
	ldi XH,HIGH(sram_spielerbalken_player2) //+1
	ldi XL,LOW(sram_spielerbalken_player2) //+1 //10
	rjmp back_draw_spielerbalken2 //12

draw_spielerbalken1: // 7
	cpi ZeileL,start_spielerbalken1_vertikal+spielerbalken1_high
	brcc back_fail_draw_spielerbalken1 // wenn Zeile größer ist
	ldi XH,HIGH(sram_spielerbalken_player1) //+1
	ldi XL,LOW(sram_spielerbalken_player1) //+1 //11
	nop
	rjmp back_draw_spielerbalken1 //13


//===========	
//================= Fertig mit Zeichnen

finish_draw: //15	
// Insgesamt fürs Zeichnen 15+19 == 34 Takte fürs Zeichnen  + vorher 15 Nops --> 49 Takte bisher in BACK_from_cmp
	nop
	nop
	ldi tmp,0x00 //schwärz bei Wiedereinstieg in display_vga_data 
HSYNC_end:
	sbi SYNC_VGA_PORT,H_SYNC // 37 Takte==> HSYNC endet
 	//jetzt noch 12 Takte Back Porch
	ldi tmp2,1 //1
	rcall NOP_BLOCK // 10
	rjmp display_vga_data // 12

NOP_BLOCK: // rcall +ret =7 ,dec //+1 , brne //+2,1 -> 6+tmp2*3 ,[9,12,15,18 etc]
	dec tmp2
	brne NOP_BLOCK
	ret 



copy_spirites_to_sram:

	ldi ZH,HIGH(2*flash_spielerbalken) //auf den Anfang vom VGA Datensegment stellen (wichtig 2* wegen byteweise Adressierung von lpm!)
	ldi ZL,LOW(2*flash_spielerbalken)
	ldi XH,HIGH(sram_IMG_spielerbalken1)
	ldi XL,LOW(sram_IMG_spielerbalken1)
	ldi tmp,BYTES_spielerbalken
 
	rcall copy_to_sram

	ldi ZH,HIGH(2*flash_spielerbalken) //auf den Anfang vom VGA Datensegment stellen (wichtig 2* wegen byteweise Adressierung von lpm!)
	ldi ZL,LOW(2*flash_spielerbalken)
	ldi XH,HIGH(sram_IMG_spielerbalken2)
	ldi XL,LOW(sram_IMG_spielerbalken2)
	ldi tmp,BYTES_spielerbalken
 
	rcall copy_to_sram


	ldi ZH,HIGH(2*flash_spielball_part_1) //auf den Anfang vom VGA Datensegment stellen (wichtig 2* wegen byteweise Adressierung von lpm!)
	ldi ZL,LOW(2*flash_spielball_part_1)
	ldi XH,HIGH(sram_IMG_spielball_part_1)
	ldi XL,LOW(sram_IMG_spielball_part_1)
	ldi tmp,BYTES_spielball
 
	rcall copy_to_sram

	ldi ZH,HIGH(2*flash_spielball_part_2) //auf den Anfang vom VGA Datensegment stellen (wichtig 2* wegen byteweise Adressierung von lpm!)
	ldi ZL,LOW(2*flash_spielball_part_2)
	ldi XH,HIGH(sram_IMG_spielball_part_2)
	ldi XL,LOW(sram_IMG_spielball_part_2)
	ldi tmp,BYTES_spielball
 
	rcall copy_to_sram
	
	ldi ZH,HIGH(2*flash_spielball_part_3) //auf den Anfang vom VGA Datensegment stellen (wichtig 2* wegen byteweise Adressierung von lpm!)
	ldi ZL,LOW(2*flash_spielball_part_3)
	ldi XH,HIGH(sram_IMG_spielball_part_3)
	ldi XL,LOW(sram_IMG_spielball_part_3)
	ldi tmp,BYTES_spielball

	rcall copy_to_sram

	// Workline linke_seitenbegrenzung vorbereiten
	ldi ZH,HIGH(2*flash_linke_seitenbegrenzung) //auf den Anfang vom VGA Datensegment stellen (wichtig 2* wegen byteweise Adressierung von lpm!)
	ldi ZL,LOW(2*flash_linke_seitenbegrenzung)
	ldi XH,HIGH(sram_linke_seitenbegrenzung)
	ldi XL,LOW(sram_linke_seitenbegrenzung)
	ldi tmp,BYTES_complete_line
 
	rcall copy_to_sram

	// Workline mittellinie vorbereiten
	ldi ZH,HIGH(2*flash_mittellinie) //auf den Anfang vom VGA Datensegment stellen (wichtig 2* wegen byteweise Adressierung von lpm!)
	ldi ZL,LOW(2*flash_mittellinie)
	ldi XH,HIGH(sram_mittellinie)
	ldi XL,LOW(sram_mittellinie)
	ldi tmp,BYTES_complete_line
 
	rcall copy_to_sram

	// Workline player1 vorbereiten
	ldi ZH,HIGH(2*flash_linke_seitenbegrenzung) 
	ldi ZL,LOW(2*flash_linke_seitenbegrenzung)
	ldi XH,HIGH(sram_spielerbalken_player1)
	ldi XL,LOW(sram_spielerbalken_player1)
	ldi tmp,BYTES_complete_line

	rcall copy_to_sram

	// Workline player2 vorbereiten
	ldi ZH,HIGH(2*flash_linke_seitenbegrenzung) 
	ldi ZL,LOW(2*flash_linke_seitenbegrenzung)
	ldi XH,HIGH(sram_spielerbalken_player2)
	ldi XL,LOW(sram_spielerbalken_player2)
	ldi tmp,BYTES_complete_line

	rcall copy_to_sram

	// Workline spielball_part_1 vorbereiten
	ldi ZH,HIGH(2*flash_linke_seitenbegrenzung) 
	ldi ZL,LOW(2*flash_linke_seitenbegrenzung)
	ldi XH,HIGH(sram_spielball_part_1)
	ldi XL,LOW(sram_spielball_part_1)
	ldi tmp,BYTES_complete_line

	rcall copy_to_sram

	// Workline spielball_part_2 vorbereiten
	ldi ZH,HIGH(2*flash_linke_seitenbegrenzung) 
	ldi ZL,LOW(2*flash_linke_seitenbegrenzung)
	ldi XH,HIGH(sram_spielball_part_2)
	ldi XL,LOW(sram_spielball_part_2)
	ldi tmp,BYTES_complete_line

	rcall copy_to_sram

	// Workline spielball_part_3 vorbereiten
	ldi ZH,HIGH(2*flash_linke_seitenbegrenzung) 
	ldi ZL,LOW(2*flash_linke_seitenbegrenzung)
	ldi XH,HIGH(sram_spielball_part_3)
	ldi XL,LOW(sram_spielball_part_3)
	ldi tmp,BYTES_complete_line

	rcall copy_to_sram
	

	ret


copy_to_sram: // Soll tmp Bytes von Flash in Sram kopieren
	push tmp
copy_to_sram_loop:
	lpm new_data,Z+
	st X+,new_data
	dec tmp
	brne copy_to_sram_loop
	pop tmp
	ret 
init_spielerbalken1: // Zum refreshen der Spielerbalken gedacht
	push XL
	push XH
	push ZL
	push ZH
	push tmp
	ldi ZH,HIGH(2*flash_spielerbalken) //auf den Anfang vom VGA Datensegment stellen (wichtig 2* wegen byteweise Adressierung von lpm!)
	ldi ZL,LOW(2*flash_spielerbalken)
	ldi XH,HIGH(sram_IMG_spielerbalken1)
	ldi XL,LOW(sram_IMG_spielerbalken1)
	ldi tmp,BYTES_spielerbalken
 
	rcall copy_to_sram
	pop tmp
	pop ZH
	pop ZL
	pop XH
	pop XL
	ret
init_spielerbalken2: // Zum refreshen der Spielerbalken gedacht
	push XL
	push XH
	push ZL
	push ZH
	push tmp
	ldi ZH,HIGH(2*flash_spielerbalken) //auf den Anfang vom VGA Datensegment stellen (wichtig 2* wegen byteweise Adressierung von lpm!)
	ldi ZL,LOW(2*flash_spielerbalken)
	ldi XH,HIGH(sram_IMG_spielerbalken2)
	ldi XL,LOW(sram_IMG_spielerbalken2)
	ldi tmp,BYTES_spielerbalken
 
	rcall copy_to_sram
	pop tmp
	pop ZH
	pop ZL
	pop XH
	pop XL
	ret
init_worklines:
	//spielerbalken_player1: (Y ist egal ,weil wird nicht verwendet )
	ldi YL,LOW(sram_spielerbalken_player1+startpos_spielerbalken1_horizontal)
	ldi YH,HIGH(sram_spielerbalken_player1+startpos_spielerbalken1_horizontal)
	// Z hingegen wird verwenden und zwar im Sichtbaren Bereich (entweder ZL,ZH pushen, aber eigentlich nicht notwendig weil es eh fertig werden muss, BEVOR es
	// wieder sichtbar wird)
	ldi ZL,LOW(sram_IMG_spielerbalken1)
	ldi ZH,HIGH(sram_IMG_spielerbalken1)
	ldi tmp,BYTES_spielerbalken
	rcall sram_to_workline
	
	ldi YL,LOW(sram_spielerbalken_player2+startpos_spielerbalken2_horizontal)
	ldi YH,HIGH(sram_spielerbalken_player2+startpos_spielerbalken2_horizontal)
	ldi ZL,LOW(sram_IMG_spielerbalken2)
	ldi ZH,HIGH(sram_IMG_spielerbalken2)
	ldi tmp,BYTES_spielerbalken
	rcall sram_to_workline
	
	ldi YL,LOW(sram_spielball_part_1+startpos_spielball_horizontal)
	ldi YH,HIGH(sram_spielball_part_1+startpos_spielball_horizontal)
	ldi ZL,LOW(sram_IMG_spielball_part_1)
	ldi ZH,HIGH(sram_IMG_spielball_part_1)
	ldi tmp,BYTES_spielball
	rcall sram_to_workline

	ldi YL,LOW(sram_spielball_part_2+startpos_spielball_horizontal)
	ldi YH,HIGH(sram_spielball_part_2+startpos_spielball_horizontal)
	ldi ZL,LOW(sram_IMG_spielball_part_2)
	ldi ZH,HIGH(sram_IMG_spielball_part_2)
	ldi tmp,BYTES_spielball
	rcall sram_to_workline
	
	ldi YL,LOW(sram_spielball_part_3+startpos_spielball_horizontal)
	ldi YH,HIGH(sram_spielball_part_3+startpos_spielball_horizontal)
	ldi ZL,LOW(sram_IMG_spielball_part_3)
	ldi ZH,HIGH(sram_IMG_spielball_part_3)
	ldi tmp,BYTES_spielball
	rcall sram_to_workline
	ret		
	


sram_to_workline:
	push tmp
sram_to_workline_loop:
	ld new_data,Z+
	st Y+,new_data
	dec tmp
	brne sram_to_workline_loop
	pop tmp
	ret

main:
// für die Variabeln ! Vsync benutzt nur die Variablen tmp,tmp2 -> diese werden deshalb auf den Stack gerettet.
// In main wird gesprungen,wenn grade kein Bild ausgegeben werden muss --> Das betrifft den Zeitraum der vertikalen Syncronistation
// also VSYNC_FRONT_PORCH [ 10 ] , VSYNC_ENABLED [ 2 ] und VSYNC_V_BACK_PORCH [ 32 ]
// spirch ca. 45*640 Takte Zeit für Berechnungen ! ( Wird durch Timer_Interrupts zwischendurch unterbrochen)
	
	out FARBE_VGA_PORT,null
	ldi out_of_main_flag,1 // erlaubt uns Funktionen in main zu benutzen ohne die Stabilität zu gefährden
	nop

		// Durchgänge seit letztem Rst zählen
	ldi YL,LOW(clocks_since_reset)
	ldi YH,HIGH(clocks_since_reset)
	ld tmp,Y
	inc tmp // bei Überlauf fängt er wieder bei null an

    ldi YL,LOW(cmds_for_controller)
	ldi YH,HIGH(cmds_for_controller)
	ld     tmp,Y
	ori tmp,init_cmds_for_controller
	//ldi tmp,'T'
	push	tmp
    rcall   uart_transmit                      ; Unterprogramm aufrufen
    
	ldi YL,LOW(cmds_for_controller)
	ldi YH,HIGH(cmds_for_controller)
	ldi tmp,init_cmds_for_controller
	st Y,tmp // reseten von "cmds_for_controller"


	// falls Ball sich bewegt hat , Positionen aktualisieren	
	rcall movement_ball_x
	rcall movement_ball_y
	// falls nun eine Kollision mit den Wänden aufgetreten ist -> aktualisieren
	rcall kollision_ball_x
	// Worklines mit dem Ball neu malen
	rcall load_ball_worklines
	
	rcall kollision_ball_ziel // Bei Treffer -> reseten , cmds_for_controller dementsprechend ändern
	
	//rcall uart_wait_for_receiving
	 ldi YL,LOW(UCSR0A)
     ldi YH,HIGH(UCSR0A)
uart_receiving:
	ld tmp,Y
	ldi tmp2,(1<<RXC0)
	and tmp,tmp2
	cp tmp,tmp2
	brne uart_receiving
	ldi YL,LOW(UDR0)
	ldi YH,HIGH(UDR0)
	ld tmp, Y
	ldi YL,LOW(databyte_controller)
	ldi YH,HIGH(databyte_controller)
	st Y,tmp

if_reset:
	mov tmp2,tmp
	andi tmp2,cmd_c2vga_reset
	cpi tmp2,cmd_c2vga_reset
	brne reset_false
reset_true:
	clr tmp
	ldi YL,LOW(databyte_controller)
	ldi YH,HIGH(databyte_controller)
	st Y,tmp
	rcall init_sram_data
	 // in SRAM kopieren
	rcall copy_spirites_to_sram
	// Worklines initalisieren
	rcall init_worklines
	// Befehle an  C-Modul sind immer noch leer (werden am Anfang immer resetet)
	rjmp wait_main_loop
reset_false:

	//serielle Schnittstelle abfragen (erstmal enablen ,dass was gesendet werden kann ! dann polln ! Wenn was gesendet wurde -> empfangen disablen u. abholen )
	//je nach Protokollaufbau auswerten 
	// Protokollaufbau : 8 Nutzbits
	// [0:2] mov_player1 ; [3:5] mov_player2 ; [6:7] cmd
	rcall movement_player // passt Positionen und SRAM-Blöcke den neuen Gegebenheiten an ( Berücksichtig Seitenrahmen ) (scheint zu klappen)
	sei
	rcall kollision_ball_y 

	

	rcall load_player_worklines // klappt

	//Kollisionen prüfen ( Ball <-> Spieler , Ball <-> Wand , Ball <-> Ziel , Spieler <-> Wand )
	//Kollisionen auswerten ( Ballbewegungsvektor neu berechnen oder/und Punktvergabe+Neustart )
	//Ball neu laden , Punkte neu laden , Spielerbalken neu laden

	rcall seitenlinie_fixen // zum fixen der Schäden am Koordinatensystem ('quick' && dirty lösung )
	
	// Clearen vom ControllerDatabyte
	clr tmp
	ldi YL,LOW(databyte_controller)
	ldi YH,HIGH(databyte_controller)
	st Y,tmp
	sei
wait_main_loop: //pollt ZeileL
	cpi ZeileL,(46-1) //13 bis 45 V-BackPorch
	brcc finish_main  // dannach neuer Zyklus
	rjmp wait_main_loop

finish_main: //nach nächstem Interrupt beginnt wieder ein neuer Anzeige Zyklus -> Dannach soll wieder ein neuer main-Zyklus beginnen [neue Daten etc] 
	//Einrichten des Stackpointers
	 ldi tmp,HIGH(RAMEND)
	 out SPH,tmp
	 ldi tmp,LOW(RAMEND)
	 out SPL,tmp
	 //Stack_ptr initalisiert
	 // Fake Rücksprungadresse draufpacken ( erst LOW ,dann HIGH )
	 ldi tmp,LOW(main)
	 push tmp
	 ldi tmp,HIGH(main)
	 push tmp
	 ldi tmp,0x00
	 push tmp
	 push tmp
	 // Stackpointer steht jetzt an "stack_ptr_main" bzw SPL ist 0x5B
finish_main_loop:
	nop
	rjmp finish_main_loop

//======Funktionen Main

//====== fix_it_felix

seitenlinie_fixen:
	
	//notwendig aufgrund der Pufferbytes zum cleanen und rotieren -> einfache Lösung , gitb bestimmt elegantere aber hey
	push tmp

	ldi YL,LOW(sram_spielball_part_1)
	ldi YH,HIGH(sram_spielball_part_1)
	ldi ZL,LOW(sram_linke_seitenbegrenzung)
	ldi ZH,HIGH(sram_linke_seitenbegrenzung)
	ldi tmp,linke_seitenbegrenzung
	rcall sram_to_workline

	ldi YL,LOW(sram_spielball_part_2)
	ldi YH,HIGH(sram_spielball_part_2)
	ldi ZL,LOW(sram_linke_seitenbegrenzung)
	ldi ZH,HIGH(sram_linke_seitenbegrenzung)
	ldi tmp,linke_seitenbegrenzung
	rcall sram_to_workline
	
	ldi YL,LOW(sram_spielerbalken_player1)
	ldi YH,HIGH(sram_spielerbalken_player1)
	ldi ZL,LOW(sram_linke_seitenbegrenzung)
	ldi ZH,HIGH(sram_linke_seitenbegrenzung)
	ldi tmp,linke_seitenbegrenzung
	rcall sram_to_workline
	
	ldi YL,LOW(sram_spielerbalken_player2)
	ldi YH,HIGH(sram_spielerbalken_player2)
	ldi ZL,LOW(sram_linke_seitenbegrenzung)
	ldi ZH,HIGH(sram_linke_seitenbegrenzung)
	ldi tmp,linke_seitenbegrenzung
	rcall sram_to_workline
	
	pop tmp
	ret


//==== load_player_worklines
load_player_worklines:

	ldi YL,LOW(pos_Byte_player1)
	ldi YH,HIGH(pos_Byte_player1)
	ld tmp2,Y
	ldi tmp,BYTES_spielerbalken	

	ldi YL,LOW(sram_spielerbalken_player1)
	ldi YH,HIGH(sram_spielerbalken_player1)
	add YL,tmp2 // Offset von Position
	adc YH,null
	ldi ZL,LOW(sram_IMG_spielerbalken1)
	ldi ZH,HIGH(sram_IMG_spielerbalken1)
	rcall sram_to_workline


	ldi YL,LOW(pos_Byte_player2)
	ldi YH,HIGH(pos_Byte_player2)
	ld tmp2,Y
	ldi tmp,BYTES_spielerbalken	

	ldi YL,LOW(sram_spielerbalken_player2)
	ldi YH,HIGH(sram_spielerbalken_player2)
	add YL,tmp2 // Offset von Position
	adc YH,null
	ldi ZL,LOW(sram_IMG_spielerbalken2)
	ldi ZH,HIGH(sram_IMG_spielerbalken2)
	rcall sram_to_workline
	ret
//======== kollision_ball_y
kollision_ball_y:
	push tmp
	push tmp2
	push Spielball_pos_Start_low
	
	// ( Ball <-> Spieler )
	// laden von Spieler Y-Koordinate  ( sind in Spielball_pos_Start_low und Spielball_pos_Start_high )
	// überprüfen ob im kritischem Bereich für spieler1 oder spieler2 (wenn nicht-> weg )
	mov tmp,Spielball_pos_Start_high
	cpi tmp,1
	brcs testen_kollision_mit_player1
	brne testen_kollision_mit_player2
	rjmp	kollision_ball_y_ende


testen_kollision_mit_player1:
	mov tmp,Spielball_pos_Start_low
	cpi tmp,(start_spielerbalken1_vertikal+spielerbalken1_high)
	brcc kollision_ball_y_ende // spielball_vertikal_start zu groß? dann weg
	
	ldi YL,LOW(counter_Bit_player1)
	ldi YH,HIGH(counter_Bit_player1)
	push YL
	push YH
	ldi YL,LOW(pos_Byte_player1)
	ldi YH,HIGH(pos_Byte_player1)
	push YL
	push YH

	rcall get_start_of_spielerbalken
	mov tmp_main,tmp2
	rcall get_start_of_spielball
	
	push tmp2
	ldi tmp,3 // Breite des Spielballs -> kurz unten abgelesen
	add tmp2,tmp
	cp tmp2,tmp_main // Ist Ball+Breite (Zählweise von rechts) größer als das (numerische kleinere) Ende des Spielbalkens?
	brcc testen_kollision_mit_player1_next
	pop tmp
	rjmp kollision_ball_y_ende
testen_kollision_mit_player1_next:
	pop tmp2
	ldi tmp,spielerbalken1_length
	add tmp_main,tmp
	cp tmp2,tmp_main // Position für tatsächlichen Anfang Spielerbalken
	brcs kollision_mit_player1
	rjmp kollision_ball_y_ende
kollision_mit_player1:
	ldi YL,LOW(v_Ball_y_max)
	ldi YH,HIGH(v_Ball_y_max)

	ld tmp,Y
	andi tmp,0x7F // cleart das MSB
	dec tmp // schneller machen (max. auf 1 )
	brne kollision_mit_player1_skip1// Zero Bit gesetzt ?
	inc tmp	
kollision_mit_player1_skip1:
	st Y,tmp 
	rjmp kollision_ball_y_ende

kollision_ball_y_ende:
	pop Spielball_pos_Start_low
	pop tmp2
	pop tmp
	ret	


testen_kollision_mit_player2:
	mov tmp,Spielball_pos_Start_low
	ldi tmp_main,Spielball_high
	add tmp,tmp_main  // zähl ja von oben nach unten
	cpi tmp,start_spielerbalken2_vertikal
	brcs kollision_ball_y_ende // spielball_vertikal_start zu klein? dann weg
	// herausfinden von genauem Start Spielbalken und genauem Start Spielkugel ( Momentanes Startbyte+Orginalanfangsverschiebung(bits)+counterzahl(bits)-4 (bits) [Normierung] )
	// wenns übereinstimmt dann kollisionsroutine -> erstmal nur Bewegung umkehren :)
	ldi YL,LOW(counter_Bit_player2)
	ldi YH,HIGH(counter_Bit_player2)
	push YL
	push YH
	ldi YL,LOW(pos_Byte_player2)
	ldi YH,HIGH(pos_Byte_player2)
	push YL
	push YH
	rcall get_start_of_spielerbalken
	mov tmp_main,tmp2
	rcall get_start_of_spielball


	push tmp2
	ldi tmp,3 // Breite des Spielballs -> kurz unten abgelesen
	add tmp2,tmp
	cp tmp2,tmp_main // Ist Ball+Breite (Zählweise von rechts) größer als das (numerische kleinere) Ende des Spielbalkens?
	brcc testen_kollision_mit_player2_next
	pop tmp
	rjmp kollision_ball_y_ende
testen_kollision_mit_player2_next:
	pop tmp2
	ldi tmp,spielerbalken2_length
	add tmp_main,tmp
	cp tmp2,tmp_main // Position für tatsächlichen Anfang Spielerbalken
	brcs kollision_mit_player2
	rjmp kollision_ball_y_ende
kollision_mit_player2:
	ldi YL,LOW(v_Ball_y_max)
	ldi YH,HIGH(v_Ball_y_max)
	ld tmp,Y
	andi tmp,0x7F // cleart das MSB
	dec tmp // schneller machen (max. auf 1 )
	brne kollision_mit_player2_skip1// Zero Bit gesetzt ?
	inc tmp	
kollision_mit_player2_skip1:
	ori tmp,0x80
	st Y,tmp
	rjmp kollision_ball_y_ende




//====== kollision_ball_ziel

kollision_ball_ziel:
	push tmp
	ldi tmp2,0
	mov tmp,Spielball_pos_Start_high
	cpi tmp,1
	brcs testen_kollision_ball_ziel1
	brne testen_kollision_ball_ziel2
	rjmp kollision_ball_ziel_ende
testen_kollision_ball_ziel1:
	mov tmp,Spielball_pos_Start_low
	cpi tmp,(start_spielerbalken1_vertikal)
	brcc kollision_ball_ziel_ende // spielball_vertikal_start zu groß? dann weg
	//hier Gewinn-Routine
	rcall init_sram_data
	 // in SRAM kopieren
	rcall copy_spirites_to_sram
	// Worklines initalisieren
	rcall init_worklines

	ldi tmp2,0x01
	
	rjmp kollision_ball_ziel_ende
testen_kollision_ball_ziel2:
	mov tmp,Spielball_pos_Start_low
	ldi tmp_main,Spielball_high
	add tmp,tmp_main  // zähl ja von oben nach unten
	cpi tmp,start_spielerbalken2_vertikal+3
	brcs kollision_ball_ziel_ende // spielball_vertikal_start zu groß? dann weg
	rcall init_sram_data
	 // in SRAM kopieren
	rcall copy_spirites_to_sram
	// Worklines initalisieren
	rcall init_worklines

	ldi tmp2,0x02
	
	rjmp kollision_ball_ziel_ende
kollision_ball_ziel_ende:
	
	ldi YL,LOW(cmds_for_controller)
	ldi YH,HIGH(cmds_for_controller)
	ldi tmp,init_cmds_for_controller
	or tmp,tmp2 //  verodern mit Spielstand,neuem
	st Y,tmp // speichern von Spielstandveränderung		
	
	pop tmp
	ret
//==== get_start_of_spielerbalken
get_start_of_spielerbalken:
	pop ZH
	pop ZL // Stack saven
	pop YH
	pop YL
	ld tmp2,Y
	lsl tmp2
	lsl tmp2
	pop YH
	pop YL
	ld tmp,Y
	add tmp2,tmp
	subi tmp2,init_counter_bit 
	ldi tmp,offset_null_bits_spielerbalken
	add tmp2,tmp  // der Wert von tmp2 kann zwar nicht zu zeichnen verwendeten werden [wie die Erfahrung uns zeigt] , reicht aber zum vergleich,da beide die gleiche Abstraktion erfahren
	mov tmp_main,tmp2 // Position für tatsächlichen Anfang Spielerbalken
	push ZL
	push ZH // Stack saven
	ret
//=== get_start_of_spielball
get_start_of_spielball:
	ldi YL,LOW(pos_Byte_Ball_x)
	ldi YH,HIGH(pos_Byte_Ball_x)
	ld tmp2,Y
	lsl tmp2
	lsl tmp2
	ldi YL,LOW(counter_Bit_Ball_x)
	ldi YH,HIGH(counter_Bit_Ball_x)
	ld tmp,Y
	add tmp2,tmp
	subi tmp2,init_counter_bit
	ldi tmp,offset_null_bits_spielball
	add tmp2,tmp // Position für tatsächlichen Anfang Spielball
	ret

//==== Movement Player
movement_player:
	push tmp
	push tmp2
	ldi YL,LOW(databyte_controller)
	ldi YH,HIGH(databyte_controller)
	ld tmp,Y
	mov tmp2,tmp
	andi tmp2,0x03
movement_player_loop1:
	rcall movement_player1
	dec tmp2
	brne movement_player_loop1
	
	mov tmp2,tmp
	lsr tmp2
	lsr tmp2
	lsr tmp2
	andi tmp2,0x03
movement_player_loop2:	
	rcall movement_player2
	dec tmp2
	brne movement_player_loop2

	pop tmp2
	pop tmp
	ret

movement_player1: // begrenzt ebenfalls Bewegung durch Seitelinien
	push tmp
	push tmp2
	//========= aktualisieren von current player
	push tmp
	ldi tmp,PLAYER_1
	mov current_player,tmp
	pop tmp
	//========

	andi tmp,0x07 // maskiert alles bis auf die movment_bits für player1 ( 000 = keine Bewegung ; 101 bis 111 schnell bis weniger schnell rechts ; 001 bis 011 schnell bis weniger schnell links )
	// Beschränkung auf linken und rechten Seitenrahmen
	ldi YL,LOW(pos_Byte_player1)
	ldi YH,HIGH(pos_Byte_player1)
	ld tmp_main,Y //Position in tmp_main
	cpi tmp_main,linke_seitenbegrenzung // Überschreiten der linken Seitenbegrenzung?
	brcs ply1_seitenbegrenzung_links
	ldi tmp2,BYTES_spielerbalken-3
	add tmp_main,tmp2
	cpi tmp_main,rechte_seitenbegrenzung+1 // Überschreiten der rechten Seitenbegrenzung?
	brcs ply1_check_border_finish // weder rechts noch links dran
ply1_seitenbegrenzung_rechts:
	cpi tmp,0x04
	brcs ply1_check_border_finish
	andi tmp,0x00 // Bewegung geht nach rechts und wird deshalb maskiert (weil rechte Seitenbegrenzung erreicht)
	rjmp ply1_check_border_finish

ply1_seitenbegrenzung_links:
	cpi tmp,0x04
	brcc ply1_check_border_finish
	andi tmp,0x00 // Bewegung geht nach links und wird deshalb maskiert (weil linke Seitenbegrenzung erreicht)
	rjmp ply1_check_border_finish

ply1_check_border_finish:
	ldi YL,LOW(counter_Bit_player1)
	ldi YH,HIGH(counter_Bit_player1)
	ld tmp2,Y
	cp tmp,null
	breq no_movement_player_1
	cpi tmp,0x04
	breq no_movement_player_1
		
	ldi YL,LOW(pos_Byte_player1)
	ldi YH,HIGH(pos_Byte_player1)
	push YL
	push YH
	ldi tmp_main,BYTES_spielerbalken // tmp_main
	push tmp_main
	ldi YL,LOW(sram_IMG_spielerbalken1)
	ldi YH,HIGH(sram_IMG_spielerbalken1)
	push YL
	push YH
	ldi YL,LOW(counter_Bit_player1)
	ldi YH,HIGH(counter_Bit_player1)
	push YL
	push YH
	ldi YL,LOW(last_input_ply1) // zum speichern der letzten Bewegung
	ldi YH,HIGH(last_input_ply1)
	
	cpi tmp,0x04
	brcc movement_right_player1
movement_left_player1:
	st Y,eins
	rcall movement_player_left
	rjmp no_movement_player_1
movement_right_player1:
	st Y,null
	rcall movement_player_right
no_movement_player_1:
	ldi YL,LOW(counter_Bit_player1)
	ldi YH,HIGH(counter_Bit_player1)
	st Y,tmp2
	sei
	pop tmp2
	pop tmp
	ret

movement_player2:
	push tmp
	push tmp2
	//========= aktualisieren von current player
	push tmp
	ldi tmp,PLAYER_2
	mov current_player,tmp
	pop tmp
	//========
	// Beschränkung auf linken und rechten Seitenrahmen
	lsr tmp
	lsr tmp
	lsr tmp
	andi tmp,0x07 // maskiert alles bis auf die movment_bits für player2 ( 000 = keine Bewegung ; 101 bis 111 schnell bis weniger schnell rechts ; 001 bis 011 schnell bis weniger schnell links )

	ldi YL,LOW(pos_Byte_player2)
	ldi YH,HIGH(pos_Byte_player2)
	ld tmp_main,Y
	cpi tmp_main,linke_seitenbegrenzung
	brcs ply2_seitenbegrenzung_links
	ldi tmp2,BYTES_spielerbalken-3
	add tmp_main,tmp2
	cpi tmp_main,rechte_seitenbegrenzung+1
	brcs ply2_check_border_finish // weder rechts noch links dran
ply2_seitenbegrenzung_rechts:
	cpi tmp,0x04
	brcs ply2_check_border_finish
	andi tmp,0x00 // Bewegung geht nach rechts und wird deshalb maskiert (weil rechte Seitenbegrenzung erreicht)
	rjmp ply2_check_border_finish
ply2_seitenbegrenzung_links:
	cpi tmp,0x04
	brcc ply2_check_border_finish
	andi tmp,0x00 // Bewegung geht nach links und wird deshalb maskiert (weil linke Seitenbegrenzung erreicht)
	rjmp ply2_check_border_finish
ply2_check_border_finish:
	ldi YL,LOW(counter_Bit_player2)
	ldi YH,HIGH(counter_Bit_player2)
	ld tmp2,Y
	cpi tmp,0	
	breq no_movement_player_2
		
	ldi YL,LOW(pos_Byte_player2)
	ldi YH,HIGH(pos_Byte_player2)
	push YL
	push YH
	ldi tmp_main,BYTES_spielerbalken
	push tmp_main
	ldi YL,LOW(sram_IMG_spielerbalken2)
	ldi YH,HIGH(sram_IMG_spielerbalken2)
	push YL
	push YH
	ldi YL,LOW(counter_Bit_player2)
	ldi YH,HIGH(counter_Bit_player2)
	push YL
	push YH

	ldi YL,LOW(last_input_ply2) // zum speichern der letzten Bewegung
	ldi YH,HIGH(last_input_ply2)

	cpi tmp,0x04
	brcc movement_right_player2
movement_left_player2:
	st Y,null
	rcall movement_player_left
	rjmp no_movement_player_2
movement_right_player2:
	st Y,eins
	rcall movement_player_right
no_movement_player_2:
	ldi YL,LOW(counter_Bit_player2)
	ldi YH,HIGH(counter_Bit_player2)
	st Y,tmp2
	sei
	pop tmp2
	pop tmp
	ret

	movement_player_right:
// tmp2 movment_player_right ( High(counter_bit),Low(counter_bit),High(Startadress_Datenblock),Low(Startadresse_Datenblock,Bytegröße des Datenblocks,High(pos_Byte_spielerbalken),Low(pos_Byte_spielerbalken))
// Rückgabewert in tmp2 enthält [neues] counter_bit	
	pop ZH // saven der Rücksprungadresse
	pop ZL
	pop YH	// counter_bit_H
	pop YL	// counter_bit_L
	ld tmp,Y // tmp enthält counter_bit
	pop YH	// sram_IMG_spielerbalken
	pop YL	// sram_IMG_spielerbalken
	pop tmp2 //tmp2 enthält Byte_größe des Blockes

	inc tmp
	cpi tmp,8 // wenn ==8...
	breq movment_player_turn_back //.... wird zurück gedreht
	ldi tmp_main,2
	rcall right_rotation
	mov tmp2,tmp 
	pop YH // pos_Byte_spielerbalken
	pop YL // pos_Byte_spielerbalken
	back_movment_player_turn_back:
	nop // tmp2 enthält nun neues counter Bit
	push ZL
	push ZH // für Return wieder die Rücksprungadresse drauflegen
	ret

	movment_player_turn_back:
	ldi tmp_main,6
	rcall left_rotation
	pop YH // pos_Byte_spielerbalken
	pop YL // pos_Byte_spielerbalken
	ld tmp,Y 
	inc tmp // einen weiter nach vorne
	st Y,tmp
	push tmp // refreshen der  Spielerbalken
	mov tmp,current_player
	cpi tmp,PLAYER_1
	breq turn_back_right_ply1
	rcall init_spielerbalken2
	rjmp finish_turn_back_right
turn_back_right_ply1:
	rcall init_spielerbalken1
finish_turn_back_right:
	pop tmp
	ldi tmp2,init_counter_bit

	rjmp back_movment_player_turn_back

	movement_player_left:
// tmp2 movment_player_left ( High(counter_bit),Low(counter_bit),High(Startadress_Datenblock),Low(Startadresse_Datenblock,Bytegröße des Datenblocks,High(pos_Byte_spielerbalken),Low(pos_Byte_spielerbalken))
// Rückgabewert in tmp2 enthält [neues] counter_bit	
	pop ZH // saven der Rücksprungadresse
	pop ZL
	pop YH	// counter_bit_H
	pop YL	// counter_bit_L
	ld tmp,Y // tmp enthält counter_bit
	pop YH	// sram_IMG_spielerbalken
	pop YL	// sram_IMG_spielerbalken
	pop tmp2 //tmp2 enthält Byte_größe des Blockes

	dec tmp
	cpi tmp,0 // wenn ==8...
	breq movment_player_turn_back_left //.... wird zurück gedreht
	ldi tmp_main,2
	rcall left_rotation
	mov tmp2,tmp 
	pop YH // pos_Byte_spielerbalken
	pop YL // pos_Byte_spielerbalken
	back_movment_player_turn_back_left:
	nop // tmp2 enthält nun neues counter Bit
	push ZL
	push ZH // für Return wieder die Rücksprungadresse drauflegen
	ret

	movment_player_turn_back_left:
	ldi tmp_main,6
	rcall right_rotation
	pop YH // pos_Byte_spielerbalken
	pop YL // pos_Byte_spielerbalken
	ld tmp,Y 
	dec tmp // einen nach hinten
	st Y,tmp
	//============================================
	push tmp // refreshen der  Spielerbalken
	mov tmp,current_player
	cpi tmp,PLAYER_1
	breq turn_back_left_ply1
	rcall init_spielerbalken2
	rjmp finish_turn_back_left
turn_back_left_ply1:
	rcall init_spielerbalken1
finish_turn_back_left:
	pop tmp
	ldi tmp2,init_counter_bit
	rjmp back_movment_player_turn_back_left

//==== load_ball_worklines
load_ball_worklines: // Zeichnet den Ball an die Position die in "pos_Byte_Ball_x" steht 
	ldi YL,LOW(pos_Byte_Ball_x)
	ldi YH,HIGH(pos_Byte_Ball_x)
	ld tmp2,Y
	ldi tmp,BYTES_spielball	

	ldi YL,LOW(sram_spielball_part_1)
	ldi YH,HIGH(sram_spielball_part_1)
	add YL,tmp2 // Offset von Position
	adc YH,null
	ldi ZL,LOW(sram_IMG_spielball_part_1)
	ldi ZH,HIGH(sram_IMG_spielball_part_1)
	rcall sram_to_workline

	ldi tmp,BYTES_spielball	
	ldi YL,LOW(sram_spielball_part_2)
	ldi YH,HIGH(sram_spielball_part_2)
	add YL,tmp2 // Offset von Position
	adc YH,null
	ldi ZL,LOW(sram_IMG_spielball_part_2)
	ldi ZH,HIGH(sram_IMG_spielball_part_2)
	rcall sram_to_workline

	ldi tmp,BYTES_spielball	
	ldi YL,LOW(sram_spielball_part_3)
	ldi YH,HIGH(sram_spielball_part_3)
	add YL,tmp2 // Offset von Position
	adc YH,null
	ldi ZL,LOW(sram_IMG_spielball_part_3)
	ldi ZH,HIGH(sram_IMG_spielball_part_3)
	rcall sram_to_workline

	ret
//=== Movment Ball _y
movement_ball_y:
	ldi YL,LOW(v_Ball_y_count)
	ldi YH,HIGH(v_Ball_y_count)
	ld tmp2,Y
	dec tmp2
	breq change_pos_Ball_y
	st Y,tmp2
	ret

change_pos_Ball_y:
	ldi YL,LOW(v_Ball_y_max)
	ldi YH,HIGH(v_Ball_y_max)
	ld tmp,Y
	ldi YL,LOW(pos_Ball_y_high)
	ldi YH,HIGH(pos_Ball_y_high)
	ld tmp_main,Y
	ldi YL,LOW(pos_Ball_y_low)
	ldi YH,HIGH(pos_Ball_y_low)
	ld tmp2,Y
	cpi tmp,0b10000000
	brcc change_pos_Ball_y_up // bei MSB==1 gehts nach oben
change_pos_Ball_y_down:
	add tmp2,eins
	adc tmp_main,null
	add tmp2,eins
	adc tmp_main,null	
	rjmp change_pos_Ball_y_finish

change_pos_Ball_y_up:
	sub tmp2,eins
	sbc tmp_main,null
	sub tmp2,eins
	sbc tmp_main,null
change_pos_Ball_y_finish:
	st Y,tmp2
	mov Spielball_pos_Start_low,tmp2
	ldi YL,LOW(pos_Ball_y_high)
	ldi YH,HIGH(pos_Ball_y_high)
	st Y,tmp_main
	mov Spielball_pos_Start_high,tmp_main
	ldi YL,LOW(v_Ball_y_max)
	ldi YH,HIGH(v_Ball_y_max)
	ld tmp2,Y //  Maxwerte in tmp2
	andi tmp2,0x7F // Geschwindigkeit maskieren 
	ldi YL,LOW(v_Ball_y_count)
	ldi YH,HIGH(v_Ball_y_count)
	st Y,tmp2 // Zähler wieder hochschrauben
	ret

//=== Movment Ball _x

movement_ball_x: // Wir haben 160 Pixel in insgesmat 40 Bytes ! Wir können aber nur byteweise adressieren, deshalb müssen wir die IMG entsprechend bearbeiten.
	// Die IMG haben ein min Byte Padding links und rechts als puffer , durch shiften können wir auch zwei Bit Schritte realisieren!!
	ldi YL,LOW(v_Ball_x_max)
	ldi YH,HIGH(v_Ball_x_max)
	ld tmp,Y
	//Ball Position bestimmen
	ldi YL,LOW(v_Ball_x_count)
	ldi YH,HIGH(v_Ball_x_count)
	ld tmp2,Y
	dec tmp2
	breq change_pos_Ball_x // wenn v_Ball_x_count=0
	st Y,tmp2
	ret

change_pos_Ball_x: //ändern Position einen nach rechts oder links und rsten count_x
	ldi YL,LOW(counter_Bit_Ball_x)
	ldi YH,HIGH(counter_Bit_Ball_x)
	ld tmp2,Y
	cpi tmp,0x80
	brcc change_pos_Ball_x_left // bei MSB==1 gehts nach links
change_pos_Ball_x_right: // bei MSB==0 gehts nach rechts
	add tmp2,eins  // rechts +1 auf counter_Bit_Ball_x
	rcall Ball_x_rotation_right // müssen SRAM_IMG ändern 
	rjmp change_pos_Ball_x_finish

change_pos_Ball_x_left:
	sub tmp2,eins // -1 auf counter_Bit_Ball
	rcall Ball_x_rotation_left // müssen SRAM_IMG ändern 
change_pos_Ball_x_finish:
	ldi YL,LOW(counter_Bit_Ball_x)
	ldi YH,HIGH(counter_Bit_Ball_x)
	st Y,tmp2 // Ball Position wieder sichern
	ldi YL,LOW(v_Ball_x_max)
	ldi YH,HIGH(v_Ball_x_max)
	ld tmp2,Y // neue Maxwerte in tmp2
	andi tmp2,0x7F // Geschwindigkeit maskieren 
	ldi YL,LOW(v_Ball_x_count)
	ldi YH,HIGH(v_Ball_x_count)
	st Y,tmp2 // Zähler wieder hochschrauben
	ret
//=== Ball_x_rotation_left

Ball_x_rotation_left:

	cpi tmp2,0 // wenn counter_Bit_Ball == 0
	breq ball_x_rotation_left_backrotation // ... wird zurück geschoben + pos_Byte_Ball_x dekrementiert (weil links) + tmp2 mit init_counter_ball_x geladen
	//laden und um eins nach rechts rotieren
	push tmp2
	push tmp
	ldi tmp2,BYTES_spielball
	ldi tmp_main,2 // 2 Bit nach links shiften
	
	ldi YL,LOW(sram_IMG_spielball_part_1)
	ldi YH,HIGH(sram_IMG_spielball_part_1)
	rcall left_rotation
	ldi YL,LOW(sram_IMG_spielball_part_2)
	ldi YH,HIGH(sram_IMG_spielball_part_2)
	rcall left_rotation
	ldi YL,LOW(sram_IMG_spielball_part_3)
	ldi YH,HIGH(sram_IMG_spielball_part_3)
	rcall left_rotation
	pop tmp
	pop tmp2 // Tmp2 unervändert wenn einfache Verschiebung
back_ball_x_rotation_left_four: // tmp2 enthält "init_counter_ball_x"  ,wenn auzs ball_x_rotation_left_backrotation kommend
	sei	
	ret

ball_x_rotation_left_backrotation:	
	push tmp
	push tmp2
	ldi tmp2,BYTES_spielball
	ldi tmp_main,6 // 6 Bit nach rechts shiften -> Ausgangsstellung
	
	ldi YL,LOW(sram_IMG_spielball_part_1)
	ldi YH,HIGH(sram_IMG_spielball_part_1)
	rcall right_rotation
	ldi YL,LOW(sram_IMG_spielball_part_2)
	ldi YH,HIGH(sram_IMG_spielball_part_2)
	rcall right_rotation
	ldi YL,LOW(sram_IMG_spielball_part_3)
	ldi YH,HIGH(sram_IMG_spielball_part_3)
	rcall right_rotation
	
	// Bytezähler einen weiter nach links (stetiger Übergang)
	ldi YL,LOW(pos_Byte_Ball_x)
	ldi YH,HIGH(pos_Byte_Ball_x)
	ld tmp,Y
	dec tmp
	st Y,tmp

	// Counter_Bit wieder auf Initwert
	pop tmp2
	ldi tmp2,init_counter_bit

	pop tmp
	rjmp back_ball_x_rotation_left_four

//=== Ball_x_rotation_right

Ball_x_rotation_right:

	cpi tmp2,8
	breq ball_x_rotation_right_backrotation
	//laden und um eins nach rechts rotieren
	push tmp2
	push tmp
	ldi tmp2,BYTES_spielball
	ldi tmp_main,2 // 2 Bit nach rechts shiften

	ldi YL,LOW(sram_IMG_spielball_part_1)
	ldi YH,HIGH(sram_IMG_spielball_part_1)
	rcall right_rotation
	ldi YL,LOW(sram_IMG_spielball_part_2)
	ldi YH,HIGH(sram_IMG_spielball_part_2)
	rcall right_rotation
	ldi YL,LOW(sram_IMG_spielball_part_3)
	ldi YH,HIGH(sram_IMG_spielball_part_3)
	rcall right_rotation
	pop tmp
	pop tmp2
back_ball_x_rotation_right_four:
	sei
	ret

ball_x_rotation_right_backrotation:
	push tmp2
	push tmp

	ldi tmp2,BYTES_spielball
	ldi tmp_main,6 // 8 Bit nach links shiften
	
	ldi YL,LOW(sram_IMG_spielball_part_1)
	ldi YH,HIGH(sram_IMG_spielball_part_1)
	rcall left_rotation
	ldi YL,LOW(sram_IMG_spielball_part_2)
	ldi YH,HIGH(sram_IMG_spielball_part_2)
	rcall left_rotation
	ldi YL,LOW(sram_IMG_spielball_part_3)
	ldi YH,HIGH(sram_IMG_spielball_part_3)
	rcall left_rotation
	
	// Bytezähler einen weiter nach rechts (stetiger Übergang)
	ldi YL,LOW(pos_Byte_Ball_x)
	ldi YH,HIGH(pos_Byte_Ball_x)
	ld tmp,Y
	inc tmp
	st Y,tmp

	// Counter_Bit wieder auf Initwert
	pop tmp2
	ldi tmp2,init_counter_bit

	pop tmp

	rjmp back_ball_x_rotation_right_four

//=== right_rotation
right_rotation:
//  Shiftet in einem tmp2 Byte großen Block ,der an Y beginnt, tmp_main Stellen nach rechts
//	YL,YH -> Startadresse von IMG
//  tmp2 -> Bytes in die geshiftet werden soll
//  tmp_main -> Wie oft
	push tmp_main
	push tmp
	clc // clear carry
	in SREG_SAVE,SREG // damit Carry Flag am Anfang leer ist
	push SREG_SAVE
right_rotation_loop1:
	push tmp2
	push YH
	push YL
right_rotation_loop2:	
	ld tmp,Y // holen uns den Wert
	out SREG,SREG_SAVE
	ror tmp // rotieren ihn ins carry
	in SREG_SAVE,SREG
	st Y+,tmp // und speichern ihn
	dec tmp2
	brne right_rotation_loop2
	pop YL // Startadresse neu laden
	pop YH // Startadresse neu laden
	pop tmp2 // Bytegröße neu laden
	dec tmp_main // Gesamt Wiederholungen um ein verringern.
	brne right_rotation_loop1	
	pop SREG_SAVE
	out SREG,SREG_SAVE
	sei
	pop tmp
	pop tmp_main
	ret
//==== left_rotation
left_rotation:
//  Shiftet in einem tmp2 Byte großen Block ,der an Y beginnt, tmp_main Stellen nach links
//	YL,YH -> Startadresse von IMG
//  tmp2 -> Bytes in die geshiftet werden soll
//  tmp_main -> Wie oft
	push tmp_main
	push tmp
	clc // clear carry
	in SREG_SAVE,SREG // damit Carry Flag am Anfang leer ist
	push SREG_SAVE
left_rotation_loop1:
	push tmp2 // so muss tmp2 nicht immer wieder neu geladen werden
	push YH
	push YL
	add YL,tmp2 // beginnen beim nach links shiften rechts vom Block !
	adc YH,null
left_rotation_loop2:	
	ld tmp,Y // holen uns den Wert
	out SREG,SREG_SAVE
	rol tmp
	in SREG_SAVE,SREG
	st Y,tmp
	sub YL,eins
	sbc YH,null
	dec tmp2
	brne left_rotation_loop2
	pop YL
	pop YH
	pop tmp2
	dec tmp_main
	brne left_rotation_loop1	
	pop SREG_SAVE
	out SREG,SREG_SAVE
	sei
	pop tmp
	pop tmp_main
	ret


//=== kollision_ball_x

kollision_ball_x: // bei einer Kollision mit einer Wand wird die x-Bewegungskomponente invertiert
	//Ball Position bestimmen
	ldi YL,LOW(pos_Byte_Ball_x)
	ldi YH,HIGH(pos_Byte_Ball_x)
	ld tmp,Y
	ldi YL,LOW(v_Ball_x_max)
	ldi YH,HIGH(v_Ball_x_max)
	ld tmp2,Y
	//Kollision mit Begrenzung
	// Gehen davon aus ,dass rechte Begrenzung > linke Begrenzung
	cpi tmp,rechte_seitenbegrenzung
	brcc kollision_ball_rechte_seitenbegrenzung // wenn tmp größer ist als rechte Begrenzung
	cpi tmp,linke_seitenbegrenzung
	brcs kollision_ball_linke_seitenbegrenzung // wenn tmp kleiner ist als linke Begrenzung
	ret

kollision_ball_linke_seitenbegrenzung:
	// wenn linke Seitenbegrenzung berüht ,muss der Ball dannach nach rechts fliegen ( kein inv aufgrund Ballbewegung,langsame)

	andi tmp2,0x7F // cleart das MSB
	dec tmp2 // schneller machen (max. auf 1 )
	brne kollision_ball_linke_seitenbegrenzung_skip1
	inc tmp2	
kollision_ball_linke_seitenbegrenzung_skip1:
	st Y,tmp2
	ldi YL,LOW(v_Ball_x_count)
	ldi YH,HIGH(v_Ball_x_count)
	st Y,eins // Zähler wieder hochschrauben
	ret

kollision_ball_rechte_seitenbegrenzung:
	andi tmp2,0x7F
	dec tmp2
	brne kollision_ball_rechte_seitenbegrenzung_skip1
	inc tmp2
kollision_ball_rechte_seitenbegrenzung_skip1:
	ori tmp2,0x80
	st Y,tmp2
	ldi YL,LOW(v_Ball_x_count)
	ldi YH,HIGH(v_Ball_x_count)
	st Y,eins // Zähler wieder hochschrauben
	ret


//=== Init_sram_data
init_sram_data:

	 // Ball Bewegungen mit pseudo-Zufallszahlen belegen
	 ldi YL,LOW(last_input_ply1) 
	 ldi YH,HIGH(last_input_ply1)
	 ld tmp,Y
	 ldi YL,LOW(last_input_ply2)
	 ldi YH,HIGH(last_input_ply2)
	 ld tmp2,Y
	 eor tmp2,tmp
	 tst tmp2
	 breq init_ball_y_up //wenn tmp2 == null
	 ldi tmp_main,0x80 // setzen MSB
	 rjmp init_ball_y_down_back
init_ball_y_up:
	 ldi tmp_main,0x00 // Clr MSB	
init_ball_y_down_back:
	 ldi YL,LOW(clocks_since_reset) 
	 ldi YH,HIGH(clocks_since_reset)
	 ld tmp,Y
	 ldi YL,LOW(counter_Bit_Ball_x)
	 ldi YH,HIGH(counter_Bit_Ball_x)
	 ld tmp2,Y
	 eor tmp2,tmp
	 andi tmp2,0x07 // nehmen nur die hinteren 3 Bits
	 add tmp2,eins // addieren eins hinzu ( damit nie NULL geladen wird)
	 or tmp_main,tmp2 // tmp_main ethält jetzt eine pseudo Zufallszahl für die y-Komponente		
	 ldi YL,LOW(v_Ball_y_max)
	 ldi YH,HIGH(v_Ball_y_max)
	 st Y,tmp_main 
	 ldi YL,LOW(v_Ball_y_count)
	 ldi YH,HIGH(v_Ball_y_count)
	 st Y,tmp_main //y-Komponente fertig

	 //jetzt noch x-Komponente
	 ldi YL,LOW(pos_Byte_player1) 
	 ldi YH,HIGH(pos_Byte_player1)
	 ld tmp,Y
	 ldi YL,LOW(pos_Byte_player2)
	 ldi YH,HIGH(pos_Byte_player2)
	 ld tmp2,Y
	 lsr tmp2
	 add tmp2,tmp
	 ldi YL,LOW(pos_Ball_y_low)
	 ldi YH,HIGH(pos_Ball_y_low)
	 ld tmp,Y
	 eor tmp2,tmp
	 rol tmp2
	 rol tmp2
	 andi tmp2,0x87 // Bitmaske um den Ball nicht 'unendlich' langsam werden zu lassen
	 add tmp2,eins // um NULL zu verhindern
	 mov tmp_main,tmp2
	 ldi YL,LOW(v_Ball_x_max)
	 ldi YH,HIGH(v_Ball_x_max)
	 st Y,tmp_main 
	 ldi YL,LOW(v_Ball_x_count)
	 ldi YH,HIGH(v_Ball_x_count)
	 st Y,tmp_main //x-Komponente fertig


	 //Spielball_ Startposition
	 ldi tmp,startpos_spielball_vertikal_low
	 mov Spielball_pos_Start_low,tmp
	 ldi tmp,startpos_spielball_vertikal_high
	 mov Spielball_pos_Start_high,tmp
 
	 // Datenbytes
	 ldi tmp,0
	 ldi YL,LOW(databyte_controller)
	 ldi YH,HIGH(databyte_controller)
	 st Y,tmp
 
	 ldi tmp,init_cmds_for_controller
	 ldi YL,LOW(cmds_for_controller)
	 ldi YH,HIGH(cmds_for_controller)
	 st Y,tmp

	 ldi tmp,19

	 ldi YL,LOW(pos_Byte_player1)
	 ldi YH,HIGH(pos_Byte_player1)
	 st Y,tmp
	 ldi YL,LOW(pos_Byte_player2)
	 ldi YH,HIGH(pos_Byte_player2)
	 st Y,tmp

	 // Ball Position x
	 ldi YL,LOW(pos_Byte_Ball_x) 
	 ldi YH,HIGH(pos_Byte_Ball_x)
	 ldi tmp,startpos_spielball_horizontal
	 st Y,tmp

	 // Bit_Counter
	 ldi tmp,init_counter_bit
	 ldi YL,LOW(counter_Bit_Ball_x) 
	 ldi YH,HIGH(counter_Bit_Ball_x)
	 st Y,tmp
	 ldi YL,LOW(counter_Bit_player1)
	 ldi YH,HIGH(counter_Bit_player1)
	 st Y,tmp
	 ldi YL,LOW(counter_Bit_player2)
	 ldi YH,HIGH(counter_Bit_player2)
	 st Y,tmp

	 // Ball Postion y
	 ldi YL,LOW(pos_Ball_y_low)
	 ldi YH,HIGH(pos_Ball_y_low)
	 st Y,Spielball_pos_Start_low
	 ldi YL,LOW(pos_Ball_y_high)
	 ldi YH,HIGH(pos_Ball_y_high)
	 st Y,Spielball_pos_Start_high
	 
	 // clocks_since_reset wieder auf null setzen
	 ldi YL,LOW(clocks_since_reset) 
	 ldi YH,HIGH(clocks_since_reset)
	 st Y,null
/*	 ldi YL,LOW(v_Ball_x_count)
	 ldi YH,HIGH(v_Ball_x_count)
	 st Y,tmp*/
	 ret

init_usart:

	// BAUD_RATE festlegen
	ldi tmp,BAUD_RATE_HIGH
	ldi tmp2,BAUD_RATE_LOW
	andi tmp,0x7F // löschen von URSEL (bei URSEL==0 -> schieben in UBRRH , bei URSEL==1 -> schieben in UCSRC ) 
	
	ldi YL,LOW(UBRR0H)
	ldi YH,HIGH(UBRR0H)

	st Y, tmp

	ldi YL,LOW(UBRR0L)
	ldi YH,HIGH(UBRR0L)

	st Y, tmp2

	// ENABLEN vom Receiver und Transmitter
	ldi tmp, (1<<RXEN0)|(1<<TXEN0)
	ldi YL,LOW(UCSR0B)
	ldi YH,HIGH(UCSR0B)
	st Y,tmp
	
	// Format festlegen
	clr tmp 
	ldi tmp,(1<<UCSZ01)|(1<<UCSZ00) // Bei Atmega88 getrennte Speicherplätze für UBBR0H und UCSR0C , (1<<URSEL) würde zu fehlverhalten führen!
	ldi YL,LOW(UCSR0C)
	ldi YH,HIGH(UCSR0C)
	st Y,tmp


	ret
           
uart_transmit:
	pop ZH
	pop ZL
	ldi YL,LOW(UCSR0A)
	ldi YH,HIGH(UCSR0A)
uart_transmit_loop:
	// Auf leeren TX-Buffer warten
	ld tmp,Y
	ldi tmp2,(1<<UDRE0)
	and tmp,tmp2
	cp tmp,tmp2
	brne uart_transmit_loop
	// zu übergebenden Sendeparameter senden !
	ldi YL,LOW(UDR0)
	ldi YH,HIGH(UDR0)
	pop tmp // übergebener Sendeparameter ( zu sendene Daten )
	st Y,tmp
	
	push ZL
	push ZH
	ret


// === Ende Funktionen von main

	// Byte |A,B|C,D|E,F|G,H|    // nur Farbe1 |01|01|01|01| == 0x55 // nur Farbe2 |10|10|10|10| == 0xAA // Schwarz = 0x00 // Mischung Farbe1+Farbe2 = 0xFF 
	// Byte | 1 | 2 | 3 | 4 |


  flash_linke_seitenbegrenzung: //eine Zeile == 40 Byte
	    .db 0x00,0x00,0x00,0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x00
  flash_mittellinie: //eine Zeile == 40 Byte
	    .db 0xCC,0x00,0x00,0x00,0x0F,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC
  flash_spielerbalken: // Balkenlänge == 1 Byte + jeweils 4 Bit an den Rändern
	    .db 0x00,0x00,0x05,0x55,0x50,0x00,0x00
  flash_spielball_part_1:
		.db 0x00,0x00,0x40,0x00	
  flash_spielball_part_2:
		.db 0x00,0x01,0x90,0x00
  flash_spielball_part_3:
		.db 0x00,0x05,0x54,0x00
   //Memory Map (definiert Zusammengehörigkeit der einzelnen Parts ..hier bei Spielball )
  Spielball_memory_map:  // 0x01 = part1 , 0x02 = part2 ,0x00 = part3 
		.db 0x01,0x01,0x02,0x02,0x02,0x02,0x02,0x01,0x01,0x00,0x00,0x00,0x01,0x01,0x01

 // SRAM 
 .dseg
 .org sram_anfang //beginnt erst bei 0x0060 (vorher sind noch Register und I/O Ports
 //Datensegment für Positionen

 .org databyte_controller
 .BYTE BYTES_Register
 .org cmds_for_controller 
 .BYTE BYTES_Register
 .org pos_Byte_player1
 .BYTE BYTES_Register
 .org pos_Byte_player2
 .BYTE BYTES_Register
 .org counter_Bit_player1
 .BYTE BYTES_Register
 .org counter_Bit_player2
 .BYTE BYTES_Register
 .org pos_Byte_Ball_x
 .BYTE BYTES_Register
 .org counter_Bit_Ball_x
 .BYTE BYTES_Register
 .org pos_Ball_y_low
 .BYTE BYTES_Register
 .org pos_Ball_y_high
 .BYTE BYTES_Register
 .org v_Ball_x_count
 .BYTE BYTES_Register
 .org v_Ball_y_count
 .BYTE BYTES_Register
 .org v_Ball_x_max
 .BYTE BYTES_Register
 .org v_Ball_y_max
 .BYTE BYTES_Register 
 .org last_input_ply1
 .BYTE BYTES_Register
 .org last_input_ply2
 .BYTE BYTES_Register
 .org clocks_since_reset
 .BYTE BYTES_Register
 // Images
 .org sram_IMG_spielerbalken1
 .BYTE BYTES_spielerbalken
 .org sram_IMG_spielerbalken2
 .BYTE BYTES_spielerbalken
 .org sram_IMG_spielball_part_1
 .BYTE BYTES_spielball
 .org sram_IMG_spielball_part_2
 .BYTE BYTES_spielball
 .org sram_IMG_spielball_part_3
 .BYTE BYTES_spielball
 // Worklines ( die werden letzendlich abgebildet )
 .org sram_linke_seitenbegrenzung // "    |                   "
 .BYTE BYTES_complete_line //was wenn Bytes hier nicht ausreichen-> scheiße egal :D
 .org sram_mittellinie
 .BYTE BYTES_complete_line
 .org sram_spielerbalken_player1
 .BYTE BYTES_complete_line
 .org sram_spielerbalken_player2
 .BYTE BYTES_complete_line
 .org sram_spielball_part_1
 .BYTE BYTES_complete_line
 .org sram_spielball_part_2
 .BYTE BYTES_complete_line
 .org sram_spielball_part_3
 .BYTE BYTES_complete_line




 // EPPROM
 .eseg
 .org 0000
 .db 1,2,3,4 
 .dw 1,2,3,4

 
