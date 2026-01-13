/*
 * Project_C.c
 *
 * Created: 24.11.2012 17:46:01
 *  Author: mo
 */ 
//Include Header Dateien in
// C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.4.0.65\AVRToolchain\avr\include\avr


#define F_CPU 21145000UL

#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"
#include "twi_routines.h"
#include "funktions.h"


// Rst Knopf
#define RST_PORT PORTD
#define RST_PINS PIND
#define RST_DDR DDRD
#define RST_PIN PD5
#define RST_ACTIV 1 // High Aktiv

// Watchdog
#define BORDER_WATCHDOG 21 // Ein Timer ISR wird standardmäßig mit 60Hz ausgelöst sprich alle 16,6 ms -> Multiplexing geschieht alle 1,6 ms; Grenze wird mit 21 angegeben


// Nunchk-Mode auswählen ( Stick oder Beschleunigungssensor auslesen )
#define NUNCHK_MODE_PORT PORTD
#define NUNCHK_MODE_PINS PIND
#define NUNCHK_MODE_DDR DDRD
#define NUNCHK_MODE_PIN_PLY1 PD2
#define NUNCHK_MODE_PIN_PLY2 PD3
#define NUNCHK_MODE_STICK 1
#define NUNCHK_MODE_VSENSOR 0
#define NUNCHK_MODE_STICK_SEGMENT 0
#define NUNCHK_MODE_VSENSOR_SEGMENT 2


// 'Pong'-Ton Ausgabe
#define SOUND_PORT PORTD
#define SOUND_DDR DDRD
#define SOUND_PIN PD4
#define SOUND_AKTIV 1 // immer über einen Zyklus (also ~ 1/60 s aktiv -> außer beim aufhängen .... ) -> abfange routine in Timer-ISR

// SiebenSeg
#define SEG7_OUTPUT1_PORT PORTB
#define SEG7_OUTPUT2_PORT PORTD
#define SEG7_OUTPUT1_DDR DDRB
#define SEG7_OUTPUT2_DDR DDRD
#define SEG7_OUTPUT_PINS1  0b00111111//(1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB5)  //0b00111111 // PB0 bis PB5
#define SEG7_OUTPUT_PINS2  0b01000000//(1<< PD6)  //0b01000000 // PD6 als 7ter Pin
#define NUM_OF_SEG7_DISPLAYS 4
#define NUM_OF_NUMBERS4DISPLAY 10 // Bei Änderung das Array "seg7_umwandlung_zahl2seg7" anpassen !

#define SEG7_MUX_PORT PORTC
#define SEG7_MUX_DDR DDRC
#define SEG7_MUX_PINS 0x0F // PC3:PC0

//Timer0
#define TIMER0_PRESCALER 0x05
#define TIMER0_START_VALUE (255-35) // 35 Takte sollen gezählt werden -> ~1,66 ms


// Nunchk
#define NUNCHUCK_ADR 0x52

#define NUNCHUCK_INIT_1 0xF0 // 1
#define NUNCHUCK_INIT_2 0x55
#define NUNCHUCK_INIT_3 0xFB // 2
#define NUNCHUCK_INIT_4 0x00
#define NUNCHUCK_RST_PTR_RAM 0x00 // 3

#define NUNCHUCK_TIME_INIT_ms 100
#define NUNCHUCK_TIME_RAM_ms 1

#define NUM_OF_NUNCHK_DATA 6

// Für Nunchuck Multiplexer
#define MULTI_NUCHK_PIN PD7
#define MULTIPLI_NUCHK_PORT PORTD
#define MULTIPLI_NUCHK_DDR DDRD
#define TIME_AFTER_CHANGE_NUCHK_us 1
#define NUNCHK_1 0
#define NUNCHK_2 1
#define NUNCHK_CHANGE (1<<MULTI_NUCHK_PIN) // einfach eor mit Nunchk_change um  [ eor ==  ^  ]


// Bewerten einer Bewegung -> Kommunikationsdaten vom Nunchuck
/*nunchk_buffer[     1   ] | [    2    ] | [    3    ] | [    4   ] | [    5     ] | [             6            ] -> 6 Bytes!
 	Bedeutung:	Joystick X | Joystick Y  | V-Sensor X  | V-Sensor Y |  V-Sensor Z  | LSB von X,Y,Z und Knöpfe Z,C 		
*/
#define MOVEMENT_BLANK 0 // welcher Abschnitt wird als Bewegung gewertet
#define MOVEMENT_BORDER_RIGHT 127+30 // Grenzen ab wann Bewegung als Bewegung gezählt werden kann (rausfiltern von Rauschen,leichten Schwankungen)
#define MOVEMENT_BORDER_LEFT 127-30
#define MOVEMENT_BORDER_RIGHT_FAST 127+90
#define MOVEMENT_BORDER_LEFT_FAST 127-90
#define MOVEMENT_RIGHT_PLY1_FAST 0b00000110
#define MOVEMENT_RIGHT_PLY2_FAST 0b00110000
#define MOVEMENT_RIGHT_PLY1 0b00000101
#define MOVEMENT_RIGHT_PLY2 0b00101000
#define MOVEMENT_LEFT_PLY1_FAST	0b00000010
#define MOVEMENT_LEFT_PLY2_FAST	0b00010000
#define MOVEMENT_LEFT_PLY1	0b00000001
#define MOVEMENT_LEFT_PLY2	0b00001000
/*
Kommando für VGA-MODUL : 
Aufbau Protokoll ( C-Modul -> VGA-Modul ) : [ 7:6 | 5:3 | 2:0  ]  == [ 11=rst | BewegungPlayer2 | BewegungPlayer1 ]
*/
#define CMD4VGA_RESET 0b11000000

/*
Kommando für C-MODUL : 
Aufbau Protokoll ( VGA-Modul -> C-Modul ) : [ 7:2 | 1 | 0 ] == [ reserviert ( zu belegen mit "101010" ) | ÄnderungSpielstand2 | ÄnderungSpielstand1 ]
*/
#define CMD4C_NORMAL 0b10101000
#define CMD4C_MASK_POINT_PLY1 0x01
#define CMD4C_MASK_POINT_PLY2 0x02





void nunchuck_init(void);
void nunchuck_receiveData(uint8_t *buffer);
void display_7_segmentanzeige(void);

//globale Variabeln (volatile für ISR-Verwendbarkeit)

volatile uint8_t cmd_data=CMD4VGA_RESET; // enthält zu sendendes Kommando für das VGA-Modul
volatile uint8_t seg7_aktive_anzeige=0; // gibt an welche anzeige grade aktiv ist ( notwendig aufgrund multiplexing )
volatile uint8_t watchdog_counter=0; //Reagiert wenn lange keine RSR232-ISR mehr ausgeführt wurde, um Deadlock zu verhindern. 
uint8_t volatile seg7_zahlenwerte[NUM_OF_SEG7_DISPLAYS]={1,2,3,4}; // die jeweiligen Zahlenwerte ,die angezeigt werden sollen ( 1:0 [Player1] ; 3:2 [Player2] )
uint8_t volatile seg7_umwandlung_zahl2seg7[NUM_OF_NUMBERS4DISPLAY]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F}; // High-aktiv [ hat eine Treiberstufe dahinter ] , Zahlen 0 bis 9
uint8_t volatile seg7_multiplex_werte[NUM_OF_SEG7_DISPLAYS]={0x07,0x0B,0x0D,0x0E}; // PC3:PC0 // LOW Aktiv



uint8_t tmp=0;



//char nunchk_output[42];


int main(void)
{	
	cli();
	// Init RST_PIN
	RST_DDR&=(~(1<<RST_PIN)); // ist ein Eingang !
	RST_PORT&=(~(1<<RST_PIN));
	
	// Init Sound Pin
	SOUND_PORT&=~(SOUND_AKTIV<<SOUND_PIN); // Startet im inaktiven Zustand
	SOUND_DDR|=(1<<SOUND_PIN);
	
	// Init Nunchk_mode Pins
	NUNCHK_MODE_PORT&=~((1<< NUNCHK_MODE_PIN_PLY1 ) | ( 1<< NUNCHK_MODE_PIN_PLY2));
	NUNCHK_MODE_DDR&=~((1<< NUNCHK_MODE_PIN_PLY1 ) | ( 1<< NUNCHK_MODE_PIN_PLY2));
	
	// Init Siebensegmentanzeige + deren Multiplexer
	SEG7_OUTPUT1_DDR|=SEG7_OUTPUT_PINS1;
	SEG7_OUTPUT2_DDR|=SEG7_OUTPUT_PINS2;
	SEG7_MUX_DDR|=SEG7_MUX_PINS;
	SEG7_OUTPUT1_PORT&=~SEG7_OUTPUT_PINS1;
	SEG7_OUTPUT2_PORT|=SEG7_OUTPUT_PINS2;
	SEG7_MUX_PORT&=~SEG7_MUX_PINS;
	
	// Init TimerISR
	TCCR0B=TIMER0_PRESCALER;
	TCNT0=TIMER0_START_VALUE;
	TIMSK0|=(1<<TOIE0); // ISR enablen
	
	// Init des Multiplexers für Nunchuck
	tmp = (1 << MULTI_NUCHK_PIN);
	MULTIPLI_NUCHK_DDR |= tmp;
	MULTIPLI_NUCHK_PORT &= ~tmp;
	
	// Init der Nunchuks
	twi_master_init();	

	MULTIPLI_NUCHK_PORT&=(~(1<<MULTI_NUCHK_PIN)); //ändert des Multiplexers auf Nuchk1
	_delay_ms(TIME_AFTER_CHANGE_NUCHK_us);
	nunchuck_init(); //Nunchuk in decodierten Modus versetzen
	MULTIPLI_NUCHK_PORT^=NUNCHK_CHANGE;
	_delay_ms(TIME_AFTER_CHANGE_NUCHK_us); // Berücksichtigen der Rise and Fall Time des MultiplexerTransistors
	nunchuck_init(); //Nunchuk in decodierten Modus versetzen
	MULTIPLI_NUCHK_PORT^=NUNCHK_CHANGE;
	_delay_ms(TIME_AFTER_CHANGE_NUCHK_us); // Berücksichtigen der Rise and Fall Time des MultiplexerTransistors



	USART_Init(42);
	//für RX-ISR	
	UCSR0B|= (1<<RXCIE0);

	display_7_segmentanzeige();
	
	cmd_data=CMD4VGA_RESET; //Vorher Reset damit auch alles Schön aussieht
	USART_Transmit(cmd_data);
	cmd_data=0;
	
	// Warten auf Knopfdruck
	tmp=RST_PINS & (RST_ACTIV<<RST_PIN);
	
	while(tmp!=(RST_ACTIV<<RST_PIN)){
		tmp=RST_PINS & (RST_ACTIV<<RST_PIN);
	};		
		
	// Reseten vom VGA-Modul 
	cmd_data=CMD4VGA_RESET;
	USART_Transmit(cmd_data);
	cmd_data=0;
	sei();

	
	while(1)
    { 		
	display_7_segmentanzeige();
	if(watchdog_counter>=BORDER_WATCHDOG){
		cmd_data=MOVEMENT_BLANK;  //Vermeidung von Dead-Locks
		USART_Transmit(cmd_data); //ein dummy losschicken, um anderen anzutriggern (der wartet auf eine Antwort) 
		watchdog_counter=0;
		}
	};
	
}


// Funktion zum Ansteuern der 7 Seg Anzeige
// langer Interrupt ,wobei nur anfang ununterbrechbar ist 
ISR(USART_RX_vect){
	cli();
	
	uint8_t new_data;
	uint8_t cmd_prototyp=MOVEMENT_BLANK;
	uint8_t nunchk_buffer[NUM_OF_NUNCHK_DATA];
	uint8_t nunchk_mode=NUNCHK_MODE_STICK_SEGMENT; // Default auf Stick
	
	new_data=UDR0; // liest Zeichen und speichert es in new_data
	
	SOUND_PORT&=~(SOUND_AKTIV<<SOUND_PIN); // Sound aus

	USART_Transmit(cmd_data); 	// sendet Bewegungsbefehle (vorher schon beschlossen)
	
	// wertet Zeichen aus und greift auf 7-Segmentdata zu -> ( 1:0 [Player1] ; 3:2 [Player2] )
	tmp=new_data;
	sei();
	watchdog_counter=0;
	tmp=new_data;
	
	tmp&=CMD4C_MASK_POINT_PLY1;
	//tmp/=CMD4C_MASK_POINT_PLY1; // ergibt 1 bei Übereinstimmung,ansosnten null
// Punkt Sound
	if(tmp==1){	
		SOUND_PORT|=(SOUND_AKTIV<<SOUND_PIN);
	}
	seg7_zahlenwerte[0]+=tmp;
	tmp=seg7_zahlenwerte[0]/NUM_OF_NUMBERS4DISPLAY; // 1 beim übereinstimmen
	seg7_zahlenwerte[0]=seg7_zahlenwerte[0]%NUM_OF_NUMBERS4DISPLAY; // 0 beim Ende des Zahlenspektrums
	seg7_zahlenwerte[1]+=tmp; // wird ein hochgezählt
	seg7_zahlenwerte[1]=seg7_zahlenwerte[1]%NUM_OF_NUMBERS4DISPLAY; // 0 beim Ende des Zahlenspektrums
		
	tmp=new_data;

	tmp&= CMD4C_MASK_POINT_PLY2;
	tmp=tmp >> 1;
	//tmp/=CMD4C_MASK_POINT_PLY2; // ergibt 1 bei Übereinstimmung,ansosnten null
// Punkt Sound
	if(tmp==1){	
		SOUND_PORT|=(SOUND_AKTIV<<SOUND_PIN);
	}
	seg7_zahlenwerte[2]+=tmp;
	tmp=seg7_zahlenwerte[2]/NUM_OF_NUMBERS4DISPLAY; // 1 beim übereinstimmen
	seg7_zahlenwerte[2]=seg7_zahlenwerte[2]%NUM_OF_NUMBERS4DISPLAY; // 0 beim Ende des Zahlenspektrums
	seg7_zahlenwerte[3]+=tmp; // wird ein hochgezählt
	seg7_zahlenwerte[3]=seg7_zahlenwerte[3]%NUM_OF_NUMBERS4DISPLAY; // 0 beim Ende des Zahlenspektrums
	
	
// nur bei Rst zurücksetzen

		
	
//Holen && Auswerten der Daten
/*nunchk_buffer[     1   ] | [    2    ] | [    3    ] | [    4   ] | [    5     ] | [             6            ]
		Bedeutung:	Joystick X | Joystick Y  | V-Sensor X  | V-Sensor Y |  V-Sensor Z  | LSB von X,Y,Z und Knöpfe Z,C 
*/

// MOVEMENT_UNIT -> JOYSTICKX,JOYSTICKY,V-SENSORX,V-SENSORY,V-SENSOR ;

	display_7_segmentanzeige();	

//=== Mode für Nunchk1 bestimmen === [Ob Stick oder V-Sensor für Bewegung relevant -> wird extern über Jumper netschieden ]	

	tmp=NUNCHK_MODE_PINS;
	tmp&=(1<<NUNCHK_MODE_PIN_PLY1);
	if(tmp==(NUNCHK_MODE_STICK<<NUNCHK_MODE_PIN_PLY1)){
		nunchk_mode=NUNCHK_MODE_STICK_SEGMENT;
	}
	else{
		nunchk_mode=NUNCHK_MODE_VSENSOR_SEGMENT;
	}
	
	
//=== Daten vom Nunchk 1 holen ===
	nunchuck_receiveData(&nunchk_buffer[0]);
	
	MULTIPLI_NUCHK_PORT^=NUNCHK_CHANGE;
	_delay_ms(TIME_AFTER_CHANGE_NUCHK_us); // Berücksichtigen der Rise and Fall Time des MultiplexerTransistors

	if(nunchk_buffer[nunchk_mode]>=(MOVEMENT_BORDER_RIGHT_FAST)){
		cmd_prototyp|=MOVEMENT_RIGHT_PLY1_FAST; // je nach tmp wird nur die richtige Stelle übernommen
	}
	else if(nunchk_buffer[nunchk_mode]>=(MOVEMENT_BORDER_RIGHT))
	{	cmd_prototyp|=MOVEMENT_RIGHT_PLY1; // je nach tmp wird nur die richtige Stelle übernommen
	}
	
	else if(nunchk_buffer[nunchk_mode]<=(MOVEMENT_BORDER_LEFT_FAST)){
		cmd_prototyp|=MOVEMENT_LEFT_PLY1_FAST;
	}
	else if(nunchk_buffer[nunchk_mode]<=(MOVEMENT_BORDER_LEFT)){
		cmd_prototyp|=MOVEMENT_LEFT_PLY1;
	}


//===== fertig mit Nunchk1 , weiter zu Nunchk2 =====
	display_7_segmentanzeige();
//=== Mode für Nunchk2 bestimmen ===
	tmp=NUNCHK_MODE_PINS;
	tmp&=(1<<NUNCHK_MODE_PIN_PLY2);
	if(tmp==(NUNCHK_MODE_STICK<<NUNCHK_MODE_PIN_PLY2)){
		nunchk_mode=NUNCHK_MODE_STICK_SEGMENT;
	}
	else{
		nunchk_mode=NUNCHK_MODE_VSENSOR_SEGMENT;
	}


//=== Daten vom Nunchk 2 holen ===	
	nunchuck_receiveData(&nunchk_buffer[0]);
	MULTIPLI_NUCHK_PORT^=NUNCHK_CHANGE;
	_delay_us(TIME_AFTER_CHANGE_NUCHK_us); // Berücksichtigen der Rise and Fall Time des MultiplexerTransistors

	if(nunchk_buffer[nunchk_mode]>=(MOVEMENT_BORDER_RIGHT_FAST)){
		cmd_prototyp|=MOVEMENT_RIGHT_PLY2_FAST; // je nach tmp wird nur die richtige Stelle übernommen
	}
	else if(nunchk_buffer[nunchk_mode]>=(MOVEMENT_BORDER_RIGHT)){
		cmd_prototyp|=MOVEMENT_RIGHT_PLY2; // je nach tmp wird nur die richtige Stelle übernommen	
		}
	
	else if(nunchk_buffer[nunchk_mode]<=(MOVEMENT_BORDER_LEFT_FAST)){
		cmd_prototyp|=MOVEMENT_LEFT_PLY2_FAST;
	}
	
	else if(nunchk_buffer[nunchk_mode]<=(MOVEMENT_BORDER_LEFT)){
			cmd_prototyp|=MOVEMENT_LEFT_PLY2;	
		}

//=== fertig mit Nunchks ===

//=== Rst Pin überprüfen ===
	tmp=RST_PINS;
	tmp&=(RST_ACTIV<<RST_PIN);
		
	if(tmp==(RST_ACTIV<<RST_PIN)){
			//reset
			cmd_prototyp=CMD4VGA_RESET;
			for(tmp=0;tmp<NUM_OF_SEG7_DISPLAYS;tmp++){
				seg7_zahlenwerte[tmp]=0;
			}
		}

//=== fertig -> cmd fürs nächste mal erstellen ===		
	
	
	cmd_data=cmd_prototyp; // Arbeitet mit cmd_prototyp schneller ,da wahr. in Registern 
	cmd_prototyp=0;


	
}	


ISR (TIMER0_OVF_vect){
	cli();
	TCNT0=TIMER0_START_VALUE;
	seg7_aktive_anzeige++;
	watchdog_counter++;
	if(seg7_aktive_anzeige>=NUM_OF_SEG7_DISPLAYS){
		seg7_aktive_anzeige=0;
		}

	sei();
}
void nunchuck_init(void){
	twi_master_transStart();
	twi_master_transAdresseSEND(NUNCHUCK_ADR);
	twi_master_transDaten(NUNCHUCK_INIT_1); // Adresse nunchk Register
	twi_master_transDaten(NUNCHUCK_INIT_2); // Wert für nunchk Register
	twi_master_transStop();
	
	_delay_ms(NUNCHUCK_TIME_INIT_ms);

	
	twi_master_transStart();
	twi_master_transAdresseSEND(NUNCHUCK_ADR);
	
	twi_master_transDaten(NUNCHUCK_INIT_3);
	twi_master_transDaten(NUNCHUCK_INIT_4);
	twi_master_transStop();
	
	
	
	_delay_ms(NUNCHUCK_TIME_INIT_ms);
	
}

void nunchuck_receiveData(uint8_t *buffer){
	
	twi_master_transStart();
	twi_master_transAdresseSEND(NUNCHUCK_ADR);
	twi_master_transDaten(NUNCHUCK_RST_PTR_RAM); // Neu setzen des RAM Pointers
	twi_master_transStop();
	
	_delay_ms(NUNCHUCK_TIME_RAM_ms);
	
	twi_master_transStart();
	twi_master_transAdresseRECEIVE(NUNCHUCK_ADR);
	twi_master_receiveData(buffer,NUM_OF_NUNCHK_DATA);
	twi_master_transStop();
}

void display_7_segmentanzeige(void){ 
	// gibt Werte entsprechend auf der sieben_segmentanzeige aus
	uint8_t tmp=0;
	
	tmp=SEG7_MUX_PORT;
	tmp&=~SEG7_MUX_PINS; // löschen der alten Pinwerte
	tmp=tmp | ((seg7_multiplex_werte[seg7_aktive_anzeige]) & SEG7_MUX_PINS); // draufspielen neuer Werte
	SEG7_MUX_PORT=tmp;

	tmp=SEG7_OUTPUT1_PORT;
	tmp&=~SEG7_OUTPUT_PINS1;
	tmp|=(seg7_umwandlung_zahl2seg7[seg7_zahlenwerte[seg7_aktive_anzeige]]&SEG7_OUTPUT_PINS1);
	SEG7_OUTPUT1_PORT=tmp;
	tmp=SEG7_OUTPUT2_PORT;
	tmp&=~SEG7_OUTPUT_PINS2;
	tmp|=(seg7_umwandlung_zahl2seg7[seg7_zahlenwerte[seg7_aktive_anzeige]]&SEG7_OUTPUT_PINS2); // geht weil PD6 genau das 7ze Bit deckt (ansosnten noch zusätzliche SchiebeOP)
	SEG7_OUTPUT2_PORT=tmp;
}