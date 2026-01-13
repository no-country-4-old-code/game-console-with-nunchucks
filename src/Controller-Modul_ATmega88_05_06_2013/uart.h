
#include <avr/io.h>
#include <string.h>



#ifndef F_CPU
#define F_CPU 21145000UL //Wartezeiten dürfen nicht zu kurz werden
#endif
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#define PIN_RX PD0
#define PIN_TX PD1

/*
Kurze Übersicht:
Es gibt folgende 6 Register : UDR (Daten lesen,schreiben) ,UCSRA ( StatusFlags ) , UCSRB ( ISR ) , UCSRC ( Statusregister,Formatfestlegung) ,UBRRL and UBRRH (Teiler für Takt)
Anmerkungen:
Schreiben in UDR -> sendet Daten
Lesen aus UDR -> lädt empfangene Daten ( zu schreibende Daten lesen geht nicht )
UBRRH und UCSRA besitzen die gleiche Spiecheradresse.
Bei Schreibzugriffen wird durch URSEL ausgewählt in welches Register geschrieben wird -> URSEL== 0 -> UBRRH , URSEL== 1 -> UCSRC

*/


void USART_Init( unsigned int ubrr);
unsigned char USART_Receive( void );
void USART_Transmit( unsigned char data );