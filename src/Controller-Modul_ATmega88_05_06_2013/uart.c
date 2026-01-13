
#include "uart.h"

/*
Kurze Übersicht:
Es gibt folgende 6 Register : UDR (Daten lesen,schreiben) ,UCSRA ( StatusFlags ) , UCSRB ( ISR ) , UCSRC ( Statusregister,Formatfestlegung) ,UBRRL and UBRRH (Teiler für Takt)
Anmerkungen:
Schreiben in UDR -> sendet Daten
Lesen aus UDR -> lädt empfangene Daten ( zu schreibende Daten lesen geht nicht )
UBRRH und UCSRA besitzen die gleiche Spiecheradresse.
Bei Schreibzugriffen wird durch URSEL ausgewählt in welches Register geschrieben wird -> URSEL== 0 -> UBRRH , URSEL== 1 -> UCSRC

*/

//ISRs -> Bei ISR drauf achten Variablen auf die in der ISR zugegriffen werden soll mit volatile zu deklarieren -> sonst geht die ISR nur von lokalen Vars aus
// { werden dann vorher drauf gepusht ähnlich der Parameter beim Funtkionsaufruf }
/*
ISR(USART_RXC_vect) // bei Datenzugriff auf
{	
}

*/

void USART_Init( unsigned int ubrr)
{
	/* Setzen der Baudrate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Enablen vom Receiver und Transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Format festlegen : UCSRC [ Bit7=URSEL | Bit6=ModeSelect | Bit 5:4 Parity Mode | Bit 3 Stop Bit Select | Bit 2:0 Character Size ] 
	Beispiel:
	 ModeSelect=0b0 ( Asyncro ) , ParityMode=0b00 (keine) , StopBit=0b0 (keins) , CharcterSize=0b011 ( 8 Datenbits )
	(Dem letzten Bit kommt beim SynchroBetrieb eine andere Bedeutung zu)
	*/
	UCSR0C = (3<<UCSZ00);
	
	//für RX-ISR
	//UCSRB|= (1<<RXCIE);
	
}

// empfangen ( auch mögliche über Interrupt -> RXCIE in UCSRB enablen 
// ( ISR wird immer ausgelöst,wenn Daten empfangen [RXC Bit in UCSRA wird gesetzt-> ISR ausgelöst und RXC Bit hardwaremäßig gecleant] ) 
unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	/* Get and return received data from buffer */
	return UDR0;
} 

// senden ( auch mögliche über Interrupt -> TXCIE in UCSRB enablen )
// Achtung: Weitere Interrupt möglich und zwar der "Register ist empty" Interrupt -> UDRIE in UCSRB enablen 
// Für beide am besten nochmal in die Anleitung gucken ,ob die Flags evt manuell zu löschen sind
void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}