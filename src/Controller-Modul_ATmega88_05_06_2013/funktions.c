// Allgemeine Fkten
#include <avr/io.h>

// fügt einem String eine uint8_t Nummer hinzu ( Umwandlung in Ascii )
void uint8_add_String(char* ptr_string,uint8_t number){
	uint8_t tmp=0,divide=100;
	
	tmp=number/divide;
	*ptr_string=tmp+48;
	ptr_string++;
		
		do{
		tmp=number%divide;
		divide/=10;
		tmp/=divide;
		*ptr_string=tmp+48;
		ptr_string++;				
	}while(divide>1);
	
}