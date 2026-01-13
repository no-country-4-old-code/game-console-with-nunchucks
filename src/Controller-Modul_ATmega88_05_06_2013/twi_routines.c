#include <avr/io.h>
#include "twi_routines.h"
#include "lcd-routines.h"
#include <util/delay.h>
#include <string.h>

void twi_master_init(void){
	//Pins für TWI freischalten
//	uint8_t pins=(1<<TWI_SCL) | (1<<TWI_SDA) ;
//	TWI_DDR |= pins;
//	TWI_PORT &= ~pins; //alle aus
	
	TWBR=TWI_TWBR;
	TWSR=(TWI_TWPS & 0x03);
	_delay_ms(WAIT_AFTER_FREQ_ms);
}

void twi_master_transDaten(uint8_t daten)
{
	TWDR = daten;
	TWCR = (1<<TWINT) | (1<<TWEN);

	while (!(TWCR & (1<<TWINT))){;}; // Warten bis TWINT wieder 1
	
	if ((TWSR & 0xF8) != MT_DATA_ACK ){
		ERROR(TWSR & 0xF8);}
	
}

void twi_master_transStart(void){
	//Send Start Condition
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	
	while (!(TWCR & (1<<TWINT))){;}; // Warten bis TWINT wieder 1
	
	if ((TWSR & 0xF8) != START_ACK){
		ERROR(TWSR & 0xF8);}
}

void twi_master_transStop(void){
	
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
	//wARTen 
	while ((TWCR & (1<<TWINT))){;}; 
}

void twi_master_transAdresseSEND(uint8_t adresse){
	
	// Prüfen ob die Slaveadresse+W oder nur die Slaveadresse übergeben wurde
	
	if((adresse & 0x80) == 0){
		adresse= (adresse<<1) | ((TWI_SEND) << 0);
	}
	
	TWDR = adresse;
	TWCR = (1<<TWINT) | (1<<TWEN);
	
	while (!(TWCR & (1<<TWINT))){;}; // Warten bis TWINT wieder 1
	
	if ((TWSR & 0xF8) != MT_SLA_ACK){
		ERROR(TWSR & 0xF8);}
}

void twi_master_transAdresseRECEIVE(uint8_t adresse){
	
	// Prüfen ob die Slaveadresse+W oder nur die Slaveadresse übergeben wurde
	
	if((adresse & 0x80) == 0){
		adresse= (adresse<<1) | ((TWI_RECEIVE) << 0);
	}
	
	TWDR = adresse;
	TWCR = (1<<TWINT) | (1<<TWEN);

	while (!(TWCR & (1<<TWINT))){;}; // Warten bis TWINT wieder 1

	if ((TWSR & 0xF8) != MR_SLA_ACK){
	ERROR(TWSR & 0xF8);}
}

void twi_master_receiveData(uint8_t *ptr_data,uint8_t len_data){
	// noch im Aufbau (gehen davon aus ,dass jetzt schon was abzuholen ist
	// normalerweise bis zu MR_DATA_NACK
	// wissen aber nicht welchen Befehl wir wirklich schicken müssen blubb bla
	
	uint8_t tmp=0;
	uint8_t test_count=0;
	
	
	for(tmp=0;tmp<(len_data-1);tmp++){
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); // Sendet ACK nach Antwort
		while (!(TWCR & (1<<TWINT))){;}; // Warten bis TWINT wieder 1	

		*ptr_data=TWDR;
		ptr_data++;
	
		test_count++;
	}
		
	if(!(test_count<9)){
		ERROR(66);
	}
	
	TWCR = (1<<TWINT) | (1<<TWEN);		
	while(!(TWCR & (1<<TWINT)));
	*ptr_data=TWDR;
	ptr_data++;
}

void ERROR(uint8_t err){
/*	lcd_clear();
	lcd_setcursor(0,1);
	
	
	char errorlog[SIZE_TWI_MASTER_ERRORLOG];
	switch(err){
		
		case MT_SLA_NACK:
			strcpy(errorlog,"NACK , SLAVE ADRESSE [ Transmisson Mode]");
			break;
		case MT_DATA_NACK:
			strcpy(errorlog,"NACK , SLAVE DATA [ Transmisson Mode]");
			break;
		case MR_SLA_NACK:
			strcpy(errorlog,"NACK , SLAVE ADRESSE [ Receive Mode]");			
			break;
		case MR_DATA_NACK:
			strcpy(errorlog,"NACK , SLAVE DATA [ Receive Mode]");	
			break;
		case ARIB_LOST:
			strcpy(errorlog,"NACK , ARIB LOST");		
			break;
		default:
			strcpy(errorlog,"DEFAULT: CODE= ");
			errorlog[strlen("DEFAULT: CODE= ")]=err+48;
			errorlog[strlen("DEFAULT: CODE= ")+1]='\x00';
			break;
	}
	lcd_string(errorlog);			
	
					
	while(1){;};
	*/			
	}
	
