
//Für die Einstellung der Frequenz
#ifndef F_CPU
#define F_CPU 22114500UL //Wartezeiten dürfen nicht zu kurz werden
#endif

#define TWI_TWBR 25
#define TWI_TWPS 1
#define SCL_FREQ 100 // (in kHz) -> SCL_FREQ= F_CPU/[16+2*(TWI_TWBR)*4^(TWI_TWPS)] __nach ausführen von initFrequenz
#define WAIT_AFTER_FREQ_ms 10

//Größe Errorlog
#define SIZE_TWI_MASTER_ERRORLOG 42

//TWI PORT
#define TWI_DDR DDRC
#define TWI_PORT PORTC
#define TWI_SCL PC5
#define TWI_SDA PC4

//ADRESSE [ Bits 7:1 ] , DIRECTION [ Bit 0 ]
#define TWI_RECEIVE 1
#define TWI_SEND 0

//SLAVE RESPONSE
#define START_ACK 0x08
#define RESTART_ACK 0x10
#define MT_SLA_ACK 0x18
#define MT_SLA_NACK 0x20
#define MT_DATA_ACK 0x28
#define MT_DATA_NACK 0x30
#define ARIB_LOST 0x38
#define MR_SLA_ACK 0x40
#define MR_SLA_NACK 0x48
#define MR_DATA_ACK 0x50
#define MR_DATA_NACK 0x58

void twi_master_init(void);
void twi_master_transDaten(uint8_t daten);
void twi_master_transStart(void);
void twi_master_transStop(void);
void twi_master_transAdresseSEND(uint8_t adresse);
void twi_master_transAdresseRECEIVE(uint8_t adresse);
void twi_master_receiveData(uint8_t *ptr_buffer,uint8_t len_data);
void ERROR(uint8_t err);