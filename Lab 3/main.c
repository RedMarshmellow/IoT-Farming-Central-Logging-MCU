#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <util/delay.h>
#include <avr/wdt.h>

#define F_CPU 8000000 //internal clock frequency, used for delay subroutines
#define Baud_Config  51//baud rate = 9600 and frequency of 8 MHz, 0x33 = 51
#define RESET_R 0x00
#define REPEAT_R 0xE0
#define ACKNOWLEDGE 0x40
#define LOG_R 0x20
#define CRC_GEN 0x35

unsigned char data[3200]; //used to store logged data
unsigned short dP = 0; //pointer to keep track of logged data
char user_input_buffer [5]; 
char user_input_buffer_ptr = 0;
char user_output_buffer [255];
char user_output_buffer_ptr = 0;
char bluetoothSending = 0;
char user_output_buffer_size = 0;
unsigned char sensor_input_buffer [2];
unsigned char sensor_input_buffer_ptr = 0;
unsigned char TOS = 0; //stack representation
unsigned char TOS_FULL = 0; //keeps track of stack status
unsigned short timeout = 0; //time out counter
char saved; // if timeout counter are already saved in eeprom
char wdSaved; // if watchdog counter settings are already saved in eeprom
char watchdogSetting;

void start_init();
void enableWD(); 
void sys_config();
void user_WD();
void sensor_init();
void user_transmit(char[]);
void user_transmit_handler(unsigned char); 
char eeprom_read(short);
void eeprom_write(unsigned short,char);
unsigned char crc3(unsigned char); 
unsigned char crc_check3(unsigned char); 
unsigned char crc_check11(unsigned char); 
void sensor_transmit(unsigned char); 
void timer_start(char); 
void timer_stop(char);
void treat_data(unsigned char);
void log_data(unsigned char); 
void service_readout(char);
void ascii_convert(unsigned char, unsigned char *);
unsigned char hex_convert(unsigned char[]);
void ConfigMasterWD();
void ConfigSlaveWD();

int main(void)
{
	eeprom_write(0x00, 0xFF);
	eeprom_write(0x01, 0x00);
	eeprom_write(0x02, 0x00);
	start_init();
	//enableWD();
	while (1){
		sleep_enable();
		sleep_cpu();
	}
}

void start_init(){
	sys_config();
	user_WD();
	sensor_init();
	user_transmit("\rEnter choice (and period): 1-Mem Dump 2-Last Entry 3-Restart \0");
}

void sys_config(){
	DDRB = 0x01; //watchdog timer indicator
	MCUCR = 0x80; //XMEM mode
	
	// USART 0, BLUETOOTH
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00); // setting data width to 8
	UBRR0H = (Baud_Config>>8); // setting baud rate to 9600 by setting UBBR
	UBRR0L = Baud_Config;
	UCSR0B = (1<<TXEN0) | (1<<RXCIE0)| (1<<TXCIE0) | (1<<RXEN0); // enable transmitter, receiver, and receive and transmit complete interrupts


	// USART 1, XBEE
	UCSR1C = (1<<UCSZ11)|(1<<UCSZ10); // setting data width to 8
	UBRR1H = (Baud_Config>>8); // setting baud rate to 9600
	UBRR1L = Baud_Config;
	UCSR1B = (1<<TXEN1) | (1<<RXCIE1) | (1<<RXEN1);; // enable transmitter, receiver, and receive and transmit complete interrupts

	sei(); // enable global interrupts
}

void user_WD(){
	saved = eeprom_read(0x00); //check if already configured
	wdSaved = saved;
	ConfigSlaveWD();
	ConfigMasterWD();
	user_transmit("Saving Settings");
	//write settings to eeprom
	eeprom_write(0x00,0x00);
	eeprom_write(0x01,(timeout&0xFF00)>>8);
	eeprom_write(0x02,(timeout&0xFF00));
	eeprom_write(0x03,watchdogSetting);
	timeout = 65536 - timeout;
}

void ConfigMasterWD(){
	watchdogSetting = eeprom_read(0x02);
	
	if(wdSaved){
		user_transmit("Please enter the watchdog timer duration wanted in (0-8)s (0 to disable): ");
	}
	while(wdSaved){
		sleep_enable();
		sleep_cpu();
	}
}

void ConfigSlaveWD(){
	unsigned short timer = 0;
	for(char i = 0; i < 2; i++){ //read last timer from eeprom
		timer |= eeprom_read(0x01 + i);
		if(i == 0)
		timer = timer<<8;
	}
	if(saved){ //if bytes are set, meaning that the settings aren't on eeprom
		user_transmit("Please enter the timeout duration wanted in (0-8)s (0 to disable): ");
		timeout = 0;
	}
	else{ //if saved, use saved settings
		timeout = timer;
	}
	while(saved){ //wait until user is done
		sleep_enable();
		sleep_cpu();
	}
}

char eeprom_read(short address){
	while((EECR & (1<<EEWE)) == 2)
	_delay_ms(1);
	
	EEARH = address&0xFF00;
	EEARL = address&0x00FF;
	EECR = (1<<EERE);
	return EEDR;
}

void eeprom_write(unsigned short address,char data){
	while((EECR & (1<<EEWE)) == 2)
	_delay_ms(1);
	
	EEARH = address&0xFF00;
	EEARL = address&0x00FF;
	
	EEDR = data;
	EECR = (1<<EEMWE);
	EECR = (1<<EEWE);
}

void sensor_init(){
	TOS = crc3(RESET_R);
	TOS_FULL = 1;
	sensor_transmit(TOS);
	timer_start(0); //start timeout timer
	user_transmit("Sensors Resetting");
}

unsigned char crc3(unsigned char msg){
	unsigned char crc = msg & 0xE0;// isolate first 3 bits, 11100000
	
	crc = crc>>2; // align with generator
	if( crc >= 0x20)
	crc = crc^CRC_GEN;

	for( int i = 0; i < 2; i++) {
		crc = crc<<1;
		if( crc >= 0x20)
		crc = crc^CRC_GEN;
		
	}
	
	msg |= crc;
	return msg;
}

unsigned char crc_check3(unsigned char msg){
unsigned char true_crc = crc3(msg);
unsigned char return_value = 0x00;
if ( true_crc == msg )
return_value = 0xFF;

return return_value;
}

unsigned char crc_check11(unsigned char command){
unsigned char crc_bits = command&0x1F;
unsigned char data = TOS;
unsigned char temp = 0;


command = command & 0xE0;


temp = data & 0b11;


temp = temp<<6;


data = data>>2;

if( data >= 0x20 )
data ^= CRC_GEN;

data = data<<1 | temp>> 7;
temp = temp<<1; 
if( data >= 0x20 )
data ^= CRC_GEN;


data = data<<1 | temp>>7; 
if( data >= 0x20 )
data ^= CRC_GEN;


for (int i=0; i < 8; i++){
	data = data<<1 | command>> 7;
	command = command<<1;		
	if ( data >= 0x20 )
	data ^= CRC_GEN;
}

unsigned char return_value = 0x00;

if (data == crc_bits)
return_value = 0xFF;

return return_value;
}

void sensor_transmit(unsigned char packet){ 
	while(!(UCSR1A & (1<<UDRE1)))
	_delay_ms(1);
	
	UDR1 = packet;
}

void timer_start(char timer){ //starts timer 1 or 3
	if(timer == 0 && timeout > 0){
		TCNT1H = (timeout&0xFF00)>>8;
		TCNT1L = timeout&0x00FF;
		TCCR1A = 0;
		TCCR1B = (1<<CS10) | (1<<CS12);
		TIMSK = (1<<TOIE1);
	}
	// timeout timer 3
	else if(timeout > 0){
		TCNT3H = (timeout&0xFF00)>>8;
		TCNT3L = timeout&0x00FF;
		TCCR3A = 0;
		TCCR3B = (1<<CS30) | (1<<CS32);
		ETIMSK = (1<<TOIE3);
	}
}

void timer_stop(char timer){ //stops timer 1 or 3
	if(timer == 0){
		TCCR1B &= ~(1<<CS10);
		TCCR1B &= ~(1<<CS12);
		TIMSK &= ~(1<<TOIE1);
	}
	else{
		TCCR3B &= ~(1<<CS30);
		TCCR3B &= ~(1<<CS32);
		ETIMSK &= ~(1<<TOIE3);
	}
}

void user_transmit(char msg[]){
	while (bluetoothSending) //wait until bluetooth is done sending
	{
		sleep_enable();
		sleep_cpu();
	}
	bluetoothSending = 1; //signifies active transmition, others need to wait
	strcpy(user_output_buffer, msg); //move to buffer
	user_output_buffer_size = strlen(msg)+1; 
	user_output_buffer[user_output_buffer_size-1] = '\0';
	user_output_buffer_ptr = 0; //initialize pointer
	user_transmit_handler(' ');	 //activate tx interrupt
}

void user_transmit_handler(unsigned char msg){ //handles sending data to user
	while(!(UCSR0A & (1<<UDRE0)))
	_delay_ms(1);
	
	UDR0 = msg;
}

void treat_data(unsigned char packet){ //sensor input treating 
	sensor_transmit(packet);
	if (packet&(1<<7)){ //if data packet
		TOS = packet;
		TOS_FULL = 1;
		timer_start(1);
	}else{  //if command packet
		timer_stop(1);
		if(TOS_FULL){ //if TOS has data packet
			unsigned char result = crc_check11(packet);
			if (result == 0xFF){ // crc check 11 passed
				if((packet&0xE0) == LOG_R){ //if log request
					log_data(TOS & 0x1F);
					TOS_FULL= 0;
					sensor_transmit(crc3(ACKNOWLEDGE));
				}else{
					sensor_init();
				}
			}else{ //crc check 11 failed
			TOS_FULL = 0;
			sensor_transmit(0xAA);
			sensor_transmit(crc3(REPEAT_R));
		}
	}else{ //if TOS has no data packet
	unsigned char result = crc_check3(packet);
	if(result == 0xFF){ //crc 3 passed
		if((packet&0xE0) == ACKNOWLEDGE){ //if acknowledge packet
			TOS_FULL = 0;
			timer_stop(0);
			}else{ //if not acknowledge packet
			if((packet&0xE0) == REPEAT_R){
				if (TOS_FULL)
					sensor_transmit(TOS);
			}else{
			sensor_init();}
			}
		}else{ //if crc 3 failed
		sensor_transmit(0xbb);
		sensor_transmit(crc3(REPEAT_R));
			}
		}
	}
}

void log_data(unsigned char packet){ //handles log requests
	data[dP] = packet;
	dP++;
	if(dP > 3199)
	dP = 0;
}

void service_readout(char selection){ //transmits logged data to user based on selection
	if (dP == 0){
		user_transmit("No data ");
		return;
	}
	unsigned char buffer[4];
	if (selection == 0){ 
		user_transmit("MEM DUMP: ");
		for (int i = 0; i< dP; i++){
			ascii_convert(data[i],buffer);
			user_transmit(buffer);
			_delay_ms(1000);
		}
	}else{
		user_transmit("Last Entry: ");
		ascii_convert(data[dP-1],buffer);
		user_transmit(buffer);
	}
}

void ascii_convert(unsigned char hex_dat, unsigned char * buffer){ //converts hex 8-bit number to 2 ascii characters
	buffer[0] = 0;
	buffer[1] = 0;
	
	buffer[0] = hex_dat>>4;
	if(buffer[0]>=0 && buffer[0]<=9)
	buffer[0] += '0';
	else
	buffer[0] += 'A' - 10;
	
	buffer[1] = hex_dat&0x0F;
	if(buffer[1] >= 0 && buffer[1] <= 9)
	buffer[1] += '0';
	else
	buffer[1] += 'A' - 10;
	
	buffer[2] = ',';
	buffer[3] = '\0';
}
unsigned char hex_convert(unsigned char buffer[]){ //converts 2 ascii characters into 8-bit hex number
	unsigned char number = 0;
	if(buffer[0] <= '9' && buffer[0] >= '0')
	number += (buffer[0] - '0') * 16;
	else if(buffer[0] >= 'a' && buffer[0] <= 'f')
	number += (buffer[0] - 'a' + 10) * 16;
	else if(buffer[0] >= 'A' && buffer[0] <= 'F')
	number += (buffer[0] - 'A' + 10) * 16;
	
	if(buffer[1] <= '9' && buffer[1] >= '0')
	number += (buffer[1] - '0');
	else if(buffer[1] >= 'a' && buffer[1] <= 'f')
	number += (buffer[1] - 'a' + 10);
	else if(buffer[1] >= 'A' && buffer[1] <= 'F')
	number += (buffer[1] - 'A' + 10);
	
	return number;
}
void enableWD(){
	char temp;
	switch(watchdogSetting){
		case 0:
		temp = 0;
		break;
		case 1:
		temp = WDTO_15MS;
		break;
		case 2:
		temp = WDTO_30MS;
		break;
		case 3:
		temp = WDTO_60MS;
		break;
		case 4:
		temp = WDTO_120MS;
		break;
		case 5:
		temp = WDTO_250MS;
		break;
		case 6:
		temp = WDTO_500MS;
		break;
		case 7:
		temp = WDTO_1S;
		break;
		case 8:
		temp = WDTO_2S;
		break;
	}
	if (temp){
		PORTB |= 0x01;
		wdt_enable(temp);
	}
}

ISR(TIMER3_OVF_vect){  //master watchdog expires
		sleep_disable();
		sensor_init();
}
ISR(TIMER1_OVF_vect){ //slave watchdog expires
	sleep_disable();
	sensor_transmit(crc3(REPEAT_R));
	TOS = 0;
	TOS_FULL = 0;
}
ISR(USART0_RX_vect){
	sleep_disable();
	wdt_reset(); //reset timer
	user_input_buffer[user_input_buffer_ptr++] = UDR0; //read one byte from user
	if (user_input_buffer[user_input_buffer_ptr - 1] == '.'){ //if '.' then reading done
		if(saved){ //checks if timers are setup, if not, read from user
			if(user_input_buffer[0] < '9' && user_input_buffer[0] >= '0'){
				user_transmit("RECEIVED TIMEOUT");
				timeout = (user_input_buffer[0] - '0') *  8000000 / 1024;
				saved = 0;
			}
			}else if(wdSaved){//checks if timers are setup, if not, read from user
			if(user_input_buffer[0] < '9' && user_input_buffer[0] >= '0'){
				user_transmit("RECEIVED WATCHDOG");
				watchdogSetting = user_input_buffer[0] - '0';
				wdSaved = 0;
			}
			}else{ //if timers are set, service
			char selection = user_input_buffer[0];
			switch(selection){
				case '1':
				sei(); // interrupts are reenabled since service readout relies on them
				service_readout(0);
				break;
				// last entry
				case '2':
				sei(); // interrupts are reenabled since service readout relies on them
				service_readout(1);
				break;
				// reset
				case '3':
				start_init();
				break;
				// unknown command
				default:
				user_transmit("Invalid command");
				break;
			}
		}
		user_input_buffer_ptr = 0; //reset pointer
	}
}
ISR(USART0_TX_vect){
	sleep_disable();
	if(user_output_buffer[user_output_buffer_ptr] != '\0' && bluetoothSending){  //if sending data
		user_transmit_handler(user_output_buffer[user_output_buffer_ptr++]);  //send data from buffer
		if(user_output_buffer_ptr == user_output_buffer_size || user_output_buffer[user_output_buffer_ptr] == '\0') //if last char, set status to not sending
		bluetoothSending = 0;
		}else{ //if not sending, reset pointers
		user_output_buffer_ptr = 0;
		user_output_buffer_size = 0;
	}
}
ISR(USART1_RX_vect){
	sleep_disable();
	sensor_input_buffer[sensor_input_buffer_ptr++] = UDR1; //read data
	if (sensor_input_buffer_ptr == 2) { //once full byte is received
		sensor_input_buffer_ptr = 0;
		treat_data(hex_convert(sensor_input_buffer)); //treat data
	}
}
