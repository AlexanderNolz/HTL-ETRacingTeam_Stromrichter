#include <stdio.h>
#include <string.h>

// TX buffer

uint8_t uart2_tx_buffer[100];
uint8_t uart2_rx_buffer[10];

// variablen

uint8_t enable = 0;
uint8_t trenner =0;
uint8_t status =0;
uint8_t uart_row = 0;

// hier wird defined welcher Wert übertragen wird

#define uart_broadcast_adress 0x00
#define uart_strom_adress 0x01
#define uart_strom_offset_adress 0x02
#define uart_endcoder_adress 0x03
#define uart_strom_alphabeta 0x04
#define uart_spannung_alphabeta 0x05
#define uart_spannung_ABC 0x06

int16_t floattoword_1_komma(float input){
	//diese methode nimmt einen float wert multipliziert in mit 10 und macht ihn zu einen 16 bit int
	float input_10=input*10.0f;
	int16_t output = (int16_t)(input_10);
	return output;
}
//Diese Methode schreibt die wert in abhängigkeit welche gerade drann sind in den tx Buffer
void senduart(){
	float tx_float_buffer[9];
	tx_float_buffer[0]=theta;
	tx_float_buffer[1]=Clark.alpha;
	tx_float_buffer[2]=Clark.beta;
	tx_float_buffer[3]=Park1.alpha;
	tx_float_buffer[4]=Park1.beta;
	tx_float_buffer[5]=Id;
	tx_float_buffer[6]=Iq;
	tx_float_buffer[7]=theta_is_filter.y;
	tx_float_buffer[8]=omega_rotoe;
	memcpy(uart2_tx_buffer + 20, tx_float_buffer,sizeof(tx_float_buffer));
}
