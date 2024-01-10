// TX buffer

uint8_t uart2_tx_buffer[10];
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
void senduart(uint8_t reinfolge){
	uart2_tx_buffer[0]=reinfolge;
	switch (reinfolge) {
		case uart_strom_adress:

			uart2_tx_buffer[1]=0xFF & floattoword_1_komma(Strom_UVW[0]);
			uart2_tx_buffer[2]=floattoword_1_komma(Strom_UVW[0])>>8;
			uart2_tx_buffer[3]=0xFF & floattoword_1_komma(Strom_UVW[1]);
			uart2_tx_buffer[4]=floattoword_1_komma(Strom_UVW[1])>>8;
			uart2_tx_buffer[5]=0xFF & floattoword_1_komma(Strom_UVW[2]);
			uart2_tx_buffer[6]=floattoword_1_komma(Strom_UVW[2])>>8;
			break;
		default:
			break;
	}
}
