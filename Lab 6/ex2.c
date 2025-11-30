#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

void SPI_MasterInit(void) {
	// Set MOSI, SCK, SS as output, MISO as input
	DDRB = (1<<PB3) | (1<<PB5) | (1<<PB2); // MOSI=PB3, SCK=PB5, SS=PB2
	DDRB &= ~(1<<PB4); // MISO=PB4 input
	// Enable SPI, set as Master, MSB first, Mode 3, fosk/16
	SPCR0 = (1<<SPE)|(1<<MSTR)|(1<<SPR0)| (1<<CPOL);
	SPCR1 = (1<<SPE)|(1<<MSTR)|(1<<SPR0)| (1<<CPOL);
}
uint8_t SPI_MasterTransmit(uint8_t data) {
	SPDR0 = data; // Start transmission
	while(!(SPSR0 & (1<<SPIF))); // Wait for transfer complete
	return SPDR0; // Return received byte
}
int main(void) {
	SPI_MasterInit();
	uint8_t received;
	while(1) {
		PORTB &= ~(1<<PB2); // CS LOW
		received = SPI_MasterTransmit(0xB4);
		PORTB |= (1<<PB2); // CS HIGH
		_delay_ms(1000);
	}
}