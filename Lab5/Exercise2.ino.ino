#include <avr/io.h>
#include <util/delay.h>

void UART_init() {
    // Set baud rate: UBRR = (F_CPU / (16 * Baud)) - 1
    // For 16 MHz and 115200 baud: UBRR ≈ 8
    UBRR0H = 0;
    UBRR0L = 8;

    // Enable transmitter
    UCSR0B = (1 << TXEN0);

    UCSR0C = (1 << UPM01) | (1 << USBS0) | (1 << UCSZ01);
}
void UART_sendChar(char c) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait until buffer empty
    UDR0 = c;
}

void UART_sendString(const char *str) {
    while (*str) {
        UART_sendChar(*str++);
    }
}
int main(void) {
    UART_init();

    while (1) {
        UART_sendString("Hello 7E2 Format\r\n");
        _delay_ms(1000);
    }
}