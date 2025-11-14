#include <avr/io.h>

void timer1_init() {
    // Set Timer1 to CTC mode (Clear Timer on Compare)
    TCCR1B |= (1 << WGM12);

    // Prescaler = 1024
    TCCR1B |= (1 << CS12) | (1 << CS10);

    // With 16 MHz clock:
    // Tick time = 1024 / 16,000,000 = 64 µs
    // For 1 second: 1 / 64e-6 = 15625 ticks
    OCR1A = 15624; // 1-second compare value
}

void wait_seconds(uint8_t seconds) {
    for(uint8_t i = 0; i < seconds; i++) {
        TCNT1 = 0;              // Reset counter
        TIFR1 |= (1 << OCF1A);  // Clear flag
        while(!(TIFR1 & (1 << OCF1A))); // Wait for compare match
    }
}

int main() {
    DDRB |= (1 << PB1); // PB5 as output (LED pin)

    timer1_init();

    while(1) {
        // LED ON for 3 seconds
        PORTB |= (1 << PB1);
        wait_seconds(3);

        // LED OFF for 1 second
        PORTB &= ~(1 << PB1);
        wait_seconds(1);
    }
}
