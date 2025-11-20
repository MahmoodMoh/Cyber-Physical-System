#include <avr/io.h>
#include <stdint.h>

void timer1_init() {
    TCCR1B |= (1 << WGM12);                 // CTC mode
    TCCR1B |= (1 << CS12) | (1 << CS10);    // Prescaler 1024
    OCR1A = 15624;                      // 1 second
}
void wait_seconds(uint8_t s) {
    for(uint8_t i=0; i<s; i++) {
        TCNT1 = 0;                          // Reset timer
        TIFR1 |= (1 << OCF1A);              // Clear flag
        while(!(TIFR1 & (1 << OCF1A)));     // Wait for 1 second
    }
}
int main(void) {
    DDRB |= (1 << PB1);                      // Set PB1 as output (connected to optocoupler)
    
    timer1_init();                            // Initialize Timer1 for delays

    while(1) {
        
        PORTB |= (1 << PB1);                  // PB1 HIGH → Optocoupler LED ON → Fan ON
        wait_seconds(3);                      // Keep ON for 3 seconds

        
        PORTB &= ~(1 << PB1);                 // PB1 LOW → Optocoupler LED OFF → Fan OFF
        wait_seconds(10);                     // Keep OFF for 10 seconds
    }
}
