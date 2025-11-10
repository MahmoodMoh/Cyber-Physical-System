#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

ISR(TIMER1_COMPA_vect) {
    // Wake up and perform task
    PORTB |= (1 << PB5);      // LED ON
    _delay_ms(100);           // LED stays ON for 0.1s
    PORTB &= ~(1 << PB5);     // LED OFF
}

int main(void) {
    // Set LED pin as output
    DDRB |= (1 << PB5);

    // Configure Timer1 for CTC mode
    TCCR1B |= (1 << WGM12);          // CTC mode
    OCR1A = 15624;                   // Compare value for 1s (16MHz/1024)
    TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler = 1024

    // Enable compare match interrupt
    TIMSK1 |= (1 << OCIE1A);

    // Enable global interrupts
    sei();

    // Configure sleep mode
    set_sleep_mode(SLEEP_MODE_IDLE); // or SLEEP_MODE_PWR_DOWN for more saving

    while (1) {
        sleep_mode();  // MCU sleeps until interrupt occurs
        // After ISR finishes, code returns here and sleeps again
    }
}
