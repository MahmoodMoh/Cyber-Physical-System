#include <avr/io.h>

void timer1_init() {
    // Set Timer1 to CTC mode
    TCCR1B |= (1 << WGM12);

    // Prescaler = 1024
    TCCR1B |= (1 << CS12) | (1 << CS10);

    // 1 second at 16 MHz / 1024
    OCR1A = 15625 - 1;
}

void wait_seconds(uint8_t seconds) {
    for(uint8_t i = 0; i < seconds; i++) {
        TCNT1 = 0;              // Reset counter
        TIFR1 |= (1 << OCF1A);  // Clear flag
        while(!(TIFR1 & (1 << OCF1A))); // Wait for compare match
    }
}

void timer0_pwm_init() {
    // PD6 = OC0A output
    DDRD |= (1 << PD6);

    // Fast PWM, non-inverting on OC0A
    TCCR0A |= (1 << WGM00) | (1 << WGM01);     // Fast PWM
    TCCR0A |= (1 << COM0A1);                   // Non-inverting mode

    // Prescaler = 64 -> ~976 Hz PWM
    TCCR0B |= (1 << CS01) | (1 << CS00);

    OCR0A = 0;  // Start fan OFF
}

int main() {
    timer1_init();
    timer0_pwm_init();

    // 5 PWM levels (slow → fast)
    uint8_t ramp[5] = {100, 140, 180, 220, 255};

    while(1) {

        // Wait 15 seconds before next cycle
        wait_seconds(15);

        // 5-second fan ON with speed ramp
        for(uint8_t i = 0; i < 5; i++) {
            OCR0A = ramp[i];   // Set PWM duty cycle
            wait_seconds(1);   // Hold speed for 1s
        }

        // Turn fan OFF
        OCR0A = 0;
    }
}
