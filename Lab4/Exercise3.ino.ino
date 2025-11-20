#include <avr/io.h>
#include <stdint.h>

void timer1_init() {
    TCCR1B |= (1 << WGM12);                 // CTC mode
    TCCR1B |= (1 << CS12) | (1 << CS10);    // Prescaler 1024
    OCR1A = 15625 - 1;                      // 1 second
}

void wait_seconds(uint8_t s) {
    for(uint8_t i=0; i<s; i++) {
        TCNT1 = 0;
        TIFR1 |= (1 << OCF1A);
        while(!(TIFR1 & (1 << OCF1A)));
    }
}

void pwm_init() {
    DDRD |= (1 << PD6);                     // PD6 = OC0A output
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); // Fast PWM
    TCCR0B = (1 << CS01) | (1 << CS00);     // Prescaler 64 (977 Hz)
}

// ADC
void adc_init() {
    ADMUX = (1 << REFS0); // AVcc reference, ADC0
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read() {
    ADCSRA |= (1 << ADSC);                 // Start conversion
    while(ADCSRA & (1 << ADSC));           // Wait
    return ADC;                            // Return 0-1023
}
uint8_t map_to_pwm(uint16_t x) {
    return (uint32_t)x * 255 / 1023;       // 0–1023 -> 0–255
}
int main(void) {
    timer1_init();
    pwm_init();
    adc_init();

    while(1)
    {
        OCR0A = 0;                    // Fan off
        wait_seconds(10);
        
        for(uint8_t s=0; s<6; s++)
        {
            uint16_t potValue = adc_read();       // Read pot each second
            uint8_t  pwmValue = map_to_pwm(potValue);

            OCR0A = pwmValue;                     // Update speed instantly

            wait_seconds(1);                      // Stay in this speed for 1 second
        }
        OCR0A = 0;                                // Fan OFF after 5 s
    }
}
