#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define SIGNAL_PIN PB0   // Arduino pin 8
#define FAN_PIN PD3      // Arduino pin 3 (OC2B for PWM)

// UART initialization for Serial Monitor
void UART_init() {
    UBRR0H = 0;
    UBRR0L = 103; // 9600 baud at 16MHz
    UCSR0B = (1 << TXEN0); // Enable transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data
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

// PWM setup on Timer2
void setupPWM() {
    DDRD |= (1 << FAN_PIN); // PD3 as output
    TCCR2A |= (1 << WGM20) | (1 << WGM21) | (1 << COM2B1); // Fast PWM, non-inverting
    TCCR2B |= (1 << CS22); // Prescaler 64
}

void setFanSpeed(uint8_t speed) {
    OCR2B = speed; // 0-255
}

// Ultrasonic distance measurement
uint16_t readDistance() {
    // Trigger pulse
    DDRB |= (1 << SIGNAL_PIN); // OUTPUT
    PORTB &= ~(1 << SIGNAL_PIN);
    _delay_us(2);
    PORTB |= (1 << SIGNAL_PIN);
    _delay_us(10);
    PORTB &= ~(1 << SIGNAL_PIN);

    // Switch to input
    DDRB &= ~(1 << SIGNAL_PIN);

    // Timer1 for echo duration
    TCNT1 = 0;
    TCCR1B = (1 << CS11); // Prescaler 8
    while (!(PINB & (1 << SIGNAL_PIN))); // Wait for HIGH
    TCNT1 = 0;
    while (PINB & (1 << SIGNAL_PIN)); // Measure HIGH
    uint16_t duration = TCNT1;
    TCCR1B = 0; // Stop timer

    // Convert to cm (approx)
    uint16_t distance = duration / 58; // For prescaler 8
    return distance;
}

int main(void) {
    UART_init();
    setupPWM();

    char buffer[30];

    while (1) {
        uint16_t distance = readDistance();

        // Print distance to Serial Monitor
        sprintf(buffer, "Distance: %u cm\r\n", distance);
        UART_sendString(buffer);

        // Control fan
        if (distance <= 4) {
            setFanSpeed(255);
        } else if (distance <= 20) {
            uint8_t speed = 255 - ((distance - 4) * 13); // Map manually
            setFanSpeed(speed);
        } else {
            setFanSpeed(0);
        }

        _delay_ms(2000); // 2-second delay
    }
}