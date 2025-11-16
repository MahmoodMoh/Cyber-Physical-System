#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

volatile uint8_t button_pressed = 0;
// External Interrupt 0 (INT0) ISR - Button press on Digital Pin 2
ISR(INT0_vect) {
// MCU wakes up here
button_pressed = 1;
}
void adc_init() {
// Configure ADC for reading A0 (ADC0)
ADMUX = (1 << REFS0);  // AVCC reference
// Enable ADC, set prescaler to 128 (16MHz / 128 = 125kHz)
ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

// Disable digital input on ADC0 to save power
DIDR0 = (1 << ADC0D);
}

uint16_t adc_read() {
// Start conversion
ADCSRA |= (1 << ADSC);
// Wait for conversion to complete
while (ADCSRA & (1 << ADSC));
// Return ADC result (10-bit value: 0-1023)
return ADC;
}

void setup() {
DDRD &= ~(1 << DDD2);  // Set PD2 as input
PORTD |= (1 << PORTD2);  // Enable internal pull-up resistor
EICRA |= (1 << ISC01);  // Falling edge triggers INT0
EICRA &= ~(1 << ISC00);

EIMSK |= (1 << INT0); // Enable INT0 interrupt
adc_init(); // Initialize ADC 
DDRB |= (1 << DDB5);  // PB5 as output (built-in LED)
PORTB &= ~(1 << PORTB5);  // LED initially OFF

// Disable unused peripherals to save power
power_spi_disable();
power_twi_disable();
power_timer0_disable();
power_timer1_disable();
power_timer2_disable();
power_usart0_disable();


set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep mode to Power-Down (deepest sleep)

sei();  // Enable global interrupts
}
void loop() {
sleep_enable();
ADCSRA &= ~(1 << ADEN); // Disable ADC before sleeping to save more power
sleep_cpu();
sleep_disable();

ADCSRA |= (1 << ADEN); // Re-enable ADC

if (button_pressed) {
button_pressed = 0;
PORTB |= (1 << PORTB5);

uint16_t adc_value = adc_read(); // Read value from potentiometer on A0
for (uint8_t i = 0; i < (adc_value >> 8) + 1; i++) {
PORTB ^= (1 << PORTB5);  // Toggle LED
for (volatile uint16_t j = 0; j < 10000; j++);  // Small delay
}
PORTB &= ~(1 << PORTB5);
}}
int main(void) {
setup();
while(1) {
loop();
}
return 0;
}

