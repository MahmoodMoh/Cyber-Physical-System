#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

volatile uint8_t led_counter = 0; // Counts timer0 ticks
volatile uint8_t led_on_timer_active =0; 
// Timer1 ISR: Wakes arduino every 1 second
ISR(TIMER1_COMPA_vect)  {
  PORTB |= (1 << PB5);
  led_counter = 0;
  led_on_timer_active = 1;
}
// Timer0 ISR; 1 ms intervals to turn the LED off after 0.1s
ISR(TIMER0_COMPA_vect) {
  if (led_on_timer_active) {
    led_counter ++;
    if(led_counter >= 100) {
      PORTB &= ~(1 << PB5); //Turn off led
      led_on_timer_active = 0;
    }
  }
}

int main (void) {
  DDRB |= (1 << PB5); // Pin 13 as output
  //Timer0
  TCCR0A |= (1 << WGM01); // CTC Mode for Timer0

  TCCR0B |= (1 << CS00) | (1 << CS01); // Prescaler 64
  TIMSK0 |= (1 << OCIE0A); // Enabling compare match interrupt for timer0
  OCR0A = 249;
  // Timer1
  TCCR1B |= (1 << WGM12);          // CTC mode
    OCR1A = 15624;                   // Compare value for 1s (16MHz/1024)
    TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler = 1024
    // Enable compare match interrupt
    TIMSK1 |= (1 << OCIE1A);
  sei();
  set_sleep_mode(SLEEP_MODE_IDLE);

  while (1) {
    sleep_mode(); // Arduino sleeps until Timer1 or Timer0 interrupt occrs

}
}


