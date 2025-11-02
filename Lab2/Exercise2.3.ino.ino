#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int main(void) {
  DDRB |= (1 << PB5);

  TCCR1B |= (1 << WGM12); // CTC Mode

  TCCR1B |= (1 << CS12) | (1 << CS10); // PRESCALER = 1024
  
  OCR1A = 15625;

  TIMSK1 |= (1 << OCIE1A);
  sei();

  while (1) {

}
}
ISR(TIMER1_COMPA_vect) {
  PORTB ^= (1 << PB5);
} 
