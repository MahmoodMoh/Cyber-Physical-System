#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int main(void) {
  DDRB |= (1 << PB5);
  DDRB &= ~(1 << PB0); // Push button INPUT
  PORTB |= (1 << PB0); // Enabling pullup resistor

  PCICR |= (1 << PCIE0); // Enabling pin change interrupt group 0
  PCMSK0 |= (1 << PCINT0); // Enabling pin change interrupt
  sei(); // Global interrup

  while (1) {
  // put your main code here, to run repeatedly:
}
}
ISR(PCINT0_vect){
  if (!(PINB & (1 << PB0))) {
    PORTB ^= (1 << PB5);
  }
}


