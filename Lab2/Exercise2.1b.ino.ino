#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

ISR (INT0_vect){
  PORTB ^= (1 << PB5 );
  
}
int main(void) {
  DDRB |= (1 << PB5); // Pin B as output
  DDRD &= ~(1 << PD2); 

  PORTD |= (1 << PD2); // Pull up

//iNTERRUPT eNAbled
  EIMSK |= (1 << INT0);
// FALLING edge triggring
  EICRA |= (1 << ISC01);
  EICRA &= ~(1 << ISC00);

  EIFR |= (1 << INTF0); // Manually clearing the flag
  sei();

  while (1) {
  // put your main code here, to run repeatedly:
}

}


