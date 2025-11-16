#include <avr/io.h>
#include <avr/interrupt.h>

#define Timer1_Compare_value 15624
#define Timer1_Interrupt_Count 3

volatile uint8_t Timer1_count = 0;

ISR(TIMER1_COMPA_vect){
  Timer1_count++;
  if (Timer1_count == 1) {
    PORTB |= (1 << PB5);
  }
  if (Timer1_count == 4){
    Timer1_count = 0;
    PORTB &= ~(1 << PB5);
  }
}

void Gpio_init(void) {
  DDRB |= (1 << PB5);
  PORTB &= (1 << PB5);
}

void Timer1_init(void) {
TCCR1B |= (1 << WGM12); // CTC mode for Timer1
  TCCR1B |= (1 << CS10) | (1 << CS12); // Prescaler 1024
  OCR1A = Timer1_Compare_value;
  TIMSK1 |= (1 << OCIE1A);
}
int main (void) {
  Gpio_init();
  Timer1_init();
  sei();
  while(1) {
  // put your main code here, to run repeatedly:

}
}


