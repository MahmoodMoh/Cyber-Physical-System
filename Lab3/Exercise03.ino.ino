#define F_CPU 16000000UL    

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define LED_pin PB5
#define Timer1_Compare_1s 15624
#define Timer0_Compare_Value 249
#define Timer0_Interrupt_Count 100

volatile uint8_t timer0_count = 0;


ISR(TIMER1_COMPA_vect){
  PORTB |= (1 << LED_pin);
  timer0_count = 0;
  TCNT0 = 0;
  TIMSK0 |= (1 << OCIE0A);
  TCCR0B |= (1 << CS01) | (1 << CS00);
  

}

ISR(TIMER0_COMPA_vect) {
  timer0_count++;

  if (timer0_count >= Timer0_Interrupt_Count){
    PORTB &= ~(1 << LED_pin);
    TCCR0B = 0;
    TIMSK0 &= ~(1 << OCIE0A);
    timer0_count = 0;
  }
}

void die_hard(void){
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  
}
void Timer0_init(void){
  TCCR0A |= (1 << WGM01); // CTC MODE
  OCR0A = Timer0_Compare_Value;
}
void Timer1_init(void) {
TCCR1B |= (1 << WGM12); //CTC
  TCCR1B |= (1 << CS10) | (1 << CS12); // Prescaler

  TIMSK1 |= (1 << OCIE1A); // ENABLES interrupt for compre match A
  OCR1A = Timer1_Compare_1s;
}
void GPIO_init(void) {
DDRB |= (1 << PB5); // Output
}
int main(void) {
  Timer0_init();
  Timer1_init();
  GPIO_init();
  die_hard();

  sei();

while(1) {
  sleep_mode();
}
return 0;
}
