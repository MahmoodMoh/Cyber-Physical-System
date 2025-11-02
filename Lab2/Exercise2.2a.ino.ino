int ledPin = 13;
int pushButton = 8;
volatile int ledState;
void setup() {
  pinMode(pushButton, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  sei();
}
ISR(PCINT0_vect) {
  if (digitalRead(8) == LOW) {
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
  }
}
void loop() {
  // empty
}
