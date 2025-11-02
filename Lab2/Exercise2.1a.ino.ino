int redLed = 13;
int pushButton = 2;
volatile int ledState;

void buttonInterrupt() {
  ledState = !ledState;
  digitalWrite(redLed, ledState);              

}

void setup() {
  pinMode(redLed, OUTPUT);
  pinMode(pushButton, INPUT_PULLUP);  
  attachInterrupt(digitalPinToInterrupt(pushButton), buttonInterrupt, FALLING);
}

void loop() {
  // You can add something like:

}
