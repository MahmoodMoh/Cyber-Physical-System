
  int analogVal;
  int potM = A1;
  int LEDPin = 6;
  float V2;
  int delayT = 1000;
  int delayT2 = 500;

void setup() {
  Serial.begin(9600);
  while(!Serial);
  pinMode(potM, INPUT);
  pinMode(LEDPin, OUTPUT);
  analogReadResolution(12);

}
void loop(){
  int analogval = analogRead(A1);
  float V2 = analogVal*(3.3/4095);
  Serial.print("Analog Value: ");
  Serial.println(analogVal);
  delay(delayT);
  Serial.print("Voltage: ");
  Serial.println(V2);

  digitalWrite(LEDPin, HIGH);
  delay(delayT2);
  digitalWrite(LEDPin, LOW);
  delay(delayT2);
}