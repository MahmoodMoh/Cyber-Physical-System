void setup() {
  Serial.begin(38400); // Baud rate: 38400, format: 8N1 by default
  delay(1000);         // Wait for initialization
  Serial.println("Baud Rate: 38400, Format: 8N1");
}

void loop() {
  Serial.write('1');   // Sends only one character (no newline)
  delay(2000);       // 120,000 ms = 2 minutes
}