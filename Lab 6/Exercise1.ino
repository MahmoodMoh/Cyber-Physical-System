#include <SPI.h>

// Define SS (Slave Select) pin
#define SS_PIN 8

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  while (!Serial); // Wait for serial port to connect
  
  // Set SS pin as output
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH); // Set SS high (inactive)
  
  // Initialize SPI
  SPI.begin();
  
  Serial.println("MKR1010 SPI Master Initialized");
}

void loop() {
  // Data to send
  byte dataToSend = 0xA5; // Example data (10100101 in binary)
  
  // Start SPI transaction with specified settings
  // Modified to match slave configuration: LSB first instead of MSB
  // This ensures sent and received data match correctly
  SPI.beginTransaction(SPISettings(1000000, LSBFIRST, SPI_MODE0));
  
  // Pull SS low to select slave
  digitalWrite(SS_PIN, LOW);
  
  // Send data over MOSI
  byte receivedData = SPI.transfer(dataToSend);
  
  // Pull SS high to deselect slave
  digitalWrite(SS_PIN, HIGH);
  
  // End SPI transaction
  SPI.endTransaction();
  
  // Print for debugging
  Serial.print("Sent: 0x");
  Serial.print(dataToSend, HEX);
  Serial.print(" | Received: 0x");
  Serial.println(receivedData, HEX);
  
  // Wait before sending next data
  delay(1000);
}