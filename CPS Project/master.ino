/*
 * Auruino uno Master SPI mode
 *
 * Functionality:
 * Soil Sensor ADC - A0
 * DHT11 temp & Hum Sensor - D2
 * Fan 12VDC 0.45A  D9 = PB1 > opto/mosfet       /// remake to PWM????
 * Pump 12vDC 90ml/min D8 - PB0 > opto/mosfet 
 * LCD 16x2 by I2C
 * PUSH BOTTON for LCD contro  16x2 by I2C   D4  //INT1 
 * Wakeup slave by falling edge from PC1 -> to PC3(INT1) before SPI
 
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include "DHT.h"

// === Pin Definitions ===
#define DHTPIN      2
#define DHTTYPE     DHT11
#define SOIL_PIN    0        // ADC0 == A0
#define BUTTON_PIN  4        // button
#define FAN_PIN     PB1      // Fan control

#define WAKEUP_PIN  PD7      // Slave wakeup

// SPI commands
// SPI commands
#define CMD_LED_ON			      0x10	//Lamp ON
#define CMD_LED_OFF			      0x11	//Lamp OFF
#define CMD_DOOR_OPEN		      0x20	// activate servo to OPEN vent
#define CMD_DOOR_CLOSE		    0x21	// activate servo to CLOSE vent
#define CMD_GET_DOOR_STATE	  0x30	// checking vent State: open/closed?
#define CMD_GET_STATE		      0x40	//here sould be a lot of data. (servo, door, LED, light) probably it not fit in 4 byte + 2 byte timestamp + (massage counter?/Checksum?/parity?)
#define CMD_GET_SENSOR		    0x41	// Light sensor level
#define CMD_DUMMY			        0xFF	// empty byte to read response


/* Global Variables */
// Status Definitions for Door
#define DOOR_STATE_OPEN     0
#define DOOR_STATE_CLOSED   1
#define DOOR_STATE_MOVING   2
#define DOOR_STATE_ERROR    3

typedef struct {
    uint16_t timestamp_ms;
    uint8_t light_level;
	  uint8_t	led_status;
    uint8_t door_status;      // 0=Open, 1=Closed, 2=Moving, 3=Error
    float   temp_level;
    float   hum_level;
    uint8_t soil_level;
    bool    fan_status;
    bool    pump_status;
    bool    lcd_status;
    uint8_t checksum;
} SensorData;

volatile SensorData sensor_data = {0};

// === Sensor and Display Setup ===

DHT dht(DHTPIN, DHTTYPE);
rgb_lcd lcd;  // Grove LCD V2.0

// === Timing and State and Flags ===
volatile bool sampleFlag = false;
volatile bool measurement_ready = false;
volatile bool SPI_ready = false;
volatile uint8_t tickCount = 0;
uint8_t page = 0;

// ---------- ADC ----------
void adc_init() {
  ADMUX = (1 << REFS0); // AVcc reference
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC, prescaler 64
}

uint16_t adc_read(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

// ---------- WD interup 8 sec ----------
void wd_init() {
  cli(); // stop interrupt during init

  // allow WDT
  WDTCSR = (1 << WDCE) | (1 << WDE);

  // Enable WD, prescaler ~8s
  WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
}

ISR(WDT_vect) {
  sampleFlag = true;
  
}

void go_to_sleep(void) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // mode
    sleep_enable();
    sleep_cpu(); // MCU sleep - > WDT
    sleep_disable(); // donot sleep after wakeup
}


// ---------- Timer1 ----------
void timer1_init() {
  TCCR1A = 0;
  TCCR1B = (1 << WGM12); // CTC mode
  OCR1A = 31249;         // ~0.5s interval
  TIMSK1 = (1 << OCIE1A);
  TCCR1B |= (1 << CS12); // Prescaler 256
}

ISR(TIMER1_COMPA_vect) {
  if (++tickCount >= 4) { // ~2s
    tickCount = 0;
    sampleFlag = true;
  }
}


// ---------- INT1 for botton push ----------
void INT1_init() {
  EICRA |= (1 << ISC11);   //  falling edge of INT1 
  EICRA &= ~(1 << ISC10);  // 

  EIFR =  (1 << INTF1); 

  EIMSK |= (1 << INT1);
}

ISR(INT1_vect) {
  page = (page + 1) % 4;
  
}

#define SS_PIN PB2
#define SEND_DELAY 0.5

void SPI_MasterInit(void) {
  // Set MOSI, SCK, SS as output, MISO as input
  DDRB = (1<<PB3) | (1<<PB5) | (1<<PB2); // MOSI=PB3, SCK=PB5, SS=PB2
  DDRB &= ~(1<<PB4);                       
  // MISO=PB4 input
  // Enable SPI, set as Master, MSB first, Mode 2, fck/16
  PORTB |= (1 << SS_PIN);
  
  SPCR = (1 << SPE)|(1 << MSTR)| (1 << SPR0) | (1 << CPHA);
  Serial.println(F("SPI_MasterInit "));
}

uint8_t SPI_MasterTransmit(uint8_t data) {
  SPDR = data;                        
  // Start transmission
  while(!(SPSR & (1<<SPIF)));       
  //Serial.println(F("SPI_MasterTransmit done "));
  return SPDR;                        
  // Wait for transfer complete
  // Return received byte
}

void sendCommand(uint8_t cmd) {
  SPI_ready = false;
  PORTD &= ~(1 << WAKEUP_PIN);  // Wakeup slave from power down
  _delay_ms(2);        // Time to wakeup 
  PORTD |= (1 << WAKEUP_PIN);   // Slave can go power down
  _delay_ms(2);
  PORTB &= ~(1 << SS_PIN);      // SS LOW (выбрать слейва)
  SPI_MasterTransmit(cmd);      // Отправляем команду
  _delay_ms(SEND_DELAY); 
  uint8_t end_marker = SPI_MasterTransmit(CMD_DUMMY);
  PORTB |= (1 << SS_PIN);       // SS HIGH (завершить сеанс)
  
  if (end_marker != 0x4F) {
    Serial.println("Protocol Error: Confirmation missing.");
    Serial.println(cmd);
  }
  SPI_ready = true;
}

uint8_t readData(uint8_t cmd) {
  SPI_ready = false;
  if (cmd == CMD_GET_STATE) {
    //Serial.println("CMD_GET_STATE start");
    uint8_t response;
    uint8_t timelowb, timehighb , responcheck;
    PORTD &= ~(1 << WAKEUP_PIN);  // Wakeup slave from power down
    _delay_ms(2);        // Time to wakeup 
    PORTD |= (1 << WAKEUP_PIN);   // Slave can go power down
    _delay_ms(2); 
    PORTB &= ~(1 << SS_PIN);      // Start
    SPI_MasterTransmit(cmd);      // 1. Send cmd
    _delay_ms(SEND_DELAY);           
    timehighb = SPI_MasterTransmit(CMD_DUMMY); //Timestamp Highbyte
    _delay_ms(SEND_DELAY); 
    timelowb = SPI_MasterTransmit(CMD_DUMMY);  //Timestamp lowbyte
    sensor_data.timestamp_ms = (timehighb << 8) | timelowb;
    _delay_ms(SEND_DELAY); 
    sensor_data.light_level = SPI_MasterTransmit(CMD_DUMMY); // LiGHT

    _delay_ms(SEND_DELAY); 
    sensor_data.door_status = SPI_MasterTransmit(CMD_DUMMY); //DOOR
    _delay_ms(SEND_DELAY); 
    sensor_data.led_status = SPI_MasterTransmit(CMD_DUMMY);  //LED
    _delay_ms(SEND_DELAY);
    responcheck = SPI_MasterTransmit(CMD_DUMMY);  // CHKSUM
    _delay_ms(SEND_DELAY);
    uint8_t end_marker = SPI_MasterTransmit(CMD_DUMMY); // End Marker read 
    uint8_t locCheckSum = calculate_checksum((SensorData*)&sensor_data);
    if (responcheck != locCheckSum ) {
      Serial.print(responcheck);              //recieved checkSum  
      Serial.print(" Checksum error! ");
      Serial.println(locCheckSum);            //local calc
      
    }
    if (end_marker != 0xAA) {
      Serial.println("Protocol Error: End Marker missing.");
    }
    PORTB |= (1 << SS_PIN);       // Stop
    PORTD |= (1 << WAKEUP_PIN);   // Slave can go power down
    SPI_ready = true;
    return 1;
  } else {
  SPI_ready = false;
  uint8_t response;
  PORTD &= ~(1 << WAKEUP_PIN);  // Wakeup slave from power down
  _delay_ms(4);        // Time to wakeup 
  PORTB &= ~(1 << SS_PIN);      // Start
  SPI_MasterTransmit(cmd);      // 1. Send cmd
  _delay_ms(SEND_DELAY);           
  response = SPI_MasterTransmit(CMD_DUMMY); // 2. Шлем пустышку, чтобы выкачать ответ
  PORTB |= (1 << SS_PIN);       // Stop
  PORTD |= (1 << WAKEUP_PIN);   // Slave can go power down
  SPI_ready = true;
  return response;
  }
}

/* ===== Checksum ===== */

uint8_t calculate_checksum(SensorData *data) {
    uint8_t *ptr = (uint8_t *)data;
    uint8_t sum = 0;
    // Calculate over struct excluding checksum byte itself
    for (size_t i = 0; i <5; i++) {
        sum ^= ptr[i];
    }
    return sum;
}


void LCDdata(void) {
  // --- Get data from Slave ---
  readData(CMD_GET_STATE);
  
  bool dhtOK = !(isnan(sensor_data.temp_level) || isnan(sensor_data.hum_level));

  // === LCD Display ===
  lcd.clear();
  switch (page) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print("Soil: ");
      lcd.print(sensor_data.soil_level);
      lcd.print("%");
      lcd.setCursor(0, 1);
      lcd.print("Pump: ");
      lcd.print(sensor_data.pump_status ? "ON " : "OFF");
      break;
    case 1:
      lcd.setCursor(0, 0);
      if (dhtOK) {
        lcd.print("Temp: ");
        lcd.print(sensor_data.temp_level, 1);
        lcd.print("C");
        lcd.setCursor(0, 1);
        lcd.print("Hum: ");
        lcd.print(sensor_data.hum_level, 1);
        lcd.print("%");
      } else {
        lcd.print("DHT Error");
      }
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("Fan: ");
      lcd.print(sensor_data.fan_status ? "ON " : "OFF");
      lcd.setCursor(0, 1);
      lcd.print("Vent: ");
      lcd.print(sensor_data.door_status ? "CLOSE " : "OPEN");
      break;
    case 3:
      lcd.setCursor(0, 0);
      lcd.print("Light: ");
      lcd.print(sensor_data.light_level);
      lcd.print(" lx");
      lcd.setCursor(0, 1);
      lcd.print("LED lamp: ");
      lcd.print(sensor_data.led_status ? "ON " : "OFF");
      break;
  }
  //return 0;
}

void dhtRead() {
  float emaAlpha = 0.2;
  float raw_temp = dht.readTemperature();
  float raw_hum = dht.readHumidity();
  sensor_data.temp_level = emaAlpha * raw_temp + (1 - emaAlpha) * sensor_data.temp_level;
  sensor_data.hum_level  = emaAlpha * raw_hum + (1 - emaAlpha) * sensor_data.hum_level;
}

void soilLevel() {
  // === Read Soil Moisture ===
  float emaAlpha = 0.2;
  uint16_t soilADC = adc_read(SOIL_PIN);
  const int SOIL_DRY_ADC = 650; // adjusted with calibration
  const int SOIL_WET_ADC = 850; // adjusted with calibration
  if (soilADC < 5) {
    Serial.println(F("Senor out of SOIL! "));
    return;
  }
  int soilPercent = map(soilADC, SOIL_DRY_ADC, SOIL_WET_ADC, 0, 100);

  sensor_data.soil_level = constrain(soilPercent, 0, 100);

}

void stateMachine(void) {
  
  Serial.println(sensor_data.soil_level);
  // === Fan Control ===
  sensor_data.fan_status = (sensor_data.soil_level > 80);
  if (sensor_data.fan_status)  {
    sendCommand(CMD_DOOR_OPEN);     //Open vent door
    _delay_ms(200);                  // waiting for servo
    if (!readData(CMD_GET_DOOR_STATE))  {
      PORTB |= (1 << FAN_PIN); //Fan ON
    }
  }  else {
    PORTB &= ~(1 << FAN_PIN);           //Fan OFF
    sendCommand(CMD_DOOR_CLOSE);    //Close Door
  }
  // === Lamp Control ===
  sensor_data.led_status = (sensor_data.light_level < 100);
  if (sensor_data.led_status)   sendCommand(CMD_LED_ON);
  else                           sendCommand(CMD_LED_OFF);

  // === Pump Control ===
  sensor_data.pump_status = (sensor_data.soil_level < 25);
  if (sensor_data.pump_status)   PORTB |= (1 << PB0);
  else                           PORTB &= ~(1 << PB0);
}


void setup() {

  Wire.begin();
  lcd.begin(16, 2);
  lcd.setRGB(255, 255, 0);
  lcd.print("Hello World!");

  //wakeup pin init
  DDRD  |= (1 << WAKEUP_PIN); // pin init
  PORTD |= (1 << WAKEUP_PIN); //pullup
  

  // Fan pin init
  DDRB |= (1 << FAN_PIN);   // fan output D9 = PB1
  PORTB &= ~(1 << FAN_PIN); // Fan OFF initially

  // Button init
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Init subsystems
  adc_init();
  dht.begin();
  timer1_init();
  INT1_init();
  //wd_init();
  sei(); // Enable interrupts

  // Serial Monitor
  Serial.begin(9600);
  Serial.println(F("Smart Greenhouse CPS Node"));

  //SPI Master mode init
  //SPI_MasterInit();

}

void loop() {
 if (!sampleFlag) return;
  sampleFlag = false;
  
  SPI_MasterInit(); //SPI Master mode init


  uint8_t received;

  // === Read Soil Moisture ===
  uint16_t soilADC = adc_read(SOIL_PIN);

  dhtRead(); // Temp + Hum sensor measure
  soilLevel();
  stateMachine(); // Actuators check
  
  // === LCD Display ===
  LCDdata();

  // === Serial Monitor Output ===

  bool dhtOK = !(isnan(sensor_data.temp_level) || isnan(sensor_data.hum_level));

  Serial.print(F("ADC="));
  Serial.print(soilADC);
  Serial.print(F(" | Soil="));
  Serial.print(sensor_data.soil_level);
  Serial.print(F("% | "));
  if (dhtOK) {
    Serial.print(F("Temp="));
    Serial.print(sensor_data.temp_level, 1);
    Serial.print(F("C | Hum="));
    Serial.print(sensor_data.hum_level, 1);
    Serial.print(F("% | "));
  } else {
    Serial.print(F("DHT ERROR | "));
  }
  Serial.print(F("Fan="));
  Serial.print(sensor_data.fan_status ? F("ON | ") : F("OFF | "));
  Serial.print("Light: ");
  Serial.print(sensor_data.light_level);
  Serial.print(" lx | ");
  Serial.print("LED lamp: ");
  Serial.print(sensor_data.led_status ? "ON | " : "OFF | ");
  Serial.print("Vent: ");
  Serial.print(sensor_data.door_status ? "CLOSE  | " : "OPEN | ");
  Serial.print(F("Page="));
  Serial.println(page);

  measurement_ready = true;
  
 // if (measurement_ready && SPI_ready == 1)   go_to_sleep();

}