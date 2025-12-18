/*
 * ATmega328PB Node - Bare-Metal Firmware
 *
 * Functionality:
 * - Read Light Sensor (ADC0 - PC0)
 * - Monitor Door Status:
 * PC1 -> Output LOW (Common Ground signal)
 * PC2 -> Input Pull-Up (Reads LOW when Closed)
 * PC3 -> Input Pull-Up (Reads LOW when Open)
 * - Control Servo (PWM on PB1)
 * - SLEEP_MODE_PWR_DOWN with INT1 wake-up
 * - LED_lamp 20W
 *
 * Compiler: AVR-GCC
 * MCU: ATmega328PB
 * Clock: 16 MHz
 */

/* ===== DEFINITIONS ===== */

#define F_CPU 16000000UL

#define SERVO_MIN_US 900   // 1ms pulse (0 degrees)
#define SERVO_MAX_US 2450  // 2ms pulse (180 degrees)
#define SERVO_PIN	 PB1   // Servo pin

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <string.h>
#include <util/delay.h>



// SPI commands
#define CMD_LED_ON			0x10	// Lamp ON
#define CMD_LED_OFF			0x11	// Lamp OFF
#define CMD_DOOR_OPEN		0x20	// activate servo to OPEN vent
#define CMD_DOOR_CLOSE		0x21	// activate servo to CLOSE vent
#define CMD_GET_DOOR_STATE	0x30	// checking vent State: open/closed?
#define CMD_GET_STATE		0x40	//
#define CMD_GET_SENSOR		0x41	// Light sensor level
#define CMD_DUMMY			0xFF	// empty byte to read response

/* Global Variables */
// Status Definitions for Door
#define DOOR_STATE_OPEN     0
#define DOOR_STATE_CLOSED   1
#define DOOR_STATE_MOVING   2
#define DOOR_STATE_ERROR    3

/* Bool definitions */
#define TRUE 1
#define FALSE 0

typedef struct {
    uint16_t timestamp_ms;
    uint8_t light_level;
	uint8_t	led_status;
    uint8_t door_status;      // 0=Open, 1=Closed, 2=Moving, 3=Error
    uint8_t checksum;
	//uint8_t temp_level;
	//uint8_t hum_level;
	//uint8_t soil_level;
	//uint8_t fan_status;
	//uint8_t pump_status;
	//uint8_t lcd_status;
} SensorData;

volatile SensorData sensor_data = {0};
volatile uint32_t ms_counter = 0;
volatile uint8_t measurement_ready = 0;

volatile uint8_t button_pressed = 0;

// Button Configuration (Check your board schematic!)
// On ATmega328PB Xplained Mini, SW0 is usually on PB7 or PD7. Assuming PB7 here.
#define BUTTON_PORT PORTB
#define BUTTON_PIN  PINB
#define BUTTON_DDR  DDRB
#define BUTTON_BIT  PB7 

// Door Sensor Configuration
#define DOOR_PORT   PORTC
#define DOOR_PIN    PINC
#define DOOR_DDR    DDRC
#define DOOR_SIG_PIN PC1 // Output (Common)
#define DOOR_CLOSE_PIN PC2 // Input
#define DOOR_OPEN_PIN  PC3 // Input

/* ===== UART (for debugging) ===== */

void uart_init(uint16_t baud) {
    uint16_t ubrr = F_CPU / 16 / baud - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (3 << UCSZ00);  // 8-bit data
}

void uart_send_byte(uint8_t byte) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = byte;
}

void uart_send_string(const char *str) {
    while (*str) uart_send_byte(*str++);
}

void print_int(int32_t num) {
    if (num == 0) {
        uart_send_byte('0');
        return;
    }
    
    char buffer[12];
    int i = 0;
    int neg = num < 0;
    if (neg) num = -num;
    
    while (num > 0) {
        buffer[i++] = '0' + (num % 10);
        num /= 10;
    }
    
    if (neg) uart_send_byte('-');
    while (i > 0) uart_send_byte(buffer[--i]);
}

void print_hex(uint8_t num) {
    const char hex_chars[] = "0123456789ABCDEF";
    uart_send_byte(hex_chars[(num >> 4) & 0x0F]);
    uart_send_byte(hex_chars[num & 0x0F]);
}




/* ===== TIMER0 (1ms Tick for Timestamp & Button Debounce) ===== */

void timer0_init(void) {
    // CTC mode, prescaler 64
    TCCR0A = (1 << WGM01);
    TCCR0B = (3 << CS00);  // Prescaler 64
    OCR0A = 249;  // (16MHz / 64) / 1000 - 1 = 249 ticks per 1ms
    TIMSK0 = (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
    ms_counter++;
    sensor_data.timestamp_ms = (uint16_t)ms_counter;
    
    // Button polling with debounce
    static uint8_t button_state = 1;  // 1 = released
    static uint16_t debounce_count = 0;
    
    uint8_t current_reading = (BUTTON_PIN >> BUTTON_BIT) & 1;
    
    if (current_reading != button_state) {
        debounce_count++;
        if (debounce_count > 50) {  // 50ms stable state
            button_state = current_reading;
            if (button_state == 0) {  // Button pressed (Active LOW)
                button_pressed = 1;
            }
            debounce_count = 0;
        }
    } else {
        debounce_count = 0;
    }
}

/* ===== TIMER1 (PWM for Servo on PB1) ===== */

void timer1_servo_init(void) {
    // Fast PWM mode 14 (ICR1 = TOP), Prescaler 8
    // TOP = 40000 -> 20ms period (50Hz)
    TCCR1A = (1 << COM1A1) | (1 << WGM11); 
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // CS11 = Prescaler 8
    
    ICR1 = 40000;
    DDRB |= (1 << SERVO_PIN);	// Set SERVO_PIN as Output
    OCR1A = 3000;				// Init at 1.5ms (90 degrees approx)
}

void servo_set_angle(uint8_t angle) {
    // angle: 0-180 degrees
    // Pulse: 1000us to 2000us
    // With Prescaler 8 @ 16MHz, 1 tick = 0.5us
    // 1000us = 2000 ticks
    // 2000us = 4000 ticks
    
    uint16_t pulse_us = SERVO_MIN_US + ((uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US) / 180);
    OCR1A = pulse_us * 2; // Convert us to ticks
}

/* ===== ADC (Light Sensor) ===== */

void adc_init(void) {
    ADMUX = (1 << REFS0); // AVCC ref
    ADCSRA = (1 << ADEN) | (7 << ADPS0); // Enable, Prescaler 128
}

uint8_t read_light_sensor(void) {
    ADMUX = (ADMUX & 0xF0); // Select Channel 0 (PC0)
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return (uint8_t)(ADC >> 2); // Return 8-bit value
}

/* ===== Door Status Logic ===== */

void door_sensor_init(void) {
    // 1. Configure PC1 as OUTPUT and set LOW (Virtual Ground)
    DOOR_DDR |= (1 << DOOR_SIG_PIN);
    DOOR_PORT &= ~(1 << DOOR_SIG_PIN);
    
    // 2. Configure PC2 and PC3 as INPUTS
    DOOR_DDR &= ~((1 << DOOR_CLOSE_PIN) | (1 << DOOR_OPEN_PIN));
    
    // 3. Enable Internal PULL-UPs on PC2 and PC3
    DOOR_PORT |= (1 << DOOR_CLOSE_PIN) | (1 << DOOR_OPEN_PIN);
}

uint8_t get_door_status(void) {
    // Logic: Inputs are Pull-up. 
    // If switch closes to PC1 (GND), input reads LOW (0).
    
    uint8_t is_closed_sensor_active = !(DOOR_PIN & (1 << DOOR_CLOSE_PIN)); // True if PC2 is Low
    uint8_t is_open_sensor_active   = !(DOOR_PIN & (1 << DOOR_OPEN_PIN));  // True if PC3 is Low
    
    if (is_closed_sensor_active && !is_open_sensor_active) {
        return DOOR_STATE_CLOSED;
    } else if (!is_closed_sensor_active && is_open_sensor_active) {
        return DOOR_STATE_OPEN;
    } else if (!is_closed_sensor_active && !is_open_sensor_active) {
        return DOOR_STATE_MOVING; // Neither sensor active
    } else {
        return DOOR_STATE_ERROR;  // Both sensors active (impossible physically?)
    }
}

/* ===== Reporting & Checksum ===== */

uint8_t calculate_checksum(SensorData *data) {
    uint8_t *ptr = (uint8_t *)data;
    uint8_t sum = 0;
    // Calculate over struct excluding checksum byte itself
    for (size_t i = 0; i < 5; i++) {
        sum ^= ptr[i];
    }
    return sum;
}

void print_sensor_data(void) {
    uart_send_string("\r\n--- STATUS REPORT ---\r\n");
    
    // Light
    uart_send_string("Light: ");
    print_int(sensor_data.light_level);
    uart_send_string("\r\n");
    
    // Door
    uart_send_string("Door:  ");
    switch (sensor_data.door_status) {
        case DOOR_STATE_CLOSED: uart_send_string("CLOSED"); break;
        case DOOR_STATE_OPEN:   uart_send_string("OPEN"); break;
        case DOOR_STATE_MOVING: uart_send_string("MOVING/HALF"); break;
        default:                uart_send_string("ERROR"); break;
    }
    uart_send_string("\r\n");
    
    // Checksum
    uart_send_string("CSum:  0x");
    print_hex(sensor_data.checksum);
    uart_send_string("\r\n---------------------\r\n");
}

/* ===== Watchdog & Sleep ===== */

void watchdog_init(void) {
    wdt_reset();
    WDTCSR = (1 << WDCE) | (1 << WDE);
    WDTCSR = 0; // Disabled by default, enable inside main if needed
}

ISR(WDT_vect) {
    measurement_ready = 1;
}

/* =====  INT1 for wakeup before SPI ===== */
volatile uint8_t wake_flag = 0;

void INT1_init() {
	EICRA |= (1 << ISC11);
	EICRA &= ~(1 << ISC10); //  falling edge for INT1
	
	EIFR =  (1 << INTF1);

	EIMSK |= (1 << INT1);
	
}

ISR(INT1_vect) {
	wake_flag = 1;
	//PORTC |= (1 << PC1);
}

/* ===== State package & Sleep ===== */

// State package in reply on CMD_GET_STATE (8 byte)
#define STATE_PACKET_SIZE 7
volatile uint8_t State_Packet[STATE_PACKET_SIZE];
volatile uint8_t packet_index = 0; // current sending index
volatile uint8_t sending_state = 0; // Flag: 0 = waiting cmd, 1 = in transmithion

void prepare_state_packet(void) {
    sensor_data.light_level = read_light_sensor();
    sensor_data.door_status = get_door_status();

    State_Packet[0] = (uint8_t)(sensor_data.timestamp_ms >> 8);
    State_Packet[1] = (uint8_t)sensor_data.timestamp_ms;
    State_Packet[2] = sensor_data.light_level;
    State_Packet[3] = sensor_data.door_status;
    State_Packet[4] = sensor_data.led_status;

    // Checksum [0..4]
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 5; i++) sum ^= State_Packet[i];
    State_Packet[5] = sum;

    State_Packet[6] = 0xAA;
}

/* ===== SPI slave config ===== */

volatile uint8_t next_reply = 0x00;
volatile uint8_t command_received = 0; // Flag for main loop 


void SPI_SlaveInit(void) {
	// Configuration 
	DDRB |= (1 << PB4);   // MISO Output
	DDRB &= ~(1 << PB3);  // MOSI Input
	DDRB &= ~(1 << PB5);  // SCK Input
	DDRB &= ~(1 << PB2);  // SS Input

	// Enable SPI, Enable SPI Interrupt , Slave mode , SPI mode 2
	
	SPCR0 = (1 << SPE) | (1 << SPIE) | (1 << CPHA);
	
	SPDR0 = 0; // clean buffer
	//uart_send_string("SPI_SlaveInit done \r\n");
}

ISR(SPI0_STC_vect) { 
	uint8_t input = SPDR0; // read cmd from Master
	//uart_send_string("Got messege from master! \r\n");
	
	if (sending_state == 1) {
		// waiting DUMMY from master to send back data
		
		
		packet_index++;
		
		if (packet_index < STATE_PACKET_SIZE) {
			// upload next packet for next reply
			SPDR0 = State_Packet[packet_index];
			} else {
			// if all done - reset state.
			sending_state = 0;
			packet_index = 0;
			SPDR0 = 0x00; // defolt last reply
		}
		
		return; // exit from ISR
	}
	
	switch(input) {
		case CMD_LED_ON:
			PORTC |= (1 << PC1); // LED on
			next_reply = 0x4F;   // reply "ÎÊ" 
			sensor_data.led_status = TRUE;
			break;

		case CMD_LED_OFF:
			PORTC &= ~(1 << PC1);
			//uart_send_string("CMD_LED_OFF \r\n");
			next_reply = 0x4F;
			sensor_data.led_status = FALSE;
			break;
		
		case CMD_DOOR_OPEN:
			servo_set_angle(180);
			//uart_send_string("CMD_DOOR_OPEN \r\n");
			next_reply = 0x4F;
			break;
		
		case CMD_DOOR_CLOSE:
			servo_set_angle(0);
			//uart_send_string("CMD_DOOR_CLOSE \r\n");
			next_reply = 0x4F;
			break;
			
		case CMD_GET_DOOR_STATE:
			sensor_data.door_status = get_door_status();
			//uart_send_string("CMD_GET_DOOR_STATE \r\n");
			next_reply = sensor_data.door_status;
			break;
			
		case CMD_GET_STATE:
			// 1. Collect refreshed data for State_Packet
			prepare_state_packet();
			
			// 2. initialise long connection index 0
			packet_index = 0;
			
			// 3. Upload first byte for reply
			SPDR0 = State_Packet[packet_index];
			
			// 4. sending state flag
			sending_state = 1;
			//uart_send_string("CMD_GET_STATE: Starting multi-byte transfer. \r\n");
			break;

		case CMD_GET_SENSOR:
			// Update light sensor data and send back on nexte dummy
			sensor_data.light_level = read_light_sensor();
			//uart_send_string("CMD_GET_SENSOR  \r\n");
			next_reply = sensor_data.light_level;
			break;

		case CMD_DUMMY:
			
			//uart_send_string("CMD_DUMMY \r\n");
			next_reply = 0x00;
			break;

		default:
			next_reply = 0xEE; // Error
			uart_send_string("SPI error: \r\n");
	}

	// Udload reply to buffer
	SPDR0 = next_reply;
	
}

/* ===== READY FOR SLEEP ===== */
void readyForSleep(void) {
	    // Explicitly configure pins for low power (no floating inputs)
	    // Set unused pins as outputs driven LOW
	    // Keep configured pins in safe states

	    // Port B: SPI pins stay as-is (MISO out, others in)
	    // Servo PB1: Output LOW if not used
	    DDRB |= (1 << PB1); PORTB &= ~(1 << PB1);
	    // Button PB7: Input with pull-up (if used)
	    DDRB &= ~(1 << PB7); PORTB |= (1 << PB7);
	    // Unused: PB0, PB6 as output LOW
	    DDRB |= (1 << PB0) | (1 << PB6);
	    PORTB &= ~((1 << PB0) | (1 << PB6));

	    // Port C: Door sensors stay as-is (PC1 out LOW, PC2/PC3 in pull-up)
	    // Light PC0: Input no pull-up (ADC, but disable ADC below)
	    DDRC &= ~(1 << PC0); PORTC &= ~(1 << PC0);
	    // Unused: PC4, PC5 as output LOW
	    DDRC |= (1 << PC4) | (1 << PC5);
	    PORTC &= ~((1 << PC4) | (1 << PC5));

	    // Port D: UART PD0/PD1 stay as-is if debugging
	    // INT1 PD3: Input no pull-up (external drive assumed)
	    DDRD &= ~(1 << PD3); PORTD &= ~(1 << PD3);
	    // Unused: Set others as output LOW (PD2, PD4-PD7)
	    DDRD |= (1 << PD2) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);
	    PORTD &= ~((1 << PD2) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7));

	    // --- disable peripherals ---
	    ADCSRA &= ~(1 << ADEN); // Disable ADC
	    PRR0 |= (1 << PRADC);
	    
	    // Do NOT disable SPI (keep for wake-up comms)
	    // SPCR &= ~(1 << SPE);   // Removed
	    // PRR |= (1 << PRSPI);   // Removed

	    // Disable USART0
	    UCSR0B = 0;
	    PRR0 |= (1 << PRUSART0);

	    // Disable timers
	    TCCR0A = 0; TCCR0B = 0;
	    TCCR1A = 0; TCCR1B = 0;
	    TCCR2A = 0; TCCR2B = 0;
	    PRR0 |= (1 << PRTIM0) | (1 << PRTIM1) | (1 << PRTIM2);

	    // Disable TWI (I2C)
	    PRR0 |= (1 << PRTWI0);

	    // Ensure INT1 is initialized
	    INT1_init();
	    
	    // --- sleep mode config ---
	    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	    sleep_enable();

	    // INT1 and global interrupts on
	    sei();
}
/* ===== MAIN ===== */

int main(void) {
	// Initialization
	uart_init(9600);
	uart_send_string("\r\nATmega328P Controller Started.\r\n");
	
	timer0_init();          // Milliseconds & Debounce
	timer1_servo_init();    // PWM
	adc_init();             // Light
	door_sensor_init();     // Door GPIOs
	//button_init();        // Button GPIO (handled in sleep prep)
	//watchdog_init();
	INT1_init();
	SPI_SlaveInit();        // SPI init once here
	
	// Initial State
	uint8_t target_door_closed = 1; // Logic state: 1=Want Closed, 0=Want Open
	servo_set_angle(0);             // Move to 0 degrees initially
	
	uart_send_string("Ready. Going sleep.\r\n");
	
	
	while (1) {
		readyForSleep();
		sleep_bod_disable(); // Disable BOD for lower power
		sleep_cpu();
		
		// --- 1. Handle Wake-up ---
		if (wake_flag) {
			wake_flag = 0;      // Clear flag
			_delay_ms(500);     // Time to wakeup
			
			uart_init(9600);
			timer0_init();      // Milliseconds & Debounce
			timer1_servo_init();// PWM
			adc_init();         // Light
			door_sensor_init(); // Door GPIOs
			
			measurement_ready = 1;
			/*/ Toggle Logic  //Servo check
			if (target_door_closed) {
				target_door_closed = 0; // Change to Open
				uart_send_string("Action: Opening Door (Servo -> 180)\r\n");
				servo_set_angle(180);
				} else {
				target_door_closed = 1; // Change to Closed
				uart_send_string("Action: Closing Door (Servo -> 0)\r\n");
				servo_set_angle(0);
			}*/
			
			// Wait a moment for servo to move (simple delay)
			_delay_ms(1500);
			
			// Take Measurement immediately after action
			sensor_data.light_level = read_light_sensor();
			sensor_data.door_status = get_door_status();
			sensor_data.checksum = calculate_checksum((SensorData*)&sensor_data);
			
			print_sensor_data();
		}
		
		// --- 2. Periodic Measurement, if Watchdog enabled ---
		if (measurement_ready) {
			measurement_ready = 0;
			sensor_data.light_level = read_light_sensor();
			sensor_data.door_status = get_door_status();
			sensor_data.checksum = calculate_checksum((SensorData*)&sensor_data);
			print_sensor_data();
		}
	}
	
	return 0;
}