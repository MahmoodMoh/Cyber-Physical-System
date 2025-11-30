
// Bare Metal CPS: Fan Control based on MPU6050 Angular Velocity
// ATmega328PB Specific Version

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

// I2C (TWI0) definitions for MPU6050 on ATmega328PB
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define GYRO_ZOUT_H  0x47
#define GYRO_CONFIG  0x1B

// Fan control pin (using Timer1 PWM)
// PB1 = OC1A = Pin 9 on Arduino Uno form factor
#define FAN_PIN PB1

// Fan states
typedef enum {
	FAN_OFF,
	FAN_LOW,
	FAN_MEDIUM,
	FAN_HIGH
} FanState;

FanState currentState = FAN_OFF;

// I2C/TWI Functions for ATmega328PB (TWI0)

void I2C_Init(void) {
	// Set SCL frequency to 100kHz
	// SCL = F_CPU / (16 + 2*TWBR0*Prescaler)
	// For 100kHz: TWBR0 = ((F_CPU / 100000) - 16) / 2
	TWBR0 = 72;  // For 16MHz clock
	TWSR0 = 0;   // Prescaler = 1
	TWCR0 = (1 << TWEN);  // Enable TWI0
}

void I2C_Start(void) {
	TWCR0 = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR0 & (1 << TWINT)));
}

void I2C_Stop(void) {
	TWCR0 = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	_delay_us(100);
}

void I2C_Write(uint8_t data) {
	TWDR0 = data;
	TWCR0 = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR0 & (1 << TWINT)));
}

uint8_t I2C_ReadACK(void) {
	TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR0 & (1 << TWINT)));
	return TWDR0;
}

uint8_t I2C_ReadNACK(void) {
	TWCR0 = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR0 & (1 << TWINT)));
	return TWDR0;
}

// MPU6050 Functions
void MPU6050_Init(void) {
	_delay_ms(100);  // Wait for sensor to stabilize
	
	// Wake up MPU6050 (exits sleep mode)
	I2C_Start();
	I2C_Write(MPU6050_ADDR << 1);
	I2C_Write(PWR_MGMT_1);
	I2C_Write(0x00);  // Clear sleep bit
	I2C_Stop();
	
	_delay_ms(10);
	
	// Configure Gyroscope (optional: set full scale range to ±250°/s)
	I2C_Start();
	I2C_Write(MPU6050_ADDR << 1);
	I2C_Write(GYRO_CONFIG);
	I2C_Write(0x00);  // ±250°/s
	I2C_Stop();
	
	_delay_ms(10);
}

int16_t MPU6050_ReadGyroZ(void) {
	int16_t gyroZ;
	
	// Write register address
	I2C_Start();
	I2C_Write(MPU6050_ADDR << 1);
	I2C_Write(GYRO_ZOUT_H);
	I2C_Stop();
	
	// Read 2 bytes
	I2C_Start();
	I2C_Write((MPU6050_ADDR << 1) | 1);
	uint8_t gyroZ_H = I2C_ReadACK();
	uint8_t gyroZ_L = I2C_ReadNACK();
	I2C_Stop();
	
	gyroZ = (gyroZ_H << 8) | gyroZ_L;
	return gyroZ;
}

float MPU6050_GetAngularVelocity(void) {
	int16_t rawGyroZ = MPU6050_ReadGyroZ();
	
	// Convert to degrees per second
	// Sensitivity for ±250°/s range: 131 LSB/(°/s)
	float angularVelocity = (float)rawGyroZ / 131.0;
	
	// Return absolute value
	if (angularVelocity < 0) {
		angularVelocity = -angularVelocity;
	}
	
	return angularVelocity;
}


// PWM Functions for Fan Control (Timer1)

void PWM_Init(void) {
	// Set PB1 (OC1A, Pin 9) as output
	DDRB |= (1 << FAN_PIN);
	
	// Configure Timer1 for Fast PWM, 8-bit
	// COM1A1:0 = 10 (Clear OC1A on compare match, non-inverting mode)
	// WGM13:10 = 0101 (Fast PWM, 8-bit, TOP=0x00FF)
	TCCR1A = (1 << COM1A1) | (1 << WGM10);
	
	// CS12:10 = 010 (Prescaler = 8)
	// This gives PWM frequency = 16MHz / (8 * 256) ? 7.8kHz
	TCCR1B = (1 << WGM12) | (1 << CS11);
	
	// Start with fan off
	OCR1A = 0;
}

void SetFanSpeed(FanState state) {
	switch(state) {
		case FAN_OFF:
		OCR1A = 0;      // 0% duty cycle
		break;
		case FAN_LOW:
		OCR1A = 85;     // ~33% duty cycle
		break;
		case FAN_MEDIUM:
		OCR1A = 170;    // ~66% duty cycle
		break;
		case FAN_HIGH:
		OCR1A = 255;    // 100% duty cycle
		break;
	}
	currentState = state;
}

// State Machine
FanState DetermineState(float angularVelocity) {
	if (angularVelocity <= 30) {
		return FAN_OFF;
		} else if (angularVelocity <= 100) {
		return FAN_LOW;
		} else if (angularVelocity <= 200) {
		return FAN_MEDIUM;
		} else {
		return FAN_HIGH;
	}
}
// UART for Debugging (Optional)
void UART_Init(void) {
	// Set baud rate to 9600 (for 16MHz clock)
	// UBRR = (F_CPU / (16 * BAUD)) - 1
	UBRR0H = 0;
	UBRR0L = 103;  // 9600 baud at 16MHz
	
	// Enable transmitter
	UCSR0B = (1 << TXEN0);
	
	// Set frame format: 8 data bits, 1 stop bit
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_Transmit(uint8_t data) {
	// Wait for empty transmit buffer
	while (!(UCSR0A & (1 << UDRE0)));
	
	// Put data into buffer, sends the data
	UDR0 = data;
}

void UART_PrintString(const char* str) {
	while (*str) {
		UART_Transmit(*str++);
	}
}

void UART_PrintNumber(float num) {
	char buffer[16];
	dtostrf(num, 6, 2, buffer);
	UART_PrintString(buffer);
}

void UART_PrintInt(int16_t num) {
	char buffer[16];
	itoa(num, buffer, 10);
	UART_PrintString(buffer);
}

// Send raw data for Data Visualizer
void SendDebugData(int16_t rawGyro, float angularVel, uint8_t fanState, uint8_t pwmValue) {
	// Start marker
	UART_Transmit(0xAA);
	UART_Transmit(0xBB);
	
	// Raw Gyro (2 bytes)
	UART_Transmit((rawGyro >> 8) & 0xFF);
	UART_Transmit(rawGyro & 0xFF);
	
	// Angular Velocity (send as integer * 10 for precision)
	int16_t angVelInt = (int16_t)(angularVel * 10);
	UART_Transmit((angVelInt >> 8) & 0xFF);
	UART_Transmit(angVelInt & 0xFF);
	
	// Fan State (1 byte)
	UART_Transmit(fanState);
	
	// PWM Value (1 byte)
	UART_Transmit(pwmValue);
	
	// End marker
	UART_Transmit(0xCC);
	UART_Transmit(0xDD);
}

// Main Program

int main(void) {
	// Initialize UART for debugging
	UART_Init();
	UART_PrintString("ATmega328PB CPS System Starting...\r\n");
	
	// Initialize I2C and MPU6050
	I2C_Init();
	_delay_ms(100);
	
	MPU6050_Init();
	UART_PrintString("MPU6050 Initialized\r\n");
	
	// Initialize PWM for fan control
	PWM_Init();
	UART_PrintString("PWM Initialized\r\n");
	UART_PrintString("System Ready!\r\n\r\n");
	
	while (1) {
		// Read angular velocity from MPU6050
		int16_t rawGyro = MPU6050_ReadGyroZ();
		float angularVelocity = MPU6050_GetAngularVelocity();
		
		// Determine fan state based on angular velocity
		FanState newState = DetermineState(angularVelocity);
		
		// Update fan speed if state changed
		if (newState != currentState) {
			SetFanSpeed(newState);
			
			// Debug output (human readable)
			UART_PrintString("Angular Velocity: ");
			UART_PrintNumber(angularVelocity);
			UART_PrintString(" deg/s | Fan: ");
			
			switch(currentState) {
				case FAN_OFF:
				UART_PrintString("OFF\r\n");
				break;
				case FAN_LOW:
				UART_PrintString("LOW\r\n");
				break;
				case FAN_MEDIUM:
				UART_PrintString("MEDIUM\r\n");
				break;
				case FAN_HIGH:
				UART_PrintString("HIGH\r\n");
				break;
			}
		}
		
		// Send structured data for Data Visualizer
		SendDebugData(rawGyro, angularVelocity, (uint8_t)currentState, OCR1A);
		
		// Sample every 200ms
		_delay_ms(200);
	}
	
	return 0;
}
