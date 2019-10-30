#include <Arduino.h>

#define DDR (*(volatile uint8_t *)(DDRB))
#define PORT (*(volatile uint8_t *)(PORTB))
#define PIN (*(volatile uint8_t *)(PINB))
#define BAUD_RATE 9600L
#define OUTPUT_FREQUENCY_HZ 10000
#define MICROS_DECIMAL 1000000
#define MILLIS_DECIMAL 1000
#define SERIAL_POS 0
#define SHIFT_CLOCK_POS 1
#define STORAGE_CLOCK_POS 2
#define DIP_A_POS 3
#define DIP_B_POS 4
#define DIP_C_POS 5

// Leds connected to shift register
static const uint8_t LED_A = (1 << 0);
static const uint8_t LED_B = (1 << 1);
static const uint8_t LED_C = (1 << 2);
static const uint8_t LED_D = (1 << 3);
static const uint8_t LED_E = (1 << 4);
static const uint8_t LED_F = (1 << 5);
static const uint8_t LED_G = (1 << 6);

enum DelayType {
	MICROS, MILLIS
};

uint8_t segmentOutput[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int output_delay;
DelayType delayType = MICROS;

int readDip();
void serialOut(uint8_t output);
void shiftRegisterClock();
void storageRegisterClock();
void tick();
void fillCounter();

int main() {

	output_delay = MICROS_DECIMAL / OUTPUT_FREQUENCY_HZ;
	// Micro loop alows 768us max
	if (output_delay > 768) {
		delayType = MILLIS;
		output_delay = MILLIS_DECIMAL / OUTPUT_FREQUENCY_HZ;
	}

	/* Arduino lib start for debugs */
	init();
	Serial.begin(BAUD_RATE);
	// Outputs real tick frequency
	double realFrequency = delayType == MICROS ? (double) MICROS_DECIMAL / (double) output_delay : (double) MILLIS_DECIMAL / (double) output_delay;
	// 'Rounds' the frequency to one decimal
	realFrequency = ((int) (realFrequency * 10)) / (double) 10.0;
	Serial.println("Starting tick clock with ");
	Serial.print(realFrequency);
	Serial.print(" Hz.");

	Serial.end();
	/* Arduino lib end*/

	// Fill output array
	fillCounter();

	// Setting dip A - C as input
	DDR &= ~((1 << DIP_A_POS) | (1 << DIP_B_POS) | (1 << DIP_C_POS));
	// Setting Serial Data Out, Shift clock, Storage clock as output
	DDR |= (1 << SERIAL_POS) | (1 << SHIFT_CLOCK_POS) | (1 << STORAGE_CLOCK_POS);

	// Resetting port
	PORT &= ~((1 << SERIAL_POS) | (1 << SHIFT_CLOCK_POS) | (1 << STORAGE_CLOCK_POS));

	// Starting loop with milli or micro delay
	switch (delayType) {
	case MILLIS:
		void milliLoop();
		break;
	case MICROS:
		void microLoop();
		break;
	default:
		break;
	}

}

void milliLoop() {
	while (1) {
		unsigned long loopStart = millis();
		tick();
		unsigned long delta = millis() - loopStart;
		_delay_ms(output_delay - delta);
	}
}

void microLoop() {
	while (1) {
		unsigned long loopStart = micros();
		tick();
		unsigned long delta = micros() - loopStart;
		_delay_us(output_delay - delta);
	}
}

void tick() {
	serialOut(segmentOutput[readDip()]);
}

// Arrays index matches the digit that
// should be displayed in 7 segments
void fillCounter() {
	// All but G
	segmentOutput[0] |= ~LED_G;
	// B and C
	segmentOutput[1] |= LED_B | LED_C;
	// All but F and C
	segmentOutput[2] |= ~(LED_F | LED_C);
	// All but F and E
	segmentOutput[3] |= ~(LED_F | LED_E);
	// All but A, E and D
	segmentOutput[4] |= ~(LED_A | LED_E | LED_D);
	// All but B and E
	segmentOutput[5] |= ~(LED_B | LED_E);
	// All but B
	segmentOutput[6] |= ~LED_B;
	// A, B and C
	segmentOutput[7] |= LED_A | LED_B | LED_C;
	// All
	segmentOutput[8] |= ~0;
	// All but E
	segmentOutput[9] |= ~LED_E;
}

// Reads DIP_A as LSB and DIP_C as MSB
int readDip() {
	int value = 0;
	value |= !!(PIN & 1 << DIP_A_POS) << 0;
	value |= !!(PIN & 1 << DIP_B_POS) << 1;
	value |= !!(PIN & 1 << DIP_C_POS) << 2;
	return value;
}

// Writes 8 Bit data to shift register
void serialOut(uint8_t output) {
	// Iterate over all bits in 'output'
	for (int n = 0; n < 8; n++) {
		// Resets output port
		PORT &= ~(1 << SERIAL_POS);
		// Check if n'th bit is set in output
		// And-links either 1 or 0 to SERIAL_POS in PORT
		PORT |= !!(output & (1 << n)) << SERIAL_POS;

		shiftRegisterClock();
	}

	storageRegisterClock();
}

// Outputs a HIGH signal to the shift clock
void shiftRegisterClock() {
	PORT |= (1 << SHIFT_CLOCK_POS);
	PORT &= ~(1 << SHIFT_CLOCK_POS);
}

// Outputs a HIGH signal to the storage clock
void storageRegisterClock() {
	PORT |= (1 << STORAGE_CLOCK_POS);
	PORT &= ~(1 << STORAGE_CLOCK_POS);
}
