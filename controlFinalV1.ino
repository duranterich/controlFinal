// Cool Loops Loop Control
// by: Richard Durante

#include <PWM.h>
#include <stdio.h>
#include <stdlib.h>

const byte pumpPWM = 3;					// pump control connected to digital pin 3
const byte radFanPWM = 5;				// radiator fans connected to digital pin 5
const byte exhFanPWM = 6;				// large fan connected to digital pin 6

const byte tempSensorHot = A0;			// temperature sensor hot analog pin
const byte tempSensorCold = A1;			// temperature sensor cold analog pin

const byte flowSensorHot = A2;			// flow sensor hot analog pin
const byte flowSensorCold = A3;			// flow sensor cold analog pin

const byte fanRPMPin = 2;				// fan RPM readings
const byte pumpRPMPin = 7;				// pump RPM reading

int dutyCycle = 30;						// startup duty cycle (0-255)

// int fanRPM;								// variable to output fan RPM
int rotationCount;						// counter for number of interrupts (hall effect) per second

int32_t pwmFrequency = 27500;			// set PWM frequency for system to 27.5kHz

void setup() {
	InitTimersSafe();					// initialize all timers except for 0, to save time keeping functions
	Serial.begin(9600);       
	delay(100);
	// RPM Counter Setup
	pinMode(fanRPMPin, INPUT);
	attachInterrupt(digitalPinToInterrupt(fanRPMPin), rpm, RISING);

	// Pump Setup
	SetPinFrequencySafe(pumpPWM, pwmFrequency);
	pinMode(pumpPWM, OUTPUT);
	pwmWrite(pumpPWM, dutyCycle);

	// 200mm Exhaust Fan Setup
	SetPinFrequencySafe(exhFanPWM, pwmFrequency);
	pinMode(exhFanPWM, OUTPUT);
	pwmWrite(exhFanPWM, dutyCycle);

	// 120mm Radiator Fans Setup
  	SetPinFrequencySafe(radFanPWM,pwmFrequency);
	pinMode(pumpPWM, OUTPUT);
	pwmWrite(fanPWM,dutyCycle);
}

	// Interrupt to count the fan RPM
void rpm() {
  rotationCount++;
}

void loop() {
    // Control for PWM
    pwmWrite(radFanPWM,dutyCycle);
	fanRPMOutput(rotationCount);

	pwmWrite(pumpPWM,dutyCycle);
	// punpRPMout(rotationCount);

}

int fanRPMOutput(int rpmCounter) {
	int fanRPM;

	rpmCounter = 0;							// reset rotation counter to 0
	sei();									// enable interrupts
	delay(1000);							// pause 1 second
	cli();									// disable interrupts

	fanRPM = (rpmCounter * 60) / 2;			// take count of interrupts over 1 second, multiply by 60 (for minute) and div by 2 for bipolar hall effect sensor
	Serial.print("Fan RPM: ");				// output fan RPM
	Serial.println(fanRPM, DEC);
	return;

}

int tempSense(int THot, int TCold) {

	TCold = analogRead(tempSensorCold);
	THot = analogRead(tempSensorHot);
	int deltaT = THot - TCold;
	return;

}
