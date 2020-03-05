// Cool Loops Loop Control
// by: Richard Durante

#include <PWM.h>
#include <stdio.h>
#include <stdlib.h>

const byte pumpPWM = 3;					// pump control connected to digital pin 3
const byte radFanPWM = 9;				// radiator fans connected to digital pin 5
const byte exhFanPWM = 6;				// large fan connected to digital pin 6

const byte tempSensorHot = A0;			// temperature sensor hot analog pin
const byte tempSensorCold = A1;			// temperature sensor cold analog pin

const byte flowSensorHot = A2;			// flow sensor hot analog pin
const byte flowSensorCold = A3;			// flow sensor cold analog pin

const byte fanRPMPin = 2;				// fan RPM readings
const byte pumpRPMPin = 7;				// pump RPM reading
const byte meterPin1R = 2;
const byte meterPin1L = 2;

int dutyCycleStart = 75;              // startup duty cycle (0-255)
int dutyCycleExhFan = 50;		
int dutyCycleRadFan = 140;
int dutyCyclePump = 150;

int analogOneVal;
int analogTwoVal;

// int fanRPM;								// variable to output fan RPM
int rpmCounter;						// counter for number of interrupts (hall effect) per second

int32_t pwmFrequency = 24000;                                  // set PWM frequency for system to 27.5kHz

void setup() {
	InitTimersSafe();					// initialize all timers except for 0, to save time keeping functions
	Serial.begin(115200);       
	delay(100);

	// RPM Counter Setup
	pinMode(fanRPMPin, INPUT);
	attachInterrupt(fanRPMPin, rpm, RISING);
        
        // Flow Meter Setup
        pinMode(meterPin1R, INPUT);
        attachInterrupt(meterPin1R, rpm, RISING);
        
        // Pump Setup
	SetPinFrequencySafe(pumpPWM, pwmFrequency);
	pinMode(pumpPWM, OUTPUT);
	pwmWrite(pumpPWM, dutyCycleStart);

	// 200mm Exhaust Fan Setup
	SetPinFrequencySafe(exhFanPWM, pwmFrequency);
	pinMode(exhFanPWM, OUTPUT);
	pwmWrite(exhFanPWM, dutyCycleStart);

	// 120mm Radiator Fans Setup
  	SetPinFrequencySafe(radFanPWM,pwmFrequency);
	pinMode(radFanPWM, OUTPUT);
	pwmWrite(radFanPWM,dutyCycleStart);
        delay(5000);
}

// Interrupt to count the fan RPM
 void rpm() {
  rpmCounter++;
}

void loop() {
  
    // Control for PWM
    pwmWrite(radFanPWM,dutyCycleRadFan);
    // fanRPMOutput(rotationCount);

    pwmWrite(pumpPWM,dutyCyclePump);
    // pumpRPMoutput(rotationCount);
    
    pwmWrite(exhFanPWM,dutyCycleExhFan);
    // delay(10);
    
    analogOneVal = analogRead(tempSensorHot);
    analogTwoVal = analogRead(tempSensorCold);
    
    Serial.print("Analog One: ");
    Serial.println(analogOneVal);
    
    Serial.print("Analog Two: ");
    Serial.println(analogTwoVal);
    
    delay(1000);
    
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

}

int flowMeterCold(int rpmCounter) {
	int flowMeter;

	rpmCounter = 0;							// reset rotation counter to 0
	sei();									// enable interrupts
	delay(1000);							// pause 1 second
	cli();									// disable interrupts

	flowMeter = (rpmCounter * 60) / 2;			// take count of interrupts over 1 second, multiply by 60 (for minute) and div by 2 for bipolar hall effect sensor
	Serial.print("Flow RPM: ");				// output fan RPM
	Serial.println(flowMeter, DEC);

}


int tempSense(int THot, int TCold) {

	TCold = analogRead(tempSensorCold);
	THot = analogRead(tempSensorHot);
	int deltaT = THot - TCold;

}
