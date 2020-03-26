// Cool Loops Loop Control
// by: Richard Durante

#include <PWM.h>
#include <stdio.h>
#include <stdlib.h>

const byte radFanPWM = 9;				// fans connected to digital pin 6

const byte fanRPMPin = 3;				// fan RPM readings
const byte fan2RPMPin = 2;				// pump RPM reading

int dutyCycleStart = 50;              // startup duty cycle (0-255)		
int dutyCycleRadFan = 127;

// int fanRPM;								// variable to output fan RPM
int rpmCounter;						// counter for number of interrupts (hall effect) per second
int fanRPM;


int32_t pwmFrequency = 24000;                                  // set PWM frequency for system to 27.5kHz

void setup() {
	InitTimersSafe();					// initialize all timers except for 0, to save time keeping functions
	Serial.begin(115200);       
	delay(100);

	// RPM Counter Setup
	pinMode(fanRPMPin, INPUT);
	attachInterrupt(fanRPMPin, rpm, RISING);
        
        // RPM Counter Setup
	pinMode(fan2RPMPin, INPUT);
	attachInterrupt(fan2RPMPin, rpm, RISING);

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
    
    // RPM Counters for Fans
    rpmCounter = 0;							// reset rotation counter to 0
    sei();									// enable interrupts
    delay(1000);							// pause 1 second
    cli();									// disable interrupts

    fanRPM = (rpmCounter * 60) / 2;			// take count of interrupts over 1 second, multiply by 60 (for minute) and div by 2 for bipolar hall effect sensor
    Serial.print("Fan RPM: ");				// output fan RPM
    Serial.println(fanRPM, DEC);
    delay(1000);
    
}

int fanRPMOutput(int rpmnum) {


}

