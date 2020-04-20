// Cool Loops Loop Control
// by: Richard Durante

#include <PWM.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define ThermistorPIN 0                 // Analog Pin 0

float vcc = 4.91;                       // only used for display purposes, if used
                                        // set to the measured Vcc.
float pad = 9850;                       // balance/pad resistor value, set this to
                                        // the measured resistance of your pad resistor
float thermr = 10000;                   // thermistor nominal resistance

float Thermistor(int RawADC) {
  long Resistance;  
  float Temp;                              // Dual-Purpose variable to save space.
  // Thermistor Section
  Resistance = pad * ((1024.0 / RawADC) - 1);
  Temp = log(Resistance);                 // Saving the Log(resistance) so not to calculate  it 4 times later
  Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
  Temp = Temp - 273.15;                   // Convert Kelvin to Celsius
  return Temp;
}

const byte pumpPWM = 7;					// pump control connected to digital pin 3
const byte radFanPWM = 8;				// radiator fans connected to digital pin 5
const byte exhFanPWM = 9;				// large fan connected to digital pin 6

const byte tempSensorHot = A0;			// temperature sensor hot analog pin
const byte tempSensorCold = A1;			// temperature sensor cold analog pin

const byte flowSensorHot = 2;			// flow sensor hot analog pin
const byte flowSensorCold = 3;			// flow sensor cold analog pin

const byte pumpRPMPin = 19;				// pump RPM reading
const byte radFanRPMPin = 20;				// fan RPM readings
const byte exhFanRPMPin = 21;

int dutyCycleStart = 75;              // startup duty cycle (0-255)
int dutyCycleExhFan = 50;		
int dutyCycleRadFan = 50;
int dutyCyclePump = 50;


// int fanRPM;								// variable to output fan RPM
int rpmCounter;						// counter for number of interrupts (hall effect) per second

int32_t pwmFrequency = 24000;                                  // set PWM frequency for system to 27.5kHz

void setup() {
	InitTimersSafe();					// initialize all timers except for 0, to save time keeping functions
	Serial.begin(115200);       
	delay(100);

	// RPM Counter Setup
	pinMode(radFanRPMPin, INPUT);
	attachInterrupt(radFanRPMPin, rpm, RISING);
        
        // Flow Meter Setup
        pinMode(flowSensorHot, INPUT);
        attachInterrupt(flowSensorHot, rpm, RISING);
        
        pinMode(flowSensorCold, INPUT);
        attachInterrupt(flowSensorCold, rpm, RISING);
        
        // Temperature Sensor Setup
        pinMode(tempSensorHot, INPUT);
        pinMode(tempSensorCold, INPUT);
        
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
        
        float temp;
        temp = Thermistor(analogRead(ThermistorPIN));       // read ADC and  convert it to Celsius
        Serial.print("Celsius: ");
        Serial.print(temp,1);                             // display Celsius
        //temp = (temp * 9.0)/ 5.0 + 32.0;                  // converts to  Fahrenheit
        //Serial.print(", Fahrenheit: ");
        //Serial.print(temp,1);                             // display  Fahrenheit
        Serial.println("");                                  
        delay(5000);                                      // Delay a bit...
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
