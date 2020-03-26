// Cool Loops Loop Control
// by: Richard Durante

#include <PWM.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define ThermistorPINCold 0                             // Analog Pin 0
#define ThermistorPINHot 1                  // Analog Pin 1

float ardVolt = 5.05;                       // Arduino Output Voltage 5V measured
                                        
float pulldownRes = 9990;                   // 10k Pulldown Resistor
                                        
float thermistorRes = 10000;                // thermistor nominal resistance

const byte pumpPWM = 7;			    // pump control connected to digital pin 3
const byte radFanPWM = 8;		    // radiator fans connected to digital pin 5
const byte exhFanPWM = 9;		    // large fan connected to digital pin 6


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
    
    float tempCold;
    float tempHot; 
    tempCold = ThermistorCold(analogRead(ThermistorColdPIN));   // read ADC and  convert it to Celsius
    Serial.print("Temperature Out Cool (Celsius): ");
    Serial.print(tempCold,1);                                   // display Celsius
    Serial.println("");                                  
    
    tempHot = ThermistorHot(analogRead(ThermistorHotPIN));      // read ADC and  convert it to Celsius
    Serial.print("Temperature In Hot (Celsius): ");
    Serial.print(tempHot,1);                                    // display Celsius
    Serial.println("");                                  
    delay(5000);                                                // Delay readings
    
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

float ThermistorHot(int RawADC) {
  long ResistanceCold;  
  float TempCold;                                  // Dual-Purpose variable to save space.
  // Thermistor Section
  ResistanceCold = pullupRes * ((1024.0 / RawADC) - 1);
  TempCold = log(ResistanceCold);                 // Saving the Log(resistance) so not to calculate  it 4 times later
  TempCold = 1 / (0.001129148 + (0.000234125 * TempCold) + (0.0000000876741 * TempCold * TempCold * TempCold));
  TempCold = TempCold - 273.15;                   // Convert Kelvin to Celsius
  return TempCold;
}
float ThermistorCold(int RawADC) {
  long ResistanceCold;  
  float TempCold;                                  // Dual-Purpose variable to save space.
  // Thermistor Section
  ResistanceCold = pullupRes * ((1024.0 / RawADC) - 1);
  TempCold = log(ResistanceCold);                 // Saving the Log(resistance) so not to calculate  it 4 times later
  TempCold = 1 / (0.001129148 + (0.000234125 * TempCold) + (0.0000000876741 * TempCold * TempCold * TempCold));
  TempCold = TempCold - 273.15;                   // Convert Kelvin to Celsius
  return TempCold;
}
