#include <Arduino.h>
#include <Ewma.h>

//Fire Control
const int buttonPinFlywheel = 2;  // Flywheel Rev microswitch pin number
const int buttonPinSolenoid = 3;  // Solenoid microswitch pin number
const int solenoidPin = 4;        // Solenoid MOSFET Gate pin number
const int flywheelPin = 8;        // Flywheel MOSFET Gate pin number
const int buttonPinFull = 9;      // Full auto switch pin number
const int buttonPinSemi = 10;     // Semi auto switch pin number

int buttonStateFull = 0;          // Variable for reading the Full auto switch status
int buttonStateSemi = 0;          // Variable for reading the Semi auto switch status
int buttonStateFlywheel = 0;      // Variable for reading the Flywheel Rev microswitch status
int buttonStateSolenoid = 0;      // Variable for reading the Solenoid microswitch status

int closeDelay = 80;              // Solenoid time delay for closing slide in milliseconds
int openDelay = 70;               // Solenoid time delay for opening slide in milliseconds

//Voltage Monitor
const int vPIN = A4;              // Battery Voltage Analog Pin number
float vOUT = 0.0;
float vIN = 0.0;
float R1 = 10000.0;               // Resistor 1 value in ohms (10000 ohms = 10Kohms)
float R2 = 1000.0;                // Resistor 2 value in ohms (1000 ohms  =  1Kohms)
int value = 0;
float battCutOffVoltage = 10.4;    // Set battery cutoff voltage

bool battCutOff = false;          // Battery cutoff boolean state

Ewma adcFilter1(0.1);   // Less smoothing - faster to detect changes, but more prone to noise
Ewma adcFilter2(0.01);  // More smoothing - less prone to noise, but slower to detect changes


void setup() {
  Serial.begin(9600);
  pinMode(buttonPinFull, INPUT_PULLUP);       // Initialize the Full Auto switch pin as an input
  pinMode(buttonPinSemi, INPUT_PULLUP);       // Initialize the Semi Auto switch pin as an input
  pinMode(buttonPinFlywheel, INPUT_PULLUP);   // Initialize the Flywheel microswitch pin as an input
  pinMode(buttonPinSolenoid, INPUT_PULLUP);   // Initialize the Solenoid microswitch pin as an input
  pinMode(solenoidPin, OUTPUT);               // Sets Solenoid MOSFET Gate pin as an output
  pinMode(flywheelPin, OUTPUT);               // Sets Solenoid MOSFET Gate pin as an output
  pinMode(vPIN, INPUT);                       // Battery voltage monitor pin
}

void loop() {

  //Fire Control Pin Reading
  buttonStateFlywheel = digitalRead(buttonPinFlywheel);    // Read state of Flywheel microswitch value
  buttonStateFull = digitalRead(buttonPinFull);            // Read state of Flywheel microswitch value
  buttonStateSemi = digitalRead(buttonPinSemi);            // Read state of Flywheel microswitch value
  buttonStateSolenoid = digitalRead(buttonPinSolenoid);    // Read state of Solenoid microswitch value

  //Voltage Monitor
  if ((buttonStateFlywheel == HIGH) && (buttonStateSolenoid == LOW)) { //only reads voltage when Blaster is at rest to eliminate influence of voltage drop from motors + solenoid
    value = analogRead(vPIN);
    vOUT = (value * 5.0) / 1024.0;
    vIN = vOUT / ( R2 / (R1 + R2) );

    float filtered1 = adcFilter1.filter(vIN);
    float filtered2 = adcFilter2.filter(vIN);
    Serial.print("raw voltage: ");
    Serial.print(vIN);
    Serial.print(" filter 1 voltage: ");
    Serial.print(filtered1);
    Serial.print(" filter 2 voltage: ");
    Serial.println(filtered2);

    if ((filtered1 < battCutOffVoltage) && (filtered2 < battCutOffVoltage) && (vIN < battCutOffVoltage) && (buttonStateFlywheel == HIGH) && (buttonStateSolenoid == LOW)) {
      battCutOff = true;
      Serial.print("battery cutoff");

    }    else {
      battCutOff = false;
    }
  }

  // Battery Protection Notification Protocol - flywheel motors will pulse momentarily to show battery at critical levels. Restart required.

  while (battCutOff == true) {
    digitalWrite(flywheelPin, LOW);
    digitalWrite(solenoidPin, LOW);
    delay(2000);
    digitalWrite(flywheelPin, HIGH);
    delay(5);
    digitalWrite(flywheelPin, LOW);
    delay(1000);
    digitalWrite(flywheelPin, HIGH);
    delay(5);
    digitalWrite(flywheelPin, LOW);
    delay(1000);
  }



  // FULL AUTO Fire Sequence
  if ((buttonStateFull == HIGH) && (buttonStateSemi == LOW) && (battCutOff == false))  { // Check selector switch state
    Serial.println ("full auto");

    if (buttonStateFlywheel == LOW) {                 // Check if trigger lifted off of flywheel microswitch
      digitalWrite(flywheelPin, HIGH);                // Turns flywheel on
      delay(2);                                       // debounce delay
      Serial.print (" flywheel on ");
    } else {
      digitalWrite(flywheelPin, LOW);                 // Flywheels off
      delay(2);                                       // debounce delay
      Serial.print (" flywheel off ");
    }

    if ( buttonStateSolenoid == HIGH) {
      digitalWrite(solenoidPin, HIGH);                // Switch Solenoid ON
      Serial.println(" solenoidPin HIGH");
      delay(closeDelay);                              // ON duration - adjust accordingly to achieve full solenoid stroke.
      digitalWrite(solenoidPin, LOW);                 // Switch Solenoid OFF
      delay(openDelay);                               // OFF duration - adjust accordingly to achieve full solenoid stroke
      Serial.println("solenoid fired");
    } else {
      digitalWrite(solenoidPin, LOW);                 // Switch Solenoid OFF
      delay(2);                                       // debounce delay
      Serial.println("solenoid off");
    }
  }

  // SEMI AUTO Fire Sequence
  if ((buttonStateFull == LOW) && (buttonStateSemi == HIGH) && (battCutOff == false)) {  // Check selector switch state
    Serial.println ("semi auto");
    if (buttonStateFlywheel == LOW) {                 // Check if trigger lifted off of flywheel microswitch
      digitalWrite(flywheelPin, HIGH);
      delay(2);                                       // debounce delay
      Serial.print (" flywheel on  ");
    } else {
      digitalWrite(flywheelPin, LOW);                 // Flywheels off
      delay(2);                                       // debounce delay
      Serial.print (" flywheel off ");
    }

    if ( buttonStateSolenoid == HIGH) {
      digitalWrite(solenoidPin, HIGH);                // Switch Solenoid ON
      Serial.println(" solenoidPin HIGH");
      delay(closeDelay);                              // ON duration
      digitalWrite(solenoidPin, LOW);                 // Switch Solenoid OFF

      Serial.println(" end semi fire cycle");
      while (digitalRead(buttonPinSolenoid) == HIGH); // Semi-auto while loop
      digitalWrite(solenoidPin, LOW);                 // Switch Solenoid OFF
      delay(openDelay);
    }
  }

  // SAFE Fire Sequence
  if ((buttonStateFull == HIGH) && (buttonStateSemi == HIGH) && (battCutOff == false)) {  // check selector switch state
    Serial.println("safe");
    digitalWrite(flywheelPin, LOW);                   // Flywheels deactivated
    digitalWrite(solenoidPin, LOW);                   // Solenoid deactivated
  }
}
