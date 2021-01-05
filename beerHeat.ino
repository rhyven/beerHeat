/*************************
  TO DO

   1)  Handle power failures - save target temp
   2)  Temperature calibration!!!
   3)  Scream if temp is too far out
   4)  Figure out better temp control (deadband)

  Written by Eric Light 2021-01-01
  Requires OneWriteNoResistor library from https://github.com/bigjosh/OneWireNoResistor/
  and the Dallas Temperature library from https://github.com/milesburton/Arduino-Temperature-Control-Library/
  See end for Pinouts

*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>


// Initial setup declarations
const char SW_VERSION[] = "20210104";

// Button variables
const int buttonPin = A2;
unsigned int buttonPresses = 0;
int lastButtonState = HIGH;   // the previous reading from the input pin
unsigned long lastButtonPressTime = 0;  // the last time the button was pressed
unsigned long debounceDelay = 100;    // the debounce time between valid button presses
unsigned long lastSerialTempPrint = 0;  // last time the temp was printed to Serial
unsigned long lastScreenClearTime = 0;

// Define temperatures to cycle through via button
const int targetTemps[] = {23, 18, 10, 3};
int targetC = targetTemps[0]; float tempC = targetTemps[0];
float tempRange = .5;
bool achievedTargetTemp = false;

// LCD Panel pin definitions
const int RS = 2; const int E = 3; const int D4 = 4; const int D5 = 5; const int D6 = 6; const int D7 = 7;
// Other pin constants
const int coolingRelay = 12; const int heatingRelay = 11; const int tempProbePin = A5;
// Incidator light pins
const int coolingLED = 9; const int heatingLED = 10;


// Set up thermocouple libraries
#define ONE_WIRE_BUS tempProbePin
#define TEMPERATURE_PRECISION 11  // Precision of 11 is much faster
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempProbe;

// Set up LCD screen
LiquidCrystal lcd(RS, E, D4, D5, D6, D7);


void setup()
{

  Serial.begin(9600);

  Serial.println("Starting up...");

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(coolingLED, OUTPUT); pinMode(heatingLED, OUTPUT);
  pinMode(coolingRelay, OUTPUT); pinMode(heatingRelay, OUTPUT);
  digitalWrite(coolingRelay, LOW); digitalWrite(heatingRelay, LOW);

  // turn on the heaing & cooling LED's for a moment while booting
  digitalWrite(coolingLED, HIGH); digitalWrite(heatingLED, HIGH);

  Serial.println("Pins initialised, setting up probe and screen...");

  initialise_display();
  initialise_tempProbe();
  lastButtonPressTime = millis();
  lastSerialTempPrint = millis();

  digitalWrite(coolingLED, LOW); digitalWrite(heatingLED, LOW);

  EEPROM.get(0, targetC);
  Serial.print("Retrieved targetC from EEPROM: ");
  Serial.println(targetC);

  Serial.println("Completed setup.");

}


void loop()
{

  update_screen();

  poll_temperature();

  observe_button_press();

  adjust_temperature();

  save_temp();

}


void save_temp() {

  // if last buttonpress
  if (((millis() - lastButtonPressTime) >= 10000) && ((millis() - lastButtonPressTime) <= 10060)) {
    Serial.println("Saving temp setting.");
    EEPROM.put(0, targetC);
    delay(60);
  }


}


void observe_button_press() {

  // It's a INPUT_PULLUP pin, so LOW == pressed
  if (digitalRead(buttonPin) == LOW) {

    // Only react if it's been held down for over debounceDelay time
    if ((millis() - lastButtonPressTime) > debounceDelay) {

      Serial.println("Button acknowledged");
      buttonPresses++;
      targetC = targetTemps[buttonPresses % (sizeof(targetTemps) / sizeof(targetTemps[0]))];

    }

    lastButtonPressTime = millis();
  }

}


void adjust_temperature() {

  if (tempC < -85) {
    // Probe is disconnected, bail
    no_probe();
    return;
  }

  float gap = tempC - targetC;

  // Are we aiming for the true target, or are we in the deadband?
  // Breached deadband, need to achieve actual target
  if (abs(gap) > tempRange) achievedTargetTemp = false;

  // If the gap is less than .15 degrees, we've achieved target.
  if (abs(gap) < .15) achievedTargetTemp = true;

  if (achievedTargetTemp) {

    // Either we're at temperature, or we're within the deadband
    cooling(false); heating(false);

  }
  else
  {

    // We have either not yet reached target, or we've exceeded deadband, 4so aim for true target
    if (tempC > targetC) cooling(true);
    if (tempC < targetC) heating(true);

  }

  // Some basic output to Serial
  if (millis() - lastSerialTempPrint > 1000) {
    Serial.print("Target: ");
    Serial.print(targetC);
    Serial.print("   Temp:   ");
    Serial.print(tempC, 2);
    Serial.print("   Gap: ");
    Serial.println(gap);
    lastSerialTempPrint = millis();
  }

}


void no_probe() {

  cooling(false);
  heating(false);
  Serial.println("No temperature probe detected!");

}


void cooling(bool on) {

  if (on) {
    if (digitalRead(coolingRelay) == HIGH) Serial.println("Beginning cooling.  Ensuring heating is off.");
    heating(false);
    digitalWrite(coolingLED, HIGH); digitalWrite(coolingRelay, LOW);
  }
  else {
    if (digitalRead(coolingRelay) == LOW) Serial.println("Finished cooling.");
    digitalWrite(coolingLED, LOW); digitalWrite(coolingRelay, HIGH);
  }
}


void heating(bool on) {

  if (on) {
    if (digitalRead(heatingRelay) == HIGH) Serial.println("Beginning heating.  Ensuring cooling is off.");
    cooling(false);
    digitalWrite(heatingLED, HIGH); digitalWrite(heatingRelay, LOW);
  }
  else {
    if (digitalRead(heatingRelay) == LOW) Serial.println("Finished heating.");
    digitalWrite(heatingLED, LOW); digitalWrite(heatingRelay, HIGH);
  }
}


//void cooling(bool on) {
//
//  if (on) {
//    if (digitalRead(coolingRelay) == LOW) Serial.println("Beginning cooling.  Ensuring heating is off.");
//    heating(false);
//    digitalWrite(coolingLED, HIGH); digitalWrite(coolingRelay, HIGH);
//  }
//  else {
//    if (digitalRead(coolingRelay) == HIGH) Serial.println("Finished cooling.");
//    digitalWrite(coolingLED, LOW); digitalWrite(coolingRelay, LOW);
//  }
//}
//
//
//void heating(bool on) {
//
//  if (on) {
//    if (digitalRead(heatingRelay) == LOW) Serial.println("Beginning heating.  Ensuring cooling is off.");
//    cooling(false);
//    digitalWrite(heatingLED, HIGH); digitalWrite(heatingRelay, HIGH);
//  }
//  else {
//    if (digitalRead(heatingRelay) == HIGH) Serial.println("Finished heating.");
//    digitalWrite(heatingLED, LOW); digitalWrite(heatingRelay, LOW);
//  }
//}


void update_screen() {

  // Only clear the screen twice a second
  if (millis() - lastScreenClearTime >= 500) {
    lcd.clear();
    lastScreenClearTime = millis();
  }

  lcd.setCursor(0, 0);
  lcd.print("Target:  ");
  if (targetC < 10) lcd.print(" ");
  lcd.print(targetC);
  lcd.print(".00");
  lcd.setCursor(15, 0);
  lcd.print("C");

  lcd.setCursor(0, 1);
  if (tempC < -80) {
    lcd.print("Unplugged!");
  }
  else {
    lcd.print("Current: ");
    if (tempC < 10) lcd.print(" ");
    lcd.print(tempC);
    lcd.setCursor(15, 1);
    lcd.print("C");
  }

}


void poll_temperature() {
  // Read the most recent temperature calculated by the probe, and tell it to start a new one.
  tempC = sensors.getTempC(tempProbe);
  sensors.requestTemperatures();
}


void initialise_display() {

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Starting up...");
  lcd.setCursor(0, 1);
  lcd.print("Version " + (String)SW_VERSION);
  delay(1000);
  lcd.clear();

}


void initialise_tempProbe() {

  // Initialise thermocouple
  sensors.getAddress(tempProbe, 0);
  sensors.setResolution(tempProbe, TEMPERATURE_PRECISION);
  sensors.setWaitForConversion(false);
  delay(500);  // delays here to prevent probe failing on boot
  sensors.begin();
  delay(500);
  sensors.requestTemperatures();
}

/* Pinout:

  GND
  D2 - LED screen RS
  D3 - LED screen E
  D4 - LED screen D4
  D5 - LED screen D5
  D6 - LED screen D6
  D7 - LED screen D7
  D8 -
  D9 - CoolingLED
  D10 - Heating LED
  D11 - Heating Relay control
  D12 - Cooling Relay control
  D13 - (can't use due to onboard LED coexisting - https://forum.arduino.cc/index.php?topic=493665.0)
  3.3v
  A0
  A1
  A2 - Selector button
  A3 -
  A4 -
  A5 - Temperature probe data pin  (DS18B20)
  A6 - (can't use, truly analog only)
  A7 - (can't use, truly analog only)
  5v
  GND


  Screen PINS
  A - 5v
  K - GND
  D7 - 7
  D6 - 6
  D5 - 5
  D4 - 4
  D0 through D3 - nothing
  E - 3
  RW - GND
  RS - 2
  V0 - 10k Trimpot
  VDD - +5v
  VSS - GND
*/
