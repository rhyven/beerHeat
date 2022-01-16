/*************************
  TO DO

   2)  Temperature calibration - put in calibrate mode, hold in boiling water, and save highest offset to EEPROM
   3)  Beep if temp is too far out?
   4)  If I ever rebuild this, merge the Relay and LED control variables into a single pin, derp

  Originally written by Eric Light 2021-01-01. Updates since then.
  
  Requires OneWriteNoResistor library from https://github.com/bigjosh/OneWireNoResistor/
  and the Dallas Temperature library from https://github.com/milesburton/Arduino-Temperature-Control-Library/

  See end for Pinouts

*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>


// Initial setup declarations, constants, and global variables
const char SW_VERSION[] = "20220116";
const int targetTemps[] = {38, 34, 23, 18, 10, 1};  // array of temperatures the button will cycle through; append as required

// Static pin declarations
const int coolingLED = 9; const int heatingLED = 10;  // Incidator LED's
const int coolingRelay = 12; const int heatingRelay = 11; const int tempProbePin = A5; const int buttonPin = A2;
const int RS = 2; const int E = 3; const int D4 = 4; const int D5 = 5; const int D6 = 6; const int D7 = 7;  // LCD Panel

// Fan setup
const int fanPin = A4; const int fanCycleDuration = 5; // time in minutes between fan state toggles
long fanLastToggle = 0; // Last time the fan toggled states

// Button variables
unsigned long debounceDelay = 80;           // the time required between button state changes, to consider it a valid press
unsigned int buttonPresses = 0;             // buttonPresses is an increasing incrementing count

int lastButtonState = HIGH;
unsigned long lastButtonPressTime = 0;      // last time a button state change was detected
unsigned long lastSerialTempPrint = 0;      // last time the temp was printed to Serial
unsigned long lastScreenClearTime = 0;      // last time the LCD screen was cleared (only clear periodically to reduce flicker)

// Temperature definitions                  // A deadband reduces control switching by allowing the temperature to drift a given distance from target.
float tempRange = .8;                       // This distance from target is the deadband. 0.5 is fairly tight control; 0.8 is easier on the fridge
bool achievedTargetTemp = false;            // Used to determine if we're in deadband, or if we're still trying to achieve temp
int targetC = targetTemps[0];
float tempC = targetTemps[0];

// Set up thermocouple libraries
#define ONE_WIRE_BUS tempProbePin
#define TEMPERATURE_PRECISION 11            // Precision of 11 is much faster
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempProbe;

// Set up LCD screen
LiquidCrystal lcd(RS, E, D4, D5, D6, D7);

// Begin
void setup()
{

  Serial.begin(9600); Serial.println("Starting up...");

  // Basic pin setup at boot
  pinMode(buttonPin, INPUT_PULLUP); pinMode(fanPin, OUTPUT);
  pinMode(coolingLED, OUTPUT); pinMode(heatingLED, OUTPUT);
  pinMode(coolingRelay, OUTPUT); pinMode(heatingRelay, OUTPUT);
  digitalWrite(coolingRelay, LOW); digitalWrite(heatingRelay, LOW); digitalWrite(fanPin, HIGH);
  

  digitalWrite(coolingLED, LOW); digitalWrite(heatingLED, LOW);

  Serial.println("Pins initialised, setting up probe and screen...");

  initialise_display();
  initialise_tempProbe();
  lastButtonPressTime = millis();
  lastSerialTempPrint = millis();

  EEPROM.get(0, targetC);
  Serial.print("Retrieved targetC from EEPROM: ");
  Serial.println(targetC);

  // if you're pressing the button, you're probably toggling from hot to crash-chill, or from crash-chill to hot
  // so set the "next button press" to be the last option in targetTemps[]; this reduces user button pressing
  buttonPresses = (sizeof(targetTemps) / sizeof(targetTemps[0])) -1;  
  
  Serial.println("Completed setup.");

}


void loop()
{

  // Our basic loop for the whole show

  update_screen();

  poll_temperature();

  toggle_fan();

  observe_button_press();

  adjust_temperature();

  save_temp();

}

void toggle_fan() {

  if ((millis() - fanLastToggle) > (fanCycleDuration * 60000)) {

    fanLastToggle = millis();
    digitalWrite(fanPin, !digitalRead(fanPin));
    
  }
  
}


void save_temp() {

  // If it's been ten seconds since the last button press, save it to EEPROM
  if (((millis() - lastButtonPressTime) >= 10000) && ((millis() - lastButtonPressTime) <= 10060)) {
    Serial.println("Saving temp setting.");
    EEPROM.put(0, targetC);
    delay(60);  // wait for the end of the 60ms window, so we don't accidentally write twice
  }

}


void observe_button_press() {

  // It's a INPUT_PULLUP pin, so LOW == pressed
  if (digitalRead(buttonPin) == LOW) {

    // Only react if it's been held down for a duration; otherwise buttons have a tendency to 'bounce'
    if ((millis() - lastButtonPressTime) > debounceDelay) {

      buttonPresses++;
      // This ugly thing cycles through the array of target temps; modulo makes it loop from end to start.
      // The array upper boundary is the total size of the array (in bytes), divided by the size (in bytes) of those things.
      targetC = targetTemps[buttonPresses % (sizeof(targetTemps) / sizeof(targetTemps[0]))];
      Serial.println("Button acknowledged");

    }

    lastButtonPressTime = millis();
  }

}


void adjust_temperature() {

  if (tempC < -85) {
    // When probe is disconnected, it reports -127 degrees C.
    no_probe();
    return;
  }

  float gap = tempC - targetC;

  // First we need to determine if we're aiming for the true target, or are we in the deadband?

  // Breached deadband, need to achieve actual target
  if (abs(gap) > tempRange) achievedTargetTemp = false;

  // If the gap is less than .15 degrees, we've achieved the target temp and can enter deadband.
  if (abs(gap) < .15) achievedTargetTemp = true;

  if (achievedTargetTemp) {

    // Either we've arrived at temp, or we're within the deadband.  Relax!
    cooling(false); heating(false);

  }
  else
  {

    // We have either not yet reached target, or we've exceeded deadband, so aim for true target
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

  // Basically this whole subroutine is upside-down and I don't know why.  Why is the relay LOW==on?
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

  // Basically this whole subroutine is upside-down and I don't know why.  Why is the relay LOW==on?
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


void update_screen() {

  // Only clear the screen twice a second to prevent flicker
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
  sensors.setWaitForConversion(false);      // temperature conversion takes time; this makes it asynchronous
  delay(500); sensors.begin(); delay(500);  // These delays are here to prevent probe failing on boot
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
  A4 - Fan
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
