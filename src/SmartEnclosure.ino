#include <RTCZero.h>
#include <WiFiNINA.h>
#include <Scheduler.h>
#include "secrets.h"

#include <Wire.h>
#include "Adafruit_SGP30.h"
#include "Adafruit_Si7021.h"
#include <Adafruit_HMC5883_U.h>

#define TCAADDR 0x70

// Adafruit SGP 30 VOC+eCO2 sensors
// https://www.adafruit.com/product/3709
Adafruit_SGP30 vocSensor1;  // tcaselect(1)
Adafruit_SGP30 vocSensor2;

// Adafruit si7021
// https://www.adafruit.com/product/3251
Adafruit_Si7021 tempSensor1 = Adafruit_Si7021(); // tcaselect(0)
Adafruit_Si7021 tempSensor2 = Adafruit_Si7021(); 

// Constants
static int cooloffTime = 10; // minutes. How long to keep the fan at 100% after a print has finished.
uint32_t printerCheckInterval = 2;  // minutes
uint32_t environmentCheckInterval = 15;  // seconds

String printingStates[3] = {"printing", "resuming", "pre_print"};
String nonPrintingStates[6] = {"none", "pausing", "paused", "post_print", "wait_cleanup", "wait_user_action"};

// Pin definitions
// Fan PWM pin is 2
static int fanRelayEnablePin = 4;
static int builtInLEDPin = 13;
static int flameSensorPin = 9;
static int emergencyPowerShutoffPin = 10;
static int alarmPin = 6;

// State variables
volatile bool flameWasDetected = false;
volatile int flameDetectionCounter = 0; // Increment to avoid false positives for flame counter
static int maxFlameDetectionCounter = 20; // If we detect this many flames in a minute, we probably have a flame
volatile uint32_t lastFlameDetected = 0;
volatile bool didTriggerEmergencyShutoff = false;

volatile bool printing = false;
volatile bool inCoolOffPeriod = false;
String printerResponseBuffer = String();
uint32_t lastPrinterCheck; // epoch

uint32_t lastEnvironmentCheck; // epoch

// TODO: Change
int defaultFanSpeed = 75; // percentage. Speed will increase or decrease based on VOC, temperature
int coolOffFanSpeed = 100; // percentage

int status = WL_IDLE_STATUS;
WiFiClient client;

RTCZero rtc; // Onboard real-time clock

//
/* return absolute humidity [mg/m^3] with approximation formula
* @param temperature [°C]
* @param humidity [%RH]
*/
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

// Select which i2c device to use with the multiplexer
// input between 0-7
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

//

// To update the time, manually paste in current epoch from https://www.epochconverter.com/
// then call this method from `setup`. After the time has been set, remove this, and re-compile the sketch.
void setupClock() {
  rtc.begin();
  rtc.setEpoch(1597447527);
  Serial.println("RTC set.");
}

void triggerAlarmAfterDelay(int minuteDelay) {
  Serial.println("[RTC] Setting alarm with " + String(minuteDelay) + " delay...");
  rtc.setAlarmEpoch(rtc.getEpoch() + (minuteDelay * 60));
  rtc.enableAlarm(rtc.MATCH_MMSS);
  rtc.attachInterrupt(rtcAlarmTriggered);
}

void rtcAlarmTriggered() {
  Serial.println("[Alarm] Alarm triggered.");
  inCoolOffPeriod = false;

  // Make sure we didn't start printing again
  // between noticing the printer stopping
  // and stopping the cooloff period
  if (printing == false) {
    stopActivity();
  } else {
    Serial.println("[Alarm] Cooloff period ended but printer was printing.");
  }
}

// 100% at 25 kHz
const int MAX_FANDUTY = 5000;

// Minimum
const int MIN_FANDUTY = 0;

void setupPWM()
{
  Serial.println("[PWM] Setting up fan PWM...");
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the 4 PWM channels: timer TCC0 outputs
  const uint8_t CHANNELS = 2;
  const uint8_t pwmPins[] = { 2, 3};   //PB10, PB11
  for (uint8_t i = 0; i < CHANNELS; i++)
  {
    PORT->Group[g_APinDescription[pwmPins[i]].ulPort].PINCFG[g_APinDescription[pwmPins[i]].ulPin].bit.PMUXEN = 1;
  }
  // Connect the TCC0 timer to the port outputs - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
  PORT->Group[g_APinDescription[3].ulPort].PMUX[g_APinDescription[3].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
  // PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
  //PORT->Group[g_APinDescription[5].ulPort].PMUX[g_APinDescription[5].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;    // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  // 20000 = 50Hz, 10000 = 100Hz, 2500  = 400Hz
  REG_TCC0_PER = MAX_FANDUTY;      // Set the frequency of the PWM on TCC0 to 50Hz
  while (TCC0->SYNCBUSY.bit.PER);

  // The CCBx register value corresponds to the pulsewidth in microseconds (us)
  REG_TCC0_CCB0 = MAX_FANDUTY;
  while (TCC0->SYNCBUSY.bit.CCB0);
  REG_TCC0_CCB1 = MAX_FANDUTY;
  while (TCC0->SYNCBUSY.bit.CCB1);

  // Divide the 48MHz signal by 1 giving 48MHz (20.8ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
  Serial.println("[PWM] Setup fan PWM.");
}

void setFanDutyCycle(int pcCycle)
{
    REG_TCC0_CCB1 = pcCycle;
    while (TCC0->SYNCBUSY.bit.CCB1);
    REG_TCC0_CCB0 = pcCycle;
    while (TCC0->SYNCBUSY.bit.CCB0);
}

void setFanSpeed(int percentage)
{
  int pwmValue = percentage * MAX_FANDUTY / 100;
  Serial.println("[Fan] Setting fan speed to " + String(percentage) + "%. PWM value: " + String(pwmValue));
  setFanDutyCycle(pwmValue);

  // If we're turning the fan off completely, turn off the relay.
  // If we're turning it on, make sure the relay is on.
  if (percentage <= 0) {
    digitalWrite(fanRelayEnablePin, LOW);
  } else {
    digitalWrite(fanRelayEnablePin, HIGH);
  }
}

void setup() {
  Serial.begin(9600);

  // TODO: Remove before finishing debugging
  // if (!Serial) {
  //   delay(3000);
  // }

  Serial.println("[Setup] Started.");

  // If you need to set the clock again, call this method after
  // following the instructions above to set the time.
  //
  // setupClock();

  // Setup the RTC clock
  rtc.begin();

  // Set the PWM frequency needed for the enclosure fan
  setupPWM();

  // Setup the relay we're using to turn the fan completely off.
  // We need a relay because it's 12V input. Setting the PWM to zero
  // does NOT turn off the fan completely.
  pinMode(fanRelayEnablePin, OUTPUT);

  // Built-in LED
  pinMode(builtInLEDPin, OUTPUT);

  // Flame sensor
  pinMode(flameSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(flameSensorPin), flameDetected, RISING);

  // Emergency power shutoff
  pinMode(emergencyPowerShutoffPin, OUTPUT);
  digitalWrite(emergencyPowerShutoffPin, LOW);

  // Alarm for fire detection
  pinMode(alarmPin, OUTPUT);
  digitalWrite(alarmPin, LOW);

  // To show the system is working, turn the fan on for a few seconds
  setFanSpeed(10);
  delay(5000);
  setFanSpeed(0);

  // Setup environment sensors
  Serial.println("[Setup] Setting up environment sensors...");
  setupEnvironmentSensors();

  // Setup WiFi
  connectToWiFi();
  printWifiStatus();

  // Check printer status on startup
  checkPrinterStatus();

}

void setupEnvironmentSensors() {
  Wire.begin();
  
  Serial.print("[Sensors] Configuring internal VOC (SGP30) sensor");
  tcaselect(1);
  delay(100);
  if (vocSensor1.begin()){
    Serial.print("[Sensors] Found VOC (SGP30) serial #");
    Serial.print(vocSensor1.serialnumber[0], HEX);
    Serial.print(vocSensor1.serialnumber[1], HEX);
    Serial.println(vocSensor1.serialnumber[2], HEX);
  } else {
    Serial.println("[Sensors] VOC (SGP30) sensor not found at channel 1");
    failIndefinitely();
  }

  Serial.println("[Sensors] Configuring internal temperature (Si7021) sensor ");
  tcaselect(0);
  delay(100);
  if (tempSensor1.begin()){
    Serial.print("[Sensors] Found temperature (Si7021) serial #");
    Serial.println(tempSensor1.sernum_a, HEX); Serial.println(tempSensor1.sernum_b, HEX);
  } else {
    Serial.println("[Sensors] Temperature (Si7021) sensor not found at channel 0");
    failIndefinitely();
  }
  
}

void loop() {
  // If we detected a fire, keep sounding the alarm
  // TODO: Change to alarmPin and slow down significantly
  while (flameWasDetected) {
    emergencyShutdown();

    digitalWrite(builtInLEDPin, HIGH);
    delay(250);
    digitalWrite(builtInLEDPin, LOW);
    delay(250);
  }

  uint32_t currentTime = rtc.getEpoch();
  if (lastPrinterCheck != 0 && currentTime - lastPrinterCheck > (printerCheckInterval*60)) {
    checkPrinterStatus();
  }

  if ((currentTime - lastEnvironmentCheck > environmentCheckInterval)) {  // && (printing || inCoolOffPeriod) && 
    checkEnvironment();
  }

  listenForPrinterResponse();
}

void checkEnvironment() {
  lastEnvironmentCheck = rtc.getEpoch();
  Serial.println("[Sensors] Checking environment...");

  // Internal temperature sensor
  tcaselect(0);
  delay(100);
  float internalTemperature = tempSensor1.readTemperature(); // [°C]
  float internalHumidity = tempSensor1.readHumidity(); // [%RH]
  Serial.print("[Sensors Values] Internal temperature: ");
  Serial.println((internalTemperature * 1.8) + 32, 2);
  Serial.print("[Sensors Values] Internal humidity: ");
  Serial.println(internalHumidity);

  // External temperature sensor
  // Serial.print("[Sensors] Checking external temperature (Si7021) sensor...");
  // tcaselect(2);
  // delay(100);
  // float externalTemperature = tempSensor2.readTemperature(); // [°C]
  // float externalHumidity = tempSensor2.readHumidity(); // [%RH]
  // Serial.print("[Sensors Values] External temperature: ");
  // Serial.println(externalTemperature);
  // Serial.print("[Sensors Values] External humidity: ");
  // Serial.println(externalHumidity);

  // Internal VOC sensor
  tcaselect(1);
  delay(200);

  vocSensor1.setHumidity(getAbsoluteHumidity(internalTemperature, internalHumidity));
  if (! vocSensor1.IAQmeasure()) {
    Serial.println("[Sensor] Internal VOC measurement failed.");
    return;
  }
  Serial.print("[Sensors Values] TVOC "); Serial.print(vocSensor1.TVOC); Serial.println(" ppb\t");
  Serial.print("[Sensors Values] eCO2 "); Serial.print(vocSensor1.eCO2); Serial.println(" ppm");
  if (! vocSensor1.IAQmeasureRaw()) {
    Serial.println("[Sensor] Internal Raw VOC measurement failed");
    return;
  }
  Serial.println("--------------");


}

void printWifiStatus() {
  Serial.print("[WiFi] SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("[WiFi] IP Address: ");
  Serial.println(ip);

  Serial.print("[WiFi] Firmware version: ");
  Serial.println(WiFi.firmwareVersion());
}

void connectToWiFi() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("[WiFi] Communication with WiFi module failed!");
    failIndefinitely();
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("[WiFi] Firmware may require updating");
  }

  // Try to connect to Wifi network
  int attempts = 0;
  int delayT = 300;
  static int maxDelayTime = 10 * 60; // seconds
  while (status != WL_CONNECTED) {
    Serial.print("[WiFi] Connecting... (attempt " + String(attempts+1) + ")");

    status = WiFi.begin(SSIDNAME, SSIDPASSWORD);

    int delayTime = (delayT + (delayT * attempts));
    delay(min(delayTime, maxDelayTime));
    attempts++;
  }

  Serial.println("[WiFi] Connected.");

}

void printDateAndTime() {
  // Print date
  print2digits(rtc.getDay());
  Serial.print("/");
  print2digits(rtc.getMonth());
  Serial.print("/");
  print2digits(rtc.getYear());
  Serial.print(" ");

  // and time
  print2digits(rtc.getHours());
  Serial.print(":");
  print2digits(rtc.getMinutes());
  Serial.print(":");
  print2digits(rtc.getSeconds());

  Serial.println();
}

void print2digits(int number) {
  if (number < 10) {
    Serial.print("0"); // print a 0 before if the number is < than 10
  }
  Serial.print(number);
}

void checkPrinterStatus() {
  Serial.println("[Printer] Getting printer status...");
  IPAddress server(192,168,7,198);
  
  lastPrinterCheck = rtc.getEpoch();

  if (client.connect(server, 80)) {
    Serial.println("[Printer] Connected to server. Sending request...");

    client.println("GET /api/v1/print_job/state HTTP/1.1");
    client.println("Host: 192.168.7.148");
    client.println("User-Agent: curl/7.64.1");
    client.println("Accept: application/json");
    client.println("Connection: close");
    client.println();

    Serial.println("[Printer] Finished sending request.");
  } else {
    Serial.println("[Printer] Printer request failed.");
  }
}

void listenForPrinterResponse() {
  while (client.available()) {
    char c = client.read();
    Serial.write(c); // Uncomment to debug printer API response
    printerResponseBuffer += c;
  }

  if (printerResponseBuffer.indexOf("\"") == -1) {
    // Ignore the response until we get at least one double quote
    return;
  }

  if (stringIsFoundInArray(printerResponseBuffer, printingStates, 3)) {
    Serial.println();
    Serial.println("[Printer] Printer is printing.");
    printerResponseBuffer = String();

    if (printing == false) {
      printing = true;
      inCoolOffPeriod = false;
      didStartPrinting();
    }
  } else if (stringIsFoundInArray(printerResponseBuffer, nonPrintingStates, 6)) {
    Serial.println();
    Serial.println("[Printer] Printer is idle.");
    printerResponseBuffer = String();

    if (printing == true) {
      printing = false;
      inCoolOffPeriod = true;
      didStopPrinting();
    }
  }
}

bool stringIsFoundInArray(String string, String strArray[6], int count) {
  for (int i = 0; i < count; i++) {
    if (string.indexOf(strArray[i]) != -1) {
      return true;
    }
  }
  return false;
}

void didStartPrinting() {
  setFanSpeed(calculatedFanSpeed());
}

void didStopPrinting() {
  // Enter the cool-off period that cools the print
  // and filters out VOC's
  Serial.println("[Printer] Printing stopped. Speeding up fan to remove VOC's from chamber.");
  setFanSpeed(coolOffFanSpeed);

  triggerAlarmAfterDelay(cooloffTime);
}

// Stop the fan, data collection, and anything else
void stopActivity() {
  Serial.println("[Printer] Cooloff period ended. Stopping fan.");
  setFanSpeed(0);

  // TODO: Stop data collection
}

int calculatedFanSpeed() {
  // TODO: Calculate fan speed based on environment temperature, VOC level, etc
  return defaultFanSpeed;
}

void flameDetected() {
  if (didTriggerEmergencyShutoff) {
    // We already shutoff - we can ignore these signals
    Serial.println("[Flame] Flame detected but we already shut down.");
    return;
  }

  uint32_t currEpoch = rtc.getEpoch();
  if (currEpoch - lastFlameDetected > 1000) {
    // If we haven't detected a flame for over a minute, we've probably just been getting
    // noise in the signal. Reset everything.
    flameDetectionCounter = 0;
  }

  lastFlameDetected = currEpoch;
  flameDetectionCounter++;

  Serial.print("[Flame] Potential flame detected. Incremented to: ");
  Serial.println(flameDetectionCounter);
  
  if (flameDetectionCounter > maxFlameDetectionCounter) {
    flameWasDetected = true;
    emergencyShutdown();
  }
}

void emergencyShutdown() {
  if (didTriggerEmergencyShutoff) {
    return;
  }
  didTriggerEmergencyShutoff = true;

  Serial.println("[Flame] Emergency shutoff!");

  // We may have just detected a fire. 
  // Stop everything immediately and shut off the power.
  turnOff3DPrinterPower();
  stopActivity();

  // Sound the alarm
  digitalWrite(alarmPin, HIGH);
}

void turnOff3DPrinterPower() {
  digitalWrite(emergencyPowerShutoffPin, HIGH);
}

void failIndefinitely() {
  // Something went catastrophically wrong.
  // Blink the LED and stop running the code.
  while (true) {
    digitalWrite(builtInLEDPin, HIGH);
    delay(500);
    digitalWrite(builtInLEDPin, LOW);
    delay(500);
  }
}
