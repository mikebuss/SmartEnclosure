#include <RTCZero.h>
#include <WiFiNINA.h>
#include "secrets.h"

static int fanRelayEnablePin = 3; // D3

#pragma region "WiFi"

int status = WL_IDLE_STATUS;
WiFiClient client;

#pragma endregion

#pragma region "RTC"

RTCZero rtc; // Onboard real-time clock

// To update the time, manually paste in current epoch from https://www.epochconverter.com/
// then call this method from setup(). After the time has been set, remove this, and re-compile the sketch.
void setupClock() {
  rtc.begin();
  rtc.setEpoch(1597447527);
  Serial.println("RTC set.");
}

void triggerAlarmAfterDelay(int minuteDelay) {
  Serial.println("[RTC] Setting alarm with " + String(minuteDelay) + "delay...");
  rtc.setAlarmEpoch(rtc.getEpoch() + (minuteDelay * 60));
  rtc.enableAlarm(rtc.MATCH_MMSS);
  rtc.attachInterrupt(rtcAlarmTriggered);
}

void rtcAlarmTriggered() {
  Serial.println("[Alarm] Alarm triggered.");
}

#pragma endregion

#pragma region "Fan Control"

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

#pragma endregion

void setup() {
  Serial.begin(9600);

  // TODO: REMOVE BEFORE DEPLOYING
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

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

  // Initially, set the fan speed to zero.
  setFanSpeed(0);

  triggerAlarmAfterDelay(1);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  connectToAP();    // Connect to Wifi Access Point
  printWifiStatus();

  // Get the status initially
  getPrinterStatus();

  // End setup
}

void loop() {
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  delay(1000);
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

void connectToAP() {
  // Try to connect to Wifi network
  while ( status != WL_CONNECTED) {
    Serial.print("[WiFi] Attempting to connect to SSID: ");
    Serial.println(SSIDNAME);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(SSIDNAME, SSIDPASSWORD);

    // wait 1 second for connection:
    delay(1000);
    Serial.println("[WiFi] Connected.");
  }
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

void getPrinterStatus() {
  Serial.println("[Printer] Getting printer status...");
  IPAddress server(192,168,7,198);

  if (client.connect(server, 80)) {
    client.println("GET /api/v1/printer/status HTTP/1.1");
    client.println("Accept: application/json");
    client.println("Connection: close");
    client.println();
  } else {
    Serial.println("[Printer] Printer request failed.");
  }

}



