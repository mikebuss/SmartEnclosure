#include <RTCZero.h> 
RTCZero rtc; // Onboard real-time clock

// To update the time, manually paste in current epoch from https://www.epochconverter.com/
// then call this method from setup(). After the time has been set, remove this, and re-compile the sketch.
void setupClockOnce() {
  rtc.begin();
  rtc.setEpoch(1597275532);
  Serial.println("RTC set.");
}

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
  Serial.println("Setting fan speed to " + String(percentage) + "%. PWM value: " + String(pwmValue));
  setFanDutyCycle(pwmValue);
}

#pragma endregion

void setup() {
  Serial.begin(9600);
  Serial.println("[Setup] Started.");

  setupPWM();
  setFanSpeed(0);                    
}

void loop() {
  Serial.println("Testing platformio...");
  delay(1000);
}
