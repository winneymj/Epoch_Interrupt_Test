
#include <Time.h>         //http://www.arduino.cc/playground/Code/Time  
#include <TimeLib.h>         //http://www.arduino.cc/playground/Code/Time  
#include <Wire.h>         //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include <DS3232RTC.h>    //http://github.com/JChristensen/DS3232RTC

#define LED 6
#define MBUT 4
#define UBUT 14
#define DBUT 22
#define RTC_INT 9


#define EVERY_SECOND
//#define EVERY_MINUTE

volatile boolean buttonRead = false; //variables in ISR need to be volatile
volatile boolean buttonFired = false; //variables in ISR need to be volatile
volatile boolean rtcRead = false; //variables in ISR need to be volatile
volatile boolean rtcFired = false; //variables in ISR need to be volatile

DS3232RTC MyDS3232;

void enableInterrupts()
{
  // Enable EIC
  EIC->CTRL.bit.ENABLE = 1;
  while (EIC->STATUS.bit.SYNCBUSY == 1) { }
}

void disableInterrupts()
{
  // Enable EIC
  EIC->CTRL.bit.ENABLE = 0;
  while (EIC->STATUS.bit.SYNCBUSY == 1) { }
}

void disableUSB()
{
  //Disable USB
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;
  PM->AHBMASK.reg &= ~PM_AHBMASK_USB;
  PM->APBBMASK.reg &= ~PM_APBBMASK_USB;
}

void enableUSB()
{
//Enable USB
  PM->APBBMASK.reg |= PM_APBBMASK_USB;
  PM->AHBMASK.reg |= PM_AHBMASK_USB;
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
}

void RTC_int() //ISR for RTC interrupt overy minute
{
  disableInterrupts();
  rtcRead = !rtcRead;
  rtcFired = true;
  enableInterrupts();
}

void myISR() //ISR for button presses
{
  disableInterrupts();  // This must exist, especially with the LOW trigger.  For some reason
  // when a LOW is interrupt trigger is received it continues to fire and locks the processor.
  // I am using a LOW trigger as I am planning to put the processor into deep sleep where the clocks
  // are disabled. Other triggers (FALLING, RISING etc) need the clock running to be triggered.
  
  buttonRead = !buttonRead;
  buttonFired = true;
  enableInterrupts();
}

void setup() {
  Serial.begin(9600);
//  while (!Serial); 
  
  Wire.begin();

  // Set pullups for all the interrupts
  pinMode(MBUT, INPUT_PULLUP);
  pinMode(UBUT, INPUT_PULLUP);
  pinMode(DBUT, INPUT_PULLUP);
  pinMode(RTC_INT, INPUT_PULLUP); // RTC Interrupt

  pinMode(LED, OUTPUT);

  // Disable the RTC interrupts for the moment.
  MyDS3232.alarmInterrupt(ALARM_1, false);
  MyDS3232.alarmInterrupt(ALARM_2, false);


  // Attach button and RTC interrupt routine to the pins.
  attachInterrupt(digitalPinToInterrupt(MBUT), myISR, LOW); // when button B is pressed display 
  attachInterrupt(digitalPinToInterrupt(UBUT), myISR, LOW); // when button C is pressed display 
  attachInterrupt(digitalPinToInterrupt(DBUT), myISR, LOW); // when button C is pressed display battery status
  attachInterrupt(digitalPinToInterrupt(RTC_INT), RTC_int, LOW); // RTC Interrupt

  // Set RTC to interrupt every second for now just to make sure it works.
  // Will finally set to one minute.
#ifdef EVERY_SECOND
  MyDS3232.setAlarm(ALM1_EVERY_SECOND, 0, 0, 0);
  MyDS3232.alarmInterrupt(ALARM_1, true);
#endif
#ifdef EVERY_MINUTE
  MyDS3232.setAlarm(ALM2_EVERY_MINUTE, 0, 0, 0);
  MyDS3232.alarmInterrupt(ALARM_2, true);
#endif
}

void loop() {
  //wait-for-interrupt has no effect unless the sleep bit is set in the
  //System Control Register (SCR)(see setup, in the attachInterrupt area)
  //if the sleep bit is set, we wait after this instruction for an interrupt 
  PM->SLEEP.reg |= PM_SLEEP_IDLE_CPU;
//  PM->SLEEP.reg |= PM_SLEEP_IDLE_APB;

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;  // set to deep sleep (bit-1)

  // Clear the alarm interrupt in the RTC, else we will never wake up from sleep.
  // Very strange happening that only exhibits self when interrupt trigger is LOW.
#ifdef EVERY_SECOND
  uint8_t stat = MyDS3232.alarm(ALARM_1); 
#endif
#ifdef EVERY_MINUTE
  uint8_t stat = MyDS3232.alarm(ALARM_2); 
#endif
  
  __WFI(); // Now wait for interrupt.

  digitalWrite(LED, rtcRead ? HIGH : LOW);
}
