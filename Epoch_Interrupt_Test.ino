
#include <Time.h>         //http://www.arduino.cc/playground/Code/Time  
#include <TimeLib.h>         //http://www.arduino.cc/playground/Code/Time  
#include <Wire.h>         //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include <DS3232RTC.h>    //http://github.com/JChristensen/DS3232RTC

#define LED 6
#define MBUT 4
#define UBUT 14
#define DBUT 22
#define RTC_INT 9


volatile boolean buttonRead = false; //variables in ISR need to be volatile
volatile boolean buttonFired = false; //variables in ISR need to be volatile
volatile boolean rtcRead = false; //variables in ISR need to be volatile
volatile boolean rtcFired = false; //variables in ISR need to be volatile
//RTCx DS3231M(RTCx::DS3231MAddress, RTCx::DS3231M);

DS3232RTC MyDS3232;

void RTC_int() //ISR for RTC interrupt overy minute
{
//  noInterrupts();
  detachInterrupt(digitalPinToInterrupt(RTC_INT)); // MUST remove the LOW interrupt else locks up and I think interrupt continues to fire.
  rtcRead = !rtcRead;
  rtcFired = true;
//interrupts();
}

void myISR() //ISR for button presses
{
  detachInterrupt(digitalPinToInterrupt(MBUT));
  detachInterrupt(digitalPinToInterrupt(UBUT));
  detachInterrupt(digitalPinToInterrupt(DBUT));
    buttonRead = !buttonRead;
    buttonFired = true;
}

void setup() {
  Serial.begin(9600);
  while (!Serial); 
  
  Wire.begin();

//  noInterrupts();

  pinMode(MBUT, INPUT_PULLUP);
  pinMode(UBUT, INPUT_PULLUP);
  pinMode(DBUT, INPUT_PULLUP);
  pinMode(RTC_INT, INPUT_PULLUP); // RTC Interrupt
  pinMode(LED, OUTPUT);

//  MyDS3232.setAlarm(ALM2_EVERY_MINUTE, 0, 0, 0);
//  MyDS3232.alarmInterrupt(ALARM_2, true);
  // Something wierd about attaching the interrupt when the signal is perhaps low as it locks up.
  MyDS3232.alarmInterrupt(ALARM_1, false);
  MyDS3232.alarmInterrupt(ALARM_2, false);
//  MyDS3232.setAlarm(ALM2_EVERY_MINUTE, 0, 0, 0);


//  MyDS3232.setAlarm(ALM1_EVERY_SECOND, 0, 0, 0);
//  MyDS3232.alarmInterrupt(ALARM_1, true);

  attachInterrupt(digitalPinToInterrupt(MBUT), myISR, LOW); // when button B is pressed display 
  Serial.println("after attach1");
  attachInterrupt(digitalPinToInterrupt(UBUT), myISR, LOW); // when button C is pressed display 
  Serial.println("after attach2");
  attachInterrupt(digitalPinToInterrupt(DBUT), myISR, LOW); // when button C is pressed display battery status
  Serial.println("after attach3");
  attachInterrupt(digitalPinToInterrupt(RTC_INT), RTC_int, LOW); // RTC Interrupt
  Serial.println("after attach4");

  MyDS3232.setAlarm(ALM1_EVERY_SECOND, 0, 0, 0);
  MyDS3232.alarmInterrupt(ALARM_1, true);

//                //on the zero (as in Feather M0), interrup#=pin#; we use digitalPintToInterrupt here to provide some portability
//                //if we change to a different board AND that board allows interrups on the same pins, we don't have to change anything to get the interrupt number
//                //if we're using the battery function, VBATPIN is A7, also D9, and button A uses D9, so we avoid conflict
//
  //set System Control Register (SCR) sleep bit to deep sleep (do once so wfi (wait for interrupt)  in loop waits)
  //There are 2 sleep modes, idle and standby (deep) Sleep bit is SCR bit 2, 0 is idle, 1 is standby
  // SCB->SCR |= 0b00000100; //just a test to see how to code binary--this works
  //SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;  // set to deep sleep (bit-1) by ORing with 100
  // SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;   //set to idle (bit=0) by ANDing with the inverse of above--leaves all bita alone except sleep, which ANDs to 0
  //  SCB->SCR &= 0b11111011;  //another test
  //There are 3 idle modes, 0 (CPU),1 (CPU,AHB clock),2 (CPU,AHB,APB). Set this if using IDLE
  // PM->SLEEP.reg |= PM_SLEEP_IDLE_APB;  //idle turn off CPU, AFB, APB //Advanced Peripheral Bus (APB)   Advanced High-performance Bus (AHB)

//  Serial.println("3");

  Serial.println("3 to sleep");
  delay(1000);
  Serial.println("2 to sleep");
  delay(1000);
  Serial.println("1 to sleep");
  delay(1000);

//  interrupts(); //enable interrupts (should not need to do this, but just for drill...)
//  attachInterrupt(digitalPinToInterrupt(RTC_INT), RTC_int, LOW); // RTC Interrupt
}

void loop() {
  delay(100);
  
//  MyDS3232.alarmInterrupt(ALARM_2, true);
  //the wfi() means we only progress in loop on an interrupt, either button B or C pressed invoking headingISR or batteryISR
  //which set the corresponding booleans
  //If it was C, we display the battery status for 3 seconds and go on
  //in either case, we display the compass heading, complete the loop, and wait for the next button press
    
  //wait-for-interrupt has no effect unless the sleep bit is set in the
  //System Control Register (SCR)(see setup, in the attachInterrupt area)
//  while (!buttonRead) { //if an ISR has not set one of the booleans, wait (they're both initalized to false)
  //if the sleep bit is set, we wait after this instruction for an interrupt 
//  PM->SLEEP.reg |= PM_SLEEP_IDLE_CPU;
  //PM->SLEEP.reg |= PM_SLEEP_IDLE_APB;

//  system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
//  system_sleep();
//SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;  // set to deep sleep (bit-1) by ORing with 100
//  __WFI();  //Double underscore!! (took me a few looks to see that)
//  }

//    digitalWrite(LED, LOW);

//  uint8_t stat = DS3231M.getStatus();
//  Serial.print("DS3231M.getStatus():");
//  Serial.println(stat, HEX);

//  uint8_t rtcInt = digitalRead(RTC_INT);
//  Serial.print(" rtcInt:");
//  Serial.print(rtcInt, HEX);
//  Serial.print(" rtcRead:");
//  Serial.println(rtcRead, HEX);
//  uint8_t but = digitalRead(DBUT);
//  Serial.print(" DBUT:");
//  Serial.print(but, HEX);
  
  uint8_t stat = MyDS3232.alarm(ALARM_1); // Clear the interrupt
//  Serial.print("1)MyDS3232.alarm(ALARM_1):");
//  Serial.print(stat, HEX);
//  Serial.print(" buttonRead:");
//  Serial.print(buttonRead, HEX);
//  Serial.print(" rtcRead:");
//  Serial.println(rtcRead, HEX);
  
//  if (buttonRead) // Alarm triggered
//  {
//      digitalWrite(LED, stat ? HIGH : LOW);
//  }

    digitalWrite(LED, rtcRead ? HIGH : LOW);
//    digitalWrite(LED, buttonRead ? HIGH : LOW);
    // Must reattach interrupt each time as the handler MUST remove the interrupt when mode is LOW.  Other modes do not need this.
    if (rtcFired)
    {
      Serial.println("attach");
      attachInterrupt(digitalPinToInterrupt(RTC_INT), RTC_int, LOW); // RTC Interrupt
      rtcFired = false;
    }

    if (buttonFired)
    {
      attachInterrupt(digitalPinToInterrupt(MBUT), myISR, LOW); // when button B is pressed display 
      attachInterrupt(digitalPinToInterrupt(UBUT), myISR, LOW); // when button C is pressed display 
      attachInterrupt(digitalPinToInterrupt(DBUT), myISR, LOW); // when button C is pressed display battery status
      buttonFired = false;
    }
}
