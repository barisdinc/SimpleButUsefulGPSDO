#include <Arduino.h>

/*
 * Simple GPS Disciplined Oscilaltor
 * Author  : Baris DINC
 *   Date  : August 2020
 * License : GPL Gnu Public License  
 */

/* TODO LIST
 *  Speed up corrections (not every 40 seconds, make it every second)
 *  
 *  
 */
 
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <ctype.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Wire.h>
#include <si5351.h>
//#include <EEPROMex.h>

// The TinyGPS++ object
TinyGPSPlus gps;

#define RXPIN 6
#define TXPIN 7
#define GPSBAUD 9600
SoftwareSerial mySerial = SoftwareSerial (RXPIN, TXPIN);

Si5351 si5351;

// Set up MCU pins
#define ppsPin                   2
#define LED1                    10
#define LED2                    11
#define LED3                    12
#define LED4                    13


// configure variables
unsigned long XtalFreq = 400000000;
unsigned long XtalFreq_old = 400000000;
long stab;
long correction = 0;
float counter_error = 30;
byte stab_count = 44;
unsigned long mult = 0;
//int second = 0, minute = 0, hour = 0;
unsigned int tcount = 0;
unsigned int tcount2 = 0;
int validGPSflag = false;
char c;
boolean newdata = false;
boolean GPSstatus = true;
byte new_freq = 1;
//unsigned long freq_step = 1000;
//byte menu = 4;
//boolean time_enable = true;
unsigned long pps_correct;
byte pps_valid = 1;

static void GPSproces(unsigned long ms);
void correct_si5351();
void calculate_correction();
void PPSinterrupt();



//*************************************************************************************
//                                    SETUP
//*************************************************************************************
void setup()
{
  TCCR1B = 0;                                    //Disable Timer5 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag
  pinMode(ppsPin, INPUT);                        // Inititalize GPS 1pps input
  digitalWrite(ppsPin, HIGH);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_4MA);
//  //si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_6MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);

  Serial.begin(9600);
  mySerial.begin(GPSBAUD);
digitalWrite(LED1,HIGH);
digitalWrite(LED2,HIGH);
digitalWrite(LED3,HIGH);

  Serial.println("GPS Discipllined Oscillator v1.0 (2020)");
  Serial.println("---------------------------------------");

  // Set CLK0 to output 2,5MHz
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);
  si5351.set_freq(250000000ULL, SI5351_CLK0);
  //si5351.set_ms_source(SI5351_CLK2, SI5351_PLLB);
  ////si5351.set_freq(4000000000ULL, SI5351_CLK1);
  si5351.set_freq(4000000000ULL, SI5351_CLK2);
  si5351.update_status();

  Serial.print("GPS sinyali bekleniyor...");

  GPSproces(6000);
  Serial.println("[OK]");

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("GPS bulunamadi!!!");
    delay(5000);
    GPSstatus = false;
  }
  if (GPSstatus == true) {
    Serial.print("GPS Fix bekleniyor.....");
    do {
      GPSproces(1000);
    } while (gps.satellites.value() == 0);
    Serial.println("[OK]");

    attachInterrupt(0, PPSinterrupt, RISING);
    TCCR1B = 0;
    tcount = 0;
    mult = 0;
    validGPSflag = 1;
  }
}

void loop()
{
  if (tcount2 != tcount) {
    tcount2 = tcount;
    pps_correct = millis();
  }
  if (tcount < 4 ) {
    GPSproces(0);
  }
  if (new_freq == 1) {
    correct_si5351();
    new_freq = 0;
  }
  
  if (millis() > pps_correct + 1200) {
    pps_valid = 0;
    pps_correct = millis();
  }
}

void PPSinterrupt()
{
  tcount++;
  stab_count--;
  if (tcount == 4)                               // Start counting the 2.5 MHz signal from Si5351A CLK0
  {
    TCCR1B = 7;                                  //Clock on rising edge of pin 5
    Serial.println(" [INT] ");
  }
  if (tcount == 44)                              //The 40 second gate time elapsed - stop counting
  {
    TCCR1B = 0;                                  //Turn off counter
    if (pps_valid == 1) {
      XtalFreq_old = XtalFreq;
  
      XtalFreq = mult * 0x10000 + TCNT1;           // 65536 (timer max sayim) * mult (kac sayim) + TCNT1 (timer da kalan artiklar) 
      new_freq = 1;

    }
    TCNT1 = 0;                                   //Reset count to zero
    mult = 0;
    tcount = 0;                                  //Reset the seconds counter
    pps_valid = 1;
    stab_count = 44;
    calculate_correction();
  }
      
  char sz[32];
  sprintf(sz, "%d ", stab_count);
  Serial.print(sz);
}

ISR(TIMER1_OVF_vect)
{
  mult++;                                          //Increment multiplier
  //Serial.print(".");
  digitalWrite(LED4,mult && 2);
  TIFR1 = (1 << TOV1);                             //Clear overlow flag
}


void calculate_correction() {
  stab = (XtalFreq - 400000000) * 10;

  if (stab > 100 || stab < -100) {
    correction = correction + stab;
  }
  else if (stab > 20 || stab < -20) {
    correction = correction + stab / 2;
  }
  else correction = correction + stab / 4;
  
  char sz[32];
  sprintf(sz, " DUZELTME : %ld mHz", stab);
  Serial.println(sz);
}

void correct_si5351()
{
  si5351.set_correction(correction, SI5351_PLL_INPUT_XO);
  si5351.set_freq(250000000ULL, SI5351_CLK0);
  si5351.set_freq(4000000000ULL, SI5351_CLK2);

}

static void GPSproces(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (mySerial.available())
      //char c = mySerial.read();
      //Serial.print(c);
      //gps.encode(c);
      gps.encode(mySerial.read());
      
  } while (millis() - start < ms);
}
