
/*
 * Simple GPS Disciplint Oscilaltor
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
#include <Wire.h>
#include <si5351.h>

//TinyGPSPlus gps;
Si5351 oscillator;

#define OnePPSinput              2
#define LED1                     10
#define LED2                     11  // 1 PPS 
#define LED3                     12

unsigned long XtalFreq = 100000000;      //100MHz   40 sec * 2.500.000 = 100.000.000
unsigned long XtalFreq_old = 100000000;

long stab;
long correction_size = 0;
byte stab_count = 44;

unsigned long correction_counter = 0;
unsigned long Freq1 = 10000000;  //Channel1 Frequency
unsigned long Freq2 = 27000000;  //Channel2 Frequency

unsigned int tick_count = 0;
unsigned int tick_count2 = 0;
int GPSisValid = false;
boolean GPSstatus = true;
unsigned long pps_correct;
byte pps_valid = 1;

/*
 * Environment setup
 */
void setup()
{
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(OnePPSinput, INPUT);                   //GPS 1pps input
  //digitalWrite(OnePPSinput, HIGH);

  TCCR1B = 0;                                    //TIMER5 DISABLED
  TCCR1A = 0;                                    //RESET TIMER
  TCNT1  = 0;                                    //RESET COUNTER
  TIFR1  = 1;                                    //RESET OVERFLOW
  TIMSK1 = 1;                                    //OVERFLOW FLAG

  //Serial port for configuration and debug information
  Serial.begin(9600);
  Serial.println("Simple GPS Disciplined Oscillator");


  //Initialize the si5351 Oscillators
  oscillator.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  oscillator.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  oscillator.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);

  // CLK0 output 2,5MHz - atmega328 can capture
  oscillator.set_ms_source(SI5351_CLK0, SI5351_PLLA);
  oscillator.set_freq(250000000ULL, SI5351_CLK0);      //Set sampling output to 2.5 MHz
  oscillator.set_ms_source(SI5351_CLK1, SI5351_PLLB);
  oscillator.set_freq(Freq1 * SI5351_FREQ_MULT, SI5351_CLK1);
  //oscillator.set_freq(Freq2 * SI5351_FREQ_MULT, SI5351_CLK2);
  oscillator.update_status();

/*
  GPSproces(6000);

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS connected");
    delay(5000);
    GPSstatus = false;
  }
  if (GPSstatus == true) {
    Serial.println("Waiting for SAT");
    do {
      GPSproces(1000);
    } while (gps.satellites.value() == 0);
*/
    attachInterrupt(0, OnePPSintVect, RISING);
    TCCR1B = 0;
    tick_count = 0;
    correction_counter = 0;
    GPSisValid = 1;
//  }
}


void loop()
{
  if (tick_count2 != tick_count) {
    tick_count2 = tick_count;
    pps_correct = millis();
  }
  if (tick_count < 4 ) {
    //GPSproces(0);
  }

  if (millis() > pps_correct + 1200) {
    pps_valid = 0;
    pps_correct = millis();

  }
}



/*
 * One PPS interrupt
 */
void OnePPSintVect()
{
  //Serial.println(correction_counter);
  tick_count++;
  stab_count--;
  if (tick_count == 4)                               // Start counting the 2.5 MHz signal from Si5351A CLK0
  {
    Serial.println("TC 4");
    TCCR1B = 7;                                  //Clock on rising edge of pin 5
  }
  if (tick_count == 44)                              //The 40 second gate time elapsed - stop counting
  {
    Serial.println("TC 44");
    TCCR1B = 0;                                  //Turn off counter
    if (pps_valid == 1) {
      XtalFreq_old = XtalFreq;
      XtalFreq = correction_counter * 0x10000 + TCNT1;           //Calculate correction factor
    }
    TCNT1 = 0;                                       //Reset count to zero
    correction_counter = 0;
    tick_count = 0;                                  //Reset the seconds counter
    pps_valid = 1;
    //Serial.begin(9600);
    stab_count = 44;
  }
}

//*******************************************************************************
// TIMER1 Overflow Interrupt
ISR(TIMER1_OVF_vect)
{
  //digitalWrite(LED3,!digitalRead(LED3));
  //digitalWrite(LED2,HIGH);
  //digitalWrite(LED3,HIGH);
  //Serial.print("X");
  correction_counter++;                                          
  TIFR1 = (1 << TOV1);                             //OVERFLOW FLAG CLEAR
}

//********************************************************************************
//                                STAB on LCD  stabilnośc częstotliwości
//********************************************************************************
void stab_on_lcd() {
  float stab_float;
  long pomocna;

  stab = XtalFreq - 100000000;
  stab = stab * 10 ;
  if (stab > 100 || stab < -100) {
    correction_size = correction_size + stab;
  }
  else if (stab > 20 || stab < -20) {
    correction_size = correction_size + stab / 2;
  }
  else correction_size = correction_size + stab / 4;
  pomocna = (10000 / (Freq1 / 1000000));
  stab = stab * 100;
  stab = stab / pomocna;
  stab_float = float(stab);
  stab_float = stab_float / 10;
}

void correct_oscillator()
{
  oscillator.set_correction(correction_size, SI5351_PLL_INPUT_XO);
}

static void GPSproces(unsigned long ms)
{
  unsigned long start = millis();
//  do
//  {
//    while (Serial.available())
//      gps.encode(Serial.read());
//  } while (millis() - start < ms);
}
