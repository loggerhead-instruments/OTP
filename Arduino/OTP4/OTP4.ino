// OpenTag Ping 
// 

#include <SPI.h>    // arduino pro/pro mini, AtMega 3.3V 8 MHz
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>
#include <prescaler.h>
#include <avr/wdt.h>

//boolean tagID[32] = {0,1,0,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1};
//boolean tagID[16] = {0,0,1,0,0,0,1,0, 0,0,0,0,0,0,0,0};
//boolean tagID[4] = {0,0,1,0};
boolean tagID[1] = {0};
int16_t threshold = 100; // threshold for detecting signal
uint16_t thresholdCount = 100; // n values need to exceed threshold in one buffer to trigger
uint8_t pulse = 0; // index into tagID - we can also use register counting with timer1 and pin5: https://forum.arduino.cc/index.php?topic=494744.0
boolean firstPulse = 1; // flag for whether first pulse in sequence. Used to start PWM.
uint16_t bufCounter = 0; // counter for number of buffers processed. Used to control signal of tagID regardless of whether sound detected

// store number of bits - or use DEFINE when finalized
//uint8_t NBIT = *(&tagID + 1) - tagID;
#define NBIT (*(&tagID + 1) - tagID)


#define LED A2
#define PWMPIN 3
#define DATAOUT 11      //MOSI
#define DATAIN 12       //MISO
#define SPICLOCK 13      //sck
#define chipSelectPinAccel 9  
#define INT1 2


// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// default number of channels and sample rate
// can be changed by setup.txt file
// change lis2SpiFifoRead if choosing sample rate combo of 12 bit acc data
int nchan = 1;
int srate = 800;
int accelScale = 2;

// when storing magnitude of acceleraton watermark threshold are represented by 1Lsb = 3 samples
// max buffer is 256 sets of 3-axis data
#define FIFO_WATERMARK (0x80) // samples 0x0C=12 0x24=36; 0x2A=42; 0x80 = 128
#define bufLength 384 // samples: 3x watermark
int16_t accel[bufLength]; // hold up to this many samples


void setup() {
  //setClockPrescaler(1); //slow down clock to run at 1.8V; makes 4 MHz clock
  pinMode(PWMPIN, OUTPUT); // output pin for OCR2B
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  
  // initalize the  data ready and chip select pins:
  pinMode(chipSelectPinAccel, OUTPUT);
  digitalWrite(chipSelectPinAccel, HIGH);
  pinMode(SPICLOCK, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);

  // initialize watchdog timer. Remember to continuously re-initialize
  //wdt_enable(WDTO_1S); // watchdog timer with 8 s timeout

  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0)); // with breadboard, speeds higher than 1MHz fail

  int testResponse = lis2SpiTestResponse();
  while (testResponse != 67) {
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    testResponse = lis2SpiTestResponse();
  }

  lis2SpiInit();
  digitalWrite(LED, LOW);
  //wdt_reset();  

  // LED sequence for successful start: 3 medium, 1 long flash
  for(int i=0; i<3; i++) {
    delay(300);
    digitalWrite(LED, HIGH);
    delay(300);
    digitalWrite(LED, LOW);
  }
  delay(300);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  
}


void loop() {
  //processBuf(); // process buffer first to empty FIFO so don't miss watermark
  //system_sleep();

  // Regular transmissions
  pulsePattern(1);

  delay(500);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  delay(200);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  delay(200);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);

  
  //wdt_reset();  
     
  // 2: FSK
//  for (int j=0; j<50; j++){
//      
//    //tagID = {0,0,0,0,0,0,0,0};
//    for (int j1=8; j1<16; j1++){
//      tagID[j1] = 0;
//    }
//    
//    digitalWrite(LED, HIGH);
//    pulsePattern(1);
//    delay(200);
//    digitalWrite(LED, LOW);
//
//    //tagID = {0,0,0,0,1,1,1,1};
//    for (int j1=12; j1<16; j1++){
//      tagID[j1] = 1;
//    } 
//    pulsePattern(1);
//    delay(200);
//
//    //tagID = {1,0,1,0,1,1,1,1};
//    tagID[8] = 1;
//    tagID[10] = 1;
//    pulsePattern(1);
//    delay(300);
//
//    // Reset watchdog timer
//    wdt_reset();
//  }

}


void processBuf(){
  while((lis2SpiFifoPts() * 3 > bufLength)){
    bufCounter++;
    
    lis2SpiFifoRead(bufLength);  //samples to read
    if(detectSound()){ 
      pulsePattern(1);      
    }  
    else{
    // ping after bufCounter buffers
    if(bufCounter>30){
      digitalWrite(LED, HIGH);
      pulsePattern(0);  // pulse pattern when sound not detected
      bufCounter = 0;
      }
    }
  }
  digitalWrite(LED, LOW);
}


// simple algorithm to detect whether buffer contains sound
int diffData;

boolean detectSound(){
  // High-pass filter options:
  // diff()
  // IIR
  // FIR

  // Threshold options:
  // -fixed
  // -dynamic (e.g. 4 * SD)

  uint16_t nGtThreshold = 0;
  for (int i=1; i<bufLength; i++){
    diffData = accel[i] - accel[i-1];
    if (diffData > threshold){
      nGtThreshold += 1;
    }
  }
  if(nGtThreshold > thresholdCount) 
    return 1;
  else
    return 0;
}


void pulsePattern(boolean soundFlag){
  pulse = 0;

  // Information on timers here
  // https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
  // http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf

  // initialize PWM
  TCCR2B = 0; // turn off
 
  // Start Timer 1 interrupt that will control changing of frequency or phase of each pulse in ping
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 449;  // compare match register - OCR1A = 40 [cycles per bit] x (44+1) [ticks per cycle] - 1 = 1799 [for some reason, seems to be off by x2]
  TCCR1B |= (1 << WGM12); // CTC Mode
  TCCR1B |= (1 << CS10); //  no prescaler - 16-bit counter can accomodate up to 1400 cycles per bit...
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

  // Do we want to sleep here until pulse is done to prevent collisions?
  
}

ISR(TIMER1_COMPA_vect){

  // check if time to turn off
  if(pulse>NBIT){
    TCCR2A = 0; // turn off timer2 PWM
    TCCR2B = 0; // turn off PWM
    TCCR1A = 0; // turn off Timer 1
    TCCR1B = 0;
    TCNT1 = 0;
    return;
  }
  
  // FSK mode: Bit determines PWM2 pulse rate - OCR2A controls PWM period
  if(tagID[pulse]==0){
    OCR2A = 44; // 44=178 kHz (frequency, kHz = 1/ ( [OCR2A+1]/clock_speed) )
  }
  else {
    OCR2A = 46; // 46=170 kHz
  }
  
  // Initialize if this is first pulse
  if (pulse==0){
    TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    OCR2B = 22;   // PWM high length
    TCCR2B = _BV(WGM22) | _BV(CS20);  // start Fast PWM; no prescaler
  }
  
  pulse++;
}


void watermark(){
  // wake up
  
}

//****************************************************************  
// set system into the sleep state 
// system wakes up when interrupt detected
void system_sleep() {
  // make all pin inputs and enable pullups to reduce power
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  power_all_disable();
  attachInterrupt(digitalPinToInterrupt(INT1), watermark, LOW);
  sleep_mode();  // go to sleep
  // ...sleeping here....  
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(INT1));
  power_all_enable();
}
