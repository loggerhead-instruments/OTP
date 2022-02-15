// OpenTag Ping 
// OTP6: Support for pressure sensor
// 

// Fuses
// Extended: FF
// High: D9
// Low: E2

// To Do:
// - depth with ping

// baseline current no transmission: 295 uA
// baseline current  (transmitting single pulse): 330 uA
// 256 ms for pulse, 1.87 mA

#include <SPI.h>    // arduino pro/pro mini, AtMega328p 3.3V 8 MHz
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>
#include <prescaler.h>
#include <avr/wdt.h>

// ID Codes
// first position is reserved for indicating type of signal 0 = detection; 1 = no detection ping
// boolean tagID[8] = {1,1,1,1,0,0,0,0};  // tag 1
boolean tagID[8] = {0,0,0,0,1,1,1,1};  // tag 2
// boolean tagID[8] = {1,1,1,1,1,1,1,1};  // tag 3
// boolean tagID[8] = {0,0,0,0,0,0,0,0};  // tag 4
// boolean tagID[8] = {1,0,0,1,1,1,0,0};  // tag 5
// boolean tagID[8] = {1,1,1,0,0,0,1,1};  // tag 6

uint8_t pulse = 0; // index into tagID - we can also use register counting with timer1 and pin5: https://forum.arduino.cc/index.php?topic=494744.0

// Detector settings
#define DET_THRESHOLD 450 // Detection threshold - 110 [mg] / 0.2441 [mg/sample]
#define DET_CRIT 8        // Critical number of detected blocks
#define DET_BLOCK 8       // Number of accelerometer samples per block

// OLD DETECTOR SETTINGS
int16_t threshold = 100; // threshold for detecting signal
uint16_t thresholdCount = 100; // n values need to exceed threshold in one buffer to trigger
uint16_t bufCounter = 0; // counter for number of buffers processed. Used to control signal of tagID regardless of whether sound detected

// calculate number of bits based on tagID
//uint8_t NBIT = *(&tagID + 1) - tagID;
#define NBIT (*(&tagID + 1) - tagID)


#define LED A2
#define PWMPIN 3
#define DATAOUT 11      //MOSI
#define DATAIN 12       //MISO
#define SPICLOCK 13      //sck
#define chipSelectPinAccel 9  
#define INT1 2


// defines for setting and clearing register bits - check if we use these, switch to _BV
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
  delay(2000);
  setClockPrescaler(1); //slow down clock to run at 1.8V; makes 4 MHz clock
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
  wdt_enable(WDTO_8S); // watchdog timer with 8 s timeout

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
  wdt_reset();  

  // LED sequence for successful start: 10 flash
  for(int i=0; i<3; i++) {
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
  }  
}

int loopCounter = 0;
void loop() {
  // Process accelerometer data and look for calls
  processBuf(); // process buffer first to empty FIFO so don't miss watermark

  // Go to sleep
  wdt_reset(); // reset watchdog timer (will wake up system after 8s)
  system_sleep();

    // sleep here, then end loop
}


void processBuf(){
  while((lis2SpiFifoPts() * 3 > bufLength)){
    bufCounter++;
    
    lis2SpiFifoRead(bufLength);  //samples to read
    if(detectSound()){ 
    //if(detectVocs()){ 
      digitalWrite(LED, HIGH);
      pulsePattern(1); 
      delay(100);     
    }  
    else{
    // ping after bufCounter buffers
    if(bufCounter>120){
      digitalWrite(LED, HIGH);
      pulsePattern(0);  // pulse pattern when sound not detected
      bufCounter = 0;
      delay(100);
      }
    }
  }
  digitalWrite(LED, LOW);
}

// simple algorithm to detect whether buffer contains sound
int diffData;

boolean detectVocs() {
  
  // Create variables for detector
  int dt = 0; 
  boolean blockdone = 0;
  int blockrem = 0;

  // Go through sequence
  for (int i=1; i<bufLength; i++){
    blockrem = i%DET_BLOCK; // calculate how far from new block
    if ( blockrem == 0) {
      blockdone=0; // initialize new block
    } 
    
    // check if detected
    if (blockdone==0) {
      // Calculate differential acceleration
      diffData = accel[i] - accel[i-1];      
      if (abs(diffData)>DET_THRESHOLD) {
        dt++;
        blockdone=1; // only count one detection per block
      }
    }

  }
 
  if (dt>DET_CRIT) {
    // Found a call!
    return 1;
  }
  else { 
    return 0;
  }
}

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

  tagID[0] = soundFlag;
 
  // Start Timer 1 interrupt that will control changing of frequency or phase of each pulse in ping
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 3600;  //449 compare match register - OCR1A = 40 [cycles per bit] x (44+1) [ticks per cycle] - 1 = 1799 [for some reason, seems to be off by x2]
  TCCR1B |= (1 << WGM12); // CTC Mode
  TCCR1B |= (1 << CS10); //  no prescaler - 16-bit counter can accomodate up to 1400 cycles per bit...
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

  // Do we want to sleep here until pulse is done to prevent collisions?
  
}

// Fast PWM Mode with OCRA top
// OCR2A value is the top value of the timer. So, it counts sawtooth 0 to OCR2A, 0-OCR2A, 0-OCR2A
// OCR2B value goes high until counter reaches the OCR2B value

// Ping generation for FSK mode
ISR(TIMER1_COMPA_vect){

  // check if time to turn off
  if(pulse>=NBIT){
    TCCR2A = 0; // turn off timer2 PWM
    TCCR2B = 0; // turn off PWM
    TCCR1A = 0; // turn off Timer 1
    TCCR1B = 0;
    TCNT1 = 0;
    return;
  }
  
  // FSK mode: Bit determines PWM2 pulse rate - OCR2A controls PWM period
//  50  78431.37255
//51  76923.07692
//52  75471.69811
//53  74074.07407
// 319 12500
// 399 10000
  if(tagID[pulse]==0){
    OCR2A = 50; // (frequency, kHz = 1/ ( [OCR2A+1]/clock_speed) )
  }
  else {
    OCR2A = 51; //
  }
  
  // Initialize if this is first pulse
  if (pulse==0){
    OCR2B = 25;   // PWM high length, set to half of OCR2A (round down)
    TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(WGM22) | _BV(CS20);  // start Fast PWM (WGM22, WGM21, WGM20 all 1); no prescaler
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
