#include <SPI.h>    // arduino pro/pro mini, AtMega 3.3V 8 MHz
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>
#include <prescaler.h>

boolean tagID[32] = {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1};
int16_t threshold = 100; // threshold for detecting signal
uint16_t thresholdCount = 100; // n values need to exceed threshold in one buffer to trigger


#define LED 4
#define PWMPIN 5
#define DATAOUT 11      //MOSI
#define DATAIN 12       //MISO
#define SPICLOCK 13      //sck
#define chipSelectPinAccel 9  
#define INT0 2
#define INT1 3

// pulseDelay = 3 for 400 kHz
// 19 works well for ring piezo (less ringing) determined empirically
//#define pulseDelay 19
// 100 is easy to hear
#define pulseDelay 100

#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)

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
  setClockPrescaler(1); //slow down clock to save battery 4 = 16x slower // makes 4 MHz clock
  pinMode(PWMPIN, OUTPUT); // output pin for OCR0B
  pinMode(LED, OUTPUT);
  
  // initalize the  data ready and chip select pins:
  pinMode(chipSelectPinAccel, OUTPUT);
  digitalWrite(chipSelectPinAccel, HIGH);
  pinMode(SPICLOCK, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);

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
}


void loop() {
     processBuf(); // process buffer first to empty FIFO so don't miss watermark
     //if(lis2SpiFifoStatus()==0) system_sleep();
     //if(lis2SpiFifoPts() < 128) system_sleep();
     system_sleep();
     // ... ASLEEP HERE...
}


void processBuf(){
  while((lis2SpiFifoPts() * 3 > bufLength)){
    lis2SpiFifoRead(bufLength);  //samples to read
    if(detectSound()){
      digitalWrite(LED, HIGH);
      pulsePattern();
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

void pulsePattern(){
  // 32-bit code
  for(int i=0; i<32; i++){
    pulseOut();
    for(int k=0; k<2*(tagID[i]+1); k++){
       DELAY_CYCLES(100);
    }
  }
}

void pulseOut(){
  // make a pulse of 2 cycles
  // using 3 cycle delay makes a 400 kHz square wave
  // when use loop, get extra delay for low side
  // when remove loop, don't get an output
  for(int n=0; n<1; n++){
    sbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseDelay);
    cbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseDelay);
    sbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseDelay);
    cbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseDelay);
  }
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
