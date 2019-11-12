#include <SPI.h>    // arduino pro/pro mini, AtMega 3.3V 8 MHz
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>
#include <prescaler.h>

boolean tagID[32] = {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1};
int16_t threshold = 100; // threshold for detecting signal
uint16_t thresholdCount = 100; // n values need to exceed threshold in one buffer to trigger


#define LED A2
#define PWMPIN 3
#define DATAOUT 11      //MOSI
#define DATAIN 12       //MISO
#define SPICLOCK 13      //sck
#define chipSelectPinAccel 9  
#define INT1 2

// pulseDelay = 3 for 400 kHz
// 19 works well for ring piezo (less ringing) determined empirically
// 11 cycles per half-cycle of a 182 kHz sine wave - at 4 MHz
// 13 cycles per half-cycle of a 154 kHz sine wave - at 4 MHz (157 kHz piezo)
// 16 cycles per half-cycle of a 125 kHz sine wave - at 4 MHz (127 kHz piezo)
// 100 is easy to hear
#define pulseDelay 26
#define offDelay 2

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
  digitalWrite(LED, HIGH);

//// https://www.eprojectszone.com/how-to-modify-the-pwm-frequency-on-the-arduino-part1/
//TCCR0A=0;//reset the register
//TCCR0B=0;//reset tthe register
//TCCR0A=0b00110011;// fast pwm mode on pin 5 (COM0B1)
////TCCR0B=0b00000010;// prescaler 8
//TCCR0B=0b00001001;// no prescaler; WGM02 is 1 (toggle Oc0A on Compare Match)
//OCR0A=72; // 4 MHz/72 = 55.5 kHz
////OCR0A=255;  // 4 MHz/255 (largest value because 8-bit timer) = 15.6 kHz
  
  
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
     //system_sleep();

     //pulseOut();
     delay(10);
     digitalWrite(LED, LOW);

     
     // ... ASLEEP HERE...
}


void processBuf(){
  while((lis2SpiFifoPts() * 3 > bufLength)){
    lis2SpiFifoRead(bufLength);  //samples to read
    if(detectSound()){
      digitalWrite(LED, HIGH);
     // pulsePattern();
      pulseOut();
    }  
  }
 // digitalWrite(LED, LOW);
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
  // PWM
  // when tried this got random bursts 
//  TCCR0A=0;//reset the register
//  TCCR0B=0;//reset the register
//  TCCR0A=0b00110011;// fast pwm mode on pin 5 (COM0B1)
//  TCCR0B=0b00001001;// no prescaler; WGM02 is 1 (toggle Oc0A on Compare Match)
//  OCR0A=23; // 4 MHz/72 = 55.5 kHz; 4 MHz/23 = 173.9 kHz
//
//  DELAY_CYCLES(20);
//
//  TCCR0A=0;//reset the register
//  TCCR0B=0;//reset the register

  // when use loop, get extra delay
  // amplitude is higher when pulseDelay is longer
  // amplitude is highest when offDelay is 0 (up to 40V)
  
    sbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseDelay);
    cbi(PORTD, PWMPIN);
    DELAY_CYCLES(offDelay);

    sbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseDelay);
    cbi(PORTD, PWMPIN);
    DELAY_CYCLES(offDelay);
      
    sbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseDelay);
    cbi(PORTD, PWMPIN);
    DELAY_CYCLES(offDelay);

    sbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseDelay);
    cbi(PORTD, PWMPIN);
    DELAY_CYCLES(offDelay);
      
    sbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseDelay);
    cbi(PORTD, PWMPIN);
    DELAY_CYCLES(offDelay);

    sbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseDelay);
    cbi(PORTD, PWMPIN);
    DELAY_CYCLES(offDelay);
      
    sbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseDelay);
    cbi(PORTD, PWMPIN);
    DELAY_CYCLES(offDelay);

    sbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseDelay);
    cbi(PORTD, PWMPIN);
    DELAY_CYCLES(offDelay);
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
