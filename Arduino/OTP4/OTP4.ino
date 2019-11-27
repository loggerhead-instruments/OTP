#include <SPI.h>    // arduino pro/pro mini, AtMega 3.3V 8 MHz
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>
#include <prescaler.h>

boolean tagID[32] = {0,1,0,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1};
int16_t threshold = 100; // threshold for detecting signal
uint16_t thresholdCount = 100; // n values need to exceed threshold in one buffer to trigger
uint8_t pulse;
boolean firstPulse = 1;

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
     //processBuf(); // process buffer first to empty FIFO so don't miss watermark
     //system_sleep();

      pulsePattern();  // test pulse pattern
     delay(1000);
     

     
     // ... ASLEEP HERE...
}


void processBuf(){
  while((lis2SpiFifoPts() * 3 > bufLength)){
    lis2SpiFifoRead(bufLength);  //samples to read
    if(detectSound()){
      digitalWrite(LED, HIGH);
      pulsePattern();      
      digitalWrite(LED, LOW);
    }  
  }
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
  pulse = 0;
  firstPulse = 1;

  // initialize PWM
  // https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
  TCCR2B = 0; // turn off
 
  // Start Timer 1 interrupt that will control changing of frequency or phase of each pulse in ping
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 20;  // compare match register  // compare match register 4MHz/256  15625 = 1s; 156 = 10 ms
  TCCR1B |= (1 << WGM12); // CTC Mode
  TCCR1B |= (1 << CS12); // 256 prescaler
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
}

ISR(TIMER1_COMPA_vect){
  if (firstPulse==1){
    firstPulse = 0;
    TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    OCR2B = 5;   // PWM high length
    TCCR2B = _BV(WGM22) | _BV(CS20);  // start Fast PWM; no prescaler
  }

  // check if time to turn off
  if(pulse>=32){
    TCCR2B = 0; // turn off PWM
    TCCR1A = 0; // turn off Timer 1
    TCCR1B = 0;
    TCNT1 = 0;
    digitalWrite(PWMPIN, LOW);
    return;
  }


  // OCR2A controls PWM period
  if(tagID[pulse]==0){
    OCR2A = 10; // 10=181.8 kHz
  }
  else OCR2A = 11; // 11=166.6 kHz
  
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
