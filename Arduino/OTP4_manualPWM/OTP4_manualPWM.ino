// OpenTag Ping 
// 

#include <SPI.h>    // arduino pro/pro mini, AtMega 3.3V 8 MHz
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>
#include <prescaler.h>
#include <avr/wdt.h>

// Transmission settings
//boolean tagID[32] = {0,1,0,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1};
boolean tagID[16] = {0,0,1,0,0,0,0,1,  0,0,0,0,0,0,0,0};
//boolean tagID[4] = {0,0,1,0};
//boolean tagID[1] = {0};
uint8_t pulse = 0; // index into tagID - we can also use register counting with timer1 and pin5: https://forum.arduino.cc/index.php?topic=494744.0

// Detector settings
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
#define CHG 6 // charge the piezo

// pulseDelay = 3 for 400 kHz
// 19 works well for ring piezo (less ringing) determined empirically
// 11 cycles per half-cycle of a 182 kHz sine wave - at 4 MHz
// 13 cycles per half-cycle of a 154 kHz sine wave - at 4 MHz (157 kHz piezo) - NO, this is 111 kHz
// 16 cycles per half-cycle of a 125 kHz sine wave - at 4 MHz (127 kHz piezo)
// 100 is easy to hear

// Define individual pulse settings
#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)
#define chargeDelay 2 // charging right now does not affect stimulus amplitude
#define pulseOnDelay 38 // (38+2) cycles = 160 kHz
#define pulseOffDelay 2 // 
#define pulseOnDelayB 40 // (40+2) cycles = 154 kHz


// Define bit settings
#define numBits 16   // Number of bits for ID signal
#define bitCycles 40 // Number of sine waves per bit
#define bitShift 20   // Number of clock cycles for phase shift
#define bitInt 20     // Number of clock cycles between bits

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
  //setClockPrescaler(1); //slow down clock to run at 1.8V; makes 4 MHz clock
  pinMode(PWMPIN, OUTPUT); // output pin for OCR2B
  //pinMode(CHG, OUTPUT);
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
  digitalWrite(LED, LOW);

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

    //Acoustic testing in tank
    //pulseOut();
    //pulsePattern();
    //digitalWrite(LED, HIGH);
    //delay(500);
    //digitalWrite(LED, LOW);
    //delay(500);
    
    
    // Eel Pond ID encoding

    // Start Transmission
    delay(1000);
    pulseOutMarker();
    delay(100);
    pulseOutMarker();
    delay(100);
    pulseOutMarker();
    delay(1000);
    
    // 1: BPSK
    for (int j=0; j<20; j++){
      
      //tagID = {0,0,0,0,0,0,0,0};
      for (int j1=8; j1<16; j1++){
        tagID[j1] = 0;
      }
      digitalWrite(LED, HIGH);
      pulsePattern();
      delay(200);
      digitalWrite(LED, LOW);

      //tagID = {0,0,0,0,1,1,1,1};
      for (int j1=12; j1<16; j1++){
        tagID[j1] = 1;
      } 
      pulsePattern();
      delay(200);

      //tagID = {1,0,1,0,1,1,1,1};
      tagID[8] = 1;
      tagID[10] = 1;
      pulsePattern();
      delay(400);
    }
   

    // Switch transmission
    delay(1000);
    pulseOutMarker();
    delay(100);
    pulseOutMarker();
    delay(1000);

    // 2: FSK
    for (int j=0; j<20; j++){
      
      //tagID = {0,0,0,0,0,0,0,0};
      for (int j1=8; j1<16; j1++){
        tagID[j1] = 0;
      }
      digitalWrite(LED, HIGH);
      pulsePatternFSK();
      delay(200);
      digitalWrite(LED, LOW);

      //tagID = {0,0,0,0,1,1,1,1};
      for (int j1=12; j1<16; j1++){
        tagID[j1] = 1;
      } 
      pulsePatternFSK();
      delay(200);

      //tagID = {1,0,1,0,1,1,1,1};
      tagID[8] = 1;
      tagID[10] = 1;
      pulsePatternFSK();
      delay(400);
    }

}


void processBuf(){
  while((lis2SpiFifoPts() * 3 > bufLength)){
    bufCounter++;
    
    lis2SpiFifoRead(bufLength);  //samples to read
    if(detectSound()){ 
      pulsePattern();  // NEED INFO ON SOUND DETECTED    
    }  
    else{
    // ping after bufCounter buffers
    if(bufCounter>30){
      digitalWrite(LED, HIGH);
      pulsePattern();  // pulse pattern when sound not detected
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

void pulsePattern(){
  // 32-bit code tagID
  for(int i=0; i<numBits; i++){
    if (tagID[i]) {
      DELAY_CYCLES(bitShift); // Phase shifted by bitShift cycles
      pulseOut();
    }
    else {
      pulseOut();  
      DELAY_CYCLES(bitShift);
    }
  DELAY_CYCLES(bitInt);
  }
}

// FSK ENCODING

void pulsePatternFSK(){
  // 32-bit code tagID
  for(int i=0; i<numBits; i++){
    if (tagID[i]) {
      pulseOut();
    }
    else {
      pulseOutB();  
    }
  //DELAY_CYCLES(bitInt);
  }
}

void pulseOut(){
  // make a pulse of bitCycles cycles
  // using 3 cycle delay makes a 400 kHz square wave
  // when use loop, get extra delay for low side
  // when remove loop, don't get an output
    
  for(int n=0; n<bitCycles; n++){
    sbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseOnDelay);
    cbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseOffDelay);
  }
}

void pulseOutB(){
    
  for(int n=0; n<bitCycles; n++){
    sbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseOnDelayB);
    cbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseOffDelay);
  }
}

void pulseOutMarker(){
    
  for(int n=0; n<6400; n++){
    sbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseOnDelay);
    cbi(PORTD, PWMPIN);
    DELAY_CYCLES(pulseOffDelay);
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
