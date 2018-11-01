#include <SPI.h>
#include <avr/io.h>
#include <util/delay.h>
#include <prescaler.h>
#include <MsTimer2.h>

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
#define pulseDelay 19

#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

boolean tagID[32] = {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1};

// default number of channels and sample rate
// can be changed by setup.txt file
int nchan = 1;
int srate = 1600;
int accelScale = 2;

// when storing magnitude of acceleraton watermark threshold are represented by 1Lsb = 3 samples
// max buffer is 256 sets of 3-axis data
#define FIFO_WATERMARK (0x80) // samples 0x0C=12 0x24=36; 0x2A=42; 0x80 = 128
int bufLength = 128; // samples: 3x watermark
int16_t accel[384]; // hold up to this many samples

void setup() {
  setClockPrescaler(1); //slow down clock to save battery 4 = 16x slower
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

  DDRD = DDRD | B00100000;
  MsTimer2::set(50, ping); // 500ms period
  MsTimer2::start();
}


void loop() {


}
void ping(){
  static boolean output = HIGH;
  digitalWrite(LED, output);
  output = !output;
  pulsePattern();
  
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
  // make a pulse of 4 cycles
  // using 3 cycle delay makes a 400 kHz square wave
  // when use loop, get extra delay for low side
  // when remove loop, don't get an output
  for(int n=0; n<1; n++){
    sbi(PORTD, 5);
    DELAY_CYCLES(pulseDelay);
    cbi(PORTD, 5);
    DELAY_CYCLES(pulseDelay);
    sbi(PORTD, 5);
    DELAY_CYCLES(pulseDelay);
    cbi(PORTD, 5);
    DELAY_CYCLES(pulseDelay);
  }
}
