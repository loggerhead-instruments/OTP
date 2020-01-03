#include <SPI.h>    // arduino pro/pro mini, AtMega 3.3V 8 MHz
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>
#include <prescaler.h>

//boolean tagID[32] = {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1};
boolean tagID[16] = {0,0,1,0,0,0,1,0, 0,0,0,0,0,0,0,0};
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
#define IR 0 // IR remote control
#define CHG 6 // charge the piezo

// pulseDelay = 3 for 400 kHz
// 19 works well for ring piezo (less ringing) determined empirically
// 11 cycles per half-cycle of a 182 kHz sine wave - at 4 MHz
// 13 cycles per half-cycle of a 154 kHz sine wave - at 4 MHz (157 kHz piezo) - NO, this is 111 kHz
// 16 cycles per half-cycle of a 125 kHz sine wave - at 4 MHz (127 kHz piezo)
// 100 is easy to hear

// Define individual pulse settings
#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)
#define pulseOnDelay 44  // keep pwm activated for this duration
#define pulseOnDelayB 30 // Alternative pulseOn duration for FSK encoding - higher freq
#define pulseOffDelay 0  // 

// Define bit settings
#define numBits 1   // Number of bits for ID signal
#define bitCycles 1 // Number of sine waves per bit
#define bitShift 22  // Number of clock cycles for phase shift
volatile int pulseCounter;

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// default number of channels and sample rate can be changed by setup.txt file
// change lis2SpiFifoRead if choosing sample rate combo of 12 bit acc data
int nchan = 1;
int srate = 800;
int accelScale = 2;

// when storing magnitude of acceleration watermark threshold are represented by 1Lsb = 3 samples
// max buffer is 256 sets of 3-axis data
#define FIFO_WATERMARK (0x80) // samples 0x0C=12 0x24=36; 0x2A=42; 0x80 = 128
#define bufLength 384 // samples: 3x watermark
int16_t accel[bufLength]; // hold up to this many samples
  
void setup() {
  //setClockPrescaler(1); //slow down clock to save battery 4 = 16x slower // makes 4 MHz clock
  pinMode(PWMPIN, OUTPUT); // output pin for OCR0B
  pinMode(CHG, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(IR, INPUT);
  
  
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

    // Semideep tank:
    // Individual pulses
    //pulseOut();
    //delay(400);

    // Semideep tank:
    // Individual pulses with charge (chargeDelay 2 and 10 tried)
    //for (int j2=0; j2<100; j2++) {
    //    sbi(PORTD, CHG);
    //    DELAY_CYCLES(chargeDelay);
    //    cbi(PORTD, CHG);
    //    DELAY_CYCLES(chargeDelay);
    //}
    //pulseOut();
    //delay(400);

    // Semideep tank: PulsePattern
    digitalWrite(LED, HIGH);
    pulsePattern();
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);

     // Let's just pulse away
     //pulsePattern();
     //delay(10);
     //pulseOut();
     //delay(1000);
     
     
     //deactivated to troubleshoot
     //processBuf(); // process buffer first to empty FIFO so don't miss watermark
     
     //if(lis2SpiFifoStatus()==0) system_sleep();
     //if(lis2SpiFifoPts() < 128) system_sleep();

     //deactivated to troubleshoot
     //system_sleep();
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


// Transmit 32-bit tagID code
void pulsePattern(){
  // Configure timer2
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= (1 << WGM21); // Set CTC mode

  // Configure interrupt
  OCR2A = pulseOnDelay;  // Set compare match value
  TIMSK2 |= (1<<OCIE2A); // Activate interrupt at compare match (will be in OCF2A)
  sbi(PORTD, PWMPIN);      // start power to PWM pin
  sei();                   // Enable interrupts
  
  // Set prescaler and start clock
  TCCR2B |= (1 << CS20);   // Prescaler 1 (only cs20 true)

  // Generate each bit
  for(int i=0; i<numBits; i++){
    // Update first delay
    //if (tagID[i]) {
    //  OCR2A = OCR2A + bitShift;
    //}
    pulseCounter=0;
        
    //while (pulseCounter<1) {
    //  DELAY_CYCLES(1);
      // wait for pulse
    //}

    // change delay for next pulses
    //OCR2A = pulseOnDelay;            // Reset to normal

    // Keep generating pings
    while (pulseCounter<bitCycles) {
      DELAY_CYCLES(1);
      // wait for pulses
    }
    
    // Update delay for first ping of next bit 
    //if (!tagID[i]) {
    //  OCR2A = OCR2A + 2*bitShift;
    //}
    //else {
    //  OCR2A = OCR2A + 1*bitShift;
    //}
    
  }

  // Deactivate timer
  TCCR2B = 0; // Deactivate timer
  TIMSK2 = 0; // Deactivate interrupt

  cbi(PORTD, PWMPIN); // turn off voltage to PWM - generate pulse
}


// interrupt to generate bits
ISR(TIMER2_COMPA_vect) {
  cbi(PORTD, PWMPIN); // reset PWM pin - send out ping
  pulseCounter++;
  sbi(PORTD, PWMPIN); // activate PWM pin  
}

void pulseOut(){   
  for(int n=0; n<bitCycles; n++){
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
