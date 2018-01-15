#include <RFM69.h>
#include <RFM69registers.h>
#include <RFM69_stream.h>
#include <ADC.h> // Teensy ADC library
#include <IntervalTimer.h> // Teenst Timer library
#if defined(__MKL26Z64__) // TeensyLC
  #include <i2c_t3.h>
  #include "Adafruit_Teensy_LIS3DH.h"
#else
  //NOTE: due to the way arduino compiles files, the Adafruit_Teensy_LIS3DH needs to be physically removed from the folder when not used
  #include <Wire.h>
  #include <Adafruit_LIS3DH.h>
#endif
#include <Adafruit_Sensor.h>
// chirp sound file
#include "tng_chirp.h"

// TODO:
// 2) add low pass filter to DAC output
// 3) Use 3.3V regulator with higher output power
// 4) Sleep Teensy instead of delay
// 5) Sleep radio during RX mode with Listen Mode

// Bit-rate / bandwidth NOTES:
// We transmit a 61-bit packet every 4880 us
// That mease we sample every 80us
// The packet transmission creates a tone of 200Hz (power supply noise)
// The bit-rate results in a BW of 6.25 kHz
// Human speach ranges from 200Hz to 5kH.
// We can use a low-pass filter to remove the 6.25kHz DAC/sampling noise
#define BIT_RATE      80 // in micro-seconds

#define NETWORKID     0   // Must be the same for all nodes
// For simplicity, both transmitter and receiver have the same ID
// They are both set to the broadcast address of the RFM69HCW library
// This way we can have multiple receivers
#define MYNODEID      RF_BROADCASTADDRESS_VALUE   // Source node ID
#define TONODEID      RF_BROADCASTADDRESS_VALUE   // Destination node ID

// RFM69 frequency, uncomment the frequency of your module:
//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ

#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

#define RFM69_CS      10
#define RFM69_IRQ     2
#define RFM69_RST     9

// I2C Accelerometer Setup
#if defined(__MKL26Z64__) // TeensyLC
  Adafruit_Teensy_LIS3DH lis = Adafruit_Teensy_LIS3DH();
#else
  Adafruit_LIS3DH lis = Adafruit_LIS3DH();
#endif
// Adjust this number for the sensitivity of the 'click' force
#define CLICKTHRESHHOLD 80

// Create a library object for our RFM69HCW module:
RFM69_stream radio = RFM69_stream(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQ);

ADC *adc = new ADC(); // adc object

IntervalTimer myTimer;
unsigned long startTime;
unsigned long roundTripTime;
unsigned long wakeTime;
uint8_t timerRunning;
int sound_index;
uint8_t radioMode;
uint8_t tapDetected;
enum radioModeEnum {
  M_RX, // receiver on, read to get packet (default mode)
  M_CONNECT_TX, // Play connect chirp for TX
  M_CONNECT_RX, // Play connect chirp for RX
  M_STREAM_RX, // streaming audio receiver
  M_STREAM_TX, // streaming audio transmitter
  M_TX, // transmiting data packet
  M_END, // we are playing the disconnect sound after a transmission
  M_SLEEP // sleeping radio to save power
};

// Port definitions
const int AUDIO =  A12;
const int PA_EN = 22;
const int MIC =  A1;
const int TAP = 8;
const int SCL0 = 19;
const int SDA0 = 18;

void setup() {

  Serial.begin(115200);
  delay(1000);
  
  pinMode(PA_EN, OUTPUT);
  pinMode(TAP, INPUT);
  
  digitalWrite(PA_EN, 0);

  Serial.print("Setting up ADC...");
  analogWriteResolution(8);
  adc->setAveraging(16); // set number of averages (lower noise)
  adc->setResolution(12); // set bits of resolution
  adc->setReference(1, ADC_0); // maybe not needed
  adc->enableInterrupts(ADC_0);
  Serial.println("done");

  #if !defined(__MKL26Z64__) // TeensyLC
    //use internal I2C pullups, there are none on the board
    pinMode(SDA0, INPUT_PULLUP);
    pinMode(SCL0, INPUT_PULLUP);
  #endif
  if (! lis.begin(0x19)) {
    Serial.println("Couldnt start LIS3DH");
    //while (1); //we just hope it is still there... seems like i2c port is slow ro boot up some times
  }
  Serial.print("LIS3DH found...");

  lis.setRange(LIS3DH_RANGE_8_G);   // 2, 4, 8 or 16 G!
  lis.setClick(1, CLICKTHRESHHOLD); // 1 = single click only interrupt output
  Serial.println("confugured");

  // Hard Reset the RFM module
  Serial.print("Resetting radio...");
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);
  Serial.println("done");

  Serial.print("initializing radio...");
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  radio.encrypt(ENCRYPTKEY);
  radio.dataRate200k();
  Serial.println("done");

  radioMode = M_RX;

  // Start listenting for Tap
  tapDetected = 0;
  attachInterrupt(TAP, tapISR, RISING);

  Serial.println("ready");
}

void loop() {

  if(tapDetected) {
    tapDetected = 0;
    Serial.println("tap found");
    if(radioMode == M_STREAM_TX) {
      // end transmission
      stopStream();
    } else if(radioMode == M_RX) {
      initiateStream();
    }
  }
  
  if(radioMode == M_STREAM_TX) {
    if(!radio.sendStreamBuffer(TONODEID)) {
      // the transmission has timed out (longer than 40 seconds)
      Serial.println("Stream timed out");
      stopStream();
    }
  } else
  //check if something was received (could be an interrupt from the radio)
  if ((radioMode == M_RX || radioMode == M_STREAM_RX) && radio.receiveDone())
  {
    // for printing dubug stuff
    printPacketDebug();

    // If the sender requested an ACK then the packet is for coms setup, not a stream buffer
    if (radio.ACKRequested())
    {
      //check if received message contains a Cypher
      if (strstr((char *)radio.DATA, "Cypher"))
      {
        // Set the cypher that we receieved from the TX, if we have the wrong encrypted cypher, we will hear garbage...
        radio.setCypher((uint8_t *)radio.DATA+8);
        
        radio.sendACK();
        radio.encrypt(0);
        radioMode = M_CONNECT_RX;
        playConnectAudio();
      }
    } else if(radioMode == M_STREAM_RX){
      // If no ack was requested it was a stream packet
      radio.receiveStreamBuffer();
      if(!timerRunning) {
        timerRunning = 1;
        myTimer.begin(getData, BIT_RATE);
      }
    }
  } else if(radioMode == M_STREAM_RX && !radio.isStreamConnected() ){
    // If we stopp receieving packets (or lost the connection), kill the stream
    stopStream();
  } else if(radioMode != M_STREAM_RX) {
    // Sleep for 100ms, each TX connect packet takes 126ms to timeout
    if(radioMode == M_SLEEP) {
      radioMode = M_RX;
    } else if((micros() - wakeTime) > 255000) {
      // If we have been awake for 255ms that is long enough to get two packets
      // so if we haven't gotten anything we can sleep
      //radioMode = M_SLEEP;
      //radio.sleep();
      //delay(255); //replace with legit Teensy sleep
      //wakeTime = micros();
      
    }
  }
}

// this is called to greate a cypher and start the TX of a stream
void initiateStream() {
  radioMode = M_CONNECT_TX;
  // Generate a random cypher.
  // We can't enctrpy the stream because it is too slow and we don't get the Bandwidth we need
  // for audio data. Therefore, we send an encrypted cypher than ony the TX and RX know.
  // This will be XORed at the TX and then again at the RX (to reverse the cypher) with each byte sent.
  // A single byte cypher would mask the transmission a little bit, but a 30-byte cypher will totally 
  // mask the data from anyone understanding it if they have the wrong cypher.
  uint8_t cypher[RF69_MAX_DATA_LEN/2];
  radio.generateCypher(cypher);
  char radiopacket[9+RF69_MAX_DATA_LEN/2] = "Cypher: ";
  for(int i = 0; i < RF69_MAX_DATA_LEN/2; i++) {
    radiopacket[8+i] = cypher[i];
  }
  radiopacket[8+RF69_MAX_DATA_LEN/2] = '\0';
  Serial.println(radiopacket);
  // Send start packet
  uint8_t tryCounter = 0;
  while (!radio.sendWithRetry(TONODEID, radiopacket, 9+RF69_MAX_DATA_LEN/2)) { //target node Id, message as string or byte array, message length
    Serial.println("Waiting");
    if(tryCounter++ > 10) {
      Serial.println("CONNECT FAILED");
      radioMode = M_RX;
      break;
    }
  }
  if(tryCounter < 10) {
    // Connection successful
    radio.encrypt(0);
    playConnectAudio();
  }
}
// called every time a stream has ended to return ro RX mode and reset stream vars
void stopStream()
{
  myTimer.end();
  radio.encrypt(ENCRYPTKEY);
  timerRunning = 0;
  radio.streamInit();
  Serial.println("Stream finished");
  playDisconnectAudio();
}
// Called when tap interrupt is found
void tapISR() {
  tapDetected = 1;
}

void playConnectAudio()
{
  noInterrupts();
  Serial.println("playConnectAudio");
  digitalWrite(PA_EN, 1);
  sound_index = 0;
  myTimer.begin(playConnectSample, 31.250); //32 kHz
}

void playConnectSample() {
  if(sound_index >= sounddata_length) {
    myTimer.end();
    //digitalWrite(PA_EN, 0);
    if(radioMode == M_CONNECT_TX) {
      radioMode = M_STREAM_TX;
      timerRunning = 1;
      Serial.println("starting stream");
      myTimer.begin(sendADCData, BIT_RATE);
    } else {
      radioMode = M_STREAM_RX;
    }
    interrupts();
  }
  uint8_t data = pgm_read_byte(&sounddata_data[sound_index]);
  analogWrite(AUDIO, data);
  sound_index++;
}

void playDisconnectAudio()
{
  noInterrupts();
  // play connect chirp in reverse
  digitalWrite(PA_EN, 1);
  sound_index = sounddata_length -1;
  radioMode = M_END;
  myTimer.begin(playDisconnectSample, 31.250); //32 kHz
}

void playDisconnectSample() {
  uint8_t data = pgm_read_byte(&sounddata_data[sound_index]);
  analogWrite(AUDIO, data);
  sound_index--;
  
  if(sound_index <= 0) {
    digitalWrite(PA_EN, 0);
    myTimer.end();
    radioMode = M_RX;
    interrupts();
  }
}

void sendADCData()
{
    adc->startSingleRead(MIC, ADC_0);  
}

// when the measurement finishes, this will be called
void adc0_isr() {
  int16_t adc_data = adc->readSingle();
  adc_data -= 1280; //subtract DC
  if(adc_data > 127) {
    adc_data = 127;
  } else if(adc_data < -127) {
    adc_data = -127;
  }

  radio.bufferStreamByte((uint8_t)((adc_data + 128)));
  //Serial.println(adc_data);

  // restore ADC config if it was in use before being interrupted by the analog timer
  if (adc->adc0->adcWasInUse) {
    // restore ADC config, and restart conversion
    adc->adc0->loadConfig(&adc->adc0->adc_config);
    // avoid a conversion started by this isr to repeat itself
    adc->adc0->adcWasInUse = false;
  }
}

void getData()
{
  uint8_t data;
  data = radio.receiveStreamByte();
  analogWrite(AUDIO, data);
}

void printPacketDebug() {
  roundTripTime = micros() - startTime;
  if(roundTripTime > 6000)
    Serial.println(roundTripTime);
  //Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.println("]");
  startTime = micros();
  //print message received to serial
  //Serial.print('[');Serial.print(radio.SENDERID);Serial.print("] ");
  //for (uint8_t i = 0; i < radio.DATALEN; i++)
    //Serial.println(radio.DATA[i]);
  //Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.println("]");
}

