#include <RFM69.h>
#include <RFM69registers.h>
#include <RFM69_stream.h>

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

////////////////////////////
//RFM69 Setup
////////////////////////////
#define NETWORKID     0   // Must be the same for all nodes
#define MYNODEID      1   // My node ID
#define TONODEID      2   // Destination node ID
// RFM69 frequency, uncomment the frequency of your module:
//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

#define RFM69_CS      10
#define RFM69_IRQ     2 // was 3
#define RFM69_RST     9

// Create a library object for our RFM69HCW module:
RFM69_stream radio = RFM69_stream(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQ);

// I2C
#if defined(__MKL26Z64__) // TeensyLC
  Adafruit_Teensy_LIS3DH lis = Adafruit_Teensy_LIS3DH();
#else
  Adafruit_LIS3DH lis = Adafruit_LIS3DH();
#endif
// Adjust this number for the sensitivity of the 'click' force
#define CLICKTHRESHHOLD 80

static char sendbuffer[64];
int buffersize;

uint8_t data;
const int MIC =  A1;
const int TAP = 8;
const int PA_EN = 22;
const int AUDIO =  A12; //D16
IntervalTimer myTimer;
IntervalTimer chirpTimer;
int sound_index;
enum radioModeEnum {M_SLEEP, M_RX, M_TX, M_TX_END};
uint8_t radioMode;
uint8_t cypher;
uint16_t zeroPackets; //used to track end transmiission frames;

void setup()
{
  pinMode(MIC, INPUT);
  pinMode(TAP, INPUT);
  pinMode(PA_EN, OUTPUT);
  
  digitalWrite(PA_EN, 0);
  pinMode(AUDIO, OUTPUT);
  analogWriteResolution(8);
  sound_index = 0;
  
  Serial.begin(115200);
  //while (!Serial);
  delay(100);
  Serial.println("Transmitter test");
  delay(100);
  #if !defined(__MKL26Z64__) // TeensyLC
    //use internal I2C pullups, there are none on the board
    pinMode(18,INPUT_PULLUP);
    pinMode(19,INPUT_PULLUP);
  #endif
  if (! lis.begin(0x19)) {
    Serial.println("Couldnt start LIS3DH");
    //while (1); //we just hope it is still there... seems like i2c port is slow ro boot up some times
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_8_G);   // 2, 4, 8 or 16 G!
  lis.setClick(1, CLICKTHRESHHOLD); // 1 = single click only interrupt output
  
  ////////////////////////////////
  // Hard Reset the RFM module
  ////////////////////////////////
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  radioSleep();

  // for XOR ciypher
  randomSeed(analogRead(0));
  cypher = random(255);

  // Start listenting for Tap
  attachInterrupt(TAP, tapDetected, RISING);
}

void loop()                     
{
  //check if something was received (could be an interrupt from the radio)
  if (radioMode == M_SLEEP)
  {
    if(radio.receiveDone())
    {
      //print message received to serial
      Serial.print('[');Serial.print(radio.SENDERID);Serial.print("] ");
      Serial.print((char*)radio.DATA);
      Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");
   
      //check if received message contains Hello World
      if (strstr((char *)radio.DATA, "Cypher"))
      {
        cypher = *(radio.DATA+8);
        //check if sender wanted an ACK
        if (radio.ACKRequested())
        {
          radio.sendACK();
          Serial.println(" - ACK sent, start stream RX");
        }
        startStreamingRX();
      }  
    }

    //radio.receiveDone(); //put radio in RX mode
    //Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
  }
}

// Send microphone data
void sendAudioData()
{
  if(!digitalRead(RFM69_IRQ)) {
    if(radioMode == M_TX_END) {
      if(zeroPackets < 500) {
        radio.sendStreamByte(0xAA);
        zeroPackets++;
      } else {
        myTimer.end();
        changeMode();
      }
    } else {
      data = analogRead(MIC);
      radio.sendStreamByte(data);
      //Serial.println(data);
    }
  }
}

void getAudioData()
{
  uint8_t data;
  if(digitalRead(RFM69_IRQ))
  {
    data = radio.receiveStreamByte();
    analogWrite(AUDIO, data);
    //Serial.println(data);
    if(data == 0xAA || data == 0x55) {
      zeroPackets++;
      if(zeroPackets > 100) {
        // more than 100 zeros in a row means the transmission is over
        myTimer.end();
        changeMode();
        zeroPackets = 0;
      }
    } else if(zeroPackets > 0){
      zeroPackets--;
    }
  }
}

// Called when tap interrupt is found
void tapDetected() {
  Serial.println("tap found");
  changeMode();
}

void changeMode() {
  if(radioMode == M_SLEEP) {
    radioMode = M_TX;

    cypher = random(255);
    char radiopacket[10] = "Cypher: ";
    radiopacket[8] = cypher;
    radiopacket[9] = '\0';
    Serial.print("Sending cypher...:");
    Serial.println(radiopacket);
    
    if (radio.sendWithRetry(TONODEID, radiopacket, 10)) { //target node Id, message as string or byte array, message length
      Serial.println("OK");
    }

    Serial.println("start Streaming");
    playConnectAudio();
    radio.streamingMode();
    startStreaming();
  } else if(radioMode == M_TX) {
    radioMode = M_TX_END;
    Serial.println("send end transmission frame");
  } else if(radioMode == M_TX_END) {
    Serial.println("start Sleep");
    playDisconnectAudio();
    radioSleep();
  } else if(radioMode == M_RX) {
    Serial.println("start Sleep");
    radioSleep();
  }
}

void startStreamingRX() {
  radioMode = M_RX;
  radio.streamingModeRX();
  zeroPackets = 0;
  myTimer.begin(getAudioData, 80);  // 64 us = 15.625 kHz
  Serial.println("Start Stream RX");
}

void startStreaming() {
  zeroPackets = 0;
  // intialize buffer to send packet start sequence to synchronize reciever
  buffersize = 64;
  for(int i = 0; i<64;i++) {
    sendbuffer[i] = 0xAA;
  }
  Serial.print("filling fifo...");
  radio.stream(TONODEID, sendbuffer, buffersize);
  Serial.println("done");
  myTimer.begin(sendAudioData, 80);  // 64 us = 15.625 kHz
  data = 0;
}

void playConnectAudio()
{
  noInterrupts();
  digitalWrite(PA_EN, 1);
  sound_index = 0;
  chirpTimer.begin(playConnectSample, 31.250); //32 kHz
}

void playConnectSample() {
  if(sound_index >= sounddata_length) {
    digitalWrite(PA_EN, 0);
    chirpTimer.end();
    interrupts();
  }
  int8_t data = sounddata_data[sound_index] - 128;
  analogWrite(AUDIO, data);
  sound_index++;
}

void playDisconnectAudio()
{
  // play connect chirp in reverse
  noInterrupts();
  digitalWrite(PA_EN, 1);
  sound_index = sounddata_length -1;
  chirpTimer.begin(playDisconnectSample, 31.250); //32 kHz
}

void playDisconnectSample() {
  int8_t data = sounddata_data[sound_index] - 128;
  analogWrite(AUDIO, data);
  sound_index--;
  
  if(sound_index <= 0) {
    digitalWrite(PA_EN, 0);
    chirpTimer.end();
    interrupts();
  }
}

void radioSleep() {
  radioMode = M_SLEEP;
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.dataRate10k();
  radio.setHighPower(); // Always use this for RFM69HCW
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  radio.encrypt(ENCRYPTKEY);
}

