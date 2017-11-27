
// chirp sound file
//#include "tng_chirp.h"
#include "unproductive.h"


const int PA_EN = 22;
const int AUDIO =  A12; //D16
IntervalTimer myTimer;
int sound_index;
char sinetable [32];

void setup(void) {

  pinMode(PA_EN, OUTPUT);
  digitalWrite(PA_EN, 0);
  analogWriteResolution(8);
  sound_index = 0;

  Serial.begin(115200);
  //while (!Serial);
  delay(100);
  Serial.println("Speaker test");
  delay(100);

  sinetable[0]=127;  // Put 32 step 8 bit sine table into array.
  sinetable[1]=152;
  sinetable[2]=176;
  sinetable[3]=198;
  sinetable[4]=217;
  sinetable[5]=233;
  sinetable[6]=245;
  sinetable[7]=252;
  sinetable[8]=254;
  sinetable[9]=252;
  sinetable[10]=245;
  sinetable[11]=233;
  sinetable[12]=217;
  sinetable[13]=198;
  sinetable[14]=176;
  sinetable[15]=152;
  sinetable[16]=128;
  sinetable[17]=103;
  sinetable[18]=79;
  sinetable[19]=57;
  sinetable[20]=38;
  sinetable[21]=22;
  sinetable[22]=10;
  sinetable[23]=3;
  sinetable[24]=0;
  sinetable[25]=3;
  sinetable[26]=10;
  sinetable[27]=22;
  sinetable[28]=38;
  sinetable[29]=57;
  sinetable[30]=79;
  sinetable[31]=103;

  myTimer.begin(playAudio, 100);//31.250); //32 kHz

}

void loop() {


  digitalWrite(PA_EN, 1);
  Serial.println("PA On");
  delay(1000);
  
  //myTimer.end();
  Serial.println("Audio Done");
  //delay(1000);
  //myTimer.begin(playAudio, 31250); //32 kHz
  Serial.println("Audio Start");
  
}

void playAudio() {
  if(sound_index >= sounddata_length) {
    sound_index = 0;
    delay(1000);
  }
  uint8_t data = sounddata_data[sound_index];
  //analogWrite(AUDIO, (data-128)*2 + 128);
  //uint8_t data = sinetable[sound_index % 16]/3;
  analogWrite(AUDIO, data);
  
  //Serial.println(data);
  sound_index++;
}

