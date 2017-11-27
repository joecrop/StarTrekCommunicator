
const int MIC =  A1;

void setup(void) {

  pinMode(MIC, INPUT);

  Serial.begin(115200);
  while (!Serial);
  delay(100);
  Serial.println("MIC test");
  delay(100);

}

void loop() {

  uint8_t data = analogRead(MIC);
  Serial.println(data);

}

