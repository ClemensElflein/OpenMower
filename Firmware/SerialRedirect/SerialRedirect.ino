/**
 * Use this Arduino Sketch to turn on your Raspberry Pi and redirect the console via USB.
 * 
 */
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  // Enable Raspi
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);
  // Enable ESCs
  pinMode(20, OUTPUT);
  digitalWrite(20, HIGH);
  // Enable GPS
  pinMode(22, OUTPUT);
  digitalWrite(22, HIGH);
  
}

void loop() {
  // redirect serial interface
  while(Serial.available())
    Serial1.write(Serial.read());
  while(Serial1.available())
    Serial.write(Serial1.read());
}
