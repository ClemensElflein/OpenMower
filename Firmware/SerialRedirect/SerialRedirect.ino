/**
 * Use this Arduino Sketch to turn on your Raspberry Pi and redirect the console via USB.
 * 
 */
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(21, OUTPUT);
  // Disable Raspi
  digitalWrite(21, LOW);

  // Wait for Serial to be available
  while(!Serial);
  delay(1000);

  // Drain serial input
  while(Serial.available())
    Serial.read();

  do {
    Serial.println("Send anything to start Raspi");
    delay(1000);
  } while(!Serial.available());

  // Drain serial input again
  while(Serial.available())
    Serial.read();

  // Enable Raspi
  digitalWrite(21, HIGH);
}

void loop() {
  // redirect serial interface
  while(Serial.available())
    Serial1.write(Serial.read());
  while(Serial1.available())
    Serial.write(Serial1.read());
}
