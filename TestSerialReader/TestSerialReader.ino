void setup() {
  Serial1.begin(115200);
}
void loop() {
  if (Serial1.available()) {
    uint8_t val = Serial1.read();
    if (val < 0x10) Serial.print("0");
    Serial.println(val, HEX);
  }
}

