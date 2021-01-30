#include "examples.h"

void setup() {
  Serial.begin(115200);
  testADC();
}

void loop() {
  Serial.println("hello");
  delay(1000);
}
