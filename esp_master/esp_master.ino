#include "lab1.h"
#include "lab2.h"
#include "lab3.h"
#include "lab5.h"
//#include "adc_example.h"
//#include "hal.h"

void setup() {
  Serial.begin(115200);
  setUpLab1A();
  setUpLab1B();
}

void loop() {
  runLab1A();
  runLab1B();
}
