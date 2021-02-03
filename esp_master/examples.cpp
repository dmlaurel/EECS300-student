#include "examples.h"
#include "hal.h"

void timerCallbackTest();
void timerCallbackTest2();
void timerCallbackTest3();

uint16_t val = 0;
uint8_t new_val = 0;

void testTimer() {
    setUpTimer(0, timerCallbackTest,  100000);
    startTimer(0);
}

void testADC() {
    setUpTimer(0, timerCallbackTest2, 4000);
    startTimer(0);
}

void testUART() {
    setUpUART(1, 9600);
    setUpTimer(0, timerCallbackTest3, 100000);
    startTimer(0);
}

void timerCallbackTest() {
    Serial.println("hello world");
}

void timerCallbackTest2() {
    Serial.printf("%d \n",val);
}

void timerCallbackTest3() {
    writeToUART(1, "hello world");
}
