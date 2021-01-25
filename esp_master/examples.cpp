#include "examples.h"

void timerCallbackTest();
void timerCallbackTest2();
void timerCallbackTest3();

uint16_t val = 0;

void testTimer() {
    setUpTimer(0, timerCallbackTest,  100000);
    startTimer(0);
}

void testADC() {
    setUpADC(100000);
    setUpTimer(0, adcCallbackTest2, 4000);
    startTimer(0);
}

void testUART() {
    setUpUART(1, 9600);
    setUpTimer(0, adcCallbackTest3, 10000);
    startTimer(0);
}

void timerCallbackTest() {
    Serial.println("hello world");
}

void timerCallbackTest2() {
    val = readADC();
    Serial.printf("%d \n",val);
}

void timerCallbackTest3() {
    writeToUART(1, "hello world");
}
