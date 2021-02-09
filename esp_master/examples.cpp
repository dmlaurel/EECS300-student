#include "hal.h"
#include "fft.h"
#include <Arduino.h>
#include "web_communication.h"
#include "ArduinoJson.h"

void timerCallbackTest();
void timerCallbackTest2();
void timerCallbackTest3();
float runFFTOnce(double fft_signal_p[], float total_time_p);


DynamicJsonDocument doc(1024);
int i = 0;

uint16_t val = 0;
uint8_t new_val = 0;

float mag = 0;
float freq = 0;

float max_magnitude = 0;
float fundamental_freq = 0;
//float total_time; //The time in which data was captured. This is equal to FFT_N/sampling_freq

float fft_input[FFT_N];
float fft_output[FFT_N];

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


float runFFTOnce(double fft_signal_p[], float total_time_s) { //total_time_s is in seconds
  
  fft_config_t *real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);

  for (int k = 0 ; k < FFT_N ; k++)
    real_fft_plan->input[k] = (float)fft_signal_p[k];

  //long int t1 = micros();
  
  // Execute transformation
  fft_execute(real_fft_plan);

  max_magnitude = 0;
  
  // Print the output
  for (int k = 1 ; k < real_fft_plan->size / 2 ; k++)
  {
    /*The real part of a magnitude at a frequency is followed by the corresponding imaginary part in the output*/
    mag = sqrt(pow(real_fft_plan->output[2*k],2) + pow(real_fft_plan->output[2*k+1],2))/1;
    freq = k*1.0/total_time_s;

    if(mag > max_magnitude)
    {
        max_magnitude = mag;
        fundamental_freq = freq;
    }
  }
  
  //long int t2 = micros();
  
  //Serial.println();
  
  /*Multiply the magnitude of the DC component with (1/FFT_N) to obtain the DC component*/
  /*Multiply the magnitude at all other frequencies with (2/FFT_N) to obtain the amplitude at that frequency*/
  //Serial.print("Time taken: ");
  //Serial.print((t2-t1)*1.0/1000);
  //Serial.println(" milliseconds!");
  
  fft_destroy(real_fft_plan);
  return fundamental_freq;
  
}

void setUpWebComm() {
  setUpWifi();
  setUpPostServer();
  doc["chair"] = "blue";
}

void runWebComm() {
  delay(10);
  if (i > 100) {
    sendPostRequest(doc);
    i = 0;
  }
  runPostServer();
  i++;
}
