#include "ArduinoJson.h"
#include "hal.h"
#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc.h"
#include <driver/i2s.h>
#include <driver/adc.h>
#include "driver/gpio.h"
#include <driver/adc.h>
#include "driver/uart.h"
//#include "driver/i2c.h"
#include <driver/dac.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "sdkconfig.h"
#include "esp_system.h"
//#include "nvs_flash.h"
//#include "soc/uart_struct.h"
//#include "freertos/queue.h"
#include <Wire.h>


#define ADC_SAMPLES_PER_READ 20
#define ADC_INPUT ADC1_CHANNEL_4 //pin 32
#define OUTPUT_PIN 27
#define OUTPUT_VALUE 3800
#define READ_DELAY 1 //microseconds
#define BUF_SIZE (1024)


uint32_t i2s_sample_rate = 10000; //Hz

void i2sInit();

StaticJsonDocument<256> semaphores;
StaticJsonDocument<256> muxes;

volatile SemaphoreHandle_t my_semaphore_0;
volatile SemaphoreHandle_t my_semaphore_1;
volatile SemaphoreHandle_t my_semaphore_2;
volatile SemaphoreHandle_t my_semaphore_3;

portMUX_TYPE my_mux_0;
portMUX_TYPE my_mux_1;
portMUX_TYPE my_mux_2;
portMUX_TYPE my_mux_3;

hw_timer_t * timer_0 = NULL;
hw_timer_t * timer_1 = NULL;
hw_timer_t * timer_2 = NULL;
hw_timer_t * timer_3 = NULL; // reserved for ADC

uint8_t semaphore_count = 0;
uint8_t mux_count = 0;
SemaphoreHandle_t my_semaphores[4] = {my_semaphore_0, my_semaphore_1, my_semaphore_2, my_semaphore_3};
portMUX_TYPE my_muxes[4] = {my_mux_0, my_mux_1, my_mux_2, my_mux_3};

volatile uint16_t adc_reading;
uint16_t prev_adc_reading = 0;
portMUX_TYPE adc_mux = portMUX_INITIALIZER_UNLOCKED;
volatile SemaphoreHandle_t adc_semaphore;

void giveSemaphore(char* semaphore_name) {
    uint8_t semaphore_index = semaphores[semaphore_name];
    xSemaphoreGiveFromISR(my_semaphores[semaphore_index], NULL);
}

bool checkSemaphore(char* semaphore_name) {
  uint8_t semaphore_index = semaphores[semaphore_name];
  if (xSemaphoreTake(my_semaphores[semaphore_index], 0) == pdTRUE) {
    return true;
  } else {
    return false;
  }
}

void setUpSemaphore(char* semaphore_name) {
  semaphores[semaphore_name] = semaphore_count;
  my_semaphores[semaphore_count] = xSemaphoreCreateBinary();
  semaphore_count++;
}

void setUpCriticalState(char* state_name) {
  muxes[state_name] = mux_count;
  my_muxes[mux_count] = portMUX_INITIALIZER_UNLOCKED;
  mux_count++;
}

void enterCriticalState(char* state_name) {
  uint8_t mux_index = muxes[state_name];
  portENTER_CRITICAL_ISR(&my_muxes[mux_index]);
}

void exitCriticalState(char* state_name) {
  uint8_t mux_index = muxes[state_name];
  portEXIT_CRITICAL_ISR(&my_muxes[mux_index]);
}


void setUpTimer(uint8_t timer_index, void (*f)(), uint32_t period) {

    switch(timer_index){
      case 0:
        timer_0 = timerBegin(timer_index, 80, true);
        timerAttachInterrupt(timer_0, f, true);
        timerAlarmWrite(timer_0, period, true);
        break;
      case 1:
        timer_1 = timerBegin(timer_index, 80, true);
        timerAttachInterrupt(timer_1, f, true);
        timerAlarmWrite(timer_1, period, true);
        break;
      case 2:
        timer_2 = timerBegin(timer_index, 80, true);
        timerAttachInterrupt(timer_2, f, true);
        timerAlarmWrite(timer_2, period, true);
        break;
      case 3:
        timer_3 = timerBegin(timer_index, 80, true);
        timerAttachInterrupt(timer_3, f, true);
        timerAlarmWrite(timer_3, period, true);
        break;
      default:
        break;
    }
}

void startTimer(uint8_t timer_index) {
  switch(timer_index){
      case 0:
        timerAlarmEnable(timer_0);
        break;
      case 1:
        timerAlarmEnable(timer_1);
        break;
      case 2:
        timerAlarmEnable(timer_2);
        break;
      case 3:
        timerAlarmEnable(timer_3);
        break;
      default:
        break;
    }
}

void stopTimer(uint8_t timer_index) {
    switch(timer_index){
      case 0:
        timerEnd(timer_0);
        break;
      case 1:
        timerEnd(timer_1);
        break;
      case 2:
        timerEnd(timer_2);
        break;
      case 3:
        timerEnd(timer_3);
        break;
      default:
        break;
    }

}

void changeTimerPeriod(uint8_t timer_index, uint32_t period) {
  switch(timer_index){
      case 0:
        timerAlarmWrite(timer_0, period, true);
        break;
      case 1:
        timerAlarmWrite(timer_1, period, true);
        break;
      case 2:
        timerAlarmWrite(timer_2, period, true);
        break;
      case 3:
        timerAlarmWrite(timer_3, period, true);
        break;
      default:
        break;
    }
}

void setUpDAC() {
  dac_output_enable(DAC_CHANNEL_1);
}

void writeToDAC(uint8_t value) {
  dac_output_voltage(DAC_CHANNEL_1, value);
}

void i2sInit()
{
   i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate =  i2s_sample_rate,              // The format of the signal using ADC_BUILT_IN
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 8,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
   };
   i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
   i2s_set_adc_mode(ADC_UNIT_1, ADC_INPUT);
   i2s_adc_enable(I2S_NUM_0);
}

void reader(void *pvParameters) {
  uint32_t read_counter = 0;
  uint64_t read_sum = 0;
// The 4 high bits are the channel, and the data is inverted
  uint16_t offset = (int)ADC_INPUT * 0x1000 + 0xFFF;
  size_t bytes_read;
  while(1){
    uint16_t buffer[2] = {0};
    i2s_read(I2S_NUM_0, &buffer, sizeof(buffer), &bytes_read, 15);
    //Serial.printf("%d  %d\n", offset - buffer[0], offset - buffer[1]);
    if (bytes_read == sizeof(buffer)) {
      read_sum += offset - buffer[0];
      read_sum += offset - buffer[1];
      read_counter++;
    } else {
      Serial.println("buffer empty");
    }
    if (read_counter == ADC_SAMPLES_PER_READ) {
      //portENTER_CRITICAL_ISR(&adc_mux);
      adc_reading = read_sum / ADC_SAMPLES_PER_READ / 2;
      //portEXIT_CRITICAL_ISR(&adc_mux);
      //xSemaphoreGiveFromISR(adc_semaphore, NULL);
      //Serial.printf("avg: %d millis: \n", adc_reading);
      //Serial.println(millis());
      read_counter = 0;
      read_sum = 0;
      i2s_adc_disable(I2S_NUM_0);
      delay(READ_DELAY);
      i2s_adc_enable(I2S_NUM_0);
    }
  }
}

void setUpADC() {
  /*i2s_sample_rate = period;
  i2sInit();
  xTaskCreatePinnedToCore(reader, "ADC_reader", 2048, NULL, 1, NULL, 1);
  //setUpTimer(3, f, period);
*/
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);
  //startTimer(3);
}

uint16_t val2;
uint16_t readADC() {
  /*if (xSemaphoreTake(adc_semaphore, 0) == pdTRUE){
    portENTER_CRITICAL(&adc_mux);
    prev_adc_reading = adc_reading;
    portEXIT_CRITICAL(&adc_mux); 
*/
  val2 = adc1_get_raw(ADC1_CHANNEL_0);

  return val2;
}

void writeToDigitalPin(uint8_t pin, bool digital_output) {
  if (digital_output) {
    digitalWrite(pin, HIGH);
  } else {
    digitalWrite(pin,LOW);
  }
}

bool readFromDigitalPin(uint8_t pin) {
  if (digitalRead(pin) == LOW) {
    return false;
  } else {
    return true;
  }
}

void setUpInputPin(uint8_t pin) {
  pinMode(pin, INPUT);
}

void setUpOutputPin(uint8_t pin) {
  pinMode(pin, OUTPUT);
}

void setUpUART(uint8_t uart_channel, uint16_t baud_rate) {

  uart_port_t uart_port_num;
  int tx = 0;
  int rx = 0;
  Serial.print("cont");
  Serial.println(uart_channel);
  if (uart_channel == 1) {
    uart_port_num = UART_NUM_1;
    tx = 17;
    rx = 16;
  } else if(uart_channel == 0) {
    uart_port_num = UART_NUM_0;
    tx = 1;
    rx = 3;
  } else return;

  uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,    
        .rx_flow_ctrl_thresh = 122,
    };
    //Configure UART1 parameters
    esp_err_t er1 = uart_param_config(uart_port_num, &uart_config);
    Serial.printf("er %d",er1);
    //Set UART1 pins(TX: IO4, RX: I05)
    er1 = uart_set_pin(uart_port_num, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    Serial.printf("er %d",er1);
    //Install UART driver (we don't need an event queue here)
    //In this example we don't even use a buffer for sending data.
    er1 = uart_driver_install(uart_port_num, BUF_SIZE * 2, 0, 0, NULL, 0);
    Serial.printf("er %d",er1);

   //setUpTimer(uart_channel, echo_task, 
  
  //xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
  // Install UART driver using an event queue here
  
}

void writeToUART(uint8_t uart_channel, char message[]) {
  //char* test_str = "AA";
  uart_port_t uart_port_num_temp;
  
  if (uart_channel == 1) {
    uart_port_num_temp = UART_NUM_1;
  } else if(uart_channel == 2) {
    uart_port_num_temp = UART_NUM_2;
  } else return;
  
  uart_write_bytes(uart_port_num_temp, (const char*)message, strlen(message));
}

void readFromUART(uint8_t uart_channel, String &message) {
  //const int uart_num = UART2;
  uart_port_t uart_port_num_temp;

  if (uart_channel == 1) {
    uart_port_num_temp = UART_NUM_1;
  } else if(uart_channel == 2) {
    uart_port_num_temp = UART_NUM_2;
  } else return;
  
  //uart_write_bytes(uart_port_num_temp, (const char*)message, strlen(message));
  
  uint8_t data[128];
  int length = 0;
  ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_port_num_temp, (size_t*)&length));

  length = uart_read_bytes(uart_port_num_temp, data, length, 100);

  message = (char*) data;
}

uint8_t deviceAddr;
uint8_t CTRL_REG1_val;

void setUpI2C(uint8_t addr) {
  deviceAddr = addr;
  Wire.begin();
  CTRL_REG1_val = 0b01110111;//set to normal mode, 400Hz, all 3 axes anabled
  writeToI2C(0x20, &CTRL_REG1_val, 1);// this is the bare minimum configuration to get it working. 
}

void writeToI2C(uint8_t subAddr, uint8_t * txDataBuffer, uint8_t numBytes) {
  //void LIS3DHwrite(uint8_t subAddr, uint8_t * txDataBuffer, uint8_t numBytes)
   // {
    Wire.beginTransmission(deviceAddr);
    Wire.write(subAddr);
    for(uint8_t i_i2c = 0; i_i2c < numBytes; ++i_i2c)
    {
      Wire.write(txDataBuffer[i_i2c]);
    }
    Wire.endTransmission(true);
   // }
}

void readI2C(uint8_t subAddr, uint8_t * rxDataBuffer, uint8_t numBytes) {
   
    Wire.beginTransmission(deviceAddr);
    Wire.write(subAddr);
    Wire.endTransmission(true);
    Wire.requestFrom(deviceAddr, numBytes, true);// this is technically wrong since the LIS3DH expects a repeated sttart (instead of a stop condiditon)
                                                    // between the writing of the sub address and the subsequent reads
                                                    // however, this still works since when you start a new read, the LIS3DH uses the last written sub address
    delay(100);//let the warp field stabilize
    uint8_t i_i2c = 0;
    while(Wire.available() && (i_i2c < numBytes) )
    {
      rxDataBuffer[i_i2c++] = Wire.read();
    }
}

void setUpPWM(uint8_t pin, uint8_t duty) {
  ledcSetup(0, 100, 8);
  ledcAttachPin(pin, 0);
  ledcWrite(0, duty);
}

void changePWMDuty(uint8_t channel, uint8_t duty) {
  ledcWrite(channel, duty);
}
