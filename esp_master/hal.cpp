#include "ArduinoJson.h"
#include "hal.h"
#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc.h"
#include <driver/i2s.h>
#include <driver/adc.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "sdkconfig.h"
#include "esp_system.h"
//#include "nvs_flash.h"
//#include "soc/uart_struct.h"
//#include "freertos/queue.h"

#define ADC_SAMPLES_PER_READ 100
#define ADC_INPUT ADC1_CHANNEL_4 //pin 32
#define OUTPUT_PIN 27
#define OUTPUT_VALUE 3800
#define READ_DELAY 2 //microseconds
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

void setUpDAC(uint8_t pin, uint8_t channel) {
  sigmaDeltaSetup(channel, 312500);
  sigmaDeltaAttachPin(pin, channel);
}

void writeToDAC(uint8_t channel, uint8_t value) {
  sigmaDeltaWrite(channel, value);
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

void setUpADC(uint32_t period) {
  i2s_sample_rate = period;
  i2sInit();
  xTaskCreatePinnedToCore(reader, "ADC_reader", 2048, NULL, 1, NULL, 1);
  //setUpTimer(3, f, period);
  //startTimer(3);
}

uint16_t readADC() {
  /*if (xSemaphoreTake(adc_semaphore, 0) == pdTRUE){
    portENTER_CRITICAL(&adc_mux);
    prev_adc_reading = adc_reading;
    portEXIT_CRITICAL(&adc_mux); 
  }*/

  return adc_reading;
  //return adc_reading;
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

  if (uart_channel == 1) {
    uart_port_num = UART_NUM_1;
    tx = 17;
    rx = 16;
  } else if(uart_channel == 2) {
    uart_port_num = UART_NUM_2;
    tx = 10;
    rx = 9;
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

void readFromUART(uint8_t uart_channel, char message[]) {
  //const int uart_num = UART2;
  uart_port_t uart_port_num_temp;

  if (uart_channel == 1) {
    uart_port_num_temp = UART_NUM_1;
  } else if(uart_channel == 2) {
    uart_port_num_temp = UART_NUM_2;
  } else return;
  
  uart_write_bytes(uart_port_num_temp, (const char*)message, strlen(message));
  
  uint8_t data[128];
  int length = 0;
  ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_port_num_temp, (size_t*)&length));
  length = uart_read_bytes(uart_port_num_temp, data, length, 100);

  message = (char*) data;
}

void setUpI2C() {
  i2c_port_t i2c_master_port = (i2c_port_t)0;
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = (gpio_num_t) I2C_MASTER_SDA_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = (gpio_num_t) I2C_MASTER_SCL_IO,
      .scl_pullup_en = GPIO_PULLUP_ENABLE
      // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
  };

  conf.master.clk_speed = 10000;

  esp_err_t err = i2c_param_config(i2c_master_port, &conf);
  i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void writeByteToI2C(uint8_t slave_address) {
  
}

uint8_t* readByteFromI2C(uint8_t slave_address, size_t size) {
    uint8_t *data_rd = (uint8_t *)malloc(size);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_address << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, (i2c_ack_type_t) ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, (i2c_ack_type_t) NACK_VAL);
    i2c_master_stop(cmd);
    i2c_port_t p =  (i2c_port_t) 0;
    esp_err_t ret = i2c_master_cmd_begin(p, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return data_rd;
}
