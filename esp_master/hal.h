#include <Arduino.h>
//#include "soc/soc.h"
//#include "soc/rtc.h"

#define PUSH_BUTTON_PIN 0
#define LED_PIN 2

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_SCL_IO 22             /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21             /*!< gpio number for I2C master data  */

void giveSemaphore(char* semaphore_name);
bool checkSemaphore(char* semaphore_name);
void setUpSemaphore(char* semaphore_name);

void setUpCriticalState(char* state_name);
void enterCriticalState(char* state_name);
void exitCriticalState(char* state_name);

void setUpTimer(uint8_t timer_index, void (*f)(), uint32_t period); //period is in us, index is from [0,3], inclusive
//timer 1 and 2 are used for UART

void startTimer(uint8_t timer_index);
void stopTimer(uint8_t timer_index);
void changeTimerPeriod(uint8_t timer_index, uint32_t period);

void setUpDAC(); //GPIO pin 25
void writeToDAC(uint8_t value);

void setUpADC(); //period is in us
uint16_t readADC();

//void setUpDMAADC(uint32_t period); //period is in us
//uint16_t readDMAADC();

void writeToDigitalPin(uint8_t pin, bool digital_output);
bool readFromDigitalPin(uint8_t pin);
void setUpInputPin(uint8_t pin);
void setUpOutputPin(uint8_t pin);

void setUpUART(uint8_t uart_channel, uint16_t baud_rate);
void writeToUART(uint8_t uart_channel, char message[]);
void readFromUART(uint8_t uart_channel, char message[]);

void setUpI2C();
void writeByteToI2C(uint8_t slave_address);
uint8_t* readByteFromI2C(uint8_t slave_address, size_t size);
