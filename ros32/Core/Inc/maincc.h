#ifndef MAINCC_H_
#define MAINCC_H_

#include "stm32f4xx_hal.h"
#ifdef __cplusplus
 extern "C" {
#endif

void setup(UART_HandleTypeDef* leftPort, UART_HandleTypeDef* rightPort, SPI_HandleTypeDef* imuPort, GPIO_TypeDef* portx, uint16_t pin);
void loop(void);


#ifdef __cplusplus
}
#endif


#endif
