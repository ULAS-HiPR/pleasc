#ifdef F4
#ifndef STM_F4_H
#define STM_F4_H

#include "stm32f4xx_hal.h"
#define LED_PIN GPIO_PIN_5 
#define LED_GPIO_PORT GPIOA 
#define LED_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() 

#define I2C_SCL_PIN GPIO_PIN_8 
#define I2C_SDA_PIN GPIO_PIN_9 
#define I2C_GPIO_PORT GPIOB 
#define I2C_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE() 

#define SPI_SCK_PIN GPIO_PIN_5 
#define SPI_MISO_PIN GPIO_PIN_6 
#define SPI_MOSI_PIN GPIO_PIN_7 
#define SPI_GPIO_PORT GPIOA 
#define SPI_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() 

#define BARO_CS_PIN GPIO_PIN_4
#define BARO_CS_PORT GPIOA

// Extern handles for use by I2C/SPI handlers
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

// Initialization functions
void MX_I2C1_Init();
void MX_SPI1_Init();

#endif // STM_F4_H
#endif // F4