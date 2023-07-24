#ifndef BMSDRIVER_H
#define BMSDRIVER_H

#include "stm32f3xx_hal.h"
#include "delay.h"

#define BMS_CHIP_SELECT_GPIO_PORT	GPIOA
#define BMS_CHIP_SELECT_GPIO_PIN	GPIO_PIN_15

extern SPI_HandleTypeDef hspi3;

void cs_low(void);

void cs_high(void);

void spi_write(uint8_t data); 			//Bytes to be written on the SPI port

void spi_write_array(uint8_t len, 		//Option: Number of bytes to be written on the SPI port
                     uint8_t data[] 	//Array of bytes to be written on the SPI port
                    );

void spi_write_read(uint8_t tx_Data[],	//array of data to be written on SPI port
                    uint8_t tx_len, 	//length of the tx data arry
                    uint8_t *rx_data,	//Input: array that will store the data read by the SPI port
                    uint8_t rx_len 		//Option: number of bytes to be read from the SPI port
                   );

uint8_t spi_read_byte(uint8_t tx_dat); 	//name conflicts with linduino also needs to take a byte as a parameter

void HAL_SPI_TransmitReceiveFast(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
        uint32_t Timeout);

#endif
