#include "BMSDriver.h"

/*******************************************************************************/
// MICROCONTROLLER SPECIFIC FUNCTIONS FOR CONTROLLING :

void cs_low()
{
	HAL_GPIO_WritePin(BMS_CHIP_SELECT_GPIO_PORT, BMS_CHIP_SELECT_GPIO_PIN, GPIO_PIN_RESET);
}

void cs_high()
{
	HAL_GPIO_WritePin(BMS_CHIP_SELECT_GPIO_PORT, BMS_CHIP_SELECT_GPIO_PIN, GPIO_PIN_SET);
}

void delay_u(uint16_t micro)
{
	delay_us(micro);
}

void delay_m(uint16_t milli)
{
	delay_us(milli*1000);
}

void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                    )
{
	SPI_HandleTypeDef *pspi=&hspi3;
    uint8_t ret_val;
    uint8_t i;

    for ( i = 0; i < len; i++ )
    {
        HAL_SPI_TransmitReceive(pspi, (uint8_t*)&data[i], &ret_val, 1, HAL_MAX_DELAY);
    }
}

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   )
{
	SPI_HandleTypeDef *pspi=&hspi3;
    uint8_t i;
    uint8_t data;

    // Transfer data to LTC681x
    for ( i = 0; i < tx_len; i++ )
    {
        // Transmit byte.
        HAL_SPI_TransmitReceive(pspi, (uint8_t*)&tx_Data[i], &data, 1, HAL_MAX_DELAY);
    }

    // Receive data from DC2259A board.
    for ( i = 0; i < rx_len; i++ )
    {
        // Receive byte.
        HAL_SPI_TransmitReceive(pspi, (uint8_t*)0xFF, (uint8_t*)&rx_data[i], 1, HAL_MAX_DELAY);
    }
}

uint8_t spi_read_byte(uint8_t tx_dat)
{
	SPI_HandleTypeDef *pspi=&hspi3;
	uint8_t data;
    if ( HAL_SPI_TransmitReceive(pspi, (uint8_t*)0xFF, (uint8_t*)&data, 1, HAL_MAX_DELAY) == HAL_OK )
    {
        return(data);
    }
	return(1);
}
