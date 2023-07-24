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
        HAL_SPI_TransmitReceiveFast(pspi, (uint8_t*)&data[i], &ret_val, 1, 5);
    }
}

void spi_write_read(uint8_t *tx_data,//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data array
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   )
{
	SPI_HandleTypeDef *pspi=&hspi3;
    uint8_t i;
    uint8_t rxDummy;
    uint8_t txDummy=0xFF;

    // Transfer data to LTC681x
    for ( i = 0; i < tx_len; i++ )
    {
        // Transmit byte.
        HAL_SPI_TransmitReceiveFast(pspi, (uint8_t*)&tx_data[i], (uint8_t*)&rxDummy, 1, 5);
    }

    // Receive data from DC2259A board.
    for ( i = 0; i < rx_len; i++ )
    {
        // Receive byte.
        HAL_SPI_TransmitReceiveFast(pspi, (uint8_t*)&txDummy, (uint8_t*)&rx_data[i], 1, 5);
    }
}

uint8_t spi_read_byte(uint8_t tx_data)
{
	SPI_HandleTypeDef *pspi=&hspi3;
	uint8_t rx_data;
//    if ( HAL_SPI_TransmitReceive(pspi, (uint8_t*) &tx_data, (uint8_t*)&rx_data, 1, 5) == HAL_OK )
//    {
//        return(rx_data);
//    }
	HAL_SPI_TransmitReceiveFast(pspi, (uint8_t*) &tx_data, (uint8_t*)&rx_data, 1, 5);
	return(1);
}

void HAL_SPI_TransmitReceiveFast(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
        uint32_t Timeout)
{
	SPI_TypeDef *SPIx= hspi->Instance;
	uint16_t count=Size;

//	if (!((SPIx)->CR1 & SPI_CR1_SPE)) {return;}
	__HAL_SPI_ENABLE(hspi);

	while (count--)
	{
		/* Wait busy */
		while ((SPIx->SR & SPI_FLAG_TXE) == 0 || (SPIx->SR & SPI_FLAG_BSY));

		/* Fill output buffer with data */
		*(__IO uint8_t *)&SPIx->DR = *pTxData++;

		/* Wait for SPI to end everything */
		while ((SPIx->SR & SPI_FLAG_RXNE) == 0 || (SPIx->SR & SPI_FLAG_BSY));

		/* Read data register */
		*pRxData++ = *(__IO uint8_t *)&SPIx->DR;
	}
}
