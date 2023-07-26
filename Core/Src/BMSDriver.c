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
    uint8_t ret_val[len];
//    uint8_t i;

//    for ( i = 0; i < len; i++ )
//    {
//        HAL_SPI_TransmitReceiveFast(pspi, (uint8_t*)&data[i], &ret_val, 1, BMS_SPI_TIMEOUT_MS);
//    }

    HAL_SPI_TransmitReceiveFast(pspi, (uint8_t*)&data[0], &ret_val[0], len, BMS_SPI_TIMEOUT_MS);

    /*DMA SPI*/
	//uint8_t ret_val[len];
	//HAL_SPI_TransmitReceive_DMA(pspi, (uint8_t*) data, (uint8_t*)ret_val, len);
}

void spi_write_read(uint8_t *tx_data,//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data array
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   )
{
	SPI_HandleTypeDef *pspi=&hspi3;
    uint8_t i;
    uint8_t rxDummy[tx_len];
    uint8_t txDummy[rx_len];

    for(i=0; i<tx_len; i++)
    {
    	txDummy[i]=0xFF;
    }

//    // Transfer data to LTC681x
//    for ( i = 0; i < tx_len; i++ )
//    {
//        // Transmit byte.
//        HAL_SPI_TransmitReceiveFast(pspi, (uint8_t*)&tx_data[i], (uint8_t*)&rxDummy, 1, BMS_SPI_TIMEOUT_MS);
//    }

//    // Receive data from DC2259A board.
//    for ( i = 0; i < rx_len; i++ )
//    {
//        // Receive byte.
//        HAL_SPI_TransmitReceiveFast(pspi, (uint8_t*)&txDummy, (uint8_t*)&rx_data[i], 1, BMS_SPI_TIMEOUT_MS);
//    }

    HAL_SPI_TransmitReceiveFast(pspi, (uint8_t*)&tx_data[0], (uint8_t*)&rxDummy[0], tx_len, BMS_SPI_TIMEOUT_MS);
    HAL_SPI_TransmitReceiveFast(pspi, (uint8_t*)&txDummy[0], (uint8_t*)&rx_data[0], rx_len, BMS_SPI_TIMEOUT_MS);

	/*DMA SPI*/
    //HAL_SPI_TransmitReceive_DMA(pspi, (uint8_t*)&tx_data[0], (uint8_t*)&rxDummy[0], tx_len);
    //HAL_SPI_TransmitReceive_DMA(pspi, (uint8_t*)&txDummy[0], (uint8_t*)&rx_data[0], rx_len);
}

uint8_t spi_read_byte(uint8_t tx_data)
{
	SPI_HandleTypeDef *pspi=&hspi3;
	uint8_t rx_data;
//    if ( HAL_SPI_TransmitReceiveFast(pspi, (uint8_t*) &tx_data, (uint8_t*)&rx_data, 1, BMS_SPI_TIMEOUT_MS) == HAL_OK )
//    {
//        return(rx_data);
//    }


	HAL_SPI_TransmitReceiveFast(pspi, (uint8_t*) &tx_data, (uint8_t*)&rx_data, 1, BMS_SPI_TIMEOUT_MS);

	/*DMA SPI*/
	//HAL_SPI_TransmitReceive_DMA(pspi, (uint8_t*) &tx_data, (uint8_t*)&rx_data, 1);
	return(rx_data);
}

/* WORKING IMPLEMENTATION (but with pauses)*/
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

//void HAL_SPI_TransmitReceiveFast(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
//	uint32_t Timeout)
//{
//	SPI_TypeDef *SPIx = hspi->Instance;
//	__HAL_SPI_ENABLE(hspi);
//	while (Size)
//	{
//		uint32_t rSR = SPIx->SR;
//		if (rSR & SPI_FLAG_TXE)
//		{
//			*(volatile uint8_t *)&SPIx->DR = *pTxData++;
//		}
//		if (rSR & SPI_FLAG_RXNE)
//		{
//			*pRxData++ = *(volatile uint8_t *)&SPIx->DR;
//			--Size;
//		}
//	}
//	while ((SPIx->SR & SPI_FLAG_BSY));
//}


//void HAL_SPI_TransmitReceiveFast(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
//{
//    SPI_TypeDef *SPIx = hspi->Instance;
//    uint16_t count = Size;
//    uint16_t txCount = 0;
//    uint16_t rxCount = 0;
//
//    // Enable the SPI peripheral
//    __HAL_SPI_ENABLE(hspi);
//
//    // Wait for the SPI to be ready before transmitting
//    while ((SPIx->SR & SPI_FLAG_TXE) == 0 || (SPIx->SR & SPI_FLAG_BSY));
//
//    // Fill the TX FIFO initially, up to its depth (16 bytes)
//    while (txCount < 16 && count > 0)
//    {
//        *(__IO uint8_t *)&SPIx->DR = *pTxData++;
//        txCount++;
//        count--;
//    }
//
//    // Transmit and receive data in a loop
//    while (count > 0)
//    {
//        // Check if TX FIFO is empty and can be filled
//        if ((SPIx->SR & SPI_FLAG_TXE) && txCount < 16)
//        {
//            *(__IO uint8_t *)&SPIx->DR = *pTxData++;
//            txCount++;
//            count--;
//        }
//
//        // Check if RX FIFO has data to be read
//        if ((SPIx->SR & SPI_FLAG_RXNE) && rxCount < 16)
//        {
//            *pRxData++ = *(__IO uint8_t *)&SPIx->DR;
//            rxCount++;
//        }
//    }
//
//    // Wait for the last byte to be received
//    while ((SPIx->SR & SPI_FLAG_RXNE) == 0);
//
//    // Read the last received byte
//    *pRxData++ = *(__IO uint8_t *)&SPIx->DR;
//
//    // Wait until the last data has been sent out before disabling the SPI peripheral
//    while ((SPIx->SR & SPI_FLAG_TXE) == 0 || (SPIx->SR & SPI_FLAG_BSY));
//
//    // Disable the SPI peripheral
//    __HAL_SPI_DISABLE(hspi);
//}


//void HAL_SPI_TransmitReceiveFast(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
//{
//    SPI_TypeDef *SPIx = hspi->Instance;
//    uint16_t count = Size;
//    uint16_t txCount = 0;
//    uint16_t rxCount = 0;
//
//    // Enable the SPI peripheral
//    __HAL_SPI_ENABLE(hspi);
//
//    // Fill the TX FIFO initially, up to its depth (16 bytes)
//    while (txCount < 16 && count > 0)
//    {
//        *(__IO uint8_t *)&SPIx->DR = *pTxData++;
//        txCount++;
//        count--;
//    }
//
//    // Transmit and receive data in a loop
//    while (count)
//    {
//        // Check if RX FIFO has data to be read
//        if (SPIx->SR & SPI_FLAG_RXNE)
//        {
//        	//Read the data
//            *pRxData++ = *(__IO uint8_t *)&SPIx->DR;
//            // Add the next data to TxFIFO if it exists
//            if(txCount<Size)
//            {
//                *(__IO uint8_t *)&SPIx->DR = *pTxData++;
//                txCount++;
//                count--;
//            }
//        }
//    }
//
//	// Wait for the last byte to be received
//	while ((SPIx->SR & SPI_FLAG_RXNE) == 0);
//
//	// Read the last received byte
//	*pRxData++ = *(__IO uint8_t *)&SPIx->DR;
//
//    // Wait until the last data has been sent out before disabling the SPI peripheral
//    while (SPIx->SR & SPI_FLAG_BSY);
//
//    // Disable the SPI peripheral
//    __HAL_SPI_DISABLE(hspi);
//}

