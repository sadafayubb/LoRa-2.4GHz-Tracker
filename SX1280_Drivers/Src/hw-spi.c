#include "hw.h"

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef SpiHandle;
volatile bool blockingDmaFlag;

void SpiDeInit( void )
{
    HAL_SPI_DeInit( &SpiHandle );
}

#define WAIT_FOR_BLOCKING_FLAG         while( blockingDmaFlag ) { }
/*!
 * @brief Sends txBuffer and receives rxBuffer
 *
 * @param [IN] txBuffer Byte to be sent
 * @param [OUT] rxBuffer Byte to be sent
 * @param [IN] size Byte to be sent
 */
void SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size )
{
    HAL_SPIEx_FlushRxFifo( &SpiHandle );
    #ifdef USE_DMA
        blockingDmaFlag = true;
        HAL_SPI_TransmitReceive_DMA( &SpiHandle, txBuffer, rxBuffer, size );
        WAIT_FOR_BLOCKING_FLAG
    #else
        HAL_SPI_TransmitReceive( &SpiHandle, txBuffer, rxBuffer, size, HAL_MAX_DELAY );
    #endif
}

void SpiIn( uint8_t *txBuffer, uint16_t size )
{
    #ifdef USE_DMA
        blockingDmaFlag = true;
        HAL_SPI_Transmit_DMA( &SpiHandle, txBuffer, size );
        WAIT_FOR_BLOCKING_FLAG
    #else
        HAL_SPI_Transmit( &SpiHandle, txBuffer, size, HAL_MAX_DELAY );
    #endif
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    blockingDmaFlag = false;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    blockingDmaFlag = false;
}
