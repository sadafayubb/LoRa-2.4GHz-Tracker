#include "hw.h"
#include "spi.h"

// Replace all instances of SpiHandle with hspi1
extern SPI_HandleTypeDef hspi1; // Add this at the top
volatile bool blockingDmaFlag;


void SpiInit( void )
{
    // Remove all SPI initialization code here
    // Keep only this line:
    MX_SPI1_Init(); // Use CubeMX-generated initialization
}

void SpiDeInit( void ){
    HAL_SPI_DeInit( &hspi1 );
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
//	HAL_SPIEx_FlushRxFifo( &hspi1 ); // Use hspi1
    #ifdef USE_DMA
        blockingDmaFlag = true;
        HAL_SPI_TransmitReceive_DMA( &hspi1, txBuffer, rxBuffer, size );
    #else
        HAL_SPI_TransmitReceive( &hspi1, txBuffer, rxBuffer, size, HAL_MAX_DELAY );
    #endif
}

void SpiIn( uint8_t *txBuffer, uint16_t size )
{
    #ifdef USE_DMA
        blockingDmaFlag = true;
        HAL_SPI_Transmit_DMA( &hspi1, txBuffer, size );
        WAIT_FOR_BLOCKING_FLAG
    #else
        HAL_SPI_Transmit( &hspi1, txBuffer, size, HAL_MAX_DELAY );
    #endif
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    blockingDmaFlag = false;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    blockingDmaFlag = false;
}
