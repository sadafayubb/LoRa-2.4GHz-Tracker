/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Matthieu Verdy and Benjamin Boulet
*/
#include "hw.h"
#include "sx1280-hal.h"
#include "radio.h"
#include <string.h>
#include "spi.h"
extern SPI_HandleTypeDef hspi1;

/*!
 * \brief Define the size of tx and rx hal buffers
 *
 * The Tx and Rx hal buffers are used for SPI communication to
 * store data to be sent/receive to/from the chip.
 *
 * \warning The application must ensure the maximal useful size to be much lower
 *          than the MAX_HAL_BUFFER_SIZE
 */
#define MAX_HAL_BUFFER_SIZE   0xFFF

#define IRQ_HIGH_PRIORITY  0

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1280Init,
    SX1280HalReset,
    SX1280GetStatus,
    SX1280HalWriteCommand,
    SX1280HalReadCommand,
    SX1280HalWriteRegisters,
    SX1280HalWriteRegister,
    SX1280HalReadRegisters,
    SX1280HalReadRegister,
    SX1280HalWriteBuffer,
    SX1280HalReadBuffer,
    SX1280HalGetDioStatus,
    SX1280GetFirmwareVersion,
    SX1280SetRegulatorMode,
    SX1280SetStandby,
    SX1280SetPacketType,
    SX1280SetModulationParams,
    SX1280SetPacketParams,
    SX1280SetRfFrequency,
    SX1280SetBufferBaseAddresses,
    SX1280SetTxParams,
    SX1280SetDioIrqParams,
    SX1280SetSyncWord,
    SX1280SetRx,
    SX1280GetPayload,
    SX1280SendPayload,
    SX1280SetRangingRole,
    SX1280SetPollingMode,
    SX1280SetInterruptMode,
    SX1280SetRegistersDefault,
    SX1280GetOpMode,
    SX1280SetSleep,
    SX1280SetFs,
    SX1280SetTx,
    SX1280SetRxDutyCycle,
    SX1280SetCad,
    SX1280SetTxContinuousWave,
    SX1280SetTxContinuousPreamble,
    SX1280GetPacketType,
    SX1280SetCadParams,
    SX1280GetRxBufferStatus,
    SX1280GetPacketStatus,
    SX1280GetRssiInst,
    SX1280GetIrqStatus,
    SX1280ClearIrqStatus,
    SX1280Calibrate,
    SX1280SetSaveContext,
    SX1280SetAutoTx,
    SX1280StopAutoTx,
    SX1280SetAutoFS,
    SX1280SetLongPreamble,
    SX1280SetPayload,
    SX1280SetSyncWordErrorTolerance,
    SX1280SetCrcSeed,
    SX1280SetBleAccessAddress,
    SX1280SetBleAdvertizerAccessAddress,
    SX1280SetCrcPolynomial,
    SX1280SetWhiteningSeed,
    SX1280EnableManualGain,
    SX1280DisableManualGain,
    SX1280SetManualGainValue,
    SX1280SetLNAGainSetting,
    SX1280SetRangingIdLength,
    SX1280SetDeviceRangingAddress,
    SX1280SetRangingRequestAddress,
    SX1280GetRangingResult,
    SX1280SetRangingCalibration,
    SX1280GetRangingPowerDeltaThresholdIndicator,
    SX1280RangingClearFilterResult,
    SX1280RangingSetFilterNumSamples,
    SX1280GetFrequencyError,
};

static uint8_t halTxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};
static uint8_t halRxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};

/*!
 * \brief Used to block execution waiting for low state on radio busy pin.
 *        Essentially used in SPI communications
 */
void SX1280HalWaitOnBusy(void) {
    uint32_t timeout = 10000;  // 0.01s timeout
    while(HAL_GPIO_ReadPin(RADIO_BUSY_PORT, RADIO_BUSY_PIN) == GPIO_PIN_SET) {
        if(--timeout == 0) {
            printf("BUSY Timeout!\r\n");
            break;
        }
    }
}

void SX1280HalInit( DioIrqHandler **irqHandlers )
{
    SX1280HalReset( );
    SX1280HalIoIrqInit( irqHandlers );
}

void SX1280HalIoIrqInit( DioIrqHandler **irqHandlers )
{
#if( RADIO_DIO1_ENABLE )
    GpioSetIrq( RADIO_DIO1_PORT, RADIO_DIO1_PIN, IRQ_HIGH_PRIORITY, irqHandlers[0] );
#endif
#if( RADIO_DIO2_ENABLE )
	GpioSetIrq( RADIO_DIO2_PORT, RADIO_DIO2_PIN, IRQ_HIGH_PRIORITY, irqHandlers[0] );
#endif
#if( RADIO_DIO3_ENABLE )
	GpioSetIrq( RADIO_DIO3_PORT, RADIO_DIO3_PIN, IRQ_HIGH_PRIORITY, irqHandlers[0] );
#endif
#if( !RADIO_DIO1_ENABLE && !RADIO_DIO2_ENABLE && !RADIO_DIO3_ENABLE )
#error "Please define a DIO"
#endif
}

void SX1280HalReset( void )
{
    HAL_Delay( 20 );
    HAL_GPIO_WritePin( RADIO_nRESET_PORT, RADIO_nRESET_PIN, 0 );
    HAL_Delay( 50 );
    HAL_GPIO_WritePin( RADIO_nRESET_PORT, RADIO_nRESET_PIN, 1 );
    HAL_Delay( 20 );
}

// fixed function
void SX1280HalClearInstructionRam(void) {
    uint16_t halSize = 3 + IRAM_SIZE;
    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = (IRAM_START_ADDRESS >> 8) & 0x00FF;
    halTxBuffer[2] = IRAM_START_ADDRESS & 0x00FF;

    // Fill with zeros
    memset(halTxBuffer + 3, 0x00, IRAM_SIZE);

    SX1280HalWaitOnBusy();
    HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, halTxBuffer, halSize, 1000); //  HAL defined function
    HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_SET);
    SX1280HalWaitOnBusy();
}

// fixed function
void SX1280HalWakeup(void) {
    __disable_irq();

    halTxBuffer[0] = RADIO_GET_STATUS;
    halTxBuffer[1] = 0x00;

    SX1280HalWaitOnBusy();
    HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, halTxBuffer, 2, 100); // HAL Defined
    HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_SET);

    // Wait for chip to be ready.
    SX1280HalWaitOnBusy();
    __enable_irq();
}

// Fixed HAL WriteCommand
void SX1280HalWriteCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size) {
	halTxBuffer[0] = (uint8_t)command;
    if(size > 0) {
        memcpy(halTxBuffer + 1, buffer, size);
    }

    SX1280HalWaitOnBusy();
    HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, halTxBuffer, size + 1, 1000); // Single transmit
    HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_SET);
    SX1280HalWaitOnBusy();
}

// fixed command
void SX1280HalReadCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size) {
    halTxBuffer[0] = (uint8_t)command;
    halTxBuffer[1] = 0x00; // Dummy byte

    for( uint16_t index = 0; index < size; index++ ){
            halTxBuffer[2+index] = 0x00;
    }

    SX1280HalWaitOnBusy();
    HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, halTxBuffer, halRxBuffer, size + 2, 1000);
    memcpy(buffer, halRxBuffer + 2, size); // Skip command & status bytes
    HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_SET);
    SX1280HalWaitOnBusy();
}

void SX1280HalWriteRegisters(uint16_t address, uint8_t *data, uint16_t size) {

    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = (address >> 8) & 0xFF;
    halTxBuffer[2] = address & 0xFF;
    memcpy(halTxBuffer + 3, data, size);

    SX1280HalWaitOnBusy();
    HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, halTxBuffer, size + 3, 1000);
    HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_SET);
    SX1280HalWaitOnBusy();
}

void SX1280HalWriteRegister( uint16_t address, uint8_t value )
{
    SX1280HalWriteRegisters( address, &value, 1 );
}

//fixed function that is actually needed
void SX1280HalReadRegisters(uint16_t address, uint8_t *buffer, uint16_t size) {
    uint16_t halSize = 4 + size;
    halTxBuffer[0] = RADIO_READ_REGISTER;
    halTxBuffer[1] = (address >> 8) & 0xFF;
    halTxBuffer[2] = address & 0xFF;
    halTxBuffer[3] = 0x00; // Dummy byte

    // Clear receive buffer
    memset(halRxBuffer, 0, sizeof(halRxBuffer));

    SX1280HalWaitOnBusy();
    HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, halTxBuffer, halRxBuffer, halSize, 1000); // ✅ HAL
    HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_SET);

    memcpy(buffer, halRxBuffer + 4, size);
    SX1280HalWaitOnBusy();
}

uint8_t SX1280HalReadRegister( uint16_t address )
{
    uint8_t data;

    SX1280HalReadRegisters( address, &data, 1 );

    return data;
}

void SX1280HalWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
	uint8_t halBuffer[MAX_HAL_BUFFER_SIZE];
	halBuffer[0] = RADIO_WRITE_BUFFER;
	halBuffer[1] = offset;
	memcpy(halBuffer + 2, buffer, size);

	SX1280HalWaitOnBusy();
	HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, halBuffer, size + 2, 100); // ✅ HAL
	HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_SET);
}

void SX1280HalReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) {
    uint16_t halSize = 3 + size;
    halTxBuffer[0] = RADIO_READ_BUFFER;
    halTxBuffer[1] = offset;
    halTxBuffer[2] = 0x00; // Dummy byte

    // Clear receive buffer
    memset(halRxBuffer, 0, sizeof(halRxBuffer));

    SX1280HalWaitOnBusy();
    HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, halTxBuffer, halRxBuffer, halSize, 1000); // ✅ HAL
    HAL_GPIO_WritePin(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PIN_SET);

    memcpy(buffer, halRxBuffer + 3, size);
    SX1280HalWaitOnBusy();
}

uint8_t SX1280HalGetDioStatus( void )
{
	uint8_t Status = HAL_GPIO_ReadPin( RADIO_BUSY_PORT, RADIO_BUSY_PIN );
	
#if( RADIO_DIO1_ENABLE )
	Status |= (HAL_GPIO_ReadPin( RADIO_DIO1_PORT, RADIO_DIO1_PIN ) << 1);
#endif
#if( RADIO_DIO2_ENABLE )
	Status |= (HAL_GPIO_ReadPin( RADIO_DIO2_PORT, RADIO_DIO2_PIN ) << 2);
#endif
#if( RADIO_DIO3_ENABLE )
	Status |= (HAL_GPIO_ReadPin( RADIO_DIO3_PORT, RADIO_DIO3_PIN ) << 3);
#endif
#if( !RADIO_DIO1_ENABLE && !RADIO_DIO2_ENABLE && !RADIO_DIO3_ENABLE )
#error "Please define a DIO" 
#endif
	
	return Status;
}
