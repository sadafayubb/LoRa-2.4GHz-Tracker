/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lptim.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "radio.h"
#include "sx1280.h"
#include "hw.h"
#include "sx1280-hal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Define MASTER or SLAVE in project settings
int DEMO_SETTING_ENTITY = 1; // 1 master, 0 slave

// Common parameters
#define RF_FREQUENCY         2402000000UL  // Hz
#define TX_POWER             13          // dBm (max)
#define ADDRESS              0x10000000  // 32-bit device address
#define CALIBRATION_VALUE    13000       // Example value for SF10

// Modulation Parameters (SF6, BW 1.6 MHz, CR 4/5)

ModulationParams_t mod_params = {
    .PacketType = PACKET_TYPE_RANGING,
    .Params.LoRa = {
        .SpreadingFactor = LORA_SF7,
        .Bandwidth = LORA_BW_1600,
        .CodingRate = LORA_CR_4_7
    }
};

// Packet Parameters (40 symbol preamble, explicit header)
PacketParams_t packet_params = {
    .PacketType = PACKET_TYPE_RANGING,
    .Params.LoRa = {
        .PreambleLength = 32u,
        .HeaderType = LORA_PACKET_VARIABLE_LENGTH,
        .CrcMode = LORA_CRC_ON,
        .InvertIQ = LORA_IQ_NORMAL
    }
};

uint16_t irqMaskM = IRQ_RANGING_MASTER_RESULT_VALID | IRQ_RANGING_MASTER_RESULT_TIMEOUT;
uint16_t irqMaskS = IRQ_RANGING_SLAVE_REQUEST_VALID|IRQ_RANGING_SLAVE_RESPONSE_DONE;

uint16_t last_status = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Retarget printf to UART2 (for PuTTY)
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 1000);
	return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_4) { // PB4 (DIO1, etc.)
        //printf("IRQ FIRED! Status = 0x%04X\n", Radio.GetIrqStatus());
        uint16_t irqStatus = SX1280GetIrqStatus();
        SX1280ClearIrqStatus(IRQ_RADIO_ALL); // Clear all IRQ flags

        if (irqStatus & IRQ_TX_DONE)
            printf("TX Done\n");
        if (irqStatus & IRQ_RX_DONE)
            printf("RX Done\n");
        if (irqStatus & IRQ_SYNCWORD_VALID)
            printf("Syncword Valid\n");
        if (irqStatus & IRQ_SYNCWORD_ERROR)
            printf("Syncword Error\n");
        if (irqStatus & IRQ_HEADER_VALID)
            printf("Header Valid\n");
        if (irqStatus & IRQ_HEADER_ERROR)
            printf("Header Error\n");
        if (irqStatus & IRQ_CRC_ERROR)
            printf("CRC Error\n");
        if (irqStatus & IRQ_RANGING_SLAVE_RESPONSE_DONE)
            printf("Ranging Slave Response Done\n");
        if (irqStatus & IRQ_RANGING_SLAVE_REQUEST_DISCARDED)
            printf("Ranging Slave Request Discarded\n");
        if (irqStatus & IRQ_RANGING_MASTER_RESULT_VALID)
            printf("Ranging Master Result Valid\n");
        if (irqStatus & IRQ_RANGING_MASTER_RESULT_TIMEOUT)
            printf("Ranging Master Result Timeout\n");
        if (irqStatus & IRQ_RANGING_SLAVE_REQUEST_VALID)
            printf("Ranging Slave Request Valid\n");
        if (irqStatus & IRQ_CAD_DONE)
            printf("CAD Done\n");
        if (irqStatus & IRQ_CAD_ACTIVITY_DETECTED)
            printf("CAD Activity Detected\n");
        if (irqStatus & IRQ_RX_TX_TIMEOUT)
            printf("RX/TX Timeout\n");
        if (irqStatus & IRQ_PREAMBLE_DETECTED)
            printf("Preamble Detected\n");
    }
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_LPTIM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  if(DEMO_SETTING_ENTITY){
	  printf("MASTER Starting...\n");
  }else {
	  printf("SLAVE Starting...\n");
  }

  // 1. Reset Radio
  printf("Resetting radio...\r\n");
  SX1280HalReset();

  // 2. Basic Radio Check
  printf("Reading firmware...\r\n");
  uint16_t fw_version = Radio.GetFirmwareVersion();
  printf("Firmware: 0x%04X\r\n", fw_version);

  // vs

  uint16_t reg = 0x153;
  uint8_t txData[4];
  uint8_t rxData[3];

  txData[0] = 0x19; // Read Register Command (0x1D)
  txData[1] = (uint8_t)(reg >> 8);      // High Byte of Address
  txData[2] = (uint8_t)(reg & 0xFF);    // Low Byte of Address
  txData[3] = 0x00;                     // Dummy Byte for Clocking Data Out

  HAL_StatusTypeDef txStatus;
  HAL_StatusTypeDef rxStatus;


  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  txStatus = HAL_SPI_Transmit(&hspi1, txData, 3, HAL_MAX_DELAY);
  rxStatus = HAL_SPI_Receive(&hspi1, rxData, 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  printf("Firmware: 0x%X 0x%X\r\n", rxData[1], rxData[2]);

     // ... rest of init ...

  SX1280SetStandby(STDBY_RC);
  SX1280SetPacketType(PACKET_TYPE_RANGING);
  SX1280SetModulationParams(&mod_params);
  SX1280SetPacketParams(&packet_params);

  // 5. Configure RF
  SX1280SetRfFrequency(RF_FREQUENCY); // 2.403 GHz
  SX1280SetTxParams(13, RADIO_RAMP_20_US);

  // Set calibration (adjust based on SF)
  //SX1280SetRangingCalibration(13528); // found in demoRanging.c

  // Additional SF & FEC Configuration (Datasheet Page 131)
  SX1280HalWriteRegister(0x925, 0x37);  // SetAdditionalSFConf
  SX1280HalWriteRegister(0x093C, 0x01);  // SetFECR

  if(DEMO_SETTING_ENTITY){
	  // 3. Set Slave Address
	  SX1280SetRangingRequestAddress(ADDRESS);

	  // Master configuration: enable both IRQs, and route RangingMasterResultValid to DIO1.
	  SX1280SetDioIrqParams(irqMaskM,
			  irqMaskM,  // Route primary event to DIO1
			  IRQ_RADIO_NONE,                               // No IRQ on DIO2
			  IRQ_RADIO_NONE);                              // No IRQ on DIO3

	  // With full 16-bit calibration:
	  SX1280HalWriteRegister(0x92C, 0x34);  // High byte
	  //SX1280HalWriteRegister(0x92D, 0xD8);  // Low byte

	  SX1280SetRangingRole(RADIO_RANGING_ROLE_MASTER);
	  SX1280SetTx(RX_TX_CONTINUOUS);

  } else {

	  // 3. Set Own Address
	  SX1280SetDeviceRangingAddress(ADDRESS);
	  SX1280SetRangingIdLength(RANGING_IDCHECK_LENGTH_32_BITS);
	  uint32_t length = Radio.ReadRegister(0x931);
	  printf("Length 0x%08lX\n", (unsigned long)length);

	  // Slave configuration: enable both IRQs, and route RangingSlaveResponseDone to DIO1.
	  SX1280SetDioIrqParams(irqMaskS,
			  irqMaskS, // Route the response done event to DIO1
			  IRQ_RADIO_NONE,                              // No IRQ on DIO2
			  IRQ_RADIO_NONE);                             // No IRQ on DIO3

	  // With full 16-bit calibration:
	  SX1280HalWriteRegister(0x92C, 0x34);  // High byte
	  //SX1280HalWriteRegister(0x92D, 0xD8);  // Low byte

	  SX1280SetRangingRole(RADIO_RANGING_ROLE_SLAVE);
	  SX1280SetRx(RX_TX_CONTINUOUS);
  }

  GpioWrite(LED_TX_PORT, LED_TX_PIN, 1);

  // more debuggies

  HAL_Delay(10); // a short delay might help
  // After all configurations, add:
  printf("\n=== Critical Register Verification ===\n");

  // Verify SF Configuration (0x925)
  uint8_t sf_conf = Radio.ReadRegister(0x925);
  printf("SF Config (0x925): 0x%02X %s\n",
         sf_conf,
         (sf_conf == 0x37) ? "(OK-SF7)" :
         (sf_conf == 0x1E) ? "(OK-SF5/6)" :
         (sf_conf == 0x32) ? "(OK-SF9-12)" : "(WRONG!)");

  // Verify FEC Configuration (0x093C)
  uint8_t fec_conf = Radio.ReadRegister(0x093C);
  printf("FEC Config (0x093C): 0x%02X %s\n",
         fec_conf,
         (fec_conf == 0x01) ? "(OK)" : "(WRONG!)");

  // Verify Calibration Value
  uint8_t cal_high = Radio.ReadRegister(0x92C);
  uint8_t cal_low = Radio.ReadRegister(0x92D);
  uint16_t full_cal = (cal_high << 8) | cal_low;
  printf("Calibration (0x92C-0x92D): 0x%04X %s\n",
         full_cal,
         (full_cal == 0x34D8) ? "(OK-SF7/BW1600)" : "(WRONG!)");

  // Verify Address Registers
  if(DEMO_SETTING_ENTITY) {
      uint8_t addr[4];
      addr[0] = Radio.ReadRegister(0x912);
      addr[1] = Radio.ReadRegister(0x913);
      addr[2] = Radio.ReadRegister(0x914);
      addr[3] = Radio.ReadRegister(0x915);
      printf("Master Address: 0x%02X%02X%02X%02X\n",
             addr[0], addr[1], addr[2], addr[3]);
  } else {
      uint8_t addr[4];
      addr[0] = Radio.ReadRegister(0x916);
      addr[1] = Radio.ReadRegister(0x917);
      addr[2] = Radio.ReadRegister(0x918);
      addr[3] = Radio.ReadRegister(0x919);
      printf("Slave Address: 0x%02X%02X%02X%02X\n",
             addr[0], addr[1], addr[2], addr[3]);
  }

  // end debuggies

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //more debuggies

	  // In main loop

	  uint16_t current_status = SX1280GetIrqStatus();
	  SX1280ClearIrqStatus(IRQ_RADIO_ALL); // Clear all IRQ flags
	  if(last_status != current_status){
		  if (current_status & IRQ_RANGING_MASTER_RESULT_VALID)
			  printf("Ranging Master Result Valid\n");
		  if (current_status & IRQ_RANGING_MASTER_RESULT_TIMEOUT)
			  printf("Ranging Master Result Timeout\n");
		  last_status = current_status;
	  }

	  //end debuggies

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
