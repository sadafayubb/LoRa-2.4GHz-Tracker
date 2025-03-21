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
#include "hw-gpio.h"

#define BOARD_NUCLEO_L073RZ
#define SHIELD_PCB_E394V02A

#include "boards/boards.h"
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
        .SpreadingFactor = LORA_SF6,
        .Bandwidth = LORA_BW_1600,
        .CodingRate = LORA_CR_4_5
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
    if(GPIO_Pin == GPIO_PIN_4) { // PB4 (DIO1)
    	 printf("IRQ FIRED!\n");  // Check if this prints
        uint16_t irqStatus = Radio.GetIrqStatus();
        Radio.ClearIrqStatus(IRQ_RADIO_ALL);

        if(irqStatus & IRQ_RANGING_MASTER_RESULT_VALID) {
            double distance = Radio.GetRangingResult(RANGING_RESULT_RAW);

            // Direct printf to UART2
            printf("Distance: %.2f m\r\n", distance);
        }else { // Slave
            if(irqStatus & IRQ_RANGING_SLAVE_REQUEST_VALID) {
                printf("Slave: Request Received!\r\n");
            }
        }
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

  Radio.SetStandby(STDBY_RC);
  Radio.SetPacketType(PACKET_TYPE_RANGING);
  Radio.SetModulationParams(&mod_params);

  // 5. Configure RF
  Radio.SetRfFrequency(RF_FREQUENCY); // 2.403 GHz
  Radio.SetTxParams(13, RADIO_RAMP_20_US);

  printf("Default Initialization done...\n");

  if(DEMO_SETTING_ENTITY){
	  // 3. Set Slave Address
	  Radio.SetRangingRequestAddress(ADDRESS);
	  Radio.SetRangingRole(RADIO_RANGING_ROLE_MASTER);
	  Radio.SetTx(RX_TX_CONTINUOUS);

  } else {
	  // 3. Set Own Address
	  Radio.SetDeviceRangingAddress(ADDRESS);
	  Radio.SetRangingRole(RADIO_RANGING_ROLE_SLAVE);
	  Radio.SetRx(RX_TX_CONTINUOUS);
  }

  printf("Setting Defined initialization done...\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
