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
#include "i3c.h"
#include "icache.h"
#include "memorymap.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define Direct_READ             0x09
#define TEMPERATURE_REGISTER    0x00
#define MAX_DEVICES             10


__IO uint32_t uwTargetCount = 0;
uint32_t ubTargetIndex;
volatile uint8_t DAACplt = 0;

TargetDesc_t TargetDesc1 ={ "TARGET_ID1",  DEVICE_ID1,  0x0000000000000000, 0x00, TARGET1_DYN_ADDR, };
TargetDesc_t TargetDesc2 ={ "TARGET_ID2",  DEVICE_ID2,  0x0000000000000000, 0x00, TARGET2_DYN_ADDR, };
TargetDesc_t TargetDesc3 ={ "TARGET_ID3",  DEVICE_ID3,  0x0000000000000000, 0x00, TARGET3_DYN_ADDR, };
TargetDesc_t TargetDesc4 ={ "TARGET_ID4",  DEVICE_ID4,  0x0000000000000000, 0x00, TARGET4_DYN_ADDR, };
TargetDesc_t TargetDesc5 ={ "TARGET_ID5",  DEVICE_ID5,  0x0000000000000000, 0x00, TARGET5_DYN_ADDR, };
TargetDesc_t TargetDesc6 ={ "TARGET_ID6",  DEVICE_ID6,  0x0000000000000000, 0x00, TARGET6_DYN_ADDR, };
TargetDesc_t TargetDesc7 ={ "TARGET_ID7",  DEVICE_ID7,  0x0000000000000000, 0x00, TARGET7_DYN_ADDR, };
TargetDesc_t TargetDesc8 ={ "TARGET_ID8",  DEVICE_ID8,  0x0000000000000000, 0x00, TARGET8_DYN_ADDR, };
TargetDesc_t TargetDesc9 ={ "TARGET_ID9",  DEVICE_ID9,  0x0000000000000000, 0x00, TARGET9_DYN_ADDR, };
TargetDesc_t TargetDesc10={ "TARGET_ID10", DEVICE_ID10, 0x0000000000000000, 0x00, TARGET10_DYN_ADDR, };

TargetDesc_t *aTargetDesc[MAX_DEVICES] = {
    &TargetDesc1, &TargetDesc2, &TargetDesc3, &TargetDesc4, &TargetDesc5,
    &TargetDesc6, &TargetDesc7, &TargetDesc8, &TargetDesc9, &TargetDesc10
};

I3C_DeviceConfTypeDef DeviceConf[COUNTOF(aTargetDesc)] = {0};


uint8_t aRxBuffer[MAX_DEVICES][2];
uint8_t aTxBuffer[1];
uint32_t aControlBuffer[0xFF];
I3C_XferTypeDef buffer;

float temperature[MAX_DEVICES];
volatile uint8_t RxCplt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart3,(uint8_t*)ptr,len,100);
  return len;
}

void ReadTemperatures(void) {
  for (uint32_t i = 0; i < uwTargetCount; i++) {
  //for (uint32_t i = 0; i < 1; i++) {

    aTxBuffer[0] = TEMPERATURE_REGISTER;

    buffer.CtrlBuf.pBuffer = aControlBuffer;
    buffer.CtrlBuf.Size    = sizeof(aControlBuffer);
    buffer.RxBuf.pBuffer   = aRxBuffer[i];
    buffer.RxBuf.Size      = 2;
    buffer.TxBuf.pBuffer   = aTxBuffer;
    buffer.TxBuf.Size      = 1;

    I3C_CCCTypeDef aCCCList[] = {
        {DeviceConf[i].TargetDynamicAddr,
         Direct_READ,               // CCC príkaz: Čítanie
         {aTxBuffer, 2},            // Adresa registra teploty
         LL_I3C_DIRECTION_READ},    // Smer prenosu: Čítanie
    };

    if (HAL_I3C_AddDescToFrame(&hi3c1,
        aCCCList,
        NULL,
        &buffer,
        COUNTOF(aCCCList),
        I3C_DIRECT_WITHOUT_DEFBYTE_RESTART) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_I3C_Ctrl_ReceiveCCC_IT(&hi3c1, &buffer) != HAL_OK) {
        Error_Handler();
    }

    // TODO add TMO
    while(!RxCplt);
    RxCplt = 0;
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
  MX_USART3_UART_Init();
  MX_I3C1_Init();
  MX_ICACHE_Init();
  /* USER CODE BEGIN 2 */

  printf("I3C test\r\n");

  // Dynamic Address Assignation (DAA) start
  if (HAL_I3C_Ctrl_DynAddrAssign_IT(&hi3c1, I3C_RSTDAA_THEN_ENTDAA) != HAL_OK) {
    Error_Handler();
  }
  while (HAL_I3C_GetState(&hi3c1) != HAL_I3C_STATE_READY) { }

  for(ubTargetIndex = 0; ubTargetIndex < uwTargetCount; ubTargetIndex++) {
    DeviceConf[ubTargetIndex].DeviceIndex        = (ubTargetIndex+1);
    DeviceConf[ubTargetIndex].TargetDynamicAddr  = aTargetDesc[ubTargetIndex]->DYNAMIC_ADDR;
    DeviceConf[ubTargetIndex].IBIAck             = __HAL_I3C_GET_IBI_CAPABLE(__HAL_I3C_GET_BCR(aTargetDesc[ubTargetIndex]->TARGET_BCR_DCR_PID));
    DeviceConf[ubTargetIndex].IBIPayload         = __HAL_I3C_GET_IBI_PAYLOAD(__HAL_I3C_GET_BCR(aTargetDesc[ubTargetIndex]->TARGET_BCR_DCR_PID));
    DeviceConf[ubTargetIndex].CtrlRoleReqAck     = __HAL_I3C_GET_CR_CAPABLE(__HAL_I3C_GET_BCR(aTargetDesc[ubTargetIndex]->TARGET_BCR_DCR_PID));
    DeviceConf[ubTargetIndex].CtrlStopTransfer   = DISABLE;

    if (HAL_I3C_Ctrl_ConfigBusDevices(&hi3c1, &DeviceConf[ubTargetIndex], 1U) != HAL_OK)
      Error_Handler();
  }

  while(!DAACplt);
  printf("DAA done, found %ld devices\r\n", uwTargetCount);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t tick = HAL_GetTick();
  while (1)
  {
    if( (HAL_GetTick() - tick) > 250) {
      tick = HAL_GetTick();
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

      ReadTemperatures();
      printf("Senzor: ");
      for (uint32_t i = 0; i < uwTargetCount; i++) {
        printf("T[%lu]=%.2f C, ", i + 1, temperature[i]);
      }
      printf("\r\n");
    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 31;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 2048;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/* USER CODE BEGIN 4 */

void HAL_I3C_TgtReqDynamicAddrCallback(I3C_HandleTypeDef *hi3c, uint64_t targetPayload) {
  aTargetDesc[uwTargetCount]->TARGET_BCR_DCR_PID = targetPayload;
  HAL_I3C_Ctrl_SetDynAddr(hi3c, aTargetDesc[uwTargetCount++]->DYNAMIC_ADDR);
}

void HAL_I3C_CtrlDAACpltCallback(I3C_HandleTypeDef *hi3c) {
  DAACplt = 1;
}

void HAL_I3C_NotifyCallback(I3C_HandleTypeDef *hi3c, uint32_t eventId) {
  //
}

void HAL_I3C_CtrlRxCpltCallback(I3C_HandleTypeDef *hi3c) {
  if (hi3c->Instance == I3C1) {
    for (uint32_t i = 0; i < uwTargetCount; i++) {
    //for (uint32_t i = 0; i < 1; i++) {
      uint16_t raw_temperature = (aRxBuffer[i][0] << 8) | aRxBuffer[i][1];
      temperature[i] =  ((float)(raw_temperature>>4))*0.0625f;
    }
    RxCplt = 1;
  }
}
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
