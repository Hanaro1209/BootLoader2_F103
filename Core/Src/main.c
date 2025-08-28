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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*p_function)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 메모리 맵 정의 (STM32F103CB 128KB 플래시 기준)
// Flash Page Size는 1KB 입니다.
#define FLASH_PAGE_SIZE 0x400 // 1KB
#define BOOTLOADER_START_ADDRESS \
  0x08000000 // 부트로더: 16KB (0x08000000 - 0x08003FFF)
#define APP1_START_ADDRESS \
  0x08004000 // 애플리케이션 1: 32KB (0x08004000 - 0x0800BFFF)
#define APP2_START_ADDRESS \
  0x0800C000 // 애플리케이션 2: 32KB (0x0800C000 - 0x08013FFF)
#define APP3_START_ADDRESS 0x08014000 // 애플리케이션 3: 32KB (0x08014000 - 0x0801BFFF)

// 부팅할 앱 선택 정보를 저장하는 공유 데이터 페이지 (Flash의 마지막 페이지
// 사용)
#define SHARED_DATA_PAGE_ADDRESS 0x0801FC00

// 애플리케이션 유효성 검사
// 유효한 애플리케이션은 스택 포인터가 SRAM 영역 내에 있어야 합니다.
// STM32F103CB는 20KB SRAM (0x20000000 to 0x20004FFF)을 가집니다.
#define SRAM_START 0x20000000
#define SRAM_SIZE (20 * 1024)
#define SRAM_END (SRAM_START + SRAM_SIZE)
/* USER CODE END PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void jump_to_application(uint32_t app_address);
uint8_t check_for_application(uint32_t app_address);
void bootloader_uart_handler(void);
// uint8_t get_boot_app_number(void);
// void set_boot_app_number(uint8_t app_num);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission
   */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/**
 * @brief  주어진 주소에 유효한 애플리케이션이 있는지 확인합니다.
 * @param  app_address: 애플리케이션의 시작 주소.
 * @retval 유효하면 1, 그렇지 않으면 0.
 */
uint8_t check_for_application(uint32_t app_address)
{
  uint32_t stack_pointer = *(__IO uint32_t *)app_address;
  if (stack_pointer >= SRAM_START && stack_pointer < SRAM_END)
  {
    return 1; // 유효한 스택 포인터
  }
  return 0; // 유효하지 않은 스택 포인터
}

/**
 * @brief  애플리케이션으로 점프합니다.
 * @param  app_address: 애플리케이션의 시작 주소.
 * @retval None
 */
void jump_to_application(uint32_t app_address)
{
  p_function p_jump_to_app;

  // 1. 부트로더에서 사용한 모든 주변 장치들을 비활성화합니다.
  HAL_UART_DeInit(&huart2);
  HAL_GPIO_DeInit(LD2_GPIO_Port, LD2_Pin);
  HAL_GPIO_DeInit(B1_GPIO_Port, B1_Pin);
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

  // 2. 모든 인터럽트를 비활성화합니다.
  __disable_irq();

  // 3. HAL 및 Systick을 비활성화합니다.
  HAL_DeInit();

  // 4. 벡터 테이블 오프셋 레지스터(VTOR)를 애플리케이션의 벡터 테이블 주소로
  // 설정합니다.
  SCB->VTOR = app_address;

  // 5. 메인 스택 포인터(MSP)를 설정합니다.
  __set_MSP(*(__IO uint32_t *)app_address);

  // 6. 애플리케이션의 리셋 핸들러 주소를 가져와 점프합니다.
  p_jump_to_app = (p_function)(*(__IO uint32_t *)(app_address + 4));
  p_jump_to_app();

  // 이 부분은 실행되지 않아야 합니다.
  while (1)
    ;
}

/**
 * @brief  부트로더 UART 명령어 핸들러입니다.
 * @retval None
 */
void bootloader_uart_handler(void)
{
  // 이 함수는 UART를 통해 새로운 펌웨어를 수신하고,
  // 플래시에 쓰는 로직을 구현해야 합니다. (예: Y-Modem 프로토콜 사용)
  // 현재는 구현되지 않은 상태로, LED를 토글하며 대기합니다.
  printf("Bootloader command handler is not yet implemented.\r\n");
  printf("System Halted.\r\n");
  while (1)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(500);
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
  uint32_t app_address_to_boot = 0;
  uint8_t app_to_boot = 1; // 기본적으로 App 1을 부팅 시도
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  /* USER CODE BEGIN 2 */
  printf("\r\n\r\nSTM32 Multi-App Bootloader\r\n");

  uint8_t boot_cmd = 0;
  HAL_StatusTypeDef status;

  printf("Send 'b' within 2 seconds to enter bootloader mode.\r\n");

  // 2초 동안 'b' 명령어를 기다립니다.
  status = HAL_UART_Receive(&huart2, &boot_cmd, 1, 2000);

  if (status == HAL_OK && boot_cmd == 'b')
  {
    // 부트로더 모드로 진입
    printf("Entering bootloader mode.\r\n");
    bootloader_uart_handler(); // 이 함수는 무한 루프입니다.
  }
  else
  {
    // 기본 부팅 절차
    printf("Attempting to boot application...\r\n");
    // TODO: 공유 메모리에서 어떤 앱을 부팅할지 읽어오는
    // 로직(get_boot_app_number)을 구현해야 합니다. app_to_boot =
    // get_boot_app_number();
    printf("Boot choice: App %d\r\n", app_to_boot);

    switch (app_to_boot)
    {
    case 1:
      app_address_to_boot = APP1_START_ADDRESS;
      break;
    case 2:
      app_address_to_boot = APP2_START_ADDRESS;
      break;
    case 3:
      app_address_to_boot = APP3_START_ADDRESS;
      break;
    default:
      app_address_to_boot = 0;
      break; // 발생하면 안됨
    }

    if (app_address_to_boot != 0 &&
        check_for_application(app_address_to_boot))
    {
      printf("Valid application found. Jumping to 0x%08lX\r\n",
             app_address_to_boot);
      jump_to_application(app_address_to_boot);
    }
    else
    {
      printf("No valid application found at selected slot. Entering bootloader "
             "mode.\r\n");
      bootloader_uart_handler(); // 이 함수는 무한 루프입니다.
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 이 부분은 실행되지 않아야 합니다.
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
