/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "w25qxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Start and end addresses of the user application. */
#define FLASH_APP_START_ADDRESS ((uint32_t)0x08002800u)
#define FLASH_APP_END_ADDRESS   ((uint32_t)FLASH_BANK1_END-0x10u) /**< Leave a little extra space at the end. */

/* Status report for the functions. */
typedef enum {
  F_OK              = 0x00u, /**< The action was successful. */
  F_ERROR_SIZE      = 0x01u, /**< The binary is too big. */
  F_ERROR_WRITE     = 0x02u, /**< Writing failed. */
  F_ERROR_READBACK  = 0x04u, /**< Writing was successful, but the content of the memory is wrong. */
  F_ERROR           = 0xFFu  /**< Generic error. */
} flash_status_t;

flash_status_t flash_erase(uint32_t address);
flash_status_t flash_write(uint32_t address, uint32_t *data, uint32_t length);
void flash_jump_to_app(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef rx_hdr;
CAN_TxHeaderTypeDef tx_hdr;
uint32_t tx_mailbox;
uint8_t can_rx_data[8];
const uint8_t can_response[] = "ready\n";
volatile uint8_t can_flag = 0;

typedef void (*p_main_program) (void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief   This function erases the memory.
 * @param   address: First address to be erased (the last is the end of the flash).
 * @return  status: Report about the success of the erasing.
 */
flash_status_t flash_erase(uint32_t address) {
  flash_status_t status = F_ERROR;
  uint32_t error = 0u;

  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef erase_init = {
    .TypeErase = FLASH_TYPEERASE_PAGES,
    .PageAddress = address,
    .Banks = FLASH_BANK_1,
    /* Calculate the number of pages from "address" and the end of flash. */
    .NbPages = (FLASH_BANK1_END - address) / FLASH_PAGE_SIZE
  };

  /* Do the actual erasing. */
  if (HAL_OK == HAL_FLASHEx_Erase(&erase_init, &error)) status = F_OK;
  HAL_FLASH_Lock();
  return status;
}

/**
 * @brief   This function flashes the memory.
 * @param   address: First address to be written to.
 * @param   *data:   Array of the data that we want to write.
 * @param   *length: Size of the array.
 * @return  status: Report about the success of the writing.
 */
flash_status_t flash_write(uint32_t address, uint32_t *data, uint32_t length) {
  flash_status_t status = F_OK;
  HAL_FLASH_Unlock();

  /* Loop through the array. */
  for (uint32_t i = 0u; (i < length) && (F_OK == status); i++) {
    /* If we reached the end of the memory, then report an error and don't do anything else.*/
    if (FLASH_APP_END_ADDRESS <= address) {
      status |= F_ERROR_SIZE;
    } else {
      /* The actual flashing. If there is an error, then report it. */
      if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data[i])) {
        status |= F_ERROR_WRITE;
      }
      /* Read back the content of the memory. If it is wrong, then report an error. */
      // if (((data[i])) != (*(volatile uint32_t*)address)) {
      //   status |= F_ERROR_READBACK;
      // }
      /* Shift the address by a word. */
      address += 4u;
    }
  }
  HAL_FLASH_Lock();
  return status;
}

// void LoadStackAndGo(void* sp, void* entry) {
//   __asm (
//       "mov sp, r0 \n"
//       "mov pc, r1 \n"
//   );
// }

/* Function pointer to the address of the user application. */
volatile p_main_program p_jump_to_main_program;

/**
 * @brief   Actually jumps to the user application.
 * @param   void
 * @return  void
 */ 
void flash_jump_to_app(void) {
  // /* Function pointer to the address of the user application. */
  // p_main_program p_jump_to_main_program;

  __disable_irq();

  // HAL_DeInit();
  RCC->APB1RSTR = 0xFFFFFFFFU;
  RCC->APB1RSTR = 0x00;
  RCC->APB2RSTR = 0xFFFFFFFFU;
  RCC->APB2RSTR = 0x00;

  // SysTick DeInit
  SysTick->CTRL = 0;
  SysTick->VAL = 0;
  SysTick->LOAD = 0;

  // NVIC DeInit
  __set_BASEPRI(0);
  __set_CONTROL(0);
  // NVIC->ICER[0] = 0xFFFFFFFF;
  // NVIC->ICPR[0] = 0xFFFFFFFF;
  // NVIC->ICER[1] = 0xFFFFFFFF;
  // NVIC->ICPR[1] = 0xFFFFFFFF;
  // NVIC->ICER[2] = 0xFFFFFFFF;
  // NVIC->ICPR[2] = 0xFFFFFFFF;

	/* Clear Interrupt Enable Register & Interrupt Pending Register */
	for (uint8_t i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}

  /* Set the clock to the default state */
	HAL_RCC_DeInit();

  __enable_irq();

  /* Change the main and local stack pointer. */
  // __set_MSP(*(volatile uint32_t*) FLASH_APP_START_ADDRESS);
  // LoadStackAndGo((void *) FLASH_APP_START_ADDRESS, (void *) (FLASH_APP_START_ADDRESS + 4));
  // OLD: SCB->VTOR =* (volatile uint32_t*) FLASH_APP_START_ADDRESS;
  SCB->VTOR = FLASH_APP_START_ADDRESS;
  p_jump_to_main_program = (p_main_program)(*(volatile uint32_t*) (FLASH_APP_START_ADDRESS + 4u));
  // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);/////////////////
  p_jump_to_main_program();
}

static inline void can_init(void) {
	tx_hdr.DLC = sizeof(can_response);
	tx_hdr.ExtId = 0;
	tx_hdr.IDE = CAN_ID_STD;
	tx_hdr.RTR = CAN_RTR_DATA;

	/* CAN filter configuration */
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterIdHigh = 0;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14; // 0 - 27
	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
	  Error_Handler();
	}
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_hdr, can_rx_data) == HAL_OK) {
    can_flag = 1;
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
  uint8_t w25q_rx_buf[4];
  volatile uint8_t boot_flag = 0;
  volatile uint32_t boot_size = 0;
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
  MX_CAN_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  can_init();
  /** XXX:
   * !! на будущее: если не обнаружена W25Q на шине SPI, то искать эти данные следует во FLASH-памяти
   * */
  if (W25qxx_Init()) {
    W25qxx_ReadPage(w25q_rx_buf, 0, 0, sizeof(w25q_rx_buf));
    boot_flag = w25q_rx_buf[0];
    boot_size = w25q_rx_buf[1] | (w25q_rx_buf[2] << 8) | (w25q_rx_buf[3] << 16);
  }
  // HAL_Delay(1000);
  // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);/////////////////
  if (boot_flag == 1) {
    flash_erase(FLASH_APP_START_ADDRESS);
    W25qxx_EraseSector(0); // clear new boot info
  } else {
    // HAL_Delay(1000);
    flash_jump_to_app(); // next cycle woun't be reached
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t current_address = FLASH_APP_START_ADDRESS;
  HAL_CAN_AddTxMessage(&hcan, &tx_hdr, (uint8_t *) &can_response[0], &tx_mailbox); // send ready message
  while (boot_size) {
    if (can_flag) {
      can_flag = 0;
      if (flash_write(current_address, (uint32_t *) &can_rx_data[0], rx_hdr.DLC / sizeof(uint32_t)) != F_OK) Error_Handler();
      current_address += rx_hdr.DLC;
      boot_size -= rx_hdr.DLC;
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
      HAL_CAN_AddTxMessage(&hcan, &tx_hdr, (uint8_t *) &can_response[0], &tx_mailbox);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  NVIC_SystemReset();
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  while (1) {
    HAL_Delay(250);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
    HAL_Delay(250);
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
