/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// vv kad veiktų printf į uartę3
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void CAN_Config(CAN_HandleTypeDef *phcan, uint8_t FIFO_Num);

uint8_t Can_TxMessage(CAN_HandleTypeDef *phcan, uint8_t ide, uint32_t id, uint8_t len, uint8_t *pdata);

void Can_Dome(CAN_HandleTypeDef *hcan1);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
/*
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = 0x141;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;*/



  CAN_Config(&hcan1, 0);
  //CAN_Config(&hcan1, 1);

  robo_spin();
  while (1)
  {

	  //printf("heartbeat (%d free)\n\r", HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
	  HAL_Delay(500);

	  uint8_t sta;

      robo_status();

	  if(sta != 0)
	  {
	     printf("err:%d\n\r", sta);
	  }

	 // Can_Dome(&hcan1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6; //18; //18;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart3.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint8_t Can_TxMessage(CAN_HandleTypeDef *phcan, uint8_t ide, uint32_t id, uint8_t len, uint8_t *pdata)
{
    uint32_t  TxMailbox;           //Get the email number that was sent
    CAN_TxHeaderTypeDef TxHeader;  //Send-header protocol information structure, used to fill parameters
    HAL_StatusTypeDef   HAL_RetVal; //CAN return value
    uint16_t i = 0;
    /*Fill and send header protocol*/
    if(ide == 0)
    {
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.StdId = id;
    }
    else
    {
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.ExtId = id;
    }

    TxHeader.RTR = CAN_RTR_DATA,          //The frame type of the message data frame
    TxHeader.DLC = len,                   //The length of the frame 8
    TxHeader.TransmitGlobalTime = DISABLE; //Do not capture time

    /*Ask if CAN has a free mailbox*/
    while(HAL_CAN_GetTxMailboxesFreeLevel(phcan) == 0)
    {
        i++;
        if(i > 0xfffe) {//Timeout, sending failed
            return 1;
        }
    }
    HAL_Delay(1);

    /*Send frame*/
    HAL_RetVal = HAL_CAN_AddTxMessage(phcan, &TxHeader, pdata, &TxMailbox); //Send a frame of data
    //printf("TxMailbox %d\r\n",TxMailbox);
    if(HAL_RetVal != HAL_OK)
        return 1;
    return 0;
}

/*For loopback test*/
void Can_Dome(CAN_HandleTypeDef *hcan1)
{
    uint8_t sta;
    sta = Can_TxMessage(hcan1, 0, 8, 8, "789456");
    if(sta != 0)
    {
        printf("err:%d\n\r", sta);
    }
}

void robo_spin()
{

    uint8_t sta;

    uint8_t TxData[8];
    TxData[0] = 0xA2;
    TxData[1] = 0x01;
    TxData[2] = 0x04;
    TxData[3] = 0xE5;
    TxData[4] = 0x10;
    TxData[5] = 0x27;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    sta = Can_TxMessage(&hcan1, CAN_ID_STD, 0x141, 8, &TxData);
    if(sta != 0)
    {
        printf("err:%d\n\r", sta);
    }
}

void robo_status()
{

    uint8_t sta;

    uint8_t TxData[8];
    TxData[0] = 0x9A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    sta = Can_TxMessage(&hcan1, CAN_ID_STD, 0x141, 8, &TxData);
    if(sta != 0)
    {
        printf("err:%d\n\r", sta);
    }
}


void CAN_Config(CAN_HandleTypeDef *phcan, uint8_t FIFO_Num)
{

    //Configure the filter to receive ID frames in the specified range
    CAN_FilterTypeDef CAN_FilterType;
    CAN_FilterType.FilterBank = 0;                        //Filter group [0,13]
    CAN_FilterType.SlaveStartFilterBank = 14;             //Start from the filter group [0,27]
    CAN_FilterType.FilterIdHigh = 0x0000;                 //ID high bit to be filtered [0x0000,0xFFFF]
    CAN_FilterType.FilterIdLow = 0x0000;                  //ID to be filtered low [0x0000,0xFFFF]
    CAN_FilterType.FilterMaskIdHigh = 0x0000;             //The high 16 bits of the filter must not match
    CAN_FilterType.FilterMaskIdLow = 0x0000;              //The lower 16 bits of the filter must not match
    CAN_FilterType.FilterFIFOAssignment = FIFO_Num;       //The filter is associated to (0=RX_FIFO0/1=RX_FIFO1)
    CAN_FilterType.FilterMode = CAN_FILTERMODE_IDMASK;    //Work in identifier mask bit mode
    CAN_FilterType.FilterScale = CAN_FILTERSCALE_32BIT;   //The filter bit width is a single 32 bits
    CAN_FilterType.FilterActivation = ENABLE;             //Enable filter
    if(HAL_CAN_ConfigFilter(phcan, &CAN_FilterType) != HAL_OK)
    {
        Error_Handler();
    }

    /*Open the interrupt service corresponding to the CAN channel*/
    if(FIFO_Num == 0)
    {
        if(HAL_CAN_ActivateNotification(phcan, CAN_IT_RX_FIFO0_MSG_PENDING ) != HAL_OK)
        {
            Error_Handler();
        }
    }
    else
    {
        if(HAL_CAN_ActivateNotification(phcan, CAN_IT_RX_FIFO1_MSG_PENDING ) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /*Start CAN communication*/
    if(HAL_CAN_Start(phcan) != HAL_OK)
    {
        Error_Handler();
    }
}


/*********************************************
 Function name: HAL_CAN_RxFifo0MsgPendingCallback
 Function: CAN channel 0 receives the callback function, and it will be triggered when a complete frame arrives.
 Formal parameter: hcan --CAN information structure (hcan/hcan1/hcan2)
 return value:
 Remarks:
**********************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t Rxbuff[8] = "";         //Store frame data
    CAN_RxHeaderTypeDef RxHeader;   //Store frame header protocol
    HAL_StatusTypeDef   HAL_RetVal; //CAN return value

    if(hcan->Instance == CAN1)
    {

        HAL_RetVal = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, Rxbuff); //Read a frame from channel 0 buffer
        if(HAL_RetVal == HAL_OK)
        {
            /*User-defined area*/
            if(RxHeader.IDE == 0)
            {
                printf("FIFO0,ID:%d -- Rxbuff:%x\r\n", RxHeader.StdId, Rxbuff);
                //printf("ok 1\n\r");
            }
            else
            {
                //printf("FIFO0,ID:%d -- Rxbuff:%s\r\n", RxHeader.ExtId, Rxbuff);
                printf("ok 2\n\r");
            }
        }
    }
    //if(hcan->Instance == CAN2)//When there are two CANs, share channel 0
}
/*********************************************
 Function name: HAL_CAN_RxFifo1MsgPendingCallback
 Function: CAN channel 1 receives the callback function, which will be triggered when a complete frame arrives.
 Formal parameter: hcan --CAN information structure (hcan/hcan1/hcan2)
 return value:
 Remarks:
**********************************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t Rxbuff[8] = "";         //Store frame data
    CAN_RxHeaderTypeDef RxHeader;   //Store frame header protocol
    HAL_StatusTypeDef   HAL_RetVal; //CAN return value

    if(hcan->Instance == CAN1)
    {
        HAL_RetVal = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, Rxbuff); //Read a frame from the channel 1 buffer
        if(HAL_RetVal == HAL_OK)
        {
            /*User-defined area*/
            if(RxHeader.IDE == 0)
            {
               // printf("FIFO1,ID:%d -- Rxbuff:%s\r\n", RxHeader.StdId, Rxbuff);
                printf("ok 3\n\r");
            }
            else
            {
                //printf("FIFO1,ID:%d -- Rxbuff:%s\r\n", RxHeader.ExtId, Rxbuff);
                printf("ok 4\n\r");
            }
        }
    }
    //if(hcan->Instance == CAN2)//When there are two CANs, share channel 1
}


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
