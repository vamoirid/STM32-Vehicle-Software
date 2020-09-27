/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */

// Type Definitions //
CAN_RxHeaderTypeDef canRX;
CAN_TxHeaderTypeDef canTX;
CAN_FilterTypeDef sFilterConfig;

uint8_t CAN_1000Hz_Flag = 0;
uint32_t CAN_1000Hz_Cnt = 0;
uint8_t CAN_100Hz_Flag = 0;
uint32_t CAN_100Hz_Cnt = 0;
uint8_t CAN_Rx_Flag = 0;
uint8_t CAN_Id = 0;
uint8_t CAN_Rx_Data[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_Tx_Data[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_Txing = 0;
uint16_t CAN_Msg_Id = 0;
uint32_t CAN_Tx_Mailbox = 0;

uint16_t Brake = 0;
uint16_t APPS1 = 0;
uint16_t APPS2 = 0;
uint16_t Steering = 0;
uint16_t DCDC_12V= 0;
uint16_t DCDC_5V = 0;
uint16_t DCDC_5V_APPS = 0;
uint16_t PCB_Thermistor = 0;
uint16_t SuspFL = 0;
uint16_t SuspFR = 0;
uint16_t Core_Thermistor = 0;

uint16_t Hall_Right = 0;
uint16_t Hall_Left = 0;

uint8_t Enable_Toggle = 0;
uint8_t Second_Toggle = 0;
uint8_t Third_Toggle = 0;
uint8_t Start = 0;
uint8_t Ad_Act = 0;
uint8_t Green_TSAL = 0;
uint8_t SC_State = 0;
uint8_t IMD_State = 0;
uint8_t BMS_State = 0;
uint8_t Fans_PWM = 0;
uint8_t Buzzer = 0;
uint8_t Sensor_Error = 0;
uint8_t ENABLE_SIGNAL = 0;
uint8_t Safe_State = 0;
uint8_t SC_Software = 0;
uint32_t Msg_101_Cnt = 0;
uint32_t Msg_198_Cnt = 0;
uint32_t Msg_246_Cnt = 0;
uint32_t Msg_279_Cnt = 0;

uint16_t counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);		//interrupt for Hall_Right sensor
	HAL_TIM_Base_Start_IT(&htim2);								//interrupt for  1ms timer
	HAL_TIM_Base_Start_IT(&htim6);								//interrupt for 10ms timer
	HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
	
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = 0x0000;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	
	if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}
	
	if(HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		/* CAN Module 1 Start Error */
		Error_Handler();
	}
	
	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* CAN Module 1 Receive FIFO0 Interrupt Error */
		Error_Handler();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(CAN_1000Hz_Flag)
		{
			HAL_ADC_Start(&hadc2);
			HAL_ADC_PollForConversion(&hadc2, 1);
			APPS1 = HAL_ADC_GetValue(&hadc2);	
			HAL_ADC_PollForConversion(&hadc2, 1);
			APPS2 = HAL_ADC_GetValue(&hadc2);	
			HAL_ADC_Stop(&hadc2);
			
			canTX.StdId = 101;
			canTX.DLC = 6;
			
			CAN_Tx_Data[0] = (APPS1 >> 8) & 255U;
			CAN_Tx_Data[1] = APPS1 & 255U;
			CAN_Tx_Data[2] = (APPS2 >> 8) & 255U;
			CAN_Tx_Data[3] = APPS2 & 255U;
			CAN_Tx_Data[4] = (Hall_Right >> 8) & 255U;
			CAN_Tx_Data[5] = Hall_Right & 255U;
			
			/*HAL_ADC_Start(&hadc2);
			HAL_ADC_PollForConversion(&hadc2, 1);
			Brake = HAL_ADC_GetValue(&hadc2);	
			HAL_ADC_PollForConversion(&hadc2, 1);
			APPS1 = HAL_ADC_GetValue(&hadc2);	
			HAL_ADC_Stop(&hadc2);
			HAL_ADC_PollForConversion(&hadc2, 1);
			APPS2 = HAL_ADC_GetValue(&hadc2);	
			HAL_ADC_PollForConversion(&hadc2, 1);
			Steering = HAL_ADC_GetValue(&hadc2);
			HAL_ADC_PollForConversion(&hadc2, 1);
			SuspFL = HAL_ADC_GetValue(&hadc2);
			HAL_ADC_PollForConversion(&hadc2, 1);
			SuspFR = HAL_ADC_GetValue(&hadc2);
			
			
			canTX.StdId = 101;
			canTX.DLC = 8;
			
			CAN_Tx_Data[0] = (Brake >> 8) & 255U;
			CAN_Tx_Data[1] = Brake & 255U;
			CAN_Tx_Data[2] = (APPS1 >> 8) & 255U;
			CAN_Tx_Data[3] = APPS1 & 255U;
			CAN_Tx_Data[4] = (APPS2 >> 8) & 255U;
			CAN_Tx_Data[5] = APPS2 & 255U;
			CAN_Tx_Data[6] = (Steering >> 8) & 255U;
			CAN_Tx_Data[7] = Steering & 255U;*/
			
			while(HAL_CAN_IsTxMessagePending(&hcan1,CAN_Tx_Mailbox));
			CAN_Txing = 0;
			
			if((HAL_CAN_AddTxMessage(&hcan1, &canTX, CAN_Tx_Data, &CAN_Tx_Mailbox) == HAL_OK))
			{
				CAN_Txing = 1;
			}
			
			//while(HAL_CAN_IsTxMessagePending(&hcan1,CAN_Tx_Mailbox));
			// CAN_Txing = 0;
			
			/*canTX.StdId = 198;
			canTX.DLC = 8;
			
			CAN_Tx_Data[0] = (SuspFL >> 8) & 255U;
			CAN_Tx_Data[1] = SuspFL & 255U;
			CAN_Tx_Data[2] = (SuspFR >> 8) & 255U;
			CAN_Tx_Data[3] = SuspFR & 255U;
			CAN_Tx_Data[4] = (Hall_Right >> 8) & 255U;
			CAN_Tx_Data[5] = Hall_Right & 255U;
			CAN_Tx_Data[6] = (Hall_Left >> 8) & 255U;
			CAN_Tx_Data[7] = Hall_Left & 255U;
			
			if((HAL_CAN_AddTxMessage(&hcan1, &canTX, CAN_Tx_Data, &CAN_Tx_Mailbox) == HAL_OK))
			{
				CAN_Txing = 1;
			}
			
			while(HAL_CAN_IsTxMessagePending(&hcan1,CAN_Tx_Mailbox));
			Msg_198_Cnt++;*/
			CAN_Txing = 0;
			
			CAN_1000Hz_Flag = 0;
		}
		
		if(CAN_100Hz_Flag)
		{
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion (&hadc1, 1);
			Brake = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_PollForConversion (&hadc1, 1);
			DCDC_5V_APPS = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			
			canTX.StdId = 198;
			canTX.DLC = 4;
			
			CAN_Tx_Data[0] = (Brake >> 8) & 255U;
			CAN_Tx_Data[1] = Brake & 255U;
			CAN_Tx_Data[2] = (DCDC_5V_APPS >> 8) & 255U;
			CAN_Tx_Data[3] = DCDC_5V_APPS & 255U;
			
			/*HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion (&hadc1, 1);
			DCDC_12V = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_PollForConversion (&hadc1, 1);
			DCDC_5V = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_PollForConversion (&hadc1, 1);
			DCDC_5V_APPS = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_PollForConversion (&hadc1, 1);
			PCB_Thermistor = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_PollForConversion (&hadc1, 1);
			Core_Thermistor = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			
			canTX.StdId = 246;
			canTX.DLC = 8;
			
			CAN_Tx_Data[0] = (DCDC_12V >> 8) & 255U;
			CAN_Tx_Data[1] = DCDC_12V & 255U;
			CAN_Tx_Data[2] = (DCDC_5V >> 8) & 255U;
			CAN_Tx_Data[3] = DCDC_5V & 255U;
			CAN_Tx_Data[4] = (DCDC_5V_APPS >> 8) & 255U;
			CAN_Tx_Data[5] = DCDC_5V_APPS & 255U;
			CAN_Tx_Data[6] = (PCB_Thermistor >> 8) & 255U;
			CAN_Tx_Data[7] = PCB_Thermistor & 255U;*/
			
			while(HAL_CAN_IsTxMessagePending(&hcan1,CAN_Tx_Mailbox));
			CAN_Txing = 0;
			
			if((HAL_CAN_AddTxMessage(&hcan1, &canTX, CAN_Tx_Data, &CAN_Tx_Mailbox) == HAL_OK))
			{
				CAN_Txing = 1;
			}
			
			//while(HAL_CAN_IsTxMessagePending(&hcan1,CAN_Tx_Mailbox));
			//CAN_Txing = 0;
			
			Enable_Toggle = !HAL_GPIO_ReadPin(Enable_Toggle_GPIO_Port,Enable_Toggle_Pin);
			Second_Toggle = !HAL_GPIO_ReadPin(Second_Toggle_GPIO_Port,Second_Toggle_Pin);
			Third_Toggle = !HAL_GPIO_ReadPin(Third_Toggle_GPIO_Port, Third_Toggle_Pin);
			Start = !HAL_GPIO_ReadPin(Start_GPIO_Port,Start_Pin);
			Ad_Act = HAL_GPIO_ReadPin(Ad_Act_GPIO_Port,Ad_Act_Pin);
			Green_TSAL = HAL_GPIO_ReadPin(Green_TSAL_GPIO_Port,Green_TSAL_Pin);
			SC_State = HAL_GPIO_ReadPin(SC_State_GPIO_Port,SC_State_Pin);
			IMD_State = HAL_GPIO_ReadPin(IMD_State_Input_GPIO_Port,IMD_State_Input_Pin);
			BMS_State = HAL_GPIO_ReadPin(AMS_State_Input_GPIO_Port,AMS_State_Input_Pin);
			
			canTX.StdId = 279;
			canTX.DLC = 7;
			
		/*	CAN_Tx_Data[0] = (Core_Thermistor >> 8) & 255U;
			CAN_Tx_Data[1] = Core_Thermistor & 255U;
			CAN_Tx_Data[2] = (13 * Enable_Toggle) & 255U;
			CAN_Tx_Data[2] = ((13 * Second_Toggle) << 4) & 255U;
			CAN_Tx_Data[3] = (13 * Third_Toggle) & 255U;
			CAN_Tx_Data[3] = ((13 * Start) << 4) & 255U;
			CAN_Tx_Data[4] = (13 * Ad_Act) & 255U;
			CAN_Tx_Data[4] = ((13 * Green_TSAL) << 4) & 255U;
			CAN_Tx_Data[5] = (13 * SC_State) & 255U;
			CAN_Tx_Data[5] = ((13 * IMD_State) << 4) & 255U;
			CAN_Tx_Data[6] = (13 * BMS_State) & 255U; */
			
			CAN_Tx_Data[0] = Enable_Toggle;
			CAN_Tx_Data[1] = Second_Toggle;
			CAN_Tx_Data[2] = Third_Toggle;
			CAN_Tx_Data[3] = Start;
			CAN_Tx_Data[4] = Ad_Act;
			CAN_Tx_Data[5] = Green_TSAL;
			CAN_Tx_Data[6] = SC_State;
			
			while(HAL_CAN_IsTxMessagePending(&hcan1,CAN_Tx_Mailbox));
			CAN_Txing = 0;
		  
			if((HAL_CAN_AddTxMessage(&hcan1, &canTX, CAN_Tx_Data, &CAN_Tx_Mailbox) == HAL_OK))
			{
				CAN_Txing = 1;
			}
			
			//while(HAL_CAN_IsTxMessagePending(&hcan1,CAN_Tx_Mailbox));
			//CAN_Txing = 0;
			
			CAN_100Hz_Flag = 0;
		}
		
		if(CAN_Rx_Flag)
		{
			CAN_Msg_Id = canRX.StdId;
			
			if(CAN_Msg_Id == 204)
			{
				Fans_PWM = CAN_Rx_Data[0];
				Safe_State = CAN_Rx_Data[1] & 15U;
				Buzzer = (CAN_Rx_Data[1] >> 4) & 15U;
				Sensor_Error = CAN_Rx_Data[2] & 15U;
			  ENABLE_SIGNAL = (CAN_Rx_Data[2] >> 4) & 15U;
				SC_Software = CAN_Rx_Data[3] & 255U;
				
				if(Fans_PWM > 100)
				{
					htim11.Instance->CCR1 = 100;
				}
				else
				{
					htim11.Instance->CCR1 = Fans_PWM;
				}
				if(Safe_State == 13)
				{
					HAL_GPIO_WritePin(Safe_State_GPIO_Port, Safe_State_Pin, GPIO_PIN_SET);
				}
				else
				{
					HAL_GPIO_WritePin(Safe_State_GPIO_Port, Safe_State_Pin, GPIO_PIN_RESET);
				}
				if(Buzzer == 13)
				{
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				}
				else
				{
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				}
				
				if(Sensor_Error == 13)
				{
					HAL_GPIO_WritePin(Sensor_Error_GPIO_Port,Sensor_Error_Pin,GPIO_PIN_SET);
				}
				else
				{
					HAL_GPIO_WritePin(Sensor_Error_GPIO_Port,Sensor_Error_Pin,GPIO_PIN_RESET);
				}
				
				if(ENABLE_SIGNAL == 13)
				{
					HAL_GPIO_WritePin(ENABLE_GPIO_Port,ENABLE_Pin,GPIO_PIN_SET);
				}
				else
				{
					HAL_GPIO_WritePin(ENABLE_GPIO_Port,ENABLE_Pin,GPIO_PIN_RESET);
				}
				
				if(SC_Software == 13)
				{
				HAL_GPIO_WritePin(SC_Software_GPIO_Port,SC_Software_Pin,GPIO_PIN_SET);
				}
				else
				{
				HAL_GPIO_WritePin(SC_Software_GPIO_Port,SC_Software_Pin,GPIO_PIN_RESET);
				}
			}
			
			CAN_Rx_Flag = 0;
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 90;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 90;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 180;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 100;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Buzzer_Pin|Sensor_Error_Pin|Safe_State_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SC_Software_GPIO_Port, SC_Software_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Buzzer_Pin Sensor_Error_Pin Safe_State_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|Sensor_Error_Pin|Safe_State_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Spare_Analog2_Pin Spare_Analog1_Pin */
  GPIO_InitStruct.Pin = Spare_Analog2_Pin|Spare_Analog1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IMD_State_Input_Pin Enable_Toggle_Pin Second_Toggle_Pin Third_Toggle_Pin */
  GPIO_InitStruct.Pin = IMD_State_Input_Pin|Enable_Toggle_Pin|Second_Toggle_Pin|Third_Toggle_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SC_State_Pin Green_TSAL_Pin Ad_Act_Pin */
  GPIO_InitStruct.Pin = SC_State_Pin|Green_TSAL_Pin|Ad_Act_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AMS_State_Input_Pin Start_Pin */
  GPIO_InitStruct.Pin = AMS_State_Input_Pin|Start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SC_Software_Pin */
  GPIO_InitStruct.Pin = SC_Software_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SC_Software_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM3)
	{
		Hall_Right = __HAL_TIM_GetCompare(&htim3,TIM_CHANNEL_3);
		__HAL_TIM_SetCounter(&htim3,0);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
	{
		CAN_1000Hz_Flag = 1;
		CAN_1000Hz_Cnt++;
		
		//tim2_cnt++;
	}
	else if(htim->Instance == TIM6)
	{
		CAN_100Hz_Flag = 1;
		CAN_100Hz_Cnt++;
		
		//tim6_cnt++;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
	{
		CAN_Rx_Flag = 1;
		HAL_CAN_GetRxMessage(&hcan1, CAN_FILTER_FIFO0, &canRX, CAN_Rx_Data);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
