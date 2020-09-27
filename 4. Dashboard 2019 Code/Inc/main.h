/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Buzzer_Pin GPIO_PIN_13
#define Buzzer_GPIO_Port GPIOC
#define Spare_Analog2_Pin GPIO_PIN_3
#define Spare_Analog2_GPIO_Port GPIOC
#define Brake_Pin GPIO_PIN_0
#define Brake_GPIO_Port GPIOA
#define APPS1_Pin GPIO_PIN_1
#define APPS1_GPIO_Port GPIOA
#define APPS2_Pin GPIO_PIN_2
#define APPS2_GPIO_Port GPIOA
#define Steering_Pin GPIO_PIN_3
#define Steering_GPIO_Port GPIOA
#define DCDC_12V_Pin GPIO_PIN_4
#define DCDC_12V_GPIO_Port GPIOA
#define DCDC_5V_Pin GPIO_PIN_5
#define DCDC_5V_GPIO_Port GPIOA
#define DCDC_5V_APPS_Pin GPIO_PIN_6
#define DCDC_5V_APPS_GPIO_Port GPIOA
#define PCB_Thermistor_Pin GPIO_PIN_7
#define PCB_Thermistor_GPIO_Port GPIOA
#define SuspFL_Pin GPIO_PIN_4
#define SuspFL_GPIO_Port GPIOC
#define Spare_Analog1_Pin GPIO_PIN_5
#define Spare_Analog1_GPIO_Port GPIOC
#define SuspFR_Pin GPIO_PIN_1
#define SuspFR_GPIO_Port GPIOB
#define IMD_State_Input_Pin GPIO_PIN_2
#define IMD_State_Input_GPIO_Port GPIOB
#define ENABLE_Pin GPIO_PIN_15
#define ENABLE_GPIO_Port GPIOB
#define Sensor_Error_Pin GPIO_PIN_6
#define Sensor_Error_GPIO_Port GPIOC
#define Safe_State_Pin GPIO_PIN_7
#define Safe_State_GPIO_Port GPIOC
#define HALL_Right_Pin GPIO_PIN_8
#define HALL_Right_GPIO_Port GPIOC
#define SC_State_Pin GPIO_PIN_9
#define SC_State_GPIO_Port GPIOC
#define AMS_State_Input_Pin GPIO_PIN_8
#define AMS_State_Input_GPIO_Port GPIOA
#define Start_Pin GPIO_PIN_15
#define Start_GPIO_Port GPIOA
#define Green_TSAL_Pin GPIO_PIN_11
#define Green_TSAL_GPIO_Port GPIOC
#define Ad_Act_Pin GPIO_PIN_12
#define Ad_Act_GPIO_Port GPIOC
#define SC_Software_Pin GPIO_PIN_2
#define SC_Software_GPIO_Port GPIOD
#define Enable_Toggle_Pin GPIO_PIN_4
#define Enable_Toggle_GPIO_Port GPIOB
#define Second_Toggle_Pin GPIO_PIN_5
#define Second_Toggle_GPIO_Port GPIOB
#define Third_Toggle_Pin GPIO_PIN_6
#define Third_Toggle_GPIO_Port GPIOB
#define Fans_PWM_Pin GPIO_PIN_9
#define Fans_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
