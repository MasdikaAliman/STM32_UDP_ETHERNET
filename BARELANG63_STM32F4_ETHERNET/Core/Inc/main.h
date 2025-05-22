/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LS_BAWAH_Pin GPIO_PIN_4
#define LS_BAWAH_GPIO_Port GPIOE
#define ENC2A_Pin GPIO_PIN_5
#define ENC2A_GPIO_Port GPIOE
#define LS_ATAS_Pin GPIO_PIN_6
#define LS_ATAS_GPIO_Port GPIOE
#define Proxy_Pin_Pin GPIO_PIN_13
#define Proxy_Pin_GPIO_Port GPIOC
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define ENC1B_Pin GPIO_PIN_1
#define ENC1B_GPIO_Port GPIOC
#define TPS_ADC2_Pin GPIO_PIN_2
#define TPS_ADC2_GPIO_Port GPIOC
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define PWM6_Pin GPIO_PIN_1
#define PWM6_GPIO_Port GPIOA
#define PWM7_Pin GPIO_PIN_2
#define PWM7_GPIO_Port GPIOA
#define TPS_ADC1_Pin GPIO_PIN_3
#define TPS_ADC1_GPIO_Port GPIOA
#define CCW3_Pin GPIO_PIN_5
#define CCW3_GPIO_Port GPIOA
#define CW3_Pin GPIO_PIN_6
#define CW3_GPIO_Port GPIOA
#define CCW4_Pin GPIO_PIN_7
#define CCW4_GPIO_Port GPIOA
#define W5500_CS_Pin GPIO_PIN_4
#define W5500_CS_GPIO_Port GPIOC
#define CW4_Pin GPIO_PIN_5
#define CW4_GPIO_Port GPIOC
#define PWM_Kick_Pin GPIO_PIN_0
#define PWM_Kick_GPIO_Port GPIOB
#define ENC6A_Pin GPIO_PIN_1
#define ENC6A_GPIO_Port GPIOB
#define ENC2B_Pin GPIO_PIN_7
#define ENC2B_GPIO_Port GPIOE
#define Discharge_Pin GPIO_PIN_8
#define Discharge_GPIO_Port GPIOE
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOE
#define Charge_Pin GPIO_PIN_10
#define Charge_GPIO_Port GPIOE
#define PWM2_Pin GPIO_PIN_11
#define PWM2_GPIO_Port GPIOE
#define ENC6B_Pin GPIO_PIN_12
#define ENC6B_GPIO_Port GPIOE
#define PWM3_Pin GPIO_PIN_13
#define PWM3_GPIO_Port GPIOE
#define PWM4_Pin GPIO_PIN_14
#define PWM4_GPIO_Port GPIOE
#define Full_Pin GPIO_PIN_15
#define Full_GPIO_Port GPIOE
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define PWM8_Pin GPIO_PIN_11
#define PWM8_GPIO_Port GPIOB
#define CCW1_Pin GPIO_PIN_13
#define CCW1_GPIO_Port GPIOB
#define CW1_Pin GPIO_PIN_14
#define CW1_GPIO_Port GPIOB
#define W5500_RST_Pin GPIO_PIN_8
#define W5500_RST_GPIO_Port GPIOD
#define CCW2_Pin GPIO_PIN_9
#define CCW2_GPIO_Port GPIOD
#define CW2_Pin GPIO_PIN_10
#define CW2_GPIO_Port GPIOD
#define CW6_Pin GPIO_PIN_11
#define CW6_GPIO_Port GPIOD
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define CCW6_Pin GPIO_PIN_6
#define CCW6_GPIO_Port GPIOC
#define ENC3B_Pin GPIO_PIN_7
#define ENC3B_GPIO_Port GPIOC
#define ENC7B_Pin GPIO_PIN_8
#define ENC7B_GPIO_Port GPIOC
#define ENC8A_Pin GPIO_PIN_8
#define ENC8A_GPIO_Port GPIOA
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define PWM5_Pin GPIO_PIN_15
#define PWM5_GPIO_Port GPIOA
#define I2S3_SCK_Pin GPIO_PIN_10
#define I2S3_SCK_GPIO_Port GPIOC
#define ENC4B_Pin GPIO_PIN_12
#define ENC4B_GPIO_Port GPIOC
#define ENC5A_Pin GPIO_PIN_0
#define ENC5A_GPIO_Port GPIOD
#define CCW_7_Pin GPIO_PIN_1
#define CCW_7_GPIO_Port GPIOD
#define ENC7A_Pin GPIO_PIN_2
#define ENC7A_GPIO_Port GPIOD
#define ENC4A_Pin GPIO_PIN_3
#define ENC4A_GPIO_Port GPIOD
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define ENC5B_Pin GPIO_PIN_6
#define ENC5B_GPIO_Port GPIOD
#define CW7_Pin GPIO_PIN_7
#define CW7_GPIO_Port GPIOD
#define W5500_SCK_Pin GPIO_PIN_3
#define W5500_SCK_GPIO_Port GPIOB
#define W5500_MISO_Pin GPIO_PIN_4
#define W5500_MISO_GPIO_Port GPIOB
#define W5500_MOSI_Pin GPIO_PIN_5
#define W5500_MOSI_GPIO_Port GPIOB
#define TX1_Pin GPIO_PIN_6
#define TX1_GPIO_Port GPIOB
#define RX1_Pin GPIO_PIN_7
#define RX1_GPIO_Port GPIOB
#define CCW5_Pin GPIO_PIN_0
#define CCW5_GPIO_Port GPIOE
#define CW5_Pin GPIO_PIN_1
#define CW5_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
