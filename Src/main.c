/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "usart.h"
#include <string.h>
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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
/* 送信データ用バッファ */
char tx_buffer[128];
/* 受信データ用バッファ */
char rx_buffer[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SWITCH_GPIO_PORT GPIOC
#define SWITCH_GPIO_PIN GPIO_PIN_13
#define DEBOUNCE_DELAY_MS 10      // チャタリング対策の遅延時間（ミリ秒）
#define DEBOUNCE_STABLE_COUNT 5   // 安定状態とみなすために必要な連続カウント

typedef enum {
    ACTIVE_HIGH = 0,  // 押したら HIGH（外部プルダウン等）
    ACTIVE_LOW  = 1   // 押したら LOW  （内部プルアップ）
} button_polarity_t;

static inline int normalize_pin(GPIO_PinState s, button_polarity_t pol) {
    if (pol == ACTIVE_LOW) {
        return (s == GPIO_PIN_RESET) ? 1 : 0;  // プルアップ: LOWで押下
    } else {
        return (s == GPIO_PIN_SET)   ? 1 : 0;  // プルダウン: HIGHで押下
    }
}

int get_sw_state(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, button_polarity_t pol) {
    int stable_count = 0;
    int current_state, last_state;

    last_state = normalize_pin(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin), pol);

    while (stable_count < DEBOUNCE_STABLE_COUNT) {
        current_state = normalize_pin(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin), pol);
        if (current_state == last_state) {
            stable_count++;
        } else {
            stable_count = 0;
        }
        HAL_Delay(DEBOUNCE_DELAY_MS);
        last_state = current_state;
    }
    return last_state;  // 押した=1, 離した=0
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

    int camera_id = 0;
    int record_flag = 0;
    int team_collor = 0;

    int blue_sw_prev   = get_sw_state(BLUE_SW_GPIO_Port,   BLUE_SW_Pin,   ACTIVE_LOW);
    int yellow_sw_prev = get_sw_state(YELLOW_SW_GPIO_Port, YELLOW_SW_Pin, ACTIVE_LOW);
    int red_sw_prev    = get_sw_state(RED_SW_GPIO_Port,    RED_SW_Pin,    ACTIVE_LOW);

    while (1) {
        int sw_blue   = get_sw_state(BLUE_SW_GPIO_Port,   BLUE_SW_Pin,   ACTIVE_LOW);
        int sw_yellow = get_sw_state(YELLOW_SW_GPIO_Port, YELLOW_SW_Pin, ACTIVE_LOW);
        int sw_red    = get_sw_state(RED_SW_GPIO_Port,    RED_SW_Pin,    ACTIVE_LOW);

        // 押した瞬間=0→1 の立ち上がりを検出
        if (sw_blue && !blue_sw_prev)
            camera_id = (camera_id + 1) % 3;
        if (sw_yellow && !yellow_sw_prev)
            record_flag = 1 - record_flag;
        if (sw_red && !red_sw_prev)
            team_collor = 1 - team_collor;

        HAL_Delay(10);

        blue_sw_prev   = sw_blue;
        yellow_sw_prev = sw_yellow;
        red_sw_prev    = sw_red;

        // 送信データの作成（robot_stateとして固定値＋スイッチで制御する値）
        int state_id = 2;
        int pitch_deg = 120;        // 1/10度単位（例：12.0度なら120）
        int muzzle_velocity = 154;  // mm/s（例：15.4 m/sなら154）
        int left_disks_num = 53;
        int right_disks_num = 25;
        int video_id = camera_id;

        uint8_t flags = 0;
        if (record_flag) {
            flags |= (1 << 1);  // 2ビット目を1にする
        } else {
            flags &= ~(1 << 1); // 2ビット目を0にする
        }
        
        if (team_collor) {
            flags |= (1 << 3);  // 3ビット目を1にする
        } else {
            flags &= ~(1 << 3); // 3ビット目を0にする
        }
        int reserved = 0;

        // robot_stateの文字列を生成（元の形式）
        snprintf(tx_buffer, sizeof(tx_buffer), "%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                 state_id, pitch_deg, muzzle_velocity, left_disks_num,
                 right_disks_num, video_id, flags, reserved);

        // データ送信
        HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 1000);


        /* デバッグ用（UARTが1ポートしかないので苦肉の策）
        // ここでPC側から新たに送られてくる "%d,%d,%d\n" 形式の文字列を受信する
        memset(rx_buffer, 0, sizeof(rx_buffer));
        // 10msタイムアウトで受信（受信文字数は最大 rx_buffer サイズまで
        if (HAL_UART_Receive(&huart2, (uint8_t *)rx_buffer, sizeof(rx_buffer) - 1, 10) > 0) {
            // 受信した文字列があれば、改行コードまで受信していると仮定し、確認のためエコー送信
            HAL_UART_Transmit(&huart2, (uint8_t *)rx_buffer, strlen(rx_buffer), 1000);
            // 必要に応じて、受信内容の解析やデバッグ用LEDの点灯などを実装
        }
        */

        blue_sw_prev = sw_blue;
        yellow_sw_prev = sw_yellow;
        red_sw_prev = sw_red;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
