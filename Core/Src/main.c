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

	////////////////////////7

//	IDEAS

//	uint8_t lines in matrix shape would allow image combo masking using bitmasks (and reduce storage demand)
//  3-stage pixel output stage with separate masking for R-G-B channels
//  add a few sweeping animations

	////////////////////////

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "pixel_shape_masks.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIM_CLK 100000000 // System clock at 100MHz
//#define DESIRED_FREQUENCY 625000 // Desired PWM frequency

#define DUTY_TX_ZERO 23
#define DUTY_TX_ONE 55

#define BITS_PER_COLOR_CH 24
#define MATRIX_X 8
#define MATRIX_Y 8

// master brightness control & gate edge bits to 0 to prevent glitches
#define OUTPUT_BITMASK ((uint8_t) 0b01111110)

//#define SET_PSRAND_INIT 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//static TIM_OC_InitTypeDef sConfigOC_file_global = {0};

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} color ;

const static color no_color = {.r=0x00, .g=0x00, .b=0x00};
const static color pink     = {.r=0x0f, .g=0x03, .b=0x08};
const static color red      = {.r=0x0f, .g=0x00, .b=0x00};
const static color green    = {.r=0x00, .g=0x0f, .b=0x00};
const static color blue     = {.r=0x00, .g=0x00, .b=0x0f};
const static color white    = {.r=0x08, .g=0x08, .b=0x08};

static color matrix_values[MATRIX_X][MATRIX_Y];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

TIM_HandleTypeDef htim9;

void PWM_Configuration() {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 160;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
	Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
	Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
	Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0; // Initialize pulse to 0% duty cycle
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;


  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim9);

  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
}

void SetDutyCycle(uint8_t duty_cycle) {
  // Ensure the duty cycle is within bounds (0-100%)
  if (duty_cycle > 100) {
    duty_cycle = 100;
  }

  // Calculate and set the new pulse value based on the duty cycle
  htim9.Instance->CCR1 = ((htim9.Init.Period + 1) * duty_cycle) / 100;
}

// LEDs are connected FIFO style:
// 1st row: L->R --
// 	   			  |
// 2nd row: L<-R --
// etc.
// prepare sw buffer
void write_matrix(matrix_shape mtx, color target_color){
	for(uint32_t y=0; y<MATRIX_X; y++) {
		for(uint32_t x=0; x<MATRIX_Y; x++) {
			color color = {.r=0b00000000, .g=0b00000000, .b=0b00000000};
			if(mtx[x][y]){
				color.r = target_color.r;
				color.b = target_color.g;
				color.g = target_color.b;
			}

			if((y % 2)){
				matrix_values[y][7-x] = color;
			} else {
				matrix_values[y][x] = color;
			}
		}
	}
}

void shield_reset() {
	SetDutyCycle(00);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_Delay(11);
}

void use_matrix() {
  // rst
  shield_reset();
  //HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
  __HAL_TIM_CLEAR_FLAG(&htim9, TIM_FLAG_UPDATE);

  // drive new values: G R B

  // variables to store stuff according to state in upcoming for-loops
  color color;
  uint8_t r_g_b;

  for(uint8_t pixel_idx_x=0; pixel_idx_x<MATRIX_X; pixel_idx_x++) {
	  for(uint8_t pixel_idx_y=0; pixel_idx_y<MATRIX_Y; pixel_idx_y++) {

		  color = matrix_values[pixel_idx_x][pixel_idx_y];
		  for(uint8_t byte_idx=0; byte_idx<3; byte_idx++) {
			  switch(byte_idx){
				  case 0:
					  r_g_b = color.g  & OUTPUT_BITMASK;
					  break;
				  case 1:
					  r_g_b = color.r  & OUTPUT_BITMASK;
					  break;
				  case 2:
					  r_g_b = color.b  & OUTPUT_BITMASK;
					  break;
			  }
			  // adjust PWM as the bits in matrix dictate
			  for(uint8_t bit_idx=0; bit_idx<8; bit_idx++) {

				  if((r_g_b & (1 << (7-bit_idx) ) )){
					  SetDutyCycle(DUTY_TX_ONE);
				  } else {
					  SetDutyCycle(DUTY_TX_ZERO);
				  }
				  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
				  // poll for done bit.
				  while(!__HAL_TIM_GET_FLAG(&htim9, TIM_FLAG_UPDATE));
				  // Clear the update event flag
				  __HAL_TIM_CLEAR_FLAG(&htim9, TIM_FLAG_UPDATE);
			  }
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

	write_matrix(matrix_clear, no_color);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interfaceåp and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */



  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
//  MX_TIM9_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  PWM_Configuration();

  shield_reset();
  shield_reset();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_GPIO_TogglePin(LED_TRIAL2_GPIO_Port, LED_TRIAL2_Pin);

		HAL_Delay(500);

		write_matrix(matrix_square, green);
		use_matrix();
		HAL_Delay(2000);

		write_matrix(matrix_checkers, pink);
		use_matrix();
		HAL_Delay(2000);

		write_matrix(matrix_square, blue);
		use_matrix();
		HAL_Delay(2000);

		write_matrix(matrix_left_side, pink);
		use_matrix();
		HAL_Delay(2000);

		write_matrix(matrix_right_side, red);
		use_matrix();
		HAL_Delay(2000);

		write_matrix(matrix_alt, blue);
		use_matrix();
		HAL_Delay(250);

		write_matrix(matrix_alt_2, green);
		use_matrix();
		HAL_Delay(250);

		write_matrix(matrix_alt, red);
		use_matrix();
		HAL_Delay(250);

		write_matrix(matrix_alt_2, pink);
		use_matrix();
		HAL_Delay(250);

		write_matrix(matrix_alt, white);
		use_matrix();
		HAL_Delay(250);

		write_matrix(matrix_one, white);
		use_matrix();
		HAL_Delay(1500);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 125-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */
//  HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1);


  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_TRIAL2_GPIO_Port, LED_TRIAL2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLD_WS_DRIVER_GPIO_Port, OLD_WS_DRIVER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_TRIAL2_Pin */
  GPIO_InitStruct.Pin = LED_TRIAL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_TRIAL2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLD_WS_DRIVER_Pin */
  GPIO_InitStruct.Pin = OLD_WS_DRIVER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(OLD_WS_DRIVER_GPIO_Port, &GPIO_InitStruct);

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
