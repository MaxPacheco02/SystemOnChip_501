/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @author         : rahu7p
  ******************************************************************************
  * @board			: nucleo-f103rb
  * @mcu			: stm32f103rb
  *
  *
  *
  * This code programs a blinking LED (LD2): on-and-off every second.
  *
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lcd.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void USER_RCC_Init(void);
void USER_GPIO_Init(void);
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
  /* USER CODE BEGIN 2 */
  USER_RCC_Init();
  USER_GPIO_Init();
  LCD_Init( );//				inicializamos la libreria del LCD
  LCD_Cursor_ON( );//			cursor visible activo
  LCD_Clear( );//			borra la pantalla
  LCD_Set_Cursor( 1, 0 );//		posiciona cursor en la fila 1 columna 0
  //LCD_Put_Str( "ABCDEFGHIJKLMNOd" );//	escribe un string
  //LCD_Set_Cursor(2,0);
  //LCD_Put_Str( "ABC" );//	escribe un string

  uint16_t count = 0, row = 1, tecla, found, changed = 0;

  char msg[31], msg1[16], msg2[16];
  char key[] = "0123456789ABCD ";

  for(int i = 0; i < 31 ; i++){
	  msg[i] = 0x0;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//A
/*
	  GPIOA->BSRR = GPIO_BSRR_BS5;//	LD2 ON
	  HAL_Delay(1000);//			1 second delay
	  GPIOA->BSRR = GPIO_BSRR_BR5;//	LD2 OFF
	  HAL_Delay(1000);//			1 second delay
*/

//B
/*
	  //Bouncing
	  if(!(GPIOA->IDR & GPIO_IDR_IDR10)){
		  GPIOA->ODR ^= GPIO_ODR_ODR5;
	  }
*/
/*
	  //Debounced, pull-up interno
	  if(!(GPIOA->IDR & GPIO_IDR_IDR10)){
		  GPIOA->ODR ^= GPIO_ODR_ODR5;
		  while(!(GPIOA->IDR & GPIO_IDR_IDR10)){}
		  HAL_Delay(10);
	  }
*/

//C
	  GPIOB->BSRR = (1 << 16) | (1 << 1) | (1 << 2) | (1 << 3);
	  found = 0;
	  if(!(GPIOB->IDR & GPIO_IDR_IDR4 ) & !found){
		 tecla = 0x1;
		 found = 1;
	  }
	  if(!(GPIOB->IDR & GPIO_IDR_IDR5 ) & !found){
		  tecla = 0x2;
		  found = 1;
	  }
	  if(!(GPIOB->IDR & GPIO_IDR_IDR6 ) & !found){
		  tecla = 0x3;
		  found = 1;
	  }
	  if(!(GPIOB->IDR & GPIO_IDR_IDR7 ) & !found){
		  tecla = 0xA;
		  found = 1;
	  }

	  GPIOB->BSRR = (1 << 0) | (1 << 17) | (1 << 2) | (1 << 3);
	  if(!(GPIOB->IDR & GPIO_IDR_IDR4 ) & !found){
		  tecla = 0x4;
		  found = 1;
	  }
	  if(!(GPIOB->IDR & GPIO_IDR_IDR5 ) & !found){
		  tecla = 0x5;
		  found = 1;
	  }
	  if(!(GPIOB->IDR & GPIO_IDR_IDR6 ) & !found){
		  tecla = 0x6;
		  found = 1;
	  }
	  if(!(GPIOB->IDR & GPIO_IDR_IDR7 ) & !found){
		  tecla = 0xB;
		  found = 1;
	  }

	  GPIOB->BSRR = (1 << 0) | (1 << 1) | (1 << 18) | (1 << 3);
	  if(!(GPIOB->IDR & GPIO_IDR_IDR4 ) & !found){
		  tecla = 0x7;
		  found = 1;
	  }
	  if(!(GPIOB->IDR & GPIO_IDR_IDR5 ) & !found){
		  tecla = 0x8;
		  found = 1;
	  }
	  if(!(GPIOB->IDR & GPIO_IDR_IDR6 ) & !found){
		  tecla = 0x9;
		  found = 1;
	  }
	  if(!(GPIOB->IDR & GPIO_IDR_IDR7 ) & !found){
		  tecla = 0xC;
		  found = 1;
	  }

	  GPIOB->BSRR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 19);
	  if(!(GPIOB->IDR & GPIO_IDR_IDR4 ) & !found){
		  tecla = 0xE;
		  found = 1;
	  }
	  if(!(GPIOB->IDR & GPIO_IDR_IDR5 ) & !found){
		  tecla = 0x0;
		  found = 1;
	  }
	  if(!(GPIOB->IDR & GPIO_IDR_IDR6 ) & !found){
		  tecla = 0xF;
		  found = 1;
	  }
	  if(!(GPIOB->IDR & GPIO_IDR_IDR7 ) & !found){
		  tecla = 0xD;
		  found = 1;
	  }

	  if(!found)
		  tecla = 0x10;

	  if(found){
		  changed = 1;
		  if(tecla == 0xE){
			  if(count > 0){
				  msg[count-1] = 0x0;
				  count--;

				  if(count>=16)
					  msg2[count%16] = 0x0;
				  else
					  msg1[count] = 0x0;
			  }
		  } else if(count <= 31){
			  if(tecla == 0xF)
				  msg[count] = key[0xE];
			  else
				  msg[count] = key[tecla];
			  count++;
		  }
	  }

	  for(int i = 0 ; i < 16 ; i++){
		  msg1[i] = msg[i];
		  msg2[i] = msg[i+16];
	  }

	  if(changed == 1){
		  changed = 0;
		  LCD_Clear();

		  LCD_Set_Cursor(1,0);
		  LCD_Put_Str(msg1);
		  LCD_Set_Cursor(2,0);
		  LCD_Put_Str(msg2);
		  LCD_Set_Cursor(count/16 + 1, (count+1) % 17);
	  }

	  HAL_Delay(50);


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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void USER_RCC_Init(void){
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;//		I/O port A clock enable
}

void USER_GPIO_Init(void){
//A, B
	/*
	//LED
	GPIOA->BSRR = GPIO_BSRR_BR5;//			PA5 -> 0, LD2 OFF
	//pin A5 as output push-pull max speed 10MHz
	GPIOA->CRL &= ~GPIO_CRL_CNF5 & ~GPIO_CRL_MODE5_1;
	GPIOA->CRL |= GPIO_CRL_MODE5_0;
	*/

	/*
	//Bouncing and debounced
	//pin PA10 as input floating
	GPIOA->CRH &= ~GPIO_CRH_MODE10 & ~GPIO_CRH_CNF10_1;
	GPIOA->CRH |= GPIO_CRH_CNF10_0;
	*/

	/*
	//Pull-up interno
	//pin PA10 as input pull-up
	GPIOA->CRH &= ~GPIO_CRH_MODE10 & ~GPIO_CRH_CNF10_0;
	GPIOA->CRH |= GPIO_CRH_CNF10_1;
	GPIOA->ODR |= GPIO_ODR_ODR10;
	*/

//C
	GPIOB->BSRR &= GPIO_BSRR_BR0 & GPIO_BSRR_BR1 & GPIO_BSRR_BR2 & GPIO_BSRR_BR3;
	//pins PB0-PB3 as output push-pull
	GPIOB->CRL &= ~GPIO_CRL_CNF0 & ~GPIO_CRL_MODE0_1;
	GPIOB->CRL |= GPIO_CRL_MODE0_0;
	GPIOB->CRL &= ~GPIO_CRL_CNF1 & ~GPIO_CRL_MODE1_1;
	GPIOB->CRL |= GPIO_CRL_MODE1_0;
	GPIOB->CRL &= ~GPIO_CRL_CNF2 & ~GPIO_CRL_MODE2_1;
	GPIOB->CRL |= GPIO_CRL_MODE2_0;
	GPIOB->CRL &= ~GPIO_CRL_CNF3 & ~GPIO_CRL_MODE3_1;
	GPIOB->CRL |= GPIO_CRL_MODE3_0;

	//pins PB4-PB7 as input pull-up
	GPIOB->CRL &= ~GPIO_CRL_CNF4_0 & ~GPIO_CRL_MODE4;
	GPIOB->CRL |= GPIO_CRL_CNF4_1;
	GPIOB->ODR |= GPIO_ODR_ODR4;
	GPIOB->CRL &= ~GPIO_CRL_CNF5_0 & ~GPIO_CRL_MODE5;
	GPIOB->CRL |= GPIO_CRL_CNF5_1;
	GPIOB->ODR |= GPIO_ODR_ODR5;
	GPIOB->CRL &= ~GPIO_CRL_CNF6_0 & ~GPIO_CRL_MODE6;
	GPIOB->CRL |= GPIO_CRL_CNF6_1;
	GPIOB->ODR |= GPIO_ODR_ODR6;
	GPIOB->CRL &= ~GPIO_CRL_CNF7_0 & ~GPIO_CRL_MODE7;
	GPIOB->CRL |= GPIO_CRL_CNF7_1;
	GPIOB->ODR |= GPIO_ODR_ODR7;

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
