/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @author         : rahu7p
  ******************************************************************************
  * @board	: nucleo-f103rb
  * @mcu	: stm32f103rb
  *
  *
  *
  * This code initialize the ADC1 in continuous conversion mode and stores the
  * ADC-result in a variable.
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "myprintf.h"

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
void USER_ADC_Init(void);
void USER_ADC_Calibration(void);

void USER_USART2_Init(void);
void USER_USART2_Transmit(uint8_t *pData, uint16_t size);
uint32_t USER_USART2_Receive(void);

uint16_t USER_ADC_Read( void );
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
  uint16_t dataADC;
  float v;
  uint8_t a,b;
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
  USER_ADC_Init();
  USER_ADC_Calibration();
  USER_USART2_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ADC1->CR2	|=	 ADC_CR2_ADON;//	starts the conversion
  while (1)
  {
      dataADC = USER_ADC_Read( );
      //v = (uint8_t)(dataADC*3.3/4096);
      v = (float)(dataADC*3.3/4096);
      a = v;
      b = (int8_t)(v* 1000)%1000;
      printf("%d.%3d\r\n",a,b);
	  HAL_Delay(1000);

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
	RCC->APB2ENR	|=	 RCC_APB2ENR_IOPAEN//	I/O port A clock enable
		      	|	 RCC_APB2ENR_ADC1EN;//	ADC 1 clock enable
	RCC->CFGR	|=	 RCC_CFGR_ADCPRE;//	ADC prescaler 1:8 for 8 MHz

	RCC->APB1ENR	|=	 RCC_APB1ENR_USART2EN;//  	USART2 clock enable
}

void USER_GPIO_Init(void){
	//PA0 (ADC12_IN0) as analog
	GPIOA->CRL	&=	~GPIO_CRL_CNF0 & ~GPIO_CRL_MODE0;

	//pin PA2 (USART2_TX) as alternate function output push-pull, max speed 10MHz
	GPIOA->CRL	&=	~GPIO_CRL_CNF2_0 & ~GPIO_CRL_MODE2_1;
	GPIOA->CRL	|=	 GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2_0;

	//pin PA3 (USART_RX) as input pull-up
	GPIOA->CRL	&=	~GPIO_CRL_CNF3_0 & ~GPIO_CRL_MODE3;
	GPIOA->CRL	|=	 GPIO_CRL_CNF3_1;
}
void USER_ADC_Init(void){
	ADC1->CR1	&=	~ADC_CR1_DUALMOD;//	independent mode
	ADC1->CR2	&=	~ADC_CR2_ALIGN;//	right alignment for the result
	ADC1->CR2	|=	 ADC_CR2_CONT;//	continuous conversion mode
	ADC1->SMPR2	&=	~ADC_SMPR2_SMP0;//	1.5 cycles channel sample time
	ADC1->SQR1	&=	~ADC_SQR1_L;//		1 conversion on regular channels
	ADC1->SQR3 	&=	~ADC_SQR3_SQ1;//	first and only conversion in Ch0
	ADC1->CR2	|=	 ADC_CR2_ADON;//	ADC enabled
	HAL_Delay(1);//					tstab(1us) after ADC enabled, real 1ms
}
void USER_ADC_Calibration(void){
	ADC1->CR2	|=	 ADC_CR2_CAL;//		start calibration
	while( ADC1->CR2 & ADC_CR2_CAL );//		wait until calibration is done
}
uint16_t USER_ADC_Read( void ){
	while( !( ADC1->SR & ADC_SR_EOC ) );//		wait until conversion is done
	return (uint16_t)ADC1->DR;//			return ADC data
}

void USER_USART2_Init(void){
	USART2->CR1	|=	 USART_CR1_UE;//		USART enabled
	USART2->CR1	&=	~USART_CR1_M//		  	1 start bit, 8 data bits
			&	~USART_CR1_PCE;//		parity control disabled
	USART2->CR2	&=	~USART_CR2_STOP;//  		1 stop bit
	//USART2->BRR	 =	 0xD05;//			9600 bps -> 208.33,
	//USARTDIV = 32*10^6/(16*9600)
	//NEW USARTDIV = 32*10^6/(16*115200)=17.361
	//BRR = [17->HEX=11][.361*16->HEX=6]=116
	USART2->BRR = 0x116;


	USART2->CR1	|=	 USART_CR1_TE;//		        transmitter enabled
	USART2->CR1	|=	 USART_CR1_RE;//		        receiver enabled
}

void USER_USART2_Transmit(uint8_t *pData, uint16_t size ){
	for( int i = 0; i < size; i++ ){
		while( ( USART2->SR & USART_SR_TXE ) == 0 ){}//	wait until transmit reg is empty
		USART2->DR = *pData++;//			transmit data
	}
}

uint32_t USER_USART2_Receive(void){
	while((USART2->SR & USART_SR_RXNE) == 0){}
	return USART2->DR;
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
