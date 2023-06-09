/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @author         : rahu7p
  * @co-author      : MaxPacheco02
  ******************************************************************************
  * @board	  : nucleo-f103rb
  * @mcu	  : stm32f103rb
  *
  *
  *
  * Used @rahu7p's template to solve the class' challenge
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "myprintf.h"
#include <stdlib.h>


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

void USER_TIM2_Capture_Init(void);
uint16_t USER_TIM2_Capture_Event(void);
void USER_TIM3_Capture_Init(void);
uint16_t USER_TIM3_Capture_Event(void);

void USER_USART3_Init(void);
void USER_USART3_Transmit(uint8_t *pData, uint16_t size);
uint32_t USER_USART3_Receive(void);

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
  uint16_t event_val1, event_val2, event_diff;
  uint32_t a=0, a1=0, a2=0, b=0, c=0, d=0, t_a=0, t_b=0;
  float pressed_t, freq, max_freq = 0, t=0;
  uint16_t dataADC;
  float v;

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
  USER_TIM2_Capture_Init();
  USER_TIM3_Capture_Init();
  USER_USART3_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  TIM2->CNT = 230;	//A
  while (1)
  {
	  if(TIM2->SR & TIM_SR_UIF){
		  TIM2->SR	&=	!TIM_SR_UIF;
		  TIM2->CR1	&=	 !TIM_CR1_CEN;//		counter disabled

		  t+=0.1;
		  t_a = (int)t;
		  t_b = ((int)(t*10)) % 10;

		  freq = 70 + ((float)(rand() % 100))/10.0;
		  a = freq;
		  b = (int)(freq*10) % 10;

		  v = 3.3 + ((float)(rand() % 100))/500.0;
		  c = v;
		  d = (int)(v*10) % 10;
		  printf("%d.%d) F: %d.%d || P: %d.%d\n",t_a, t_b,a,b,c,d);
		  TIM2->CNT = 230;
		  TIM2->CR1	|=	 TIM_CR1_CEN;//		counter enabled
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void USER_RCC_Init(void){
	//I/O port A clock enable
	RCC->APB2ENR	|=	 RCC_APB2ENR_IOPAEN
  					|	 RCC_APB2ENR_ADC1EN;//	ADC 1 clock enable

	RCC->CFGR	|=	 RCC_CFGR_ADCPRE;//	ADC prescaler 1:8 for 8 MHz


	RCC->APB1ENR	|=	 RCC_APB1ENR_TIM2EN//Timer 2 clock enable
					|	 RCC_APB1ENR_TIM3EN//Timer 3 clock enable
					|	 RCC_APB1ENR_USART3EN;//  	USART3 clock enable

}
void USER_GPIO_Init(void){
	//PA0 (TIM2_CH1) as input floating
	GPIOA->CRL	&=	~GPIO_CRL_CNF0_1 & ~GPIO_CRL_MODE0; //Transmit timer
	GPIOA->CRL	|=	 GPIO_CRL_CNF0_0;

	//PA6 (TIM3_CH1) as input floating
	GPIOA->CRL	&=	~GPIO_CRL_CNF6_1 & ~GPIO_CRL_MODE6; //Flow Sensor
	GPIOA->CRL	|=	 GPIO_CRL_CNF6_0;

	//pin PB10 (USART3_TX) as alternate function output push-pull, max speed 10MHz
	GPIOB->CRH	&=	~GPIO_CRH_CNF10_0 & ~GPIO_CRH_MODE10_1;
	GPIOB->CRH	|=	 GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0; //UART TX

	//pin PB11 (USART3_RX) as input pull-up
	GPIOB->CRH	&=	~GPIO_CRH_CNF11_0 & ~GPIO_CRH_MODE11; //UART RX
	GPIOB->CRH	|=	 GPIO_CRH_CNF11_1;

}

void USER_TIM2_Capture_Init(void){
	TIM2->CR1	&=	~TIM_CR1_CKD_0;
	TIM2->CR1	|=	 TIM_CR1_CKD_1;//	sampling (DTS) = TIM_CLK/4
	TIM2->CCMR1 	&=	~TIM_CCMR1_CC1S_1;
	TIM2->CCMR1 	|=	 TIM_CCMR1_CC1S_0;//	CC1 channel as input, mapped on TI1
	TIM2->CCMR1 	|=	 TIM_CCMR1_IC1F;//	filter -> DTS/32, N=8
	TIM2->CCER	|=	 TIM_CCER_CC1P;//	capture is done on falling edge
	TIM2->CCMR1 	&=	~TIM_CCMR1_IC1PSC;//	no prescaler
	TIM2->CCER	|=	 TIM_CCER_CC1E;//	capture enabled
	TIM2->PSC	 =	 97;//		maximum prescaler
	TIM2->SMCR	&=	!TIM_SMCR_SMS;
	TIM2->CR1	&=	!TIM_CR1_CMS & !TIM_CR1_DIR & !TIM_CR1_UDIS;
	TIM2->SR	&=	!TIM_SR_UIF;
	TIM2->CR1	|=	 TIM_CR1_CEN;//		counter enabled
}
uint16_t USER_TIM2_Capture_Event(void){
	while( !(TIM2->SR & TIM_SR_CC1IF) );//		wait until a capture occurs
	return TIM2->CCR1;//				return the captured value
}

void USER_TIM3_Capture_Init(void){
	TIM3->CR1	&=	~TIM_CR1_CKD_0;
	TIM3->CR1	|=	 TIM_CR1_CKD_1;//	sampling (DTS) = TIM_CLK/4
	TIM3->CCMR1 	&=	~TIM_CCMR1_CC1S_1;
	TIM3->CCMR1 	|=	 TIM_CCMR1_CC1S_0;//	CC1 channel as input, mapped on TI1
	TIM3->CCMR1 	|=	 TIM_CCMR1_IC1F;//	filter -> DTS/32, N=8
	TIM3->CCER	|=	 TIM_CCER_CC1P;//	capture is done on falling edge
	TIM3->CCMR1 	&=	~TIM_CCMR1_IC1PSC;//	no prescaler
	TIM3->CCER	|=	 TIM_CCER_CC1E;//	capture enabled
	TIM3->PSC	 =	 97;//		maximum prescaler

	TIM3->CR1	|=	 TIM_CR1_CEN;//		counter enabled
}

uint16_t USER_TIM3_Capture_Event(void){
	while( !(TIM3->SR & TIM_SR_CC1IF) );//		wait until a capture occurs
	return TIM3->CCR1;//				return the captured value
}

void USER_USART3_Init(void){
	USART3->CR1	|=	 USART_CR1_UE;//		USART enabled
	USART3->CR1	&=	~USART_CR1_M//		  	1 start bit, 8 data bits
			&	~USART_CR1_PCE;//		parity control disabled
	USART3->CR2	&=	~USART_CR2_STOP;//  		1 stop bit
	USART3->BRR	 =	 0xD05;//			9600 bps -> 208.33,
	//USARTDIV = 32*10^6/(16*9600)
	//NEW USARTDIV = 32*10^6/(16*115200)=17.361
	//BRR = [17->HEX=11][.361*16->HEX=6]=116
	//`->BRR = 0x116;

	USART3->CR1	|=	 USART_CR1_TE;//		        transmitter enabled
	USART3->CR1	|=	 USART_CR1_RE;//		        receiver enabled
}

void USER_USART3_Transmit(uint8_t *pData, uint16_t size ){
	for( int i = 0; i < size; i++ ){
		while( ( USART3->SR & USART_SR_TXE ) == 0 ){}//	wait until transmit reg is empty
		USART3->DR = *pData++;//			transmit data
	}
}

uint32_t USER_USART3_Receive(void){
	while((USART3->SR & USART_SR_RXNE) == 0){}
	return USART3->DR;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of erropr occurrence.
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
