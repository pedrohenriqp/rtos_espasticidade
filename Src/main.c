/**
  ******************************************************************************
  * File Name          	: main.c
  * Project				: Eletrogoniometro - Análise de espaticidade
  * Description        	: Main program body
  * Author			   	: Pedro Henrique Pereira
  * Version				: 1
  * Year				: 2017
  *
  *This code was configured by the author using STM32F4 HAL driver and FreeRTOS through STMCubeMX- Copyright (c) 2017 STMicroelectronics International N.V.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdlib.h"
#include "float_to_string.h"


/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
GPIO_InitTypeDef GPIO_InitStruct;
osThreadId defaultTaskHandle;
osThreadId adc_sensorHandle;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void UART2_pins_config(void);
void UART1_pins_config(void);
void ADC_pins_config(void);



int main(void)
{

  /* -------MCU Configuration------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  UART1_pins_config();			//The same as HAL_UART_MspInit(huart1) but speed setup as LOW
  UART2_pins_config();			//The same as HAL_UART_MspInit(huart2) but speed setup as LOW
  HAL_ADC_MspInit(&hadc1);

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of adc_sensor */
  osThreadDef(adc_sensor, StartTask02, osPriorityIdle, 1, 128);
  adc_sensorHandle = osThreadCreate(osThread(adc_sensor), NULL);


  /* Start scheduler */
  osKernelStart();

  while (1)
  {
  }


}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

void UART1_pins_config(void)
{
	      __GPIOA_CLK_ENABLE();
	      __USART1_CLK_ENABLE();

	      /**USART1 GPIO Configuration
	      PA9     ------> USART2_TX
	      PA10     ------> USART2_RX
	      */
	      GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
	      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	      GPIO_InitStruct.Pull = GPIO_PULLUP;
	      GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	      GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void UART2_pins_config(void)
{
	 //Using UART 2 to send data to PC

      __GPIOA_CLK_ENABLE();
      __USART2_CLK_ENABLE();

      /**USART2 GPIO Configuration
      PA2     ------> USART2_TX
      PA3     ------> USART2_RX
      */
      GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;			//Alternate Function Push Pull Mode
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void ADC_pins_config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	 __HAL_RCC_GPIOA_CLK_ENABLE();

     GPIO_InitStruct.Pin = GPIO_PIN_0;
     GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;			//Analog mode
     GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
     //GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}


/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

	/* Task : Read ADC value and send to PC through UART2 when pressed number 4 */

    char *msg1 = "Analise de espasticidade\n\rLeitura GPIO A0:\n\r";
    char *msg2 = "Eletrogoniometro\n\r";
    float  adc_value_voltage = 0;
    char adc_value_voltage_char[10];
    float vec_data[4000];
    char vec_data_converted_char[10];
    float ang = 0;
    uint8_t vetor = 0;
    uint8_t n = 1;
    uint32_t nmax = 100;
    uint16_t adc_value = 0;
    uint32_t timeout = 1000;

    HAL_ADC_MspInit(&hadc1);   //clock and pins config

	while(1){

    osDelay(5);
    HAL_UART_Receive(&huart2, &vetor, 1, 10);
    osDelay(5);

    switch(vetor)
    {
    case '1':
    	HAL_UART_Transmit(&huart2, msg1, strlen(msg1), 0xFFFF);
    	vetor = 0;
    	break;
    case '2':
    	HAL_UART_Transmit(&huart2, msg2, strlen(msg2), 0xFFFF);
    	vetor = 0;
    	break;
    case '4':
    	HAL_ADC_Start(&hadc1);
    	while(HAL_ADC_PollForConversion(&hadc1, timeout ) != HAL_OK);
    	adc_value = HAL_ADC_GetValue(&hadc1);
    	HAL_ADC_Stop(&hadc1);

    	adc_value_voltage = ((adc_value*3.3)/256); //register -> voltage
    	ang = (adc_value_voltage*180)/3.3;   		// voltage -> angle
    	vec_data[n] = ang;							//write on buffer

    	float_to_string(adc_value_voltage, adc_value_voltage_char);
    	float_to_string(vec_data[n], vec_data_converted_char);

    	HAL_UART_Transmit(&huart2, "Tensao: ", strlen("Tensao: "), timeout);
    	osDelay(1);
    	HAL_UART_Transmit(&huart2, adc_value_voltage_char, strlen(adc_value_voltage_char), timeout);
    	adc_value = 0;
    	vetor = 0;
    	osDelay(1);
    	HAL_UART_Transmit(&huart2, "\n\rAngulo correspondente: ", strlen("\n\rAngulo correspondente: "), timeout);
    	osDelay(1);
     	HAL_UART_Transmit(&huart2, vec_data_converted_char, strlen(vec_data_converted_char), timeout);
    	osDelay(1);
    	HAL_UART_Transmit(&huart2, "\n\r", strlen("\n\r"), timeout);
    	n++;
    	if(n == nmax){
    		n=0;
    	}

    	break;
    case '5':

    	break;

      }

    }

}


/* StartTask02 function */
void StartTask02(void const * argument)
{
	//Task: Read ADC1 and send to ESP8266 through UART1 continuously

    char angle_char[10];
    float adc_value_esp = 0, adc_value_voltage_esp = 0;
    uint32_t timeout = 1000;
    int nmax = 400, m = 5,t = 1;
    float buf[400];
    float ang_esp;
    float A1=2.3, A2=1.1, A0=1.4, sum=1;
	float tol = 0;
    float r1 = 0, r2 = 0,r2n = 0;
    char r1_char[10],r2_char[10], r2n_char[10];

	 HAL_ADC_MspInit(&hadc1);   //clock and pins config

	 /*read eletrogoniometer nmax samples*/
	for(int h=0; h<nmax; h++)
	{
		HAL_ADC_Start(&hadc1);
		while(HAL_ADC_PollForConversion(&hadc1, timeout ) != HAL_OK);
		adc_value_esp = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

	    osDelay(5);

	    adc_value_voltage_esp = ((adc_value_esp*3.3)/256);
    	ang_esp = ((adc_value_voltage_esp*180)/3.3);
    	buf[h] = ang_esp;

    	float_to_string(buf[h], angle_char);

    	HAL_UART_Transmit(&huart2, angle_char, strlen(angle_char), timeout);    //sendo to pc
    	HAL_UART_Transmit(&huart2, "\n\r", strlen("\n\r"), timeout);
	    HAL_UART_Transmit(&huart1, angle_char, strlen(angle_char), timeout);	// send to nodemcu
       	adc_value_esp = 0;


     }

/* sampling finished -> start the algorithm*/
	HAL_UART_Transmit(&huart2,"Fim da coleta\n\r", strlen("Fim da coleta\n\r"), timeout);

    /********* recognition of first peak and first valley *************/

	for(int j=1; j<(nmax/2); j++){
		if(  (buf[j]>(buf[j+1]+tol)) && (buf[j]>(buf[j-1]+tol))   )
			A1=buf[j];
	}

	for(int j=1; j<(nmax/2); j++){
			if(  (buf[j]<(buf[j+1]-tol)) && (buf[j]<(buf[j-1]-tol))   )
				A1=buf[j];
		}

	t = nmax-m;
	for(int k=t; k<nmax; k++){
		sum=sum + buf[k];
		}
	 	 A0 = (1*sum)/(m+1);

   		r1 = A1/((A1-A2)+0.01);          //avoid division by zero
   		r2 = A1/A0;
   		r2n = r2/1.6;

   		float_to_string(r1, r1_char);
   		HAL_UART_Transmit(&huart2,"r1", strlen("r1"), timeout);
   		HAL_UART_Transmit(&huart2,r1_char, strlen(r1_char), timeout);

   		float_to_string(r2, r2_char);
   		HAL_UART_Transmit(&huart2,"r2", strlen("r2"), timeout);
   	  	HAL_UART_Transmit(&huart2,r2_char, strlen(r2_char), timeout);

   	  	float_to_string(r2n, r2n_char);
   	    HAL_UART_Transmit(&huart2,"rn", strlen("rn"), timeout);
   	    HAL_UART_Transmit(&huart2,r2n_char, strlen(r2n_char), timeout);

   	/*************End of algorithm********/

	//loop of task
	while(1){

	}

}


void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
 }


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

//********************************End of code****************************


