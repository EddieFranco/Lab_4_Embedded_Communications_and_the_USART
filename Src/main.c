/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h> 
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Write a function that transmits a single character on the USART.
 void CharTransmitter(char character ){
	  while((USART3->ISR &= USART_ISR_TXE)) // check the USART status flag that indicates the transmit register is empty
		{
			USART3->TDR = character;
		}
	 }
	 
	 
	 //function declaration that accepts an array of characters, either in direct array form or as a pointer
	void StringTransmitter( const char *str){
		while(*str != '\0'){ 
				CharTransmitter(*str); // You can increment over the array by using a counter and array index or by incrementing
				str++;							// the pointer.
		}
		
	}
	
	
	// Check and wait on the USART status flag that indicates the receive (read) register is not
	//empty.
	char Receiver(void) {
    while(!(USART3->ISR & USART_ISR_RXNE));  // check and Wait for Receive data register not empty
    return USART3->RDR;  // Read the received data
}
	

	// Setup the blank interrupt handler
	volatile char received;
	volatile int newdata = 0;
	void USART3_IRQHandler(void) {
		//save the receive register’s value into a global variable
			received = USART3->RDR;
		// set a global variable as a flag indicating new data
			 newdata = 1;
		}
	
	
	 
	 
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */
	
	/*4.1*/
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable USART3 peripheral clock
	// Set the selected pins into alternate function mode and program the correct alternate function
	// number into the GPIO AFR registers.
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // // Enable GPIOC in the RCC
	
	GPIOC->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5); // clearing bits 
	// Set the selected pins into alternate function mode
	GPIOC->MODER |= GPIO_MODER_MODER4_1 ; 
	GPIOC->MODER |= GPIO_MODER_MODER5_1 ;
	
	GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5); // Clear alternate function bits for PC4 and PC5
	GPIOC->AFR[0] |= (1 <<GPIO_AFRL_AFSEL4_Pos) | (1 <<GPIO_AFRL_AFSEL5_Pos); // alternate function number into the GPIO AFR register

  /* USER CODE END 1 */

	// Set the Baud rate for communication to be 115200 bits/second.
	USART3->BRR = 69;
	
		
		// enable transmitter and receiver 
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;
		// USART3 is enabled
	USART3->CR1 |= USART_CR1_UE; 
		
		
		
		
		//In this final exercise use an interrupt to save the received data when it arrives
		// enable the receive register not empty interrupt
	USART3->CR1 |= USART_CR1_RXNEIE;
		
		// Enable and set the USART interrupt priority in the NVIC.
	NVIC_SetPriority(USART3_4_IRQn, 1);
		 
		 
  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
//Enabling pins 8 and 9 (green and orange)
			GPIO_InitTypeDef initc89 = {GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH,GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &initc89);
	
		//Enabling pins 6 and 7 (green and orange)
			GPIO_InitTypeDef initc67 = {GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH,GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &initc67);
  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
			
  while (1)
  {
    /* USER CODE END WHILE */
		
		// 4.3 typing a particular sequence of two keystrokes will turn on, turn off,
		// or toggle the corresponding LED,
		StringTransmitter("user");
		
		char color = Receiver();
		char num = Receiver();

	switch(color) {
    case 'r':
    case 'R':
        switch(num) {
            case '0':
                GPIOC->ODR &= ~GPIO_ODR_6; // Turn off red LED
                break;
            case '1':
                GPIOC->ODR |= GPIO_ODR_6; // Turn on red LED
						  StringTransmitter("turn_on_led_6");
                break;
            case '2':
                GPIOC->ODR ^= GPIO_ODR_6; // Toggle red LED
                break;
            default:
                StringTransmitter("Error: Unrecognized input\n\n");
                break;
        }
        break;
        
    case 'g':
    case 'G':
        switch(num) {
            case '0':
                GPIOC->ODR &= ~GPIO_ODR_9; // Turn off green LED
                break;
            case '1':
                GPIOC->ODR |= GPIO_ODR_9; // Turn on green LED
                break;
            case '2':
                GPIOC->ODR ^= GPIO_ODR_9; // Toggle green LED
                break;
            default:
                StringTransmitter("Error: Unrecognized input\n\n");
                break;
        }
        break;
        
    case 'b':
    case 'B':
        switch(num) {
            case '0':
                GPIOC->ODR &= ~GPIO_ODR_7; // Turn off blue LED
                break;
            case '1':
                GPIOC->ODR |= GPIO_ODR_7; // Turn on blue LED
                break;
            case '2':
                GPIOC->ODR ^= GPIO_ODR_7; // Toggle blue LED
                break;
            default:
                StringTransmitter("Error: Unrecognized input\n\n");
                break;
        }
        break;
        
    case 'o':
    case 'O':
        switch(num) {
            case '0':
                GPIOC->ODR &= ~GPIO_ODR_8; // Turn off orange LED
                break;
            case '1':
                GPIOC->ODR |= GPIO_ODR_8; // Turn on orange LED
                break;
            case '2':
                GPIOC->ODR ^= GPIO_ODR_8; // Toggle orange LED
                break;
            default:
                StringTransmitter("Error: Unrecognized input\n\n");
                break;
        }
        break;
        
    default:
        StringTransmitter("Error: Unrecognized color.\n\n");
        StringTransmitter("Error: Unrecognized action.\n\n");
        break;
}
		
		
		// 4.2 evelop an application that toggles
			//the correct LED whenever the character matching the first letter of the color is pressed
		//while (!(USART3->ISR & USART_ISR_RXNE)); // //check & wait until Receive Data Register (RDR) is not empty
		
		//char receiver = Receiver();  // read received data character
		
		 //switch(receiver) {
    //case 'o':
    //case 'O':
       // GPIOC->ODR ^= GPIO_ODR_8;   // Toggle orange LED
       // break;
    //case 'g':
    //case 'G':
    //    GPIOC->ODR ^= GPIO_ODR_9;   // Toggle green LED
     //   break;
    //case 'r':
    //case 'R':
       // GPIOC->ODR ^= GPIO_ODR_6;   // Toggle red LED
       // break;
    //case 'b':
    //case 'B':
       // GPIOC->ODR ^= GPIO_ODR_7;   // Toggle blue LED
       // break;
    //default:
       // StringTransmitter("Error");
       // break;
	//	}


		
		// Excercises ......
		//CharTransmitter('A');
		//StringTransmitter("Error");
		
		
		
		
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
