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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rf_driver.h"
#include "spi.h"
#include "dwt.h"
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
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */


uint8_t JETTA_KEYFOB[] = { //In fact it's 2.4Kb/s because of the Manhattan codification, see Datasheet.
	    0x0D,  // IOCFG2              GDO2 Output Pin Configuration SERIAL DATA OUT
	    0x2E,  // IOCFG1              GDO1 Output Pin Configuration
	    0x08,  // IOCFG0              GDO0 Output Pin Configuration
	    0x47,  // FIFOTHR             RX FIFO and TX FIFO Thresholds
	    0xD3,  // SYNC1               Sync Word, High Byte
	    0x91,  // SYNC0               Sync Word, Low Byte
	    0xFF,  // PKTLEN              Packet Length
	    0x00,  // PKTCTRL1            Packet Automation Control
	    0x32,  // PKTCTRL0            Packet Automation Control, IS IN ASYCH MODE
	    0x00,  // ADDR                Device Address
	    0x00,  // CHANNR              Channel Number
	    0x06,  // FSCTRL1             Frequency Synthesizer Control
	    0x00,  // FSCTRL0             Frequency Synthesizer Control
	    0x21,  // FREQ2               Frequency Control Word, High Byte
	    0x62,  // FREQ1               Frequency Control Word, Middle Byte
	    0x76,  // FREQ0               Frequency Control Word, Low Byte
	    0xF8,  // MDMCFG4             Modem Configuration
	    0xA0,  // MDMCFG3             Modem Configuration //settings should be 3.4kb
	    0x33,  // MDMCFG2             Modem Configuration //need to change to 0 or x4
	    0x22,  // MDMCFG1             Modem Configuration
	    0xF8,  // MDMCFG0             Modem Configuration
	    0x15,  // DEVIATN             Modem Deviation Setting
	    0x07,  // MCSM2               Main Radio Control State Machine Configuration
	    0x30,  // MCSM1               Main Radio Control State Machine Configuration
	    0x18,  // MCSM0               Main Radio Control State Machine Configuration
	    0x16,  // FOCCFG              Frequency Offset Compensation Configuration
	    0x6C,  // BSCFG               Bit Synchronization Configuration
	    0x04,  // AGCCTRL2            AGC Control
	    0x00,  // AGCCTRL1            AGC Control
	    0x92,  // AGCCTRL0            AGC Control
	    0x87,  // WOREVT1             High Byte Event0 Timeout
	    0x6B,  // WOREVT0             Low Byte Event0 Timeout
	    0xFB,  // WORCTRL             Wake On Radio Control
	    0x56,  // FREND1              Front End RX Configuration
	    0x11,  // FREND0              Front End TX Configuration
	    0xE9,  // FSCAL3              Frequency Synthesizer Calibration
	    0x2A,  // FSCAL2              Frequency Synthesizer Calibration
	    0x00,  // FSCAL1              Frequency Synthesizer Calibration
	    0x1F,  // FSCAL0              Frequency Synthesizer Calibration
	    0x41,  // RCCTRL1             RC Oscillator Configuration
	    0x00,  // RCCTRL0             RC Oscillator Configuration
	    0x59,  // FSTEST              Frequency Synthesizer Calibration Control
	    0x7F,  // PTEST               Production Test
	    0x3F,  // AGCTEST             AGC Test
	    0x81,  // TEST2               Various Test Settings
	    0x35,  // TEST1               Various Test Settings
	    0x09,  // TEST0               Various Test Settings

};


uint8_t HONEYWELL_KEYFOB[] = { //In fact it's 2.4Kb/s because of the Manhattan codification, see Datasheet.
    0x0D,  // IOCFG2        GDO2 Output Pin Configuration-

    0x2E,  // IOCFG1        GDO1 Output Pin Configuration

    0x2D,  // IOCFG0        GDO0 Output Pin Configuration-

    0x07,  // FIFOTHR       RX FIFO and TX FIFO Thresholds

    0xD3,  // SYNC1         Sync Word, High Byte

    0x91,  // SYNC0         Sync Word, Low Byte

    0xFF,  // PKTLEN        Packet Length-

    0x04,  // PKTCTRL1      Packet Automation Control-

    0x32,  // PKTCTRL0      Packet Automation Control-

    0x00,  // ADDR          Device Address-

    0x00,  // CHANNR        Channel Number-

    0x00,  // FSCTRL1       Frequency Synthesizer Control-

    0x00,  // FSCTRL0       Frequency Synthesizer Control-

    0x10,  // FREQ2         Frequency Control Word, High Byte

    0xAF,  // FREQ1         Frequency Control Word, Middle Byte

    0x78,  // FREQ0         Frequency Control Word, Low Byte

    0x87,  // MDMCFG4       Modem Configuration-

    0x93,  // MDMCFG3       Modem Configuration-

    0x04,  // MDMCFG2       Modem Configuration-

    0x22,  // MDMCFG1       Modem Configuration-

    0xF8,  // MDMCFG0       Modem Configuration-

    0x45,  // DEVIATN       Modem Deviation Setting-

    0x07,  // MCSM2         Main Radio Control State Machine Configuration

    0x30,  // MCSM1         Main Radio Control State Machine Configuration

    0x18,  // MCSM0         Main Radio Control State Machine Configuration-

    0x16,  // FOCCFG        Frequency Offset Compensation Configuration-

    0x6C,  // BSCFG         Bit Synchronization Configuration-

    0x07,  // AGCCTRL2      AGC Control-

    0x47,  // AGCCTRL1      AGC Control-

    0x91,  // AGCCTRL0      AGC Control-

    0x87,  // WOREVT1       High Byte Event0 Timeout

    0x6B,  // WOREVT0       Low Byte Event0 Timeout

    0xF8,  // WORCTRL       Wake On Radio Control

    0x56,  // FREND1        Front End RX Configuration-

    0x10,  // FREND0        Front End TX Configuration-

    0xE9,  // FSCAL3        Frequency Synthesizer Calibration-

    0x2A,  // FSCAL2        Frequency Synthesizer Calibration-

    0x00,  // FSCAL1        Frequency Synthesizer Calibration-

    0x1F,  // FSCAL0        Frequency Synthesizer Calibration-

    0x41,  // RCCTRL1       RC Oscillator Configuration

    0x00,  // RCCTRL0       RC Oscillator Configuration

    0x59,  // FSTEST        Frequency Synthesizer Calibration Control

    0x7F,  // PTEST         Production Test

    0x3F,  // AGCTEST       AGC Test

    0x81,  // TEST2         Various Test Settings-

    0x35,  // TEST1         Various Test Settings-

    0x09,  // TEST0         Various Test Settings-
};

uint8_t patable_power_433[]  = {0x6C,0x1C,0x06,0x3A,0x51,0x85,0xC8,0xC0};
uint8_t patable_power_315[] = {0x00,0x50,0x00,0x00,0x00,0x00,0x00,0x00};
//normal fsk 315 table: {0x17,0x1D,0x26,0x69,0x51,0x86,0xCC,0xC3};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

uint8_t jetta = 0;

#define ADC_SIZE 2000
  uint16_t adc_dat[ADC_SIZE];
  uint8_t adc_read[ADC_SIZE];
  uint8_t*adc_p = adc_read;

uint8_t bit_pat[] = {48,48,48,48,48,48,48,48,48,49,49,49,49,49,49,49,49,49,49,49,49,49};
//48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,49,49,49,49,49,49,48,48,48,48,48,48,48,49,49,49,49,49,49,49,48,48,48,48,48,48,48,48,49,49,49,49,49,49,48,48,48,48,48,48,48,48,49,49,49,49,49,49,48,48,48,48,48,48,48,48,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49};
//48,48,48,48,48,48,48,48,48,48,48,48,48,49,49,49,49,49,48,48,48,48,48,48,48,48,48,49,49,49,49,49,49,49,49,49,49,49,49,49,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,48,48,48,48,48,48,48,48,49,49,49,49,49,49,48,48,48,48,48,48,48,49,49,49,49,49,48,48,48,48,48,48,48,48,49,49,49,49,49,49,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,48,48,48,48,48,48,48,49,49,49,49,49,49,49,48,48,48,48,48,48,48};

void find_bit_pattern2(int first_index, int second_index) {
  uint8_t pattern_count = 0;
  for(int i=first_index;i<second_index;i++) {
    if(adc_read[i]==bit_pat[pattern_count]) pattern_count++;
    else pattern_count = 0;
    if(pattern_count==(sizeof(bit_pat)-1)) {
      HAL_UART_DMAStop(&huart2);
      HAL_UART_Transmit_DMA(&huart2, (uint8_t*) "DONE!\n", 6);
      HAL_ADC_Stop_DMA(&hadc1);
      while(1) {};
    }
  }
}

  

void find_bit_pattern(int first_index, int second_index) {
	uint8_t check_zero = 1;
  uint8_t zero_count = 0;
  uint8_t one_count = 0;
  uint8_t pattern_count = 0;
#define IGNORE_NUM 2
  uint8_t ignore = IGNORE_NUM;

  uint8_t zero_check_arr[3];
  uint8_t one_check_arr[3]; 

  if(jetta) {
    zero_check_arr[0] = 16;
    zero_check_arr[1] = 15;
    zero_check_arr[2] = 15;

    one_check_arr[0] = 16;
    one_check_arr[1] = 15;
    one_check_arr[2]= 15;

  } else {
    zero_check_arr[0] = 6;
    zero_check_arr[1] = 7;
    zero_check_arr[2] = 8;

    one_check_arr[0] = 5;
    one_check_arr[1] = 6;
    one_check_arr[2] = 6;
  }

  for(int i=first_index;i<second_index;i++) {
      //check for pattern five 0 five 1 five 0 five 1 five 0 five 1 three 0 three 1 three 0 three 1 three 0
      switch(check_zero) {
          case 1:
          if(adc_read[i]==0x30)	{
            zero_count++;
          } 	else if(ignore!=0) {
            ignore--;
          }
          else {
            ignore = IGNORE_NUM;
            zero_count = 0;
            pattern_count = 0;
          }
          if((zero_count==zero_check_arr[0])||(zero_count==zero_check_arr[1])||(zero_count==zero_check_arr[2])) {
            ignore = IGNORE_NUM;
            zero_count = 0;
            check_zero = 0;
            pattern_count++;
          }
          break;
          case 0:
          if(adc_read[i]==0x31) {
            one_count++;
          } else if(ignore!=0) {
            ignore--;
          } else {
            ignore = IGNORE_NUM;
            one_count = 0;
            pattern_count = 0;
          }
          if((one_count==one_check_arr[0])||(one_count==one_check_arr[1])||(one_count==one_check_arr[2])) {
            check_zero = 1;
            pattern_count++;
            one_count = 0;
            ignore = IGNORE_NUM;
          }
          break;
      }


      if(pattern_count==14) {
        //break;

        ///*
        pattern_count=0;
        HAL_UART_DMAStop(&huart2);
        HAL_UART_Transmit_DMA(&huart2, (uint8_t*) "DONE!\n", 6);
        HAL_ADC_Stop_DMA(&hadc1);
        while(1) {};
        //*/
      }

    }
}

volatile HAL_StatusTypeDef hal_stat;
volatile uint8_t uart_done = 0;
volatile uint8_t do_uart_half = 0;
volatile uint8_t do_uart_start = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	uart_done = 1;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  for(int i=0;i<ADC_SIZE/2;i++) {
    if(adc_dat[i] > 2000) {
      adc_read[i] = 0x31;
    } else {
      adc_read[i] = 0x30;
    }
  }
  find_bit_pattern(0, ADC_SIZE/2);
  do_uart_start = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	hal_stat = HAL_UART_Transmit_DMA(&huart2, adc_read, ADC_SIZE);
  for(int i=ADC_SIZE/2;i<ADC_SIZE;i++) {
    if(adc_dat[i] > 2000) {
      adc_read[i] = 0x31;
    } else {
      adc_read[i] = 0x30;
    }
  }
  find_bit_pattern(ADC_SIZE/2, ADC_SIZE);
  do_uart_half = 1;
}

int get_freq_float(void) {
  uint8_t freqs[3];
  int freq = 0;

  for(uint8_t i=0;i<3;i++) _spi_read(&hspi1, 0xC0|(FREQ0-i), &freqs[i], 1);

  for(uint8_t i=0;i<3;i++) freq |= freqs[i] << (i*8);

  return (int) (((float) freq) * 396.7285156); 

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void USART_DMA_Transmit(uint32_t peripheral, uint32_t*buff, uint32_t buff_size) {
  DMA1_Stream6->NDTR = buff_size;
  DMA1_Stream6->PAR = peripheral;
  DMA1_Stream6->M0AR = (uint32_t)buff;
  USART2->CR1 = (USART_CR1_TE|USART_CR1_RE|USART_CR1_UE);
  USART2->CR3 |= USART_CR3_DMAT;
  DMA1_Stream6->CR |=  DMA_SxCR_EN;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  adc_p += (ADC_SIZE/4);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  DwtInit();
  HAL_UART_Transmit_DMA(&huart2, (uint8_t*) "TEST!\n", 6);
  //USART_DMA_Transmit((uint32_t)&USART2->DR, (uint32_t*)init_msg, (uint32_t)6);

  uint8_t rx_data = 0;
  uint8_t read = 1;

  radio_reset(&hspi1);

  while(rx_data!=0x14) _spi_read(&hspi1, VERSION, &rx_data, 1);

  if(jetta) _spi_write_multiple(&hspi1, 0x40, JETTA_KEYFOB, sizeof(JETTA_KEYFOB));
  else _spi_write_multiple(&hspi1, 0x40, HONEYWELL_KEYFOB, sizeof(HONEYWELL_KEYFOB));
	//_spi_write_multiple(0x40, cc1100_GFSK_38_4_kb, sizeof(cc1100_GFSK_38_4_kb));

	//315 MHz: 0x0C 0x1D 0x89
  	//434 0x10 0xB0 0x71

  if(jetta) {
    _spi_write(&hspi1, FREQ2,0x0C); 
	  _spi_write(&hspi1, FREQ1,0x1D);
	  _spi_write(&hspi1, FREQ0,0x89);
  } else {
    _spi_write(&hspi1, FREQ2,0x0D); 
    _spi_write(&hspi1, FREQ1,0x45);
    _spi_write(&hspi1, FREQ0,0x35);
  }
	_spi_write_multiple(&hspi1, PATABLE_BURST, patable_power_315, 8);
	_spi_write(&hspi1, CHANNR, 0x00);
	_spi_write(&hspi1, FREND0, 0x11); //adjusts tx power

  char tx_buff[50];
  int operating_frequency = 0.0;
  int buff_size = 0;
  operating_frequency = get_freq_float();
  buff_size = snprintf(tx_buff, 50, "FREQ: %d\n", operating_frequency);
  HAL_UART_Transmit_DMA(&huart2, (uint8_t*) tx_buff, buff_size);

  if(read) _spi_write(&hspi1, PKTCTRL0, 0x32);

	uint8_t tx_dat[] = {0x10,0x01,0x10,0x30,0x10,0x01,0x10,0x30};
	_spi_strobe(&hspi1, SFTX); //flush tx buff
	_spi_strobe(&hspi1, SFRX); //flush tx buff

  uint8_t test_read = 0;
  uint8_t state_var = 0;

  //HAL_TIM_Base_Start_IT(&htim10);
  if(read) {
    _spi_strobe(&hspi1, SRX);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_dat, ADC_SIZE);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	  if(do_uart_half) {
		  do_uart_half = 0;
		  while(!uart_done) {
			  asm("NOP");
		  }
		 hal_stat = HAL_UART_Transmit_DMA(&huart2, adc_p, ADC_SIZE/4);
	  }

	  if(do_uart_start) {
		  do_uart_start = 0;
		  while(!uart_done) {
			  asm("NOP");
		  }
		  hal_stat = HAL_UART_Transmit_DMA(&huart2, adc_read, ADC_SIZE/4);
	  }
	*/

	  //for(int i=0;i<BUFF_SIZE;i++) read_arr[i] = (((GPIOA->IDR)& 0b0000000100000000) >> 8)+0x30;
	  //HAL_UART_Transmit_DMA(&huart2, read_arr, BUFF_SIZE);

	  //LL_GPIO_ReadInputPort(GPIOA);
	  /*
	  //RX
	  _spi_strobe(SRX); //Start RX
	  _spi_read(RXBYTES, &test_read, 1);
	  while(test_read==0) {
		  _spi_read(RXBYTES, &test_read, 1);
		  DwtDelay_ms(1);
	  }
	  DwtDelay_ms(10);
	  for(uint32_t i=0;i<sizeof(read_arr);i++) {
		  _spi_write(RXFIFO, read_arr[i]);
	  }
		*/

	  ///*
    if(!read) {
      //send packet size and packet to FIFO
      _spi_write(&hspi1, TXFIFO, sizeof(tx_dat));

      for(uint32_t i=0;i<sizeof(tx_dat);i++) {
        _spi_write(&hspi1, TXFIFO, tx_dat[i]);
      }

      //_spi_read(TXBYTES, &rx_data, 1);

      _spi_strobe(&hspi1, STX); //Start transmision

        HAL_Delay(10);

      _spi_read(&hspi1, MARCSTATE, &state_var, 1);
      while((state_var!=IDLE)&&(state_var!=TXFIFO_UNDERFLOW)) { //MARCSTATE returns to idle if tx is done
        _spi_read(&hspi1, MARCSTATE, &state_var, 1);
        DwtDelay_ms(1);
      }

        _spi_strobe(&hspi1, SFTX); //flush tx buff
        _spi_strobe(&hspi1, SIDLE); //return to idle (is this needed?)
        HAL_Delay(1000);
    }
    //*/

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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 10000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
