/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "math.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BMP280_ADDRESS_WRITE          0xEC
#define BMP280_ADDRESS_READ			  0xED
#define BMP280_CHIPID                 0xD0
#define BMP280_REGISTER_CONTROL       0xF4
#define BMP280_REGISTER_CONFIG        0xF5
#define BMP280_RESET                  0xE0      // Reset register
#define RESET_CODE                    0xB6      // Reset Code

#define BMP280_I_AM				  	  0x60		// chıp ıd register value

#define BMP280_REGISTER_DIG_T1 	      0x88
#define BMP280_REGISTER_DIG_T2        0x8A
#define BMP280_REGISTER_DIG_T3        0x8C

#define BMP280_REGISTER_DIG_P1        0x8E
#define BMP280_REGISTER_DIG_P2        0x90
#define BMP280_REGISTER_DIG_P3        0x92
#define BMP280_REGISTER_DIG_P4        0x94
#define BMP280_REGISTER_DIG_P5        0x96
#define BMP280_REGISTER_DIG_P6        0x98
#define BMP280_REGISTER_DIG_P7        0x9A
#define BMP280_REGISTER_DIG_P8        0x9C
#define BMP280_REGISTER_DIG_P9        0x9E

#define BMP280_REGISTER_TEMPDATA      0xFA
#define BMP280_REGISTER_PRESSUREDATA  0xF7

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

/**********************************************************************************
*
*		BMP280 GLOBAL VARIABLES
*
**********************************************************************************/

uint8_t device_adress,i2c[255],Rx_Buffer[1];
int flag;

struct
{
   uint16_t dig_T1;
   int16_t  dig_T2;
   int16_t  dig_T3;
   uint16_t dig_P1;
   int16_t  dig_P2;
   int16_t  dig_P3;
   int16_t  dig_P4;
   int16_t  dig_P5;
   int16_t  dig_P6;
   int16_t  dig_P7;
   int16_t  dig_P8;
   int16_t  dig_P9;
} params;

int32_t temp_fine,T;
float bmp280_present_pressure,bmp280_pressure_offset;
float bmp280_altitude,bmp280_temperature,bmp280_pressure;

HAL_StatusTypeDef fresult;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */

void Scan_I2C(void);
void initBMP280(void);
void readCoefficients(void);
void calibrationBMP280(void);
float bmp280_altitude_m(void);
float bmp280_altitude_cm(void);
float bmp280_readTemperature(void);
float bmp280_readPressure(void);

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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // Sensorun BMP280 sensor olup olmadigini kontrol eder
  // eger dogru ise flag'i 1 yapar degilse 0 yapar

  if(HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDRESS_READ,BMP280_CHIPID, 1, Rx_buffer, 1, 100) == HAL_OK)
  {
     flag = 1;
  }
  else
  {
     flag = 0;
  }

  initBMP280();
  calibrationBMP280();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	bmp280_temperature = bmp280_readTemperature();
	bmp280_pressure    = bmp280_readPressure();
	bmp280_altitude    = bmp280_altitude_m();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/**
  * Baslangic ayarlarini yapar
  */
void initBMP280()
{
  // Reset Code - 0xB6
  uint8_t temp_register_value = 0xB6;

  fresult = HAL_I2C_Mem_Write(&hi2c2, BMP280_ADDRESS_WRITE, BMP280_RESET, 1, &temp_register_value, 1, HAL_MAX_DELAY);
  if(fresult != HAL_OK) flag = 0;
  else if(fresult == HAL_OK) flag = 1;

  // Mode: Normal Mode - Pressure Sampling: x16 - Temperature Sampling: x16
  temp_register_value = (uint8_t)0x57;

  fresult = HAL_I2C_Mem_Write(&hi2c2, BMP280_ADDRESS_WRITE, BMP280_REGISTER_CONTROL, 1, &temp_register_value, 1, HAL_MAX_DELAY);
  if(fresult != HAL_OK) flag = 0;
  else if(fresult == HAL_OK) flag = 1;

  // standby ms: 0.5 ms , filter: x16
  temp_register_value = (uint8_t)0x08;

  fresult = HAL_I2C_Mem_Write(&hi2c2, BMP280_ADDRESS_WRITE, BMP280_REGISTER_CONTROL, 1, &temp_register_value, 1, HAL_MAX_DELAY);
  if(fresult != HAL_OK) flag = 0;
  else if(fresult == HAL_OK) flag = 1;

  // Eger bekleme olmazsa readCoefficients fonksiyonu duzgun calismiyor
  HAL_Delay(100);

  readCoefficients();
}

/**
  * Gerekli tum konfigurasyon params degerleri elde eder
  */
void readCoefficients()
{
  uint8_t temp_bmp_buffer_T[6] = {0,0,0,0,0,0};
  temp_bmp_buffer_T[0] = BMP280_REGISTER_DIG_T1;
  HAL_I2C_Master_Transmit(&hi2c2, BMP280_ADDRESS_WRITE, temp_bmp_buffer_T, 1, 200);
  HAL_I2C_Master_Receive(&hi2c2, BMP280_ADDRESS_READ, temp_bmp_buffer_T,6, 200);

  params.dig_T1   = temp_bmp_buffer_T[0] | temp_bmp_buffer_T[1] << 8;
  params.dig_T2   = (int16_t)temp_bmp_buffer_T[2] | temp_bmp_buffer_T[3] << 8;
  params.dig_T3   = (int16_t)temp_bmp_buffer_T[4] | temp_bmp_buffer_T[5] << 8;

  uint8_t temp_bmp_buffer_P[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  temp_bmp_buffer_P[0] = BMP280_REGISTER_DIG_P1;
  HAL_I2C_Master_Transmit(&hi2c2, BMP280_ADDRESS_WRITE, temp_bmp_buffer_P, 1, 200);
  HAL_I2C_Master_Receive(&hi2c2, BMP280_ADDRESS_READ, temp_bmp_buffer_P,18, 200);

  params.dig_P1   = temp_bmp_buffer_P[0] | temp_bmp_buffer_P[1] << 8;
  params.dig_P2   = (int16_t)temp_bmp_buffer_P[2]  | temp_bmp_buffer_P[3]  << 8;
  params.dig_P3   = (int16_t)temp_bmp_buffer_P[4]  | temp_bmp_buffer_P[5]  << 8;
  params.dig_P4   = (int16_t)temp_bmp_buffer_P[6]  | temp_bmp_buffer_P[7]  << 8;
  params.dig_P5   = (int16_t)temp_bmp_buffer_P[8]  | temp_bmp_buffer_P[9]  << 8;
  params.dig_P6   = (int16_t)temp_bmp_buffer_P[10] | temp_bmp_buffer_P[11] << 8;
  params.dig_P7   = (int16_t)temp_bmp_buffer_P[12] | temp_bmp_buffer_P[13] << 8;
  params.dig_P8   = (int16_t)temp_bmp_buffer_P[14] | temp_bmp_buffer_P[15] << 8;
  params.dig_P9   = (int16_t)temp_bmp_buffer_P[16] | temp_bmp_buffer_P[17] << 8;
}

/**
  * Baslangic offset basinc degeri hesaplar ve basinci hPa turune cevirir
  */
void calibrationBMP280(void)
{
  for(int i=0;i<100;++i)
	  bmp280_pressure_offset += bmp280_readPressure();

  bmp280_pressure_offset /= 100;
  bmp280_pressure_offset /= 100;			// Pa to hPa
}

/**
  * Sicaklik bilgisini okur
  */
float bmp280_readTemperature(void)
{
  int32_t var1, var2;

  uint8_t buffer_temp_data[3];
  buffer_temp_data[0] = BMP280_REGISTER_TEMPDATA;
  HAL_I2C_Master_Transmit(&hi2c2, BMP280_ADDRESS_WRITE, buffer_temp_data, 1, 100);
  HAL_I2C_Master_Receive(&hi2c2, BMP280_ADDRESS_READ, buffer_temp_data,3, 100);

  int32_t adc_T = buffer_temp_data[0];

  adc_T <<= 8;
  adc_T |= buffer_temp_data[1];
  adc_T <<= 8;
  adc_T |= buffer_temp_data[2];

  adc_T >>= 4;

  var1 = ((((adc_T >> 3) - ((int32_t)params.dig_T1 << 1))) * ((int32_t)params.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)params.dig_T1)) * ((adc_T >> 4) - ((int32_t)params.dig_T1))) >> 12) *((int32_t)params.dig_T3)) >> 14;

  temp_fine = var1 + var2;

  T = (temp_fine * 5 + 128) >> 8;

  return (float)T / 100;
}

/**
  * Basinc bilgisini okur
  */
float bmp280_readPressure(void)
{
  int64_t var1, var2, p;

  bmp280_readTemperature();

  uint8_t temp_bmp_buffer_pres_data[3] = {0,0,0};
  temp_bmp_buffer_pres_data[0] = BMP280_REGISTER_PRESSUREDATA;
  HAL_I2C_Master_Transmit(&hi2c2, BMP280_ADDRESS_WRITE, temp_bmp_buffer_pres_data, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c2, BMP280_ADDRESS_READ, temp_bmp_buffer_pres_data,3, HAL_MAX_DELAY);

  int32_t adc_P = temp_bmp_buffer_pres_data[0];

  adc_P <<= 8;
  adc_P |= temp_bmp_buffer_pres_data[1];
  adc_P <<= 8;
  adc_P |= temp_bmp_buffer_pres_data[2];

  adc_P >>= 4;

  var1 = ((int64_t)temp_fine) - 128000;
  var2 = var1 * var1 * (int64_t)params.dig_P6;
  var2 = var2 + ((var1 * (int64_t)params.dig_P5) << 17);
  var2 = var2 + (((int64_t)params.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)params.dig_P3) >> 8) + ((var1 * (int64_t)params.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)params.dig_P1) >> 33;

  if (var1 == 0)
  {
    return 0; // catch exception
  }

  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;

  var1 = (((int64_t)params.dig_P9) * (p >> 13) * (p>>13)) >> 25;
  var2 = (((int64_t)params.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)params.dig_P7) << 4);

  return ((float)p / 256);
}

/**
  * Yuksekligi cm cinsinden dondurur
  */
float bmp280_altitude_cm(void)
{
  bmp280_present_pressure = bmp280_readPressure();
  bmp280_present_pressure /= 100; bmp280_pressure_offset  /= 100;               // Pa to hPa
  return (44330.0*(1-pow((bmp280_present_pressure)/(bmp280_pressure_offset),0.1923F))) * 100;
}

/**
  * Yuksekligi metre cinsinden dondurur
  */
float bmp280_altitude_m(void)
{
   bmp280_present_pressure = bmp280_readPressure();
   bmp280_present_pressure /= 100;
   return (44330 *(1-pow((bmp280_present_pressure)/(bmp280_pressure_offset),0.1923F)));
}

/**
  * Secili I2C adresine bagli hangi cihaz varsa adresini diziye yazar eger yoksa sifir yazar
  */
void Scan_I2C(void)
{
	for(device_adress = 0;device_adress<255;++device_adress)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c2, device_adress, 1, 100) == HAL_OK)
				i2c[device_adress] = device_adress;
		else
		{
			i2c[device_adress] = 0;
		}
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
