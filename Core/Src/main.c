
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "crc.h"
#include "i2c.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


/* USER CODE BEGIN Includes */
//#include <stdio.h>
#include "nrf24l01.h"
#include "nmea_parser.h"
#include "gnss_utils.h"
#include "at45db.h"
#include "bmp280.h"
#include "mpu9250.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
nrf24l01  nrf;
at45db    dataflash;
struct    bmp280_dev  bmp280;
    char     rxbuf = 0;
    int32_t  temp32 = 0;
    uint32_t pres32 = 0;
    uint32_t pres64 = 0;
    double   temp   = 0;
    double   pres   = 0;
    uint8_t  meas_dur = 0xFF;
    
    
    
     float sum = 0;
uint32_t sumCount = 0;
    
    
    
    
    
    
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
 int fputc(int c, FILE *stream)
{
   return ITM_SendChar(c);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM14_Init();
  MX_SPI2_Init();
  MX_CRC_Init();
  MX_SPI1_Init();
  MX_I2C3_Init();
  MX_RTC_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  at45db_init(&dataflash);
  if (BMP280_Config_and_run(&bmp280) == BMP280_OK) {
 //bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp280);
 //HAL_Delay(1000);




   meas_dur = bmp280_compute_meas_time(&bmp280); 

struct bmp280_uncomp_data ucomp_data;
for (uint8_t i=0; i<10; i++) {
  

int8_t res = 0;
//printf("Measurement duration: %dms\r\n", meas_dur);

/* Loop to read out 10 samples of data */ 
    bmp280.delay_ms(meas_dur); /* Measurement time */
    do {bmp280_get_status(&bmp280.status, &bmp280);}
    while (bmp280.status.im_update); 
    res = bmp280_get_uncomp_data(&ucomp_data, &bmp280);
    if (BMP280_OK != res) {return BMP280_E_COMM_FAIL;}          /* Check if rslt == BMP280_OK, if not, then handle accordingly */

     temp32 = bmp280_comp_temp_32bit(ucomp_data.uncomp_temp, &bmp280);
     pres32 = bmp280_comp_pres_32bit(ucomp_data.uncomp_press, &bmp280);
     pres64 = bmp280_comp_pres_64bit(ucomp_data.uncomp_press, &bmp280);
     temp   = bmp280_comp_temp_double(ucomp_data.uncomp_temp, &bmp280);
     pres   = bmp280_comp_pres_double(ucomp_data.uncomp_press, &bmp280);
    char resString[300];
    sprintf(resString, "UT: %d, UP: %d, T32: %ld, P32: %ld\r\n", \
    ucomp_data.uncomp_temp, ucomp_data.uncomp_press, (long)temp32, (long)pres32);

    printf("UT: %d, UP: %d, T32: %d, P32: %d, P64: %d, P64N: %d, T: %f, P: %f\r\n", \
      ucomp_data.uncomp_temp, ucomp_data.uncomp_press, temp32, \
      pres32, pres64, pres64 / 256, temp, pres);
}
 }

  uint8_t sprf_buf[255] = {0};
 
  uint8_t whoami = mpu9250_readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  sprintf((char*)sprf_buf,"I AM 0x%x\n\r", whoami);
 // for (uint8_t i = 0; i<255; i++) {sprf_buf[i] = sprf_buf[i] - 0x40;}
  printf("%s",sprf_buf);
  sprintf((char*)sprf_buf,"I SHOULD BE 0x71\n\r");
  printf("%s",sprf_buf);
  mpu9250_initconst();
  if (whoami == 0x71) // WHO_AM_I should always be 0x68
  {  
    printf("MPU9250 is online...\n\r");
    HAL_Delay(1000);//wait(1);
    mpu9250_reset(); // Reset registers to default in preparation for device calibration
    mpu9250_calibrate(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    HAL_Delay(2000);//wait(2);
    mpu9250_init(); 
    printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    mpu9250_initAK8963(magCalibration);
    printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
    printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
    printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));
    if(Mscale == 0) printf("Magnetometer resolution = 14  bits\n\r");
    if(Mscale == 1) printf("Magnetometer resolution = 16  bits\n\r");
    if(Mmode == 2) printf("Magnetometer ODR = 8 Hz\n\r");
    if(Mmode == 6) printf("Magnetometer ODR = 100 Hz\n\r");
    HAL_Delay(2000);//wait(2);
   }
   else
   {
    printf("Could not connect to MPU9250: \n\r");
    printf("%#x \n",  whoami);
 
    //while(1) ; // Loop forever if communication doesn't happen
    }

    mpu9250_getAres(); // Get accelerometer sensitivity
    mpu9250_getGres(); // Get gyro sensitivity
    mpu9250_getMres(); // Get magnetometer sensitivity
    printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/aRes);
    printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
    printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mRes);
    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

    
    
    

















 
 
 HAL_TIM_Base_Start_IT(&htim14);
  nrf_startup(&nrf);
 
 

 
 //char tx_data1[16] = {0};
 
 uint8_t tx_data[16] = {0};
 //sprintf((char *)tx_data, "abcdefghijklmnoszxABCDEFCBDA");
 HAL_UART_Receive_IT(&huart3, (uint8_t*)&rxbuf, 1); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
 
 while (!DataDone)
 {
 }  
 //sprintf((char *)tx_data, (char *)&SLatitude); 
 //printf("data = 0x%04X\r\n",(char *)tx_data); 
 nrf_send_packet(&nrf,tx_data);
 
 //sprintf((char *)tx_data, (char *)&SLongitude);
 nrf_send_packet(&nrf,tx_data);
 //HAL_Delay(250);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void mpu9250_irq_handler(void)
{
 if(mpu9250_readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt

    mpu9250_readAccelData(accelCount);  // Read the x/y/z adc values   
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2];  
   
    mpu9250_readGyroData(gyroCount);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes - gyroBias[2];   
  
    mpu9250_readMagData(magCount);  // Read the x/y/z adc values   
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
    mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];   
  }
   
    Now = HAL_GetTick();
    deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    
    sum += deltat;
    sumCount++;
    
//    if(lastUpdate - firstUpdate > 10000000.0f) {
//     beta = 0.04;  // decrease filter gain after stabilized
//     zeta = 0.015; // increasey bias drift gain after stabilized
 //   }
    
   // Pass gyro rate as rad/s
 mpu9250_MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
 // mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = HAL_GetTick() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

    printf("ax = %f", 1000*ax); 
    printf(" ay = %f", 1000*ay); 
    printf(" az = %f  mg\n\r", 1000*az); 

    printf("gx = %f", gx); 
    printf(" gy = %f", gy); 
    printf(" gz = %f  deg/s\n\r", gz); 
    
    printf("gx = %f", mx); 
    printf(" gy = %f", my); 
    printf(" gz = %f  mG\n\r", mz); 
    
    tempCount = mpu9250_readTempData();  // Read the adc values
    temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade
    printf(" temperature = %f  C\n\r", temperature); 
    
    printf("q0 = %f\n\r", q[0]);
    printf("q1 = %f\n\r", q[1]);
    printf("q2 = %f\n\r", q[2]);
    printf("q3 = %f\n\r", q[3]);      
    
  //  lcd.clear();
  //  lcd.printString("MPU9250", 0, 0);
  //  lcd.printString("x   y   z", 0, 1);
  //  lcd.setXYAddress(0, 2); lcd.printChar((char)(1000*ax));
  // lcd.setXYAddress(20, 2); lcd.printChar((char)(1000*ay));
  //  lcd.setXYAddress(40, 2); lcd.printChar((char)(1000*az)); lcd.printString("mg", 66, 2);
    
    
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;

    printf("Yaw, Pitch, Roll: %f %f %f\n\r", yaw, pitch, roll);
    printf("average rate = %f\n\r", (float) sumCount/sum);
 
    count = HAL_GetTick(); 
    sum = 0;
    sumCount = 0; 
}
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)

{

 if(GPIO_Pin== NRF_IRQ_Pin)
  {

    nrf_irq_handler(&nrf);
  } 
 else if(GPIO_Pin== MPU9250_INT_Pin)
  {
    mpu9250_irq_handler();
    __NOP();

  }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

 
 
 HAL_UART_Receive_IT(&huart3, (uint8_t*)&rxbuf, 1);
 
 nmea_getmessage((uint8_t*)&rxbuf);  
 NMEA_Parser(rxbuf);
  
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)

{
 nrf24_timeout = 1;
 HAL_TIM_Base_MspDeInit(&htim14);
 
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
