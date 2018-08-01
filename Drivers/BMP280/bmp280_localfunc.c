#include "bmp280_localfunc.h"

static void bmp280_spi_stop(void) {
    HAL_GPIO_WritePin(BMP280_CSN_GPIO_Port, BMP280_CSN_Pin, GPIO_PIN_SET);
}

static void bmp280_spi_run(void) {
    HAL_GPIO_WritePin(BMP280_CSN_GPIO_Port, BMP280_CSN_Pin, GPIO_PIN_RESET);
}


int8_t BMP280_SPI_Read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
 HAL_StatusTypeDef res;
 uint8_t rxbuf[32] ={0};
 rxbuf[0] = reg_addr;
  if (len>1) {memcpy(rxbuf + 1,data,len);}
  bmp280_spi_run();
  res = HAL_SPI_Receive(&hspi1, rxbuf, len+1,100);  //(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
  bmp280_spi_stop();
  memcpy(data, rxbuf + 1, len);
  return res;
}

int8_t BMP280_SPI_Write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
 HAL_StatusTypeDef res;
 uint8_t txbuf[16] ={0};
 txbuf[0] = reg_addr;
 if (len>1) {memcpy(txbuf + 1,data,len);}
 bmp280_spi_run();
 res =  HAL_SPI_Transmit(&hspi1, txbuf, len+1, 100);  //(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
 bmp280_spi_stop();
 memcpy(data, txbuf + 1, len);
 return res;
}

void BMP280_SPI_Delay_ms(uint32_t period)
{
 HAL_Delay(period);
}


int8_t  BMP280_Config_and_run(struct bmp280_dev *dev)
{
  int8_t rslt;
  struct bmp280_config conf;


  /* function needed for the library*/
  //bmp280_com_fptr_t spi_read_fptr = &BMP280_SPI_Read;
  //bmp280_com_fptr_t spi_write_fptr = &BMP280_SPI_Write;
  //bmp280_delay_fptr_t delay_ms_fptr = &BMP280_SPI_Delay_ms;

  /* Sensor interface over SPI with native chip select line */
  dev->dev_id   = 0;                    // No need if using SPI
  dev->intf     = BMP280_SPI_INTF;      // SPI config
  dev->spi      = BMP280_SPI_PORT;
  dev->csn_pin  = BMP280_CSN_Pin;
  dev->csn_port = BMP280_CSN_GPIO_Port;
  dev->read     = BMP280_SPI_Read;      //spi_read_fptr;
  dev->write    = BMP280_SPI_Write;     //spi_write_fptr;
  dev->delay_ms = BMP280_SPI_Delay_ms;  //delay_ms_fptr;  
  
  rslt = bmp280_init(dev);
  if (BMP280_OK != rslt) {return BMP280_E_COMM_FAIL;}  /**ERROR HANDLING*/
  rslt = bmp280_get_config(&conf, dev);  
  if (BMP280_OK != rslt) {return BMP280_E_COMM_FAIL;}
  /* Overwrite the desired settings */
  conf.filter   = BMP280_FILTER_COEFF_2;
  conf.os_pres  = BMP280_OS_1X;
  conf.os_temp  = BMP280_OS_1X;
  conf.odr      = BMP280_ODR_62_5_MS;
  rslt = bmp280_set_config(&conf, dev);
  rslt |= bmp280_set_power_mode(BMP280_NORMAL_MODE, dev);  /* Always set the power mode after setting the configuration */    //#define BMP280_NORMAL_MODE    UINT8_C(0x03)
  if (BMP280_OK != rslt) {return BMP280_E_COMM_FAIL;}      /* Check if rslt == BMP280_OK, if not, then handle accordingly */
  return rslt;
}
