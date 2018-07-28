#include <stdbool.h>
#include <stdint.h>
#include "bmp280.h"
#include "bmp280_defs.h"

#define BMP280_POWER_MODE BMP280_NORMAL_MODE


int8_t      BMP280_SPI_Read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t      BMP280_SPI_Write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void        BMP280_SPI_Delay_ms(uint32_t period);
