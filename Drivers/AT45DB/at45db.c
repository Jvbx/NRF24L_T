#include "at45db.h"

uint8_t at45_txbuf_01[528] = {0};
uint8_t at45_txbuf_02[528] = {0};
uint8_t at45_rxbuf[528] = {0};
uint8_t at45_timeout = 0;

static void at45db_rstn_set(at45db* dev) {
    HAL_GPIO_WritePin(dev->config.rstn_port, dev->config.rstn_pin, GPIO_PIN_SET);
}

static void at45db_rstn_reset(at45db* dev) {
    HAL_GPIO_WritePin(dev->config.rstn_port, dev->config.rstn_pin, GPIO_PIN_RESET);
}

static void at45db_csn_set(at45db* dev) {
    HAL_GPIO_WritePin(dev->config.csn_port, dev->config.csn_pin, GPIO_PIN_SET);
}

static void at45db_csn_reset(at45db* dev) {
    HAL_GPIO_WritePin(dev->config.csn_port, dev->config.csn_pin, GPIO_PIN_RESET);
}

static void at45db_wpn_set(at45db* dev) {
    HAL_GPIO_WritePin(dev->config.csn_port, dev->config.csn_pin, GPIO_PIN_SET);
}

static void at45db_wpn_reset(at45db* dev) {
    HAL_GPIO_WritePin(dev->config.wpn_port, dev->config.wpn_pin, GPIO_PIN_RESET);
}







AT45DB_RESULT at45db_init(at45db* dev) 

{
		#ifndef STM32F407xx
		return AT45DB_INVALID_ARGUMENT;
		#endif
		HAL_StatusTypeDef res;
		
	//Pins
		dev->config.csn_pin 			= AT45_CSN_Pin;
		dev->config.csn_port 		= AT45_CSN_GPIO_Port;
		dev->config.rstn_pin 		= AT45_RESET_Pin;
		dev->config.rstn_port 		= AT45_RESET_GPIO_Port;	
		dev->config.wpn_pin 			= AT45_WP_Pin;
		dev->config.wpn_port 		= AT45_WP_GPIO_Port;
		at45db_rstn_set(dev);
		at45db_wpn_set(dev);
		at45db_csn_set(dev);	
	
	//buffers
		dev->config.rx_buffer 		= (uint8_t*)&at45_rxbuf;
		dev->config.tx_buffer01 	= (uint8_t*)&at45_txbuf_01;
		dev->config.tx_buffer02 	= (uint8_t*)&at45_txbuf_02;
	
	 	for (int i = 0; i<5; i++) {dev->config.devid[i] = 0x00;}
		
	//Interface
		dev->config.spi         = &hspi2;
		dev->config.spi_timeout = 10;
		

		
		
	while ( dev->config.devid[0]!= 0x1F )
	{
		at45db_getid(dev);
	}
    
	return AT45DB_OK;
}


AT45DB_RESULT at45db_send_cmd(at45db* dev, AT45DB_COMMAND cmd, const uint8_t* tx,
                            uint8_t* rx, uint8_t len) {
    uint8_t myTX[len + 1];
    uint8_t myRX[len + 1];
    myTX[0] = cmd;

    int i = 0;
    for (i = 0; i < len; i++) 
				{
					myTX[1 + i] = tx[i];
					myRX[i] = 0;
				}
				
    at45db_csn_reset(dev);

    if (HAL_SPI_TransmitReceive(dev->config.spi, myTX, myRX, 1 + len,
                                dev->config.spi_timeout) != HAL_OK) {
        return AT45DB_ERROR;
    }

    for (i = 0; i < len; i++) { rx[i] = myRX[1 + i]; }

    at45db_csn_set(dev);

    return AT45DB_OK;
}
														

AT45DB_RESULT at45db_getid(at45db* dev) 
	{
		HAL_StatusTypeDef res;
		uint8_t buf[5] = {0};
				
		
		at45db_csn_reset(dev);
		buf[0] = AT45DB_CMD_DEVID;
		res =	HAL_SPI_Transmit(dev->config.spi, buf, 1, dev->config.spi_timeout);
		res	&= HAL_SPI_Receive(dev->config.spi, buf, 5, dev->config.spi_timeout);
		at45db_csn_set(dev);
		if (res!=HAL_OK) {return AT45DB_ERROR;}
		for (int i = 0; i<5; i++) {dev->config.devid[i] = buf[i];}
		return AT45DB_OK;
	}
		
		