#include "at45db.h"

uint8_t at45_txbuf_01[528] = {0};
uint8_t at45_txbuf_02[528] = {0};
uint8_t at45_rxbuf[528] = {0};
uint8_t at45_cmdbuf[16] = {0};
uint8_t at45_timeout = 0;

static void at45db_rstn_set(at45db* dev) {
    HAL_GPIO_WritePin(dev->hw_config.rstn_port, dev->hw_config.rstn_pin, GPIO_PIN_SET);
}

static void at45db_rstn_reset(at45db* dev) {
    HAL_GPIO_WritePin(dev->hw_config.rstn_port, dev->hw_config.rstn_pin, GPIO_PIN_RESET);
}

static void at45db_csn_set(at45db* dev) {
    HAL_GPIO_WritePin(dev->hw_config.csn_port, dev->hw_config.csn_pin, GPIO_PIN_SET);
}

static void at45db_csn_reset(at45db* dev) {
    HAL_GPIO_WritePin(dev->hw_config.csn_port, dev->hw_config.csn_pin, GPIO_PIN_RESET);
}

static void at45db_wpn_set(at45db* dev) {
    HAL_GPIO_WritePin(dev->hw_config.csn_port, dev->hw_config.csn_pin, GPIO_PIN_SET);
}

static void at45db_wpn_reset(at45db* dev) {
    HAL_GPIO_WritePin(dev->hw_config.wpn_port, dev->hw_config.wpn_pin, GPIO_PIN_RESET);
}







AT45DB_RESULT at45db_init(at45db* dev) 

{
		#ifndef STM32F407xx
		return AT45DB_INVALID_ARGUMENT;
		#endif
		HAL_StatusTypeDef res;
		
	//Pins
		dev->hw_config.csn_pin 			= AT45_CSN_Pin;
		dev->hw_config.csn_port 		= AT45_CSN_GPIO_Port;
		dev->hw_config.rstn_pin 		= AT45_RESET_Pin;
		dev->hw_config.rstn_port 		= AT45_RESET_GPIO_Port;	
		dev->hw_config.wpn_pin 			= AT45_WP_Pin;
		dev->hw_config.wpn_port 		= AT45_WP_GPIO_Port;
	
	
	
		at45db_rstn_set(dev);
		at45db_wpn_set(dev);
		at45db_csn_set(dev);	
	
	//buffers
		dev->config.rx_buffer 		= (uint8_t*)&at45_rxbuf;
		dev->config.tx_buffer01 	= (uint8_t*)&at45_txbuf_01;
		dev->config.tx_buffer02 	= (uint8_t*)&at45_txbuf_02;
	
	 	for (int i = 0; i<5; i++) {dev->devid[i] = 0x00;}
		
	//Interface
		dev->hw_config.spi        	= AT45DB_SPI_PORT;
		dev->hw_config.spi_timeout 	= AT45DB_SPI_TIMEOUT;
		uint8_t retryleft = 0xFF;			//counter to avoid deadloop if flash decided to go home. Normally only one cycle pass enough
	
	while ((dev->devid[0]!= 0x1F)&(retryleft > 0))
	{
		at45db_getid(dev);
		retryleft--;
	}
  if (!retryleft) return AT45DB_ERROR;
	at45db_getstatus(dev); 
	
	
	
	
	return AT45DB_OK;
}


AT45DB_RESULT at45db_send_cmd(at45db* 				dev, 
															AT45DB_COMMAND 	cmd,
															uint16_t				page,
															uint16_t				byteoffset,
															const uint8_t* 	txbuf,
															uint8_t* 				rxbuf, 
															uint16_t 				datalen, 
															uint8_t 				cmdlen) 				//length of cmd sequence, including adress bytes; typically 5 or 8 (1 byte for opcode, 3 for adress + 1 or 4 dummy). Set to 1 if adress is not used
{
    uint8_t 								myTX[datalen];
    uint8_t 								myRX[datalen];
		uint8_t 								cmdbuf[16];
		HAL_StatusTypeDef 			res;
		uint16_t 								i;
	
	
    cmdbuf[0] = cmd;
				if (cmdlen>3) 																				//probably there is adress needed
						{
							cmdbuf[1] = (page >> 6) & 0x3F;
							cmdbuf[2] = (page << 2) & 0xFC + (byteoffset>>8);
							cmdbuf[3] =  (uint8_t)byteoffset;	
							for (i = 4; i < cmdlen; i++) 
									{
									cmdbuf[i] = 0;
									}
						}

    for (i = 0; i < datalen; i++) 
				{
					myTX[i] = txbuf[i];
					myRX[i] = 0;
				}
				
    at45db_csn_reset(dev);

    res =	 HAL_SPI_Transmit(dev->hw_config.spi, cmdbuf, cmdlen,dev->hw_config.spi_timeout);
		res &= HAL_SPI_TransmitReceive(dev->hw_config.spi, myTX, myRX, datalen,dev->hw_config.spi_timeout);
		at45db_csn_set(dev);				
				if (res != HAL_OK) 
			{
        return AT45DB_ERROR;
			}

    for (i = 0; i < datalen; i++) { rxbuf[i] = myRX[i]; }

    return AT45DB_OK;
}
														

AT45DB_RESULT at45db_getid(at45db* dev) 
	{		
		return  at45db_send_cmd(dev,AT45DB_CMD_DEVID,0,0,dev->devid,dev->devid,sizeof(dev->devid),1);
	}
		
AT45DB_RESULT at45db_getstatus(at45db* dev) 
	{
		AT45DB_RESULT at45_res;
		at45_res = at45db_send_cmd(dev,AT45DB_CMD_STATUS,0,0,&dev->registers.statusreg,&dev->registers.statusreg,sizeof(dev->registers.statusreg),1);
		dev->at45_busy = !(dev->registers.statusreg & AT45DB_RDY);
		if ((dev->registers.statusreg & AT45DB_PAGESIZE)) dev->pagesize = 512; else dev->pagesize = 528;
		return at45_res;
	}
	
AT45DB_RESULT at45db_getsize(at45db* dev) 
	{
		return AT45DB_OK;
	}
		
	
	AT45DB_RESULT at45db_read_page(at45db* dev, uint8_t* rxbuf, uint16_t pageAddr)	
	{
		return at45db_send_cmd(dev,AT45DB_CMD_R_MAINMEMPAGE,pageAddr,0,rxbuf,rxbuf,528,8);
	}	
	

	 // for 512 bytes/page chip address is transfered in form:
        // 000AAAAA AAAAAAAa aaaaaaaa
        // wcmd[1] = (pageAddr >> 7) & 0x1F;
        // wcmd[2] = (pageAddr << 1) & 0xFE;
        // wcmd[3] = 0x00;

        // 00PPPPPP PPPPPPBB BBBBBBBB
       // wcmd[1] = (pageAddr >> 6) & 0x3F;
       // wcmd[2] = (pageAddr << 2) & 0xFC;
        //wcmd[3] = 0x00;
	
	