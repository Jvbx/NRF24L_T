#include "at45db.h"

uint8_t at45_txbuf_01[528] = {0};
uint8_t at45_txbuf_02[528] = {0};
uint8_t at45_rxbuf[528] = {0};

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
		dev->config.rx_buffer 		= (uint8_t *)&at45_rxbuf;
		dev->config.tx_buffer01 	= (uint8_t *)&at45_txbuf_01;
		dev->config.tx_buffer02 	= (uint8_t *)&at45_txbuf_02;
	
	 	for (int i = 0; i<5; i++) {dev->devid[i] = 0x00;}
		
	//Interface
		dev->hw_config.spi        	= AT45DB_SPI_PORT;
		dev->hw_config.spi_timeout 	= AT45DB_SPI_TIMEOUT;
		uint8_t retryleft = 0xFF;			//counter to avoid deadloop if flash decided to go home. Normally only one cycle pass enough
	
	while ((dev->devid[0]!= 0x1F)&(retryleft))
	{
		at45db_getid(dev);
		retryleft--;
	}
  if (!retryleft) return AT45DB_ERROR;
	
	at45db_getstatus(dev); 
	at45db_sprot_read(dev);
	//at45db_sprot_disable(dev);
	//at45db_sprot_erase(dev);
	//at45db_sprot_read(dev);
	//at45db_chiperase(dev);

		for (uint16_t i = 4000; i<4095; i++) 
		{
			at45db_read_page(dev, at45_txbuf_02, i);
		}
	
	return AT45DB_OK;
}


AT45DB_RESULT at45db_send_cmd(at45db* 				dev, 
															AT45DB_COMMAND 	cmd,
															uint16_t				page,
															uint16_t				byteoffset,
															uint8_t* 				txbuf,
															uint8_t* 				rxbuf, 
															uint16_t 				datalen, 
															uint8_t 				cmdlen) 				//length of cmd sequence, including opcode, address and dummy bytes;  Set to 1 if adress is not needed. Page and byteoffset should be set to 0 in that case.
{
		uint16_t 								i;
		uint8_t 								cmdbuf[8];
		HAL_StatusTypeDef 			res;	
	  uint32_t FlashRequest = 0;

	
    cmdbuf[0] = cmd;																					//first goes the opcode
		FlashRequest = FlashRequest + (page << dev->addrshift) + byteoffset;
	 				if (cmdlen>1) 																			//probably there address or dummy bytes are needed. 
						{
							/*for 528 bytes pagesize address is transfered in form: 
							00PPPPPP PPPPPPBB BBBBBBBB
							where P - page number bit, B - bit of number of byte in page
							for 512 bytes/page chip address is transfered in form:
							000AAAAA AAAAAAAa aaaaaaaa
							where A - page number bit, a - bit of number of byte in page*/
							cmdbuf[1] = ((FlashRequest & 0xFF0000) >> 16); 
							cmdbuf[2] = ((FlashRequest & 0xFF00) >> 8);
							cmdbuf[3] = FlashRequest;
							for (i = 4; i < cmdlen; i++) 
									{
									cmdbuf[i] = 0;									
									}
						}

   for (i = 0; i < datalen; i++) 
				{
					dev->rtxbuf[i] = txbuf[i];	
				}
		
   for (i = datalen; i < dev->pagesize; i++) 
				{
					dev->rtxbuf[i] = txbuf[i];	
				}
								
				
				
    at45db_csn_reset(dev);

    res =	 HAL_SPI_Transmit(dev->hw_config.spi, cmdbuf, cmdlen,dev->hw_config.spi_timeout);
		res &= HAL_SPI_TransmitReceive(dev->hw_config.spi, dev->rtxbuf, dev->rtxbuf, datalen,dev->hw_config.spi_timeout);	
		at45db_csn_set(dev);				
				if (res != HAL_OK) 
			{
        return AT45DB_ERROR;
			}

    for (i = 0; i < datalen; i++) 
			{ 
				rxbuf[i] = dev->rtxbuf[i]; 
			}

    return AT45DB_OK;
}
														

AT45DB_RESULT at45db_getid(at45db* dev) 
	{		
		return  at45db_send_cmd(dev, AT45DB_CMD_DEVID, 0, 0, dev->devid, dev->devid, sizeof(dev->devid), 1);
	}
		
AT45DB_RESULT at45db_getstatus(at45db* dev) 
	{
		AT45DB_RESULT at45_res;
		dev->at45_busy = 1;
		at45_res = at45db_send_cmd(dev,AT45DB_CMD_STATUS,0,0,&dev->registers.statusreg,&dev->registers.statusreg,sizeof(dev->registers.statusreg),1);
		if ((dev->registers.statusreg & AT45DB_RDY) != 0) {dev->at45_busy = 0;}
		if ((dev->registers.statusreg & AT45DB_PAGESIZE)) 
					{
						dev->pagesize = 512;
						dev->addrshift = 9;
					} 
		else 
					{
						dev->pagesize = 528;
						dev->addrshift = 10;
					}
		dev->chipsize = ((dev->registers.statusreg & AT45DB_SIZE)>>2);
		return at45_res;
	}
	
AT45DB_RESULT at45db_getsize(at45db* dev) 
	{
		return AT45DB_OK;
	}
		
	
	AT45DB_RESULT at45db_read_page(at45db* dev, uint8_t* rxbuf, uint16_t pageAddr)	
	{
		return at45db_send_cmd(dev,AT45DB_CMD_R_MAINMEMPAGE_LEGACY,pageAddr,0,rxbuf,rxbuf,dev->pagesize,8);
	}	
	
	AT45DB_RESULT at45db_write_page(at45db* dev, uint8_t* rxbuf, uint16_t pageAddr)	
	{
		return at45db_send_cmd(dev,AT45DB_CMD_R_MAINMEMPAGE_LEGACY,pageAddr,0,rxbuf,rxbuf,dev->pagesize,8);
	}	
	
	
	
	
	AT45DB_RESULT at45db_sprot_read(at45db* dev) 
	{
		AT45DB_RESULT at45_res;
		uint8_t bytecount = sizeof(dev->registers.lockreg);
		at45_res = at45db_send_cmd(dev,AT45DB_CMD_R_SECTORPROTECTION,0,0,dev->registers.lockreg,dev->registers.lockreg,bytecount,4);
		return at45_res;
	}

	
	
	
	
	AT45DB_RESULT at45db_sprot_erase(at45db* dev) 					//sector protection erase
	{
		HAL_StatusTypeDef res;
		AT45DB_RESULT 		at45_res = AT45DB_OK;
		uint8_t 					*cmdbuf = AT45DB_CMD_SECTORPROTECTIONOFF;
		
		at45db_csn_reset(dev);
    res =	 HAL_SPI_Transmit(dev->hw_config.spi, cmdbuf, 4, dev->hw_config.spi_timeout);
		at45db_csn_set(dev);
		if (res != HAL_OK) return AT45DB_ERROR;
		do 
			{
			at45_res = at45db_getstatus(dev);
			} 				//waiting for ready bit
		while ((dev->at45_busy)&&(at45_res == AT45DB_OK));
	
	
	
	return at45_res; 
	}	
	
	AT45DB_RESULT at45db_sprot_disable(at45db* dev) 					//sector protection erase
	{
		HAL_StatusTypeDef res;
		AT45DB_RESULT 		at45_res = AT45DB_OK;
		uint8_t 					*cmdbuf = AT45DB_CMD_SECTORPROTECTIONDISABLE;
		
		at45db_csn_reset(dev);
    res =	 HAL_SPI_Transmit(dev->hw_config.spi, cmdbuf, 4, dev->hw_config.spi_timeout);
		at45db_csn_set(dev);
		if (res != HAL_OK) return AT45DB_ERROR;
		do 
			{
			at45_res = at45db_getstatus(dev);
			} 				//waiting for ready bit
		while ((dev->at45_busy)&&(at45_res == AT45DB_OK));
	
	
	
	return at45_res; 
	}		
	
	AT45DB_RESULT at45db_sprot_program(at45db* dev) 					//sector protection register. byte 0 - sector 0, byte 15 - sector 15. 
																																										//0xFF - protection enabled, 0x00 - disabled
	{
			HAL_StatusTypeDef res;
			AT45DB_RESULT 		at45_res = AT45DB_OK;
			uint8_t 					*cmdbuf = AT45DB_CMD_SECTORPROTECTIONON;
		 
			
		
			at45db_csn_reset(dev);
			res =	 HAL_SPI_Transmit(dev->hw_config.spi, cmdbuf, 4, dev->hw_config.spi_timeout);
			res &= HAL_SPI_Transmit(dev->hw_config.spi,dev->registers.lockreg, 16, dev->hw_config.spi_timeout);
			at45db_csn_set(dev);
			if (res != HAL_OK) return AT45DB_ERROR;
			do 
				{
				at45_res = at45db_getstatus(dev);
				} 				//waiting for ready bit
			while ((dev->at45_busy)&&(at45_res == AT45DB_OK));
	
	
	
	return at45_res; 
	}
	

	AT45DB_RESULT at45db_chiperase(at45db* dev) 					//chip erase
	{
		HAL_StatusTypeDef res;
		AT45DB_RESULT 		at45_res = AT45DB_OK;
		//uint8_t  cmdbuf[4] = AT45DB_CMD_CHIPERASE;
		
		at45db_csn_reset(dev);
    res =	 HAL_SPI_Transmit(dev->hw_config.spi,  AT45DB_CMD_CHIPERASE, 4, dev->hw_config.spi_timeout);
		at45db_csn_set(dev);
		if (res != HAL_OK) return AT45DB_ERROR;
		do 
			{
			at45_res = at45db_getstatus(dev);
			} 				//waiting for ready bit
		while ((dev->at45_busy)&&(at45_res == AT45DB_OK));
			
	return at45_res; 
 }
	
	