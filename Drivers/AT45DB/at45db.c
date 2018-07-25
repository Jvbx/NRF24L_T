#include "at45db.h"

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

/*static void at45db_wpn_reset(at45db* dev) {
    HAL_GPIO_WritePin(dev->hw_config.wpn_port, dev->hw_config.wpn_pin, GPIO_PIN_RESET);
}  */







AT45DB_RESULT at45db_init(at45db* dev) 

{
  #ifndef STM32F407xx
  return AT45DB_INVALID_ARGUMENT;
  #endif

  
 //Pins
  dev->hw_config.csn_pin    = AT45_CSN_Pin;
  dev->hw_config.csn_port   = AT45_CSN_GPIO_Port;
  dev->hw_config.rstn_pin   = AT45_RESET_Pin;
  dev->hw_config.rstn_port   = AT45_RESET_GPIO_Port; 
  dev->hw_config.wpn_pin    = AT45_WP_Pin;
  dev->hw_config.wpn_port   = AT45_WP_GPIO_Port;
 
 
 
  at45db_rstn_set(dev);
  at45db_wpn_set(dev);
  at45db_csn_set(dev); 
 
 //buffers
  //dev->config.rx_buffer   = (uint8_t *)&at45_rxbuf;
 
   for (int i = 0; i<5; i++) {dev->devid[i] = 0x00;}
  
 //Interface
  dev->hw_config.spi          = AT45DB_SPI_PORT;
  dev->hw_config.spi_timeout  = AT45DB_SPI_TIMEOUT;
  uint8_t retryleft           = 0xFF;     //counter to avoid deadloop if flash decided to go home. Normally only one cycle pass enough
 
 while ((dev->devid[0]!= 0x1F)&(retryleft))
 {
  at45db_getid(dev);
  retryleft--;
 }
  if (!retryleft) return AT45DB_ERROR;
 
 at45db_getstatus(dev); 
 
 while (at45db_isrdy(dev) != AT45DB_READY) {}
 
  for (uint16_t i = 4000; i<4095; i++) 
  {
   at45db_read_page(dev, at45_rxbuf, i);
  }
 
 return AT45DB_OK;
}


AT45DB_RESULT at45db_send_cmd(at45db*     dev, 
               AT45DB_COMMAND  cmd,
               uint16_t     page,
               uint16_t     byteoffset,
               uint8_t*     txbuf,
               uint8_t*     rxbuf, 
               uint16_t     datalen, 
               uint8_t      cmdlen)     /*length of cmd sequence, including opcode, address and dummy bytes;  
                                          Set to 1 if adress is not needed. 
                                          Page and byteoffset should be set to 0 in that case. */
{
  uint16_t                i;
  uint8_t                 tempbuf[datalen];
  uint8_t                 cmdbuf[cmdlen];
  HAL_StatusTypeDef       res; 
  uint32_t                FlashRequest = 0;
  AT45DB_DATA_DIRECTION   datadir;
 
  
  if (txbuf == NULL) {datadir = AT45DB_R;} else
  if (rxbuf == NULL) {datadir = AT45DB_W;} else 
            {datadir = AT45DB_RW;}
    cmdbuf[0] = cmd;                              //first goes the opcode
      if (cmdlen>1)                              //probably there are address and/or dummy bytes are needed. 
      {
       /*for 528 bytes pagesize address is transfered in form: 
       00PPPPPP PPPPPPBB BBBBBBBB
       where P - page number bit, B - bit of number of byte in page
       for 512 bytes/page chip address is transfered in form:
       000AAAAA AAAAAAAa aaaaaaaa
       where A - page number bit, a - bit of number of byte in page*/
              
       FlashRequest = (page << dev->addrshift) + byteoffset;         
              //dev->addrshift should contain the shift value to form the proper address
              //for 512 page it's decimal 9, for 528 - decimal  10, see address structure in the comment above
       
       cmdbuf[1] = ((FlashRequest & 0xFF0000) >> 16); 
       cmdbuf[2] = ((FlashRequest & 0xFF00) >> 8);
       cmdbuf[3] = FlashRequest;
       if (cmdlen>4) { for (i = 4; i < cmdlen; i++) {cmdbuf[i] = 0;}}  //probably there are some more dummy bytes are needed. for example for direct flash read
         
      }
   if(datadir == AT45DB_W) 
   {  
    for (i = 0; i < datalen; i++) { tempbuf[i] = txbuf[i]; }        
        memcpy(tempbuf, txbuf, datalen);
    for (i = datalen; i < dev->pagesize; i++) { tempbuf[i] = 0; }     
        memset((uint8_t*)&tempbuf + datalen, 0x00, dev->pagesize - datalen);
   }
        
    
    
    at45db_csn_reset(dev);

    res =  HAL_SPI_Transmit(dev->hw_config.spi, cmdbuf, cmdlen, dev->hw_config.spi_timeout);
  if(datadir == AT45DB_W) {res &= HAL_SPI_Transmit(dev->hw_config.spi, tempbuf, datalen,dev->hw_config.spi_timeout);} else
  if(datadir == AT45DB_R) {res &= HAL_SPI_Receive(dev->hw_config.spi,  tempbuf, datalen,dev->hw_config.spi_timeout);} else
  {res &= HAL_SPI_TransmitReceive(dev->hw_config.spi, tempbuf, tempbuf, datalen,dev->hw_config.spi_timeout);}
  at45db_csn_set(dev);    
    if (res != HAL_OK) 
   {
        return AT45DB_ERROR;
   }

    
  if (datadir != AT45DB_W) 
   {
    for (i = 0; i < datalen; i++) {rxbuf[i] = tempbuf[i];}  // if not only writing data, then we need to return what we read
   }

    return AT45DB_OK;
}
              

AT45DB_RESULT at45db_getid(at45db* dev) 
 {  
  return  at45db_send_cmd(dev, AT45DB_CMD_DEVID, 0, 0, NULL, dev->devid, sizeof(dev->devid), 1);
 }
  
AT45DB_RESULT at45db_getstatus(at45db* dev) 
 {
  AT45DB_RESULT at45_res;
  dev->at45_busy = 1;
  at45_res = at45db_send_cmd(dev,AT45DB_CMD_STATUS,0,0,NULL,&dev->registers.statusreg,sizeof(dev->registers.statusreg),1);
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
  dev->chipsize = ((dev->registers.statusreg & AT45DB_SIZE) >> 2);              /*note, that this is not an actual size in bytes, 
                                                                                  just a code, that shold be translated to actual size 
                                                                                   using datashit tables. for 16Mbit is would be 1011...
                                                                                   getting an actual size would be implemented later*/
  return at45_res;
 }
 
 
 AT45DB_RESULT at45db_isrdy(at45db* dev) 
 {
  if (at45db_send_cmd(dev,AT45DB_CMD_STATUS,0,0,NULL,&dev->registers.statusreg,sizeof(dev->registers.statusreg),1) != AT45DB_OK) return AT45DB_ERROR;
  if ((dev->registers.statusreg & AT45DB_RDY) == 0) {dev->at45_busy = 1; return AT45DB_BUSY;}
  dev->at45_busy = 0;
  return AT45DB_READY;
 }
 

 
 
 
 
 
 AT45DB_RESULT at45db_read_page(at45db* dev, uint8_t* rxbuf, uint16_t pageAddr) 
 {
  return at45db_send_cmd(dev,AT45DB_CMD_R_MAINMEMPAGE_LEGACY,pageAddr,0,NULL,rxbuf,dev->pagesize,8);
 } 
 
 
 
 
 AT45DB_RESULT at45db_w_pagethroughbuf1(at45db* dev, uint8_t* txbuf, uint16_t pageAddr, uint16_t byteAddr) 
 {
  return at45db_send_cmd(dev, AT45DB_CMD_W_MEMPAGETHROUGHBUF1, pageAddr, byteAddr, txbuf, NULL, dev->pagesize, 4);
 } 
 
 
 
 
 
  AT45DB_RESULT at45db_w_pagethroughbuf2(at45db* dev, uint8_t* txbuf, uint16_t pageAddr, uint16_t byteAddr) 
 {
  return at45db_send_cmd(dev, AT45DB_CMD_W_MEMPAGETHROUGHBUF2, pageAddr, byteAddr, txbuf, NULL, dev->pagesize, 4);
 } 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 AT45DB_RESULT at45db_sprot_read(at45db* dev) 
 {
  AT45DB_RESULT at45_res;
  uint8_t bytecount = sizeof(dev->registers.lockreg);
  at45_res = at45db_send_cmd(dev, AT45DB_CMD_R_SECTORPROTECTION, 0, 0, NULL, dev->registers.lockreg,bytecount, 4);
  return at45_res;
 }

 
 
 
 
 AT45DB_RESULT at45db_sprot_erase(at45db* dev)      //sector protection erase
 {
  HAL_StatusTypeDef res;
  AT45DB_RESULT   at45_res = AT45DB_OK;
//  uint8_t      *cmdbuf = AT45DB_CMD_SECTORPROTECTIONOFF;
  
  at45db_csn_reset(dev);
    res =  HAL_SPI_Transmit(dev->hw_config.spi, AT45DB_CMD_SECTORPROTECTIONOFF, 4, dev->hw_config.spi_timeout);
  at45db_csn_set(dev);
  if (res != HAL_OK) return AT45DB_ERROR;
  do 
   {
   at45_res = at45db_getstatus(dev);
   }     //waiting for ready bit
  while ((dev->at45_busy)&&(at45_res == AT45DB_OK));
 
 
 
 return at45_res; 
 } 
 
 AT45DB_RESULT at45db_sprot_disable(at45db* dev)      //sector protection erase
 {
  HAL_StatusTypeDef res;
  AT45DB_RESULT   at45_res = AT45DB_OK;
//  uint8_t      *cmdbuf = AT45DB_CMD_SECTORPROTECTIONDISABLE;
  
  at45db_csn_reset(dev);
    res =  HAL_SPI_Transmit(dev->hw_config.spi, AT45DB_CMD_SECTORPROTECTIONDISABLE, 4, dev->hw_config.spi_timeout);
  at45db_csn_set(dev);
  if (res != HAL_OK) return AT45DB_ERROR;
  do 
   {
   at45_res = at45db_getstatus(dev);
   }     //waiting for ready bit
  while ((dev->at45_busy)&&(at45_res == AT45DB_OK));
 
 
 
 return at45_res; 
 }  
 
 AT45DB_RESULT at45db_sprot_program(at45db* dev)      //sector protection register. byte 0 - sector 0, byte 15 - sector 15. 
                                          //0xFF - protection enabled, 0x00 - disabled
 {
   HAL_StatusTypeDef res;
   AT45DB_RESULT   at45_res = AT45DB_OK;
 //  uint8_t      *cmdbuf = AT45DB_CMD_SECTORPROTECTIONON;
   
   
  
   at45db_csn_reset(dev);
   res =  HAL_SPI_Transmit(dev->hw_config.spi, AT45DB_CMD_SECTORPROTECTIONON, 4, dev->hw_config.spi_timeout);
   res &= HAL_SPI_Transmit(dev->hw_config.spi, dev->registers.lockreg, 16, dev->hw_config.spi_timeout);
   at45db_csn_set(dev);
   if (res != HAL_OK) return AT45DB_ERROR;
   do 
    {
    at45_res = at45db_getstatus(dev);
    }     //waiting for ready bit
   while ((dev->at45_busy)&&(at45_res == AT45DB_OK));
 
 
 
 return at45_res; 
 }
 

 AT45DB_RESULT at45db_chiperase(at45db* dev)      //chip erase. 
 {
  HAL_StatusTypeDef res;
  AT45DB_RESULT   at45_res = AT45DB_OK;
  //uint8_t  cmdbuf[4] = AT45DB_CMD_CHIPERASE;
  
  at45db_csn_reset(dev);
    res =  HAL_SPI_Transmit(dev->hw_config.spi, AT45DB_CMD_CHIPERASE, 4, dev->hw_config.spi_timeout);
  at45db_csn_set(dev);
  if (res != HAL_OK) return AT45DB_ERROR;
  do 
   {
   at45_res = at45db_getstatus(dev);
   }     //waiting for ready bit
  while ((dev->at45_busy)&&(at45_res == AT45DB_OK));
   
 return at45_res; 
 }
 

 AT45DB_RESULT at45db_setpage512(at45db* dev)      //set page size equal to 512 bytes. WARNING!!! OPERATION CANNOT BE UNDONE!!! 
 {                                                 //
  HAL_StatusTypeDef res;
  AT45DB_RESULT   at45_res = AT45DB_OK;
  //uint8_t  cmdbuf[4] = AT45DB_CMD_CHIPERASE;
  
  at45db_csn_reset(dev);
    res =  HAL_SPI_Transmit(dev->hw_config.spi, AT45DB_CMD_SETPAGE512, 4, dev->hw_config.spi_timeout);
  at45db_csn_set(dev);
  if (res != HAL_OK) return AT45DB_ERROR;
  do 
   {
   at45_res = at45db_getstatus(dev);
   }     //waiting for ready bit
  while ((dev->at45_busy)&&(at45_res == AT45DB_OK));
  at45db_rstn_reset(dev); 
  HAL_Delay(150);
  at45db_rstn_set(dev); 
 return at45_res; 
 } 
