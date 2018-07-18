#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "spi.h"

#if defined STM32F100xB
#include "stm32f1xx_hal.h"
#elif defined STM32F407xx
#include "stm32f4xx_hal.h"
#endif


#define AT45DB_PAGE_SIZE   		528					//default. could be switched to 512
#define AT45DB_PAGES   				4096				// at45db161 is 16Mbit chip.
#define AT45DB_SPI_PORT				&hspi2			//spi port, what else can it be? )
#define AT45DB_SPI_TIMEOUT		10					//ftgj!	



#define 	AT45DB_CMD_SECTORPROTECTIONOFF   ((uint8_t []) {0x3D, 0x2A, 0x7F, 0xCF})
#define 	AT45DB_CMD_SECTORPROTECTIONON    ((uint8_t []) {0x3D, 0x2A, 0x7F, 0xFC})
#define 	AT45DB_CMD_CHIPERASE  					 ((uint8_t []) {0xC7, 0x94, 0x80, 0x9A})


/* Registers */


/* Commands */
typedef enum {
    AT45DB_CMD_STATUS        				 = 0xD7,
		AT45DB_CMD_DEVID  							 = 0x9F,	
    AT45DB_CMD_W_BUF1			    	     = 0x84,
		AT45DB_CMD_W_BUF2			   	       = 0x87,	
		AT45DB_CMD_BUF1_TO_MEM_ERASE		 = 0x83,
		AT45DB_CMD_BUF2_TO_MEM_ERASE		 = 0x86,
		AT45DB_CMD_AUTOREWRITEBUF1			 = 0x58,
		AT45DB_CMD_AUTOREWRITEBUF2			 = 0x59,	
		AT45DB_CMD_BUFF1TOMAINMEM				 = 0x88,						//no erase
		AT45DB_CMD_BUFF2TOMAINMEM				 = 0x89,						//no erase
		AT45DB_CMD_PAGEERASE  					 = 0x81,
		AT45DB_CMD_BLOCKERASE  					 = 0x50,	
		AT45DB_CMD_SECTORERASE  				 = 0x7C,	
		AT45DB_CMD_W_MEMPAGETHROUGHBUF1  = 0x82,
		AT45DB_CMD_W_MEMPAGETHROUGHBUF2  = 0x85,		
		AT45DB_CMD_DEEPPWRDOWN					 = 0xB9,
		AT45DB_CMD_DEEPPWRDOWNWAKE			 = 0xAB,	
		AT45DB_CMD_R_CONTARRAY_LEGACY 	 = 0xE8,
		AT45DB_CMD_R_CONTARRAY_HF				 = 0x0B,
		AT45DB_CMD_R_CONTARRAY_LF				 = 0x03,
		AT45DB_CMD_R_MAINMEMPAGE				 = 0xD2,
		AT45DB_CMD_R_BUFF1_LF  					 = 0xD1,
		AT45DB_CMD_R_BUFF2_LF						 = 0xD3,
		AT45DB_CMD_R_BUFF1_HF  					 = 0xD4,
		AT45DB_CMD_R_BUFF2_HF						 = 0xD6,		
		AT45DB_CMD_R_MAINMEMTOBUF1			 = 0x53,
		AT45DB_CMD_R_MAINMEMTOBUF2			 = 0x55,	
		AT45DB_CMD_R_MAINMEMTOBUF1COMP   = 0x60,
		AT45DB_CMD_R_MAINMEMTOBUF2COMP   = 0x61,
		AT45DB_CMD_R_SECTORLOCKDOWN 		 = 0x35,
		AT45DB_CMD_R_SECTORPROTECTION		 = 0x32
} AT45DB_COMMAND;

typedef enum {
    AT45DB_PAGESIZE 	= 0x01,
    AT45DB_PROTECT 		= 0x02,
    AT45DB_SIZE 			= 0x3C,
		AT45DB_COMP 			= 0x40,
		AT45DB_RDY 				= 0x80
} AT45DB_STATUS_REGMASK;



//typedef enum { NRF_CRC_WIDTH_1B = 0, NRF_CRC_WIDTH_2B = 1 } NRF_CRC_WIDTH;

//typedef enum { NRF_STATE_RX = 1, NRF_STATE_TX = 0 } NRF_TXRX_STATE;


typedef enum { AT45DB_OK, AT45DB_ERROR, AT45DB_INVALID_ARGUMENT } AT45DB_RESULT;

typedef struct {

    
    uint8_t* 						rx_buffer;
		uint8_t* 						tx_buffer01;
		uint8_t* 						tx_buffer02;
		
  
} at45db_config;

typedef struct {

    SPI_HandleTypeDef* 	spi;
    uint32_t           	spi_timeout;

    GPIO_TypeDef* 			csn_port;
    uint16_t      			csn_pin;

    GPIO_TypeDef* 			wpn_port;
    uint16_t      			wpn_pin;

    GPIO_TypeDef* 			rstn_port;
    uint16_t      			rstn_pin;

} at45db_hw_config;

typedef struct {

    
					uint8_t 							statusreg;
					uint8_t								lockstatus[16];
  
} at45db_registers;


typedef struct {
					at45db_config 				config;
					at45db_hw_config 			hw_config;
					at45db_registers			registers;
volatile 	uint8_t      					at45_busy;
					uint16_t							curr_page;
					uint16_t							curr_offset;
					uint8_t 							devid[5];	
					uint8_t								rxbuf[528];
					uint16_t							pagesize;
					uint8_t								chipsize;
} at45db;




AT45DB_RESULT at45db_init(at45db* dev); 
AT45DB_RESULT at45db_getid(at45db* dev);
AT45DB_RESULT at45db_getstatus(at45db* dev);
AT45DB_RESULT at45db_read_page(at45db* dev, uint8_t* rxbuf, uint16_t pageAddr);
AT45DB_RESULT at45db_sprot_read(at45db* dev);
