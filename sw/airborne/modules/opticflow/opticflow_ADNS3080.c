/** \file opticflow_ADNS3080.h
 *  \brief Handling of ADNS3080 optic flow sensor
 *
 */

/* @TODO:
 * after finishing raw data:
 * compensate for altitude
 * compensate for pitch/roll
 * ignore bad-quality measurements
 */

#include "modules/opticflow/opticflow_ADNS3080.h"
#include "modules/opticflow/opticflow_ADNS3080_srom.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

#include <stm32/gpio.h>
#include <stm32/misc.h>
#include <stm32/rcc.h>
#include <stm32/exti.h>
#include <stm32/spi.h>
#include <stm32/dma.h>

//The CS of the SPI1 header is connected to the Overo.
//So we use the PC4 (SPI1 DRDY) pin as Slave-select for the optic flow sensor.
//Connect the NCS pin of the optic flow breakout to the DRDY pin on the SPI1 header.
#define OfUnselect() GPIOC->BSRR = GPIO_Pin_4
#define OfSelect()   GPIOC->BRR = GPIO_Pin_4


void optflow_ADNS3080_init( void ) {
	optflow_ADNS3080_spi_conf();
}

void optflow_ADNS3080_spi_conf( void ) {
	GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

	//SS config
    //OfUnselect();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    OfSelect();

    /* Enable SPI1 Periph clock -------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Configure GPIOs: SCK (PA5), MISO (PA6) and MOSI(PA7)-----------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO , ENABLE);
	SPI_Cmd(SPI1, ENABLE);

	/* configure SPI */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

	//8 bit MSB first (byte is: 76543210)
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

	//prescaler 32 is 2MHz SCK
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
}

void optflow_ADNS3080_test( void ) {
	uint8_t data1,data2,data3;
	optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_PROD_ID,1,data1);
	data2=0;data3=0;
	DOWNLINK_SEND_OFLOW_DBG(DefaultChannel, &data1,&data2,&data3);
}

void optflow_ADNS3080_readRegister( uint16_t addr, uint8_t numBytes, uint8_t* data) {
	OfSelect();
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, addr);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

	for (int i = 0; i < numBytes; i++) {
		data[i] = SPI_I2S_ReceiveData(SPI1);
	}
    //data1 = 0;
	//SPI_I2S_SendData(SPI1, 0x00);
	//while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	//data2 = SPI_I2S_ReceiveData(SPI1);
    //data2=0;
	//while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	//SPI_I2S_SendData(SPI1, 0x00);
	//while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	//data3 = SPI_I2S_ReceiveData(SPI1);
    //data3=0;
}


///////////////////////////////////////////////////////////////////////
//// STUFF ABOVE HERE SHOULD STILL BE MERGED WITH MY MAVLAB LAPTOP!
///////////////////////////////////////////////////////////////////////


void optflow_ADNS3080_writeRegister(uint8_t addr, uint8_t val) {
  OfSelect();
  //the MSB of the address should be 1, in order to indicate write mode
  SPI_I2S_SendData(SPI1, (addr|(1<<7)));
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, val);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
  //maybe this can be less than 40us, check with scope!
  sys_time_usleep(40);
  OfUnselect();
}


void optflow_ADNS3080_writeSROM(void) {
	//magic sequence for initiating SROM upload
	optflow_ADNS3080_writeRegister(0x20,0x44);
	optflow_ADNS3080_writeRegister(0x23,0x07);
	optflow_ADNS3080_writeRegister(0x24,0x88);

	//We have to wait one frame period. The minimum framerate is 2000 fps, so one frame lasts 0.0005 s =  500us
	//Now to be sure that we wait long enough, we wait 600us instead of 500us.
	sys_time_usleep(600);

	//Write 0x18 to the SROM_ENABLE register to initiate the burst upload
	optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_SROM_ENABLE,0x18);

	//now we pull the CS low, and we should keep it low until the transfer is completed!
	OfSelect();
	//the MSB of the address should be 1, in order to indicate write mode
	SPI_I2S_SendData(SPI1, (OPTFLOW_ADNS3080_ADDR_SROM_LOAD|(1<<7)));
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	for (int i=0; i<1986; i++) {
		SPI_I2S_SendData(SPI1, adns3080_srom[i]);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
		//we have to sleep 10us before writing the next byte, but for safety we sleep 12us
		sys_time_usleep(12);
	}

	//now pull CS high for 10us, to exit the burst mode and return to normal operation
	OfUnselect();
	sys_time_usleep(10);
}

void optflow_ADNS3080_captureFrame(void) {
	uint8_t frame[900];
    //after capturing frames, the module has to be resetted/powercycled before it can resume normal operation!

	//initialize frame capture mode
	optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_FRAMECAP,0x83);

	//We have to wait 3 frame periods + 10us.
	//The minimum framerate is 2000 fps, so one frame lasts 0.0005 s =  500us
	//So we have to wait 3*500+10=1510us
    //Now to be sure that we wait long enough, we wait 1520us
	sys_time_usleep(1520);

	//we can not use the readRegister function to get the image, as we have to keep CS low to stay in burst mode

	//now we pull the CS low, and we should keep it low until the transfer is completed!
	OfSelect();
	SPI_I2S_SendData(SPI1, OPTFLOW_ADNS3080_ADDR_PIX_BURST);

	//maybe do this?
	//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	//SPI_I2S_SendData(SPI1, 0x00);

	//for now, we will send onlyone frame (900 pixels) a time.
	//@todo: change this to read 1 2/3 frame (1537 pixels), to get a higher framerate
	for(int i=0;i<900;i++) {
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
		frame[i] = SPI_I2S_ReceiveData(SPI1);
		//we have to sleep 10us before writing the next byte, but for safety we sleep 12us
		sys_time_usleep(12);
	}

	//now pull CS high for 10us, to stop this read session
	//this does *not* mean that the sensor returns to normal operation!
	//to resume to normal operation after burst captue, the sensor has to be power-cycled!
	OfUnselect();


	sys_time_usleep(10); //can we skip this? does the downlink send action take enough time? @todo

	DOWNLINK_SEND_OFLOW_FRAMECAP(DefaultChannel,&frame);
}
