/** \file opticflow_ADNS3080.h
 *  \brief Handling of ADNS3080 optic flow sensor
 *
 */

#include "modules/opticflow/opticflow_ADNS3080.h"
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

	//8 bit MSB
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
	optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_PROD_ID,data1);
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
