#include "modules/sonar/sonar_maxbotix_MB12XX_PWM.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/tim.h>
#include <stm32/misc.h>

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

#include "sys_time.h"

uint16_t sonar_meas;
bool_t sonar_data_available;


void maxbotix12_inito(void) {
  sonar_meas = 0;
  sonar_data_available = FALSE;

  /* TIM3 channel 4 pin (PB1) configuration */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* TIM5 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /* Time Base configuration */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period        = 0xFFFF;
  //TIM_TimeBaseStructure.TIM_Prescaler     = 0x8;
  //TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

 /* TIM3 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM3 CH4 pin (PB1)
     The Rising edge is used as active edge,
  ------------------------------------------------------------ */
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x00;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);

  /* Enable the TIM3 global Interrupt */
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);

  /* Enable the CC4 Interrupt Request */
  TIM_ITConfig(TIM3, TIM_IT_CC4|TIM_IT_Update, ENABLE);
  LED_TOGGLE(3);
}


void tim3_irq_handler0(void) {

  if(TIM_GetITStatus(TIM3, TIM_IT_CC4) == SET) {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);

    uint32_t now = TIM_GetCapture4(TIM3);
    uint8_t a = now & 0xFF;
    uint8_t b = (now & 0xFF00) >> 7;
    uint8_t c = (now & 0xFF0000) >> 14;
	DOWNLINK_SEND_OFLOW_DATA(DefaultChannel, &a,&b,&c);
  }
  else if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  }
  LED_TOGGLE(2);

}


