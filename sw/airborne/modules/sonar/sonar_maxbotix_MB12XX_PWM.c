#include "modules/sonar/sonar_maxbotix_MB12XX_PWM.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/tim.h>
#include <stm32/misc.h>

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

#include "sys_time.h"

uint16_t sonar_meas_raw;
uint16_t sonar_meas_filtered;

bool_t sonar_data_available;

void SONAR_MAXBOTIX12_IRQ_HANDLER(void);

void
maxbotix12_init(void)
{
  sonar_meas_raw = 0;
  sonar_data_available = FALSE;

  /* TIM3 channel 4 pin (PB1) configuration */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = SONAR_MAXBOTIX12_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SONAR_MAXBOTIX12_GPIO, &GPIO_InitStructure);

  /* TIM5 clock enable */
  RCC_APB1PeriphClockCmd(SONAR_MAXBOTIX12_TIM_PERIPH, ENABLE);

  /* GPIOB clock enable */
  RCC_APB2PeriphClockCmd(SONAR_MAXBOTIX12_GPIO_PERIPH, ENABLE);

  /* Time Base configuration */

  /* The sensor has a pulse duration of 58us per cm.
   * It can measure a range from about 20 cm to 1068 cm, that is a pulsewidth between 1.160 ms and 61.944 ms.
   *
   * Calculation example:
   * Our clock is running at 72 MHz
   * At 16 bits resolution, the maximum pulse we could measure would be
   * (0xffff / 72 000 000) * 1000 = 0.910 ms
   *
   * So our prescaler has to be:
   *  (58 * (10^(-6)) * 1068) / (0xffff / 72 000 000) = 68,054
   *
   *  We don't want it to overflow when it's almost at it's maximum, as that would cause the system to think
   *  the A/C is on the ground.
   *+
   *  So we want prescaler 69. We have to se 68 as value, as the clock is divided by ( 1 + prescaler ).
   *
   *  @TODO make this dependant of the APB_CLK value
   */

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = 0xffff;
  TIM_TimeBaseStructure.TIM_Prescaler = 68;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_InternalClockConfig(SONAR_MAXBOTIX12_TIM);
  TIM_TimeBaseInit(SONAR_MAXBOTIX12_TIM, &TIM_TimeBaseStructure);

  /* TIM3 configuration
   * Input signal is connected to TIM3 CH4 pin (PB1)
   * The Rising edge is used as active edge.
   */
  TIM_ICInitTypeDef TIM_ICInitStructure;
  TIM_ICInitStructure.TIM_Channel = SONAR_MAXBOTIX12_TIM_CHANNEL;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x00;

  /* Initialize PWM Input measurement
   * See page 301 of the STM32F103 reference manual. */
  TIM_PWMIConfig(SONAR_MAXBOTIX12_TIM, &TIM_ICInitStructure);

  /* Enable the TIM3 global Interrupt */
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = SONAR_MAXBOTIX12_IRQ_CHANNEL;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* we can use TI2FP2 because it is bound to TIM channel 2 , which we are using */
  TIM_SelectInputTrigger(SONAR_MAXBOTIX12_TIM, TIM_TS_TI2FP2);

  TIM_SelectSlaveMode(SONAR_MAXBOTIX12_TIM, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(SONAR_MAXBOTIX12_TIM, TIM_MasterSlaveMode_Enable);

  /* TIM3 enable counter */
  TIM_Cmd(SONAR_MAXBOTIX12_TIM, ENABLE);

  /* Enable the IRQ
   * We need CC1 only. CC2 would contain the PWM period (that is, 10Hz)
   */
  TIM_ITConfig(SONAR_MAXBOTIX12_TIM, TIM_IT_CC1, ENABLE);
}

void SONAR_MAXBOTIX12_IRQ_HANDLER(void)
{
  if (TIM_GetITStatus(SONAR_MAXBOTIX12_TIM, TIM_IT_CC1) == SET)
    {
      TIM_ClearITPendingBit(SONAR_MAXBOTIX12_TIM, TIM_IT_CC1);
      int32_t pulse_cnts = TIM_GetCapture1(SONAR_MAXBOTIX12_TIM);
      /* pulse_cnts to actual pulse width in us:
       *   pulse width = pulse_cnts * (prescaler+1)/(actual clock)
       *   with 58us per centimeter, the alt in cm is:
       */
      //int32_t alt_mm = pulse_cnts * 10 * (69/72) / 58;
      //sonar_alt
      DOWNLINK_SEND_INS_REF(DefaultChannel, &pulse_cnts, 0, 0, 0, 0, 0, 0, 0);
    }
}

