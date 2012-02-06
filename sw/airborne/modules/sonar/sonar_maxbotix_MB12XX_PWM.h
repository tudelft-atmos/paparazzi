/** \file sonar_maxbotix_MB12XX_PWM.h
 *
 * Driver for reading PWM pulses of the maxbotix MB12xx serie
 *
 * The MB12xx modules have 3 ways of outputing data
 * 1 PPM
 * 2 Serial/UART
 * 3 Analog
 *
 * We can not use option 3, as we want to feed the sensor with 5V. The ADCs at the Lisa/L have a V_ref
 * of 3.3V, so that doesn't work.
 *
 * Option 2 takes a UART port. But when we would want to connect multiple range finders,
 * we would quickly run out of UARTs.
 *
 * So... option 1.
 */

#ifndef SONAR_MAXBOTIX12_H
#define SONAR_MAXBOTIX12_H

/* Possible ports to connect the sensor to:
 * (We need a TIM channel)
 *
 * UART1_TRIG:  TIM2CH2 on PA1
 * UART2_TX:	TIM2CH3 on PA2
 * UART2_RX:	TIM2CH4 on PA3
 * SPI1_MISO:	TIM3CH1 on PA6
 * SPI1_MOSI:	TIM3CH2 on PA7
 * SERVO1:	TIM8CH1 on PC6
 * SERVO2:	TIM8CH2 on PC7
 * SERVO3:	TIM8CH3 on PC8
 * SERVO4:	TIM8CH4 on PC9
 * SERVO5:	TIM4CH3 on PB8
 * SERVO6:	TIM4CH4 on PB9
 * SPARE_PB0:	TIM3CH3 on PB0
 * SPARE_PB1:	TIM3CH4 on PB1
 */

#define SONAR_MAXBOTIX12_PORT_SPI1_MOSI

#ifdef SONAR_MAXBOTIX12_PORT_UART1_TRIG
#define SONAR_MAXBOTIX12_GPIO 		GPIOA
#define SONAR_MAXBOTIX12_GPIO_PIN 	GPIO_Pin_1
#define SONAR_MAXBOTIX12_TIM_PERIPH	RCC_APB1Periph_TIM2
#define SONAR_MAXBOTIX12_GPIO_PERIPH 	RCC_APB2Periph_GPIOA
#define SONAR_MAXBOTIX12_TIM		TIM2
#define SONAR_MAXBOTIX12_TIM_CHANNEL	TIM_Channel_2
#define SONAR_MAXBOTIX12_IRQ_HANDLER	tim2_irq_handler
#define SONAR_MAXBOTIX12_IRQ_CHANNEL    TIM2_IRQn
#ifndef USE_TIM2_IRQ
#error Please add -DUSE_TIM2_IRQ to your CFLAGS
#endif
#endif

#ifdef SONAR_MAXBOTIX12_PORT_SPARE_PB0
#define SONAR_MAXBOTIX12_GPIO 		GPIOB
#define SONAR_MAXBOTIX12_GPIO_PIN 	GPIO_Pin_0
#define SONAR_MAXBOTIX12_TIM_PERIPH	RCC_APB1Periph_TIM3
#define SONAR_MAXBOTIX12_GPIO_PERIPH 	RCC_APB2Periph_GPIOB
#define SONAR_MAXBOTIX12_TIM		TIM3
#define SONAR_MAXBOTIX12_TIM_CHANNEL	TIM_Channel_3
#define SONAR_MAXBOTIX12_IRQ_HANDLER	tim3_irq_handler
#define SONAR_MAXBOTIX12_IRQ_CHANNEL    TIM3_IRQn
#ifndef USE_TIM3_IRQ
#error Please add -DUSE_TIM3_IRQ to your CFLAGS
#endif
#endif

#ifdef SONAR_MAXBOTIX12_PORT_SPARE_PB1
#define SONAR_MAXBOTIX12_GPIO 		GPIOB
#define SONAR_MAXBOTIX12_GPIO_PIN 	GPIO_Pin_1
#define SONAR_MAXBOTIX12_TIM_PERIPH	RCC_APB1Periph_TIM3
#define SONAR_MAXBOTIX12_GPIO_PERIPH 	RCC_APB2Periph_GPIOB
#define SONAR_MAXBOTIX12_TIM		TIM3
#define SONAR_MAXBOTIX12_TIM_CHANNEL	TIM_Channel_4
#define SONAR_MAXBOTIX12_IRQ_HANDLER	tim3_irq_handler
#define SONAR_MAXBOTIX12_IRQ_CHANNEL    TIM3_IRQn
#ifndef USE_TIM3_IRQ
#error Please add -DUSE_TIM3_IRQ to your CFLAGS
#endif
#endif

#ifdef SONAR_MAXBOTIX12_PORT_SPI1_MOSI
#define SONAR_MAXBOTIX12_GPIO           GPIOA
#define SONAR_MAXBOTIX12_GPIO_PIN       GPIO_Pin_7
#define SONAR_MAXBOTIX12_TIM_PERIPH     RCC_APB1Periph_TIM3
#define SONAR_MAXBOTIX12_GPIO_PERIPH    RCC_APB2Periph_GPIOA
#define SONAR_MAXBOTIX12_TIM            TIM3
#define SONAR_MAXBOTIX12_TIM_CHANNEL    TIM_Channel_2
#define SONAR_MAXBOTIX12_IRQ_HANDLER    tim3_irq_handler
#define SONAR_MAXBOTIX12_IRQ_CHANNEL    TIM3_IRQn
#ifndef USE_TIM3_IRQ
#error Please add -DUSE_TIM3_IRQ to your CFLAGS
#endif
#endif

#ifdef SONAR_MAXBOTIX12_PORT_SERVO2
#error TIM8 is not supported yet, due to the different clock
//@FIXME
#define SONAR_MAXBOTIX12_GPIO           GPIOC
#define SONAR_MAXBOTIX12_GPIO_PIN       GPIO_Pin_7
#define SONAR_MAXBOTIX12_TIM_PERIPH     RCC_APB2Periph_TIM8
#define SONAR_MAXBOTIX12_GPIO_PERIPH    RCC_APB2Periph_GPIOC
#define SONAR_MAXBOTIX12_TIM            TIM8
#define SONAR_MAXBOTIX12_TIM_CHANNEL    TIM_Channel_2
#define SONAR_MAXBOTIX12_IRQ_HANDLER    tim8_irq_handler
#define SONAR_MAXBOTIX12_IRQ_CHANNEL    TIM8_IRQn
#ifndef USE_TIM8_IRQ
#error Please add -DUSE_TIM8_IRQ to your CFLAGS
#endif
#endif


#ifdef SONAR_MAXBOTIX12_PORT_UART2_TX
#error "Please add the Maxbotix12 port to the sonar_maxbotix_MB12XX_PWM.h file."
#endif

#ifdef SONAR_MAXBOTIX12_PORT_UART2_EX
#error "Please add the Maxbotix12 port to the sonar_maxbotix_MB12XX_PWM.h file."
#endif

#ifdef SONAR_MAXBOTIX12_PORT_SPI1_MISO
#error "Please add the Maxbotix12 port to the sonar_maxbotix_MB12XX_PWM.h file."
#endif

#ifdef SONAR_MAXBOTIX12_PORT_SERVO1
#error "Please add the Maxbotix12 port to the sonar_maxbotix_MB12XX_PWM.h file."
#endif

#ifdef SONAR_MAXBOTIX12_PORT_SERVO3
#error "Please add the Maxbotix12 port to the sonar_maxbotix_MB12XX_PWM.h file."
#endif

#ifdef SONAR_MAXBOTIX12_PORT_SERVO4
#error "Please add the Maxbotix12 port to the sonar_maxbotix_MB12XX_PWM.h file."
#endif

#ifdef SONAR_MAXBOTIX12_PORT_SERV05
#error "Please add the Maxbotix12 port to the sonar_maxbotix_MB12XX_PWM.h file."
#endif

#ifdef SONAR_MAXBOTIX12_PORT_SERVO6
#error "Please add the Maxbotix12 port to the sonar_maxbotix_MB12XX_PWM.h file."
#endif

/** parameters for the 2nd order low-pass filter
 * calculated in MATLAB using
 *   [num,den] = butter(2,(cutoff_freq/(sample_freq/2)))
 */
// values for cutoff_freq = 4Hz and sample_freq = 10Hz
#define ACCEL_BUTTER_NUM_1 +0.638945525159022
#define ACCEL_BUTTER_NUM_2 +1.277891050318045
#define ACCEL_BUTTER_NUM_3 +0.638945525159022
//warning, ACCEL_BUTTER_DEN_1 is always one for this filter, so it is omitted here.
#define ACCEL_BUTTER_DEN_2 +1.142980502539901
#define ACCEL_BUTTER_DEN_3 +0.412801598096189

#include "std.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

extern uint16_t sonar_meas;

extern bool_t sonar_data_available;

extern void
maxbotix12_init(void);
//extern void maxbotix12_read(void);
//void tim3_irq_handler(void);

#define SonarEvent(_handler) { \
  if (sonar_data_available) { \
    _handler(); \
    sonar_data_available = FALSE; \
  } \
}

#endif
