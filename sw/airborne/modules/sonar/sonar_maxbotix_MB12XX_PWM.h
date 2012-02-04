/** \file sonar_maxbotix_MB12XX_PWM.h
 *
 * driver for reading PWM pulses of the maxbotix MB12xx serie
 */

#ifndef SONAR_MAXBOTIX12_H
#define SONAR_MAXBOTIX12_H

#include "std.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

extern uint16_t sonar_meas;

extern bool_t sonar_data_available;

extern void maxbotix12_init(void);
//extern void maxbotix12_read(void);
void tim3_irq_handler(void);

//#include "subsystems/ins.h" // needed because ins is not a module

#define SonarEvent(_handler) { \
  if (sonar_data_available) { \
    _handler(); \
    sonar_data_available = FALSE; \
  } \
}

#endif
