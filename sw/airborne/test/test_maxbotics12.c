#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/uart.h"
#include "interrupt_hw.h"
#include "sys_time.h"
#include "downlink.h"
#include "datalink.h"
#include "print.h"
#include "modules/sonar/sonar_maxbotix_MB12XX_PWM.h"
static inline void main_init( void );
static inline void main_periodic( void );

int main(void) {

  main_init();

  while (1) {
    if (sys_time_periodic())
      main_periodic();
  }
  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_init();
  sys_time_usleep(9000);
  xbee_init();
  sys_time_usleep(9000);

  DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);
  maxbotix12_init();

  mcu_int_enable();
}

static inline void main_periodic( void ) {
	//uint8_t frame[900];
	//frame[0] = 100;
	RunOnceEvery(256, {DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);});
	//RunOnceEvery(1000, {LED_TOGGLE(2);});
	//DOWNLINK_SEND_OFLOW_FRAMECAP(DefaultChannel,900,frame);
	//RunOnceEvery(800, {optflow_ADNS3080_periodic();});
	//DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);
	//optflow_ADNS3080_periodic();

}



