#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/uart.h"
#include "interrupt_hw.h"
#include "sys_time.h"
#include "downlink.h"
#include "datalink.h"
#include "print.h"
#include "modules/opticflow/opticflow_ADNS3080.h"
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
  //Uart2Init();
  xbee_init();
  optflow_ADNS3080_init();
  mcu_int_enable();
  //Uart2Transmit('a');
  //Uart2Transmit("bbbbbbbbbb");
  DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);
}

static inline void main_periodic( void ) {
	//uint8_t frame[900];
	//frame[0] = 100;
	RunOnceEvery(5000, {DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);});
	//DOWNLINK_SEND_OFLOW_FRAMECAP(DefaultChannel,900,frame);
	RunOnceEvery(200, {optflow_ADNS3080_periodic();});
	//DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);
	//optflow_ADNS3080_periodic();
	//LED_PERIODIC();
}



