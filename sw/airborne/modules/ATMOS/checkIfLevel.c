#include "led.h"
#include "subsystems/ahrs.h"
#define CHIL_MARGIN_PHI 400
#define CHIL_MARGIN_THETA 1000

void atmov_checkIfLevel_init(void) {
  LED_ON(2);
}

void atmov_checkIfLevel_periodic(void) {
  if(ahrs.ltp_to_lift_euler.phi >  CHIL_MARGIN_PHI ||
     ahrs.ltp_to_lift_euler.phi < -CHIL_MARGIN_PHI ||
     ahrs.ltp_to_lift_euler.theta > CHIL_MARGIN_THETA ||
     ahrs.ltp_to_lift_euler.theta < -CHIL_MARGIN_THETA) {
     RunOnceEvery(2, LED_TOGGLE(2));
  }
  else {
      LED_OFF(2);
  }
}
