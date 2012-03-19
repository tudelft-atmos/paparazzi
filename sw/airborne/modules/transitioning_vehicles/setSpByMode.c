#include "modules/transitioning_vehicles/setSpByMode.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/autopilot.h"


uint8_t previous_mode;
uint8_t sp;

void transveh_set_sp_by_mode_init(void) {
  previous_mode = autopilot_mode;
}

void transveh_set_sp_by_mode_periodic(void) {
  if (previous_mode != autopilot_mode) {

  }
}

void transveh_smooth_transition(void) {
    //if
}
