#include "modules/transitioning_vehicles/setSpByMode.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"
#include "math/pprz_algebra_int.h"

uint8_t previous_mode;
uint8_t sp;
struct Int32Eulers modeSetpoint[3];
struct Int32Eulers oldSetpoints;
struct Int32Eulers nextSetpoints;
int32_t transitionBfpPerTick;

void transveh_set_sp_by_mode_init(void) {
  previous_mode = autopilot_mode;
  modeSetpoint[MODE_MANUAL].phi 	= SETPOINT_MANUAL_PHI;
  modeSetpoint[MODE_MANUAL].theta 	= SETPOINT_MANUAL_THETA;
  modeSetpoint[MODE_MANUAL].psi 	= SETPOINT_MANUAL_PSI;
  modeSetpoint[MODE_AUTO1].phi 		= SETPOINT_AUTO1_PHI;
  modeSetpoint[MODE_AUTO1].theta	= SETPOINT_AUTO1_THETA;
  modeSetpoint[MODE_AUTO1].psi 		= SETPOINT_AUTO1_PSI;
  modeSetpoint[MODE_AUTO2].phi 		= SETPOINT_AUTO2_PHI;
  modeSetpoint[MODE_AUTO2].theta	= SETPOINT_AUTO2_THETA;
  modeSetpoint[MODE_AUTO2].psi 		= SETPOINT_AUTO2_PSI;

  transitionBfpPerTick = ANGLE_BFP_OF_REAL((float)TRANSITION_DEG_PER_TICK);
}

void transveh_set_sp_by_mode_periodic(void) {
  //if (autopilot_mode != MODE_AUTO2)
  if (previous_mode != autopilot_mode) {
	  oldSetpoints = stab_att_sp_euler;
	  nextSetpoints = modeSetpoint[autopilot_mode];
	  previous_mode = autopilot_mode;
  }
  transveh_smooth_transition(nextSetpoints.phi,&stab_att_sp_euler.phi);
  transveh_smooth_transition(nextSetpoints.theta,&stab_att_sp_euler.theta);
  transveh_smooth_transition(nextSetpoints.psi,&stab_att_sp_euler.psi);
}

void transveh_smooth_transition(int32_t desired, int32_t *actual) {
  if (   *actual > (desired - transitionBfpPerTick)
      && *actual < (desired + transitionBfpPerTick))     {
    *actual = desired;
  }
  if (desired != *actual) {
    if (*actual < desired) {
            *actual += transitionBfpPerTick;
    }
    if (*actual > desired) {
            *actual -= transitionBfpPerTick;
    }
  }
}
