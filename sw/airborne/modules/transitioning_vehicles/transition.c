#include "modules/transitioning_vehicles/transition.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"
#include "math/pprz_algebra_int.h"
#include "firmwares/rotorcraft/actuators/supervision.h"

uint8_t sp;

struct Int32Eulers oldSetpoints;
struct Int32Eulers forwardSetpoints;
struct Int32Eulers hoverSetpoints;
int32_t transitionBfpPerTick;

void transveh_transition_init(void) {
  transitionBfpPerTick = ANGLE_BFP_OF_REAL((float)TRANSITION_DEG_PER_TICK);

  int32_t transveh_orig_thrust_coef[SUPERVISION_NB_MOTOR] = SUPERVISION_THRUST_COEF;
  float transveh_prep_thrust_coef_scaling[SUPERVISION_NB_MOTOR] = TRANSVEH_PREP_THRUST_COEF_SCALING;
}

void transveh_transition_periodic(void) {
/*  forwardSetpoints.phi = 0;
  forwardSetpoints.theta = 90;
  forwardSetpoints.psi = 0;

  hoverSetpoints.phi = 0;
  hoverSetpoints.theta = 0;
  hoverSetpoints.psi = 0;*/
}

void PrepForTransition(void) {
  for (i = 0; i < SUPERVISION_NB_MOTOR; i++) {
    thrust_coef[SUPERVISION_NB_MOTOR] =*
        (float)transveh_prep_thrust_coef_scaling[SUPERVISION_NB_MOTOR];
  }
}

void transveh_transition_doTransition(void) {
  oldSetpoints = stab_att_sp_euler;
  nextSetpoints = modeSetpoint[autopilot_mode];
  previous_mode = autopilot_mode;

  /*transveh_transition_smooth_transition(forwardSetpoints.phi,&stab_att_sp_euler.phi);
  transveh_transition_smooth_transition(forwardSetpoints.theta,&stab_att_sp_euler.theta);
  transveh_transition_smooth_transition(forwardSetpoints.psi,&stab_att_sp_euler.psi);*/

  /*int32_t pitch_rotation_angle;
  struct Int32Quat pitch_axis_quat;
  struct Int32Vect3 y_axis = { 0, 1, 0 };


  pitch_rotation_angle = ANGLE_BFP_OF_REAL(M_PI_2);
  INT32_QUAT_OF_AXIS_ANGLE(pitch_axis_quat, y_axis, pitch_rotation_angle);*/

  /*use current body position to compose setpoint */
  int32_t tang = ANGLE_BFP_OF_REAL(M_PI_2);
  reset_sp_quat(ahrs.ltp_to_body_euler.psi, tang, &ahrs.ltp_to_body_quat);
}


void transveh_transition_smooth_transition(int32_t desired, int32_t *actual) {
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

