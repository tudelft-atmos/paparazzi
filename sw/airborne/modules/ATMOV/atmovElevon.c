#include "ATMOV/atmovElevon.h"
#include "generated/airframe.h"
#include "actuators.h"
#include "subsystems/ins.h"
#include "subsystems/ahrs.h"

void GetAtmovElevonValue(void) {
  if (ahrs.ltp_to_body_euler.theta > ATMOV_ELEVON_CONTROL_START_THETA) {
      float ground_speed_squared = ins_gps_speed_m_s_ned.x*ins_gps_speed_m_s_ned.x + ins_gps_speed_m_s_ned.y*ins_gps_speed_m_s_ned.y;
      elevonDeflectionFactor
  }
}


