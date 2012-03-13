#ifndef ATMOV_ELEVON_H
#define ATMOV_ELEVON_H

#include "std.h"
#include "paparazzi.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra.h"

//50 degrees
#define ATMOV_ELEVON_CONTROL_START_THETA ANGLE_BFP_OF_REAL(0.87)

extern void GetAtmovElevonValue(void);


#define ServoSwitchOn()  ({ servo_switch_on = TRUE; FALSE; })


#endif //ATMOV_ELEVON_H

