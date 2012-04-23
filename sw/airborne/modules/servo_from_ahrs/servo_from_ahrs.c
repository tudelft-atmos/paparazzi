/*
 * $Id: $
 *
 * Copyright (C) 2012 Team ATMOS
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "servo_from_ahrs/servo_from_ahrs.h"
#include "generated/airframe.h"
#include "actuators.h"

int16_t servo_value;
int16_t servo_value_prev;

void servo_from_ahrs_init(void) {
  servo_value = SERVO_FROM_AHRS_VALUE;
  servo_from_ahrs_periodic();
}

void servo_from_ahrs_periodic(void) {
  servo_value = SERVO_FROM_AHRS_VALUE;
  SetServo(SERVO_FROM_AHRS_SERVO, servo_value)
}
