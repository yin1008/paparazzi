/*
 * Copyright (C) 2014 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 * This is code for guidance of hybrid UAVs. It needs a simple velocity
 * model to control the ground velocity of the UAV while estimating the
 * wind velocity.
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

/** @file firmwares/rotorcraft/guidance/guidance_hybrid.c
 *  Guidance controllers (horizontal and vertical) for Hybrid UAV configurations.
 *
 * Functionality:
 * 1) hover with (helicopter) thrust vectoring and align the heading with the wind vector.
 * 2) Forward flight with using pitch and a bit of thrust to control altitude and
 *    heading to control the velocity vector
 * 3) Transition between the two, with the possibility to fly at any airspeed
 */

#include "firmwares/rotorcraft/guidance/guidance_hybrid.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

/* for guidance_v_thrust_coeff */
#include "firmwares/rotorcraft/guidance/guidance_v.h"

/* for PERIODIC_FREQUENCY */
#include "generated/airframe.h"

// airspeed limits for quadshot guidance
#ifndef MAX_AIRSPEED
#define MAX_AIRSPEED       15 // m/s
#endif
#ifndef NOMINAL_AIRSPEED
#define NOMINAL_AIRSPEED    8 // m/s
#endif
#ifndef TRANSITION_AIRSPEED
#define TRANSITION_AIRSPEED 4 // m/s
#endif
#ifndef MAX_TURN_BANK
#define MAX_TURN_BANK 40      // deg
#endif

#define INT32_AIRSPEED_FRAC 8
#define AIRSPEED_BFP_OF_REAL(x) BFP_OF_REAL(x, INT32_AIRSPEED_FRAC)
#define AIRSPEED_FLOAT_OF_BFP(x) FLOAT_OF_BFP(x, INT32_AIRSPEED_FRAC)

// Variables used for settings
int32_t guidance_hybrid_airspeed_ref_norm;
float alt_pitch_gain;
float max_airspeed, nominal_airspeed, transition_airspeed;
int32_t wind_gain;
int32_t horizontal_speed_gain;
float max_turn_bank;
float turn_bank_gain;

int32_t max_airspeed_i, nominal_airspeed_i, transition_airspeed_i;
int32_t max_turn_bank_i;

// Callbacks to handle a settings change, convert convenience float to integers
void guidance_hybrid_max_airspeed_cb(float new_val)
{
  if(new_val > nominal_airspeed_i)
    max_airspeed_i = AIRSPEED_BFP_OF_REAL(new_val);
}
void guidance_hybrid_nominal_airspeed_cb(float new_val)
{
  if(new_val > transition_airspeed_i)
    nominal_airspeed_i = AIRSPEED_BFP_OF_REAL(new_val);
}
void guidance_hybrid_transition_airspeed_cb(float new_val)
{
  if (new_val < nominal_airspeed_i)
    transition_airspeed_i = AIRSPEED_BFP_OF_REAL(new_val);
}
void guidance_hybrid_max_turn_bank_cb(float new_val){ max_turn_bank_i = ANGLE_BFP_OF_REAL(RadOfDeg(new_val)); }

// Private variables
static struct Int32Eulers guidance_hybrid_ypr_sp;
static struct Int32Vect2 guidance_hybrid_airspeed_sp;
static struct Int32Vect2 guidance_h_pos_err;
static struct Int32Vect2 guidance_hybrid_airspeed_ref;
static struct Int32Vect2 wind_estimate;
static struct Int32Vect2 wind_estimate_high_res;
static struct Int32Vect2 guidance_hybrid_ref_airspeed;

static int32_t heading_diff;
static int32_t high_res_psi;
static bool force_forward_flight;
static int32_t v_control_pitch;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_hybrid_guidance(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_i *pos = stateGetPositionNed_i();
  struct NedCoor_i *speed = stateGetSpeedNed_i();
  pprz_msg_send_HYBRID_GUIDANCE(trans, dev, AC_ID,
                                &(pos->x), &(pos->y),
                                &(speed->x), &(speed->y),
                                &wind_estimate.x, &wind_estimate.y,
                                &guidance_h_pos_err.x,
                                &guidance_h_pos_err.y,
                                &guidance_hybrid_airspeed_sp.x,
                                &guidance_hybrid_airspeed_sp.y,
                                &guidance_hybrid_airspeed_ref_norm,
                                &heading_diff,
                                &guidance_hybrid_ypr_sp.phi,
                                &guidance_hybrid_ypr_sp.theta,
                                &guidance_hybrid_ypr_sp.psi);
}

#endif

void guidance_hybrid_init(void)
{

  INT_EULERS_ZERO(guidance_hybrid_ypr_sp);
  INT_VECT2_ZERO(guidance_hybrid_airspeed_sp);
  INT_VECT2_ZERO(guidance_hybrid_airspeed_ref);

  high_res_psi = 0;
  horizontal_speed_gain = 8;
  guidance_hybrid_airspeed_ref_norm = 0;
  turn_bank_gain = 0.8f;
  wind_gain = 64;
  force_forward_flight = 0;
  alt_pitch_gain = 0.3;

  max_turn_bank = MAX_TURN_BANK;
  max_airspeed = MAX_AIRSPEED;
  nominal_airspeed = NOMINAL_AIRSPEED;
  transition_airspeed = TRANSITION_AIRSPEED;

  max_turn_bank_i = ANGLE_BFP_OF_REAL(RadOfDeg(max_turn_bank));
  max_airspeed_i = AIRSPEED_BFP_OF_REAL(max_airspeed);
  nominal_airspeed_i = AIRSPEED_BFP_OF_REAL(nominal_airspeed);
  transition_airspeed_i = AIRSPEED_BFP_OF_REAL(transition_airspeed);

  INT_VECT2_ZERO(wind_estimate);
  INT_VECT2_ZERO(guidance_hybrid_ref_airspeed);
  INT_VECT2_ZERO(wind_estimate_high_res);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_HYBRID_GUIDANCE, send_hybrid_guidance);
#endif

}

void guidance_hybrid_run(void)
{
  guidance_hybrid_determine_wind_estimate();
  guidance_hybrid_position_to_airspeed();
  guidance_hybrid_airspeed_to_attitude(&guidance_hybrid_ypr_sp);
  guidance_hybrid_set_cmd_i(&guidance_hybrid_ypr_sp);
}

void guidance_hybrid_reset_heading(struct Int32Eulers *sp_cmd)
{
  guidance_hybrid_ypr_sp.psi = sp_cmd->psi;
  high_res_psi = sp_cmd->psi << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC);
  stabilization_attitude_set_rpy_setpoint_i(sp_cmd);
}

void guidance_hybrid_reset_filters(void)
{
  guidance_hybrid_airspeed_ref_norm = 0;
  VECT2_COPY(wind_estimate_high_res, wind_estimate);
  high_res_psi = 0;
  INT_VECT2_ZERO(wind_estimate);
  INT_VECT2_ZERO(wind_estimate_high_res);
}

/// Convert a required airspeed to a certain attitude for the transitioning vehicle
void guidance_hybrid_airspeed_to_attitude(struct Int32Eulers *ypr_sp)
{

  //notes:
  //in forward flight, it is preferred to first get to min(airspeed_sp, airspeed_ref) and then change heading and then get to airspeed_sp
  //in hover, just a gradual change is needed, or maybe not even needed
  //changes between flight regimes should be handled

  //determine the heading of the airspeed_sp vector
  int32_t omega = 0;
  int32_t airspeed_sp_heading = int32_atan2(guidance_hybrid_airspeed_sp.y, guidance_hybrid_airspeed_sp.x);

  //The difference of the current heading with the required heading.
  heading_diff = airspeed_sp_heading - ypr_sp->psi;
  INT32_ANGLE_NORMALIZE(heading_diff);

  //calculate the norm of the airspeed setpoint
  int32_t airspeed_sp_norm = int32_vect2_norm(&guidance_hybrid_airspeed_sp);

  //reference goes with a steady pace towards the setpoint airspeed
  //hold ref norm below transition speed until heading is aligned
  static uint32_t five_deg_i = ANGLE_BFP_OF_REAL(RadOfDeg(5.f));
  if (!((airspeed_sp_norm > transition_airspeed_i) && (guidance_hybrid_airspeed_ref_norm < transition_airspeed_i)
        && (guidance_hybrid_airspeed_ref_norm > (transition_airspeed_i - 10)) && (abs(heading_diff) > five_deg_i))) {
    // TODO what kind of filter is this?
    guidance_hybrid_airspeed_ref_norm = guidance_hybrid_airspeed_ref_norm + ((int32_t)(airspeed_sp_norm >
                                        guidance_hybrid_airspeed_ref_norm) * 2 - 1) * 3 / 2;
  }

  static int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, ypr_sp->psi);
  PPRZ_ITRIG_COS(c_psi, ypr_sp->psi);

  guidance_hybrid_ref_airspeed.x = INT_MULT_RSHIFT(guidance_hybrid_airspeed_ref_norm, c_psi, INT32_TRIG_FRAC);
  guidance_hybrid_ref_airspeed.y = INT_MULT_RSHIFT(guidance_hybrid_airspeed_ref_norm, s_psi, INT32_TRIG_FRAC);

  if (guidance_hybrid_airspeed_ref_norm < transition_airspeed_i) {
    // if required speed is lower than transition airspeed act like a rotorcraft
    // translate speed_sp into bank angle and heading

    // change heading to direction of airspeed, faster if the airspeed is higher
    omega = (airspeed_sp_norm << (INT32_ANGLE_FRAC - INT32_POS_FRAC)) / 6;
    if (heading_diff < 0) {
      omega = -omega;
    }
    BoundAbs(omega, ANGLE_BFP_OF_REAL(0.8));

    // 2) calculate roll/pitch commands
    static struct Int32Vect2 hover_sp;
    //if the setpoint is beyond transition airspeed but the ref is not, the norm of the hover sp will stay at the transition speed
    if (airspeed_sp_norm > transition_airspeed_i) {
      hover_sp.x = guidance_hybrid_airspeed_sp.x / airspeed_sp_norm;
      hover_sp.y = guidance_hybrid_airspeed_sp.y / airspeed_sp_norm;
    } else {
      hover_sp.x = guidance_hybrid_airspeed_sp.x;
      hover_sp.y = guidance_hybrid_airspeed_sp.y;
    }

    // gain of 10 means that for a transition airspeed of 5 m/s an angle of 40 degrees is needed
    static uint32_t pr_sp_gain = 10;
    ypr_sp->theta = ((- (c_psi * hover_sp.x + s_psi * hover_sp.y)) * pr_sp_gain * INT32_ANGLE_PI / 180) >> INT32_AIRSPEED_FRAC;
    ypr_sp->phi = (((- s_psi * hover_sp.x + c_psi * hover_sp.y)) * pr_sp_gain * INT32_ANGLE_PI / 180) >> INT32_AIRSPEED_FRAC;
  } else {
    /// if required speed is higher than transition airspeed act like a fixedwing
    // translate speed_sp into theta + thrust
    // coordinated turns to change heading

    // TODO make the following user settable?
    static uint32_t pitch_sp_max_speed = ANGLE_BFP_OF_REAL(RadOfDeg(78.f));
    static uint32_t pitch_sp_nominal_speed = ANGLE_BFP_OF_REAL(RadOfDeg(68.f));
    static uint32_t pitch_sp_transition_speed = ANGLE_BFP_OF_REAL(RadOfDeg(40.f));

    static uint32_t pitch_gain_nominal_speed = ANGLE_BFP_OF_REAL(RadOfDeg(2.f));
    static uint32_t pitch_gain_transition_speed = ANGLE_BFP_OF_REAL(RadOfDeg(7.f));

    // calculate required pitch angle from airspeed_sp magnitude
    if (guidance_hybrid_airspeed_ref_norm > max_airspeed_i) {
      ypr_sp->theta = -pitch_sp_max_speed;
    } else if (guidance_hybrid_airspeed_ref_norm > nominal_airspeed_i) {
      ypr_sp->theta = (-((guidance_hybrid_airspeed_ref_norm - nominal_airspeed_i) * pitch_gain_nominal_speed) >> INT32_AIRSPEED_FRAC)
          - pitch_sp_nominal_speed;
    } else {
      ypr_sp->theta = (-((guidance_hybrid_airspeed_ref_norm - transition_airspeed_i) * pitch_gain_transition_speed) >> INT32_AIRSPEED_FRAC)
          - pitch_sp_transition_speed;
    }

    // if the sp_airspeed is within hovering range, don't start a coordinated turn
    if (airspeed_sp_norm < transition_airspeed_i) {
      omega = 0;
      ypr_sp->phi = 0;
    } else { // coordinated turn
      ypr_sp->phi = (int32_t)(heading_diff * turn_bank_gain);
      BoundAbs(ypr_sp->phi, max_turn_bank_i);

      //feedforward estimate angular rotation omega = g*tan(phi)/v
      omega = (int32_t)(9.81 * pprz_itrig_tan(ypr_sp->phi) / guidance_hybrid_airspeed_ref_norm) >> INT32_AIRSPEED_FRAC;
      BoundAbs(omega, ANGLE_BFP_OF_REAL(0.7));
    }
  }

  //go to higher resolution because else the increment is too small to be added
  high_res_psi += (omega << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC)) / PERIODIC_FREQUENCY;
  INT32_ANGLE_HIGH_RES_NORMALIZE(high_res_psi);

  // go back to angle_frac
  ypr_sp->psi = high_res_psi >> (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC);
  ypr_sp->theta = ypr_sp->theta + v_control_pitch;
}

void guidance_hybrid_position_to_airspeed(void)
{
  /* compute position error    */
  VECT2_DIFF(guidance_h_pos_err, guidance_h.sp.pos, *stateGetPositionNed_i());

  // Compute ground speed setpoint
  struct Int32Vect2 guidance_hybrid_groundspeed_sp;
  VECT2_SDIV(guidance_hybrid_groundspeed_sp, guidance_h_pos_err, horizontal_speed_gain);
  INT32_VECT2_LSHIFT(guidance_hybrid_groundspeed_sp, guidance_hybrid_groundspeed_sp, INT32_AIRSPEED_FRAC - INT32_POS_FRAC);

  int32_t groundspeed_sp_norm = int32_vect2_norm(&guidance_hybrid_groundspeed_sp);

  //create setpoint groundspeed with a norm of max speed
  if (force_forward_flight) {
    //scale the groundspeed_sp to max speed if large enough to begin with
    if (groundspeed_sp_norm > AIRSPEED_BFP_OF_REAL(1)) {
      guidance_hybrid_groundspeed_sp.x = guidance_hybrid_groundspeed_sp.x * max_airspeed_i / groundspeed_sp_norm;
      guidance_hybrid_groundspeed_sp.y = guidance_hybrid_groundspeed_sp.y * max_airspeed_i / groundspeed_sp_norm;
    } else { //groundspeed_sp is very small, so continue with the current heading
      guidance_hybrid_groundspeed_sp.x = (max_airspeed_i * pprz_itrig_cos(guidance_hybrid_ypr_sp.psi)) >> (INT32_TRIG_FRAC - INT32_AIRSPEED_FRAC);
      guidance_hybrid_groundspeed_sp.y = (max_airspeed_i * pprz_itrig_sin(guidance_hybrid_ypr_sp.psi)) >> (INT32_TRIG_FRAC - INT32_AIRSPEED_FRAC);
    }
  }

  struct Int32Vect2 airspeed_sp;
  VECT2_COPY(airspeed_sp, guidance_hybrid_groundspeed_sp);
  VECT2_ADD(airspeed_sp, wind_estimate);

  int32_t airspeed_sp_norm = int32_vect2_norm(&airspeed_sp);

  //Check if the airspeed_sp is larger than the max airspeed. If so, give the wind cancellation priority.
  if (airspeed_sp_norm > max_airspeed_i && groundspeed_sp_norm > 0) {
    int32_t av = INT_MULT_RSHIFT(guidance_hybrid_groundspeed_sp.x, guidance_hybrid_groundspeed_sp.x, INT32_AIRSPEED_FRAC)
        + INT_MULT_RSHIFT(guidance_hybrid_groundspeed_sp.y, guidance_hybrid_groundspeed_sp.y, INT32_AIRSPEED_FRAC);
    int32_t bv = 2 * (INT_MULT_RSHIFT(wind_estimate.x, guidance_hybrid_groundspeed_sp.x, INT32_AIRSPEED_FRAC)
        + INT_MULT_RSHIFT(wind_estimate.y, guidance_hybrid_groundspeed_sp.y, INT32_AIRSPEED_FRAC));
    int32_t cv = INT_MULT_RSHIFT(wind_estimate.x, wind_estimate.x, 8)
        + INT_MULT_RSHIFT(wind_estimate.y, wind_estimate.y, INT32_AIRSPEED_FRAC)
        - INT_MULT_RSHIFT(max_airspeed_i, max_airspeed_i, INT32_AIRSPEED_FRAC);

    float dv = AIRSPEED_FLOAT_OF_BFP(bv) * AIRSPEED_FLOAT_OF_BFP(bv) - 4.0 * AIRSPEED_FLOAT_OF_BFP(av) * AIRSPEED_FLOAT_OF_BFP(cv);
    int32_t d_sqrt = POS_BFP_OF_REAL(sqrtf(dv));

    int32_t result = ((-bv + d_sqrt) << INT32_AIRSPEED_FRAC) / (2 * av);

    guidance_hybrid_airspeed_sp.x = wind_estimate.x + INT_MULT_RSHIFT(guidance_hybrid_groundspeed_sp.x, result, INT32_AIRSPEED_FRAC);
    guidance_hybrid_airspeed_sp.y = wind_estimate.y + INT_MULT_RSHIFT(guidance_hybrid_groundspeed_sp.y, result, INT32_AIRSPEED_FRAC);
  } else {
    // Add the wind to get the airspeed setpoint
    guidance_hybrid_airspeed_sp = guidance_hybrid_groundspeed_sp;
    VECT2_ADD(guidance_hybrid_airspeed_sp, wind_estimate);
  }

  // limit the airspeed setpoint to max airspeed, because else saturation + windup will occur
  airspeed_sp_norm = int32_vect2_norm(&guidance_hybrid_airspeed_sp);
  // add check for non-zero airspeed (in case the max_airspeed_i is ever set to zero
  if (airspeed_sp_norm > max_airspeed_i && airspeed_sp_norm > 0) {
    guidance_hybrid_airspeed_sp.x = guidance_hybrid_airspeed_sp.x * max_airspeed_i / airspeed_sp_norm;
    guidance_hybrid_airspeed_sp.y = guidance_hybrid_airspeed_sp.y * max_airspeed_i / airspeed_sp_norm;
  }
}

void guidance_hybrid_determine_wind_estimate(void)
{

  /* compute speed error    */
  struct Int32Vect2 wind_estimate_measured;
  struct Int32Vect2 measured_ground_speed;
  INT32_VECT2_RSHIFT(measured_ground_speed, *stateGetSpeedNed_i(), INT32_SPEED_FRAC - INT32_AIRSPEED_FRAC);
  VECT2_DIFF(wind_estimate_measured, guidance_hybrid_ref_airspeed, measured_ground_speed);

  //Low pass wind_estimate, because we know the wind usually only changes slowly
  //But not too slow, because the wind_estimate is also an adaptive element for the airspeed model inaccuracies
  // TODO: What is this doing exactly?
  wind_estimate_high_res.x += (((wind_estimate_measured.x - wind_estimate.x) > 0) * 2 - 1) * wind_gain;
  wind_estimate_high_res.y += (((wind_estimate_measured.y - wind_estimate.y) > 0) * 2 - 1) * wind_gain;

  wind_estimate.x = ((wind_estimate_high_res.x) >> INT32_AIRSPEED_FRAC);
  wind_estimate.y = ((wind_estimate_high_res.y) >> INT32_AIRSPEED_FRAC);
}

void guidance_hybrid_set_cmd_i(struct Int32Eulers *sp_cmd)
{
  /// @todo calc sp_quat in fixed-point

  /* orientation vector describing simultaneous rotation of roll/pitch */
  struct FloatVect3 ov;
  ov.x = ANGLE_FLOAT_OF_BFP(sp_cmd->phi);
  ov.y = ANGLE_FLOAT_OF_BFP(sp_cmd->theta);
  ov.z = 0.0;
  /* quaternion from that orientation vector */
  struct FloatQuat q_rp;
  float_quat_of_orientation_vect(&q_rp, &ov);
  struct Int32Quat q_rp_i;
  QUAT_BFP_OF_REAL(q_rp_i, q_rp);

  //   get the vertical vector to rotate the roll pitch setpoint around
  struct Int32Vect3 zaxis = {0, 0, 1};

  /* get current heading setpoint */
  struct Int32Quat q_yaw_sp;
  int32_quat_of_axis_angle(&q_yaw_sp, &zaxis, sp_cmd->psi);

  //   first apply the roll/pitch setpoint and then the yaw
  int32_quat_comp(&stab_att_sp_quat, &q_yaw_sp, &q_rp_i);

  int32_eulers_of_quat(&stab_att_sp_euler, &stab_att_sp_quat);
}

enum thrust_mode {
  THRUST_ONLY,
  THRUST_PITCH,
  PITCH_ONLY
};

static enum thrust_mode ctrl_mode = THRUST_ONLY;

void guidance_hybrid_vertical(void)
{
  if (guidance_hybrid_airspeed_ref_norm < transition_airspeed_i) {
    //if airspeed ref < transition airspeed only thrust
    stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
    v_control_pitch = 0;
    if(ctrl_mode == THRUST_PITCH)
    {
      guidance_v_kp *= 2;
      guidance_v_kd *= 2;
      guidance_v_ki *= 2;
      ctrl_mode = THRUST_ONLY;
    }
  } else if (guidance_hybrid_airspeed_ref_norm > nominal_airspeed_i) { //if airspeed ref > nominal airspeed only pitch,
    //at max speed the thrust has to be 33%
    // TODO, is this valid for all vehicles? Check if these hard coded numbers are independent of max_speed, nominal_speed ect.
    stabilization_cmd[COMMAND_THRUST] = MAX_PPRZ / 5 + (((guidance_hybrid_airspeed_ref_norm - nominal_airspeed_i) / 7 *
                                        (MAX_PPRZ / 3 - MAX_PPRZ / 5)) >> INT32_AIRSPEED_FRAC) + (guidance_v_delta_t - MAX_PPRZ / 2) / 10;
    //Control altitude with pitch, now only proportional control
    float alt_control_pitch = (guidance_v_delta_t - MAX_PPRZ * guidance_v_nominal_throttle) * alt_pitch_gain;
    v_control_pitch = ANGLE_BFP_OF_REAL(alt_control_pitch / (MAX_PPRZ * guidance_v_nominal_throttle));

    if(ctrl_mode != THRUST_PITCH)
    {
      guidance_v_kp /= 2;
      guidance_v_kd /= 2;
      guidance_v_ki /= 2;
      ctrl_mode = THRUST_PITCH;
    }
  } else { //if airspeed ref > transition airspeed && < nominal airspeed both
    int32_t airspeed_transition = AIRSPEED_BFP_OF_REAL(guidance_hybrid_airspeed_ref_norm - transition_airspeed_i) / (nominal_airspeed_i - transition_airspeed_i); // scale [0,1)
    stabilization_cmd[COMMAND_THRUST] = ((MAX_PPRZ / 5 + (guidance_v_delta_t - MAX_PPRZ / 2) / 10) * airspeed_transition +
        guidance_v_delta_t * (AIRSPEED_BFP_OF_REAL(1) - airspeed_transition)) >> INT32_AIRSPEED_FRAC;
    float alt_control_pitch = (guidance_v_delta_t - MAX_PPRZ * guidance_v_nominal_throttle) * alt_pitch_gain;
    v_control_pitch = INT_MULT_RSHIFT((int32_t)ANGLE_BFP_OF_REAL(alt_control_pitch / (MAX_PPRZ * guidance_v_nominal_throttle)), airspeed_transition, INT32_AIRSPEED_FRAC);
    if(ctrl_mode == THRUST_PITCH)
    {
      guidance_v_kp *= 2;
      guidance_v_kd *= 2;
      guidance_v_ki *= 2;
      ctrl_mode = PITCH_ONLY;
    }
  }
}
