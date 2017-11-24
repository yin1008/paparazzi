/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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

/**
 * @file subsystems/ahrs/ahrs_int_cmpl_euler.c
 *
 * Complementary filter in euler representation (fixed-point).
 *
 * Estimate the attitude, heading and gyro bias.
 *
 */

#include "ahrs_int_cmpl_euler.h"

#include "math/pprz_trig_int.h"
#include "math/pprz_algebra_int.h"

#include "generated/airframe.h"

#ifndef AHRS_FACE_REINJ_1
#define AHRS_FACE_REINJ_1 2
#endif

#ifndef AHRS_MAG_OFFSET
#define AHRS_MAG_OFFSET 0.
#endif

struct AhrsIntCmplEuler ahrs_ice;

static inline void get_phi_theta_measurement_from_accel(int32_t *phi_meas, int32_t *theta_meas, struct Int32Vect3 *accel);
static inline void get_psi_measurement_from_mag(int32_t *psi_meas, int32_t phi_est, int32_t theta_est, struct Int32Vect3 *mag);

void ahrs_ice_init(void)
{
  ahrs_ice.status = AHRS_ICE_UNINIT;
  ahrs_ice.is_aligned = false;

  /* init ltp_to_imu to zero */
  INT_EULERS_ZERO(ahrs_ice.ltp_to_imu_euler)
  INT_RATES_ZERO(ahrs_ice.imu_rate);
  INT_RATES_ZERO(ahrs_ice.gyro_bias);

  ahrs_ice.reinj_1 = AHRS_FACE_REINJ_1;
  ahrs_ice.mag_offset = ANGLE_BFP_OF_REAL(AHRS_MAG_OFFSET);
}

bool ahrs_ice_align(struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                      struct Int32Vect3 *lp_mag)
{
  // initialize all variables
  get_phi_theta_measurement_from_accel(&ahrs_ice.meas.phi,
                                      &ahrs_ice.meas.theta, lp_accel);

  INT32_EULERS_RSHIFT(ahrs_ice.ltp_to_imu_euler, ahrs_ice.meas, (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC));

  get_psi_measurement_from_mag(&ahrs_ice.meas.psi, ahrs_ice.ltp_to_imu_euler.phi, ahrs_ice.ltp_to_imu_euler.theta, lp_mag);

  EULERS_COPY(ahrs_ice.meas_lp, ahrs_ice.meas);
  EULERS_COPY(ahrs_ice.euler_est, ahrs_ice.meas);

  RATES_COPY(ahrs_ice.gyro_bias, *lp_gyro);

  ahrs_ice.status = AHRS_ICE_RUNNING;
  ahrs_ice.is_aligned = true;

  return true;
}

//#define USE_NOISE_CUT 1
//#define USE_NOISE_FILTER 1
#define NOISE_FILTER_GAIN 50

#if USE_NOISE_CUT
#include "led.h"
static inline bool cut_rates(struct Int32Rates i1, struct Int32Rates i2, int32_t threshold)
{
  struct Int32Rates diff;
  RATES_DIFF(diff, i1, i2);
  if (diff.p < -threshold || diff.p > threshold ||
      diff.q < -threshold || diff.q > threshold ||
      diff.r < -threshold || diff.r > threshold) {
    return true;
  } else {
    return false;
  }
}
#define RATE_CUT_THRESHOLD RATE_BFP_OF_REAL(1)

static inline bool cut_accel(struct Int32Vect3 i1, struct Int32Vect3 i2, int32_t threshold)
{
  struct Int32Vect3 diff;
  VECT3_DIFF(diff, i1, i2);
  if (diff.x < -threshold || diff.x > threshold ||
      diff.y < -threshold || diff.y > threshold ||
      diff.z < -threshold || diff.z > threshold) {
    LED_ON(4);
    return true;
  } else {
    LED_OFF(4);
    return false;
  }
}
#define ACCEL_CUT_THRESHOLD ACCEL_BFP_OF_REAL(20)

#endif

/*
 *
 * fc = 1/(2*pi*tau)
 *
 * alpha = dt / ( tau + dt )
 *
 *
 *  y(i) = alpha x(i) + (1-alpha) y(i-1)
 *  or
 *  y(i) = y(i-1) + alpha * (x(i) - y(i-1))
 *
 *
 */

void ahrs_ice_propagate(struct Int32Rates *gyro)
{
  /* unbias gyro             */
  static struct Int32Rates uf_rate;
  RATES_DIFF(uf_rate, *gyro, ahrs_ice.gyro_bias);
#if USE_NOISE_CUT
  static struct Int32Rates last_uf_rate = { 0, 0, 0 };
  if (!cut_rates(uf_rate, last_uf_rate, RATE_CUT_THRESHOLD)) {
#endif
    /* low pass rate */
#if USE_NOISE_FILTER
    RATES_SUM_SCALED(ahrs_ice.imu_rate, ahrs_ice.imu_rate, uf_rate, NOISE_FILTER_GAIN);
    RATES_SDIV(ahrs_ice.imu_rate, ahrs_ice.imu_rate, NOISE_FILTER_GAIN + 1);
#else
    RATES_ADD(ahrs_ice.imu_rate, uf_rate);
    RATES_SDIV(ahrs_ice.imu_rate, ahrs_ice.imu_rate, 2);
#endif
#if USE_NOISE_CUT
  }
  RATES_COPY(last_uf_rate, uf_rate);
#endif

  /* integrate eulers */
  static struct Int32Eulers euler_dot;
  int32_eulers_dot_of_rates(&euler_dot, &ahrs_ice.ltp_to_imu_euler, &ahrs_ice.imu_rate);
  INT32_EULERS_LSHIFT(euler_dot, euler_dot, (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC))
  EULERS_SDIV(euler_dot, euler_dot, PERIODIC_FREQUENCY);
  EULERS_ADD(ahrs_ice.euler_est, euler_dot);

  /* low pass measurement */
  EULERS_ADD(ahrs_ice.meas_lp, ahrs_ice.meas);
  EULERS_SDIV(ahrs_ice.meas_lp, ahrs_ice.meas_lp, 2);

  /* compute residual */
  EULERS_DIFF(ahrs_ice.residual, ahrs_ice.meas_lp, ahrs_ice.euler_est);
  INT32_ANGLE_HIGH_RES_NORMALIZE(ahrs_ice.residual.psi);

  static struct Int32Eulers correction;
  /* compute a correction */
  EULERS_SDIV(correction, ahrs_ice.residual, HIGH_RES_ANGLE_BFP_OF_REAL(ahrs_ice.reinj_1));

  /* correct estimation */
  EULERS_ADD(ahrs_ice.euler_est, correction);
  INT32_ANGLE_HIGH_RES_NORMALIZE(ahrs_ice.euler_est.psi);

  /* Compute LTP to IMU eulers */
  INT32_EULERS_RSHIFT(ahrs_ice.ltp_to_imu_euler, ahrs_ice.euler_est, (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC));
}

void ahrs_ice_update_accel(struct Int32Vect3 *accel)
{

#if USE_NOISE_CUT || USE_NOISE_FILTER
  static struct Int32Vect3 last_accel = { 0, 0, 0 };
#endif
#if USE_NOISE_CUT
  if (!cut_accel(*accel, last_accel, ACCEL_CUT_THRESHOLD)) {
#endif
#if USE_NOISE_FILTER
    VECT3_SUM_SCALED(*accel, *accel, last_accel, NOISE_FILTER_GAIN);
    VECT3_SDIV(*accel, *accel, NOISE_FILTER_GAIN + 1);
#endif
    get_phi_theta_measurement_from_accel(&ahrs_ice.meas.phi, &ahrs_ice.meas.theta, accel);
#if USE_NOISE_CUT
  }
  VECT3_COPY(last_accel, *accel);
#endif

}


void ahrs_ice_update_mag(struct Int32Vect3 *mag)
{
  get_psi_measurement_from_mag(&ahrs_ice.meas.psi, ahrs_ice.ltp_to_imu_euler.phi, ahrs_ice.ltp_to_imu_euler.theta, mag);
}

/* measures phi and theta assuming no dynamic acceleration ?!! */
__attribute__((always_inline)) static inline void get_phi_theta_measurement_from_accel(int32_t *phi_meas,
    int32_t *theta_meas, struct Int32Vect3 *accel)
{

  *phi_meas = int32_atan2(-accel->y, -accel->z);
  int32_t cphi;
  PPRZ_ITRIG_COS(cphi, *phi_meas);
  int32_t cphi_ax = -INT_MULT_RSHIFT(cphi, accel->x, INT32_TRIG_FRAC);
  *theta_meas = int32_atan2(-cphi_ax, -accel->z);

  *phi_meas   <<= (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC);
  *theta_meas <<= (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC);
}

/* measure psi by projecting magnetic vector in local tangent plan */
__attribute__((always_inline)) static inline void get_psi_measurement_from_mag(int32_t *psi_meas, int32_t phi_est,
    int32_t theta_est, struct Int32Vect3 *mag)
{
  int32_t sphi;
  PPRZ_ITRIG_SIN(sphi, phi_est);
  int32_t cphi;
  PPRZ_ITRIG_COS(cphi, phi_est);
  int32_t stheta;
  PPRZ_ITRIG_SIN(stheta, theta_est);
  int32_t ctheta;
  PPRZ_ITRIG_COS(ctheta, theta_est);

  int32_t sphi_stheta = (sphi * stheta) >> INT32_TRIG_FRAC;
  int32_t cphi_stheta = (cphi * stheta) >> INT32_TRIG_FRAC;

  const int32_t mn = ctheta * mag->x + sphi_stheta * mag->y + cphi_stheta * mag->z;
  const int32_t me = 0      * mag->x + cphi        * mag->y - sphi        * mag->z;

  int32_t m_psi = -int32_atan2(me, mn);
  *psi_meas = (m_psi - ahrs_ice.mag_offset) << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC);
}

void ahrs_ice_set_body_to_imu(struct OrientationReps *body_to_imu)
{
  ahrs_ice_set_body_to_imu_quat(orientationGetQuat_f(body_to_imu));
}

void ahrs_ice_set_body_to_imu_quat(struct FloatQuat *q_b2i)
{
  orientationSetQuat_f(&ahrs_ice.body_to_imu, q_b2i);

  if (!ahrs_ice.is_aligned) {
    /* Set ltp_to_imu so that body is zero */
    ahrs_ice.ltp_to_imu_euler = *orientationGetEulers_i(&ahrs_ice.body_to_imu);
  }
}
