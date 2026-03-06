/*
 * Copyright (c) 2015-2024 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 *					Johanata Brayan on Tue Mar 05 2026
 */
#include "acwamctrl.h"

#include <sys/time.h>
#include <err.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "wamctrl_c_types.h"

#include "codels.h"


/* --- Attribute set_saturation_weights --------------------------------- */

/** Validation codel wamctrl_set_satweights of attribute set_saturation_weights.
 *
 * Returns genom_ok.
 * Throws wamctrl_e_inval.
 */
genom_event
wamctrl_set_satweights(const wamctrl_ids_servo_s_satweight_s *satweight,
                    const genom_context self)
{
  if (satweight->thrust < 0. || satweight->tilt < 0. || satweight->head < 0.)
    return wamctrl_e_inval(&(wamctrl_e_inval_detail){"negative weight"}, self);
  return genom_ok;
}


/* --- Attribute set_mass ----------------------------------------------- */

/** Validation codel wamctrl_change_mass of attribute set_mass.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
wamctrl_change_mass(double mass, wamctrl_ids_body_s *body,
                 const genom_context self)
{
  /* update inertia matrix - this scales both the body mass and the rotors
   * mass. */

  if (mass <= 0.)
    return wamctrl_e_inval(&(wamctrl_e_inval_detail){"mass must be positive"}, self);

  if (isnan(body->mass)) return genom_ok;
  return wamctrl_scale_inertia(mass/body->mass, body->J, self);
}


/* --- Attribute set_emerg ---------------------------------------------- */

/** Validation codel wamctrl_set_emerg of attribute set_emerg.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
wamctrl_set_emerg(wamctrl_ids_servo_s_emerg_s *emerg,
               const genom_context self)
{
  emerg->dx = emerg->dx * emerg->dx / 9.;
  emerg->dq = emerg->dq * emerg->dq / 9.;
  emerg->dv = emerg->dv * emerg->dv / 9.;
  emerg->dw = emerg->dw * emerg->dw / 9.;
  return genom_ok;
}


/* --- Activity servo --------------------------------------------------- */

/** Validation codel wamctrl_check_geom of activity servo.
 *
 * Returns genom_ok.
 * Throws wamctrl_e_input, wamctrl_e_geom.
 */
genom_event
wamctrl_check_geom(bool init, const genom_context self)
{
  return init ? genom_ok : wamctrl_e_geom(self);
}


/* --- Function set_state ----------------------------------------------- */

/** Validation codel wamctrl_check_geom of function set_state.
 *
 * Returns genom_ok.
 * Throws wamctrl_e_geom.
 */
/* already defined in service servo validation */



/* --- Function set_position -------------------------------------------- */

/** Validation codel wamctrl_check_geom of function set_position.
 *
 * Returns genom_ok.
 * Throws wamctrl_e_geom.
 */
/* already defined in service servo validation */



/* --- Activity set_current_position ------------------------------------ */

/** Validation codel wamctrl_check_geom of activity set_current_position.
 *
 * Returns genom_ok.
 * Throws wamctrl_e_input, wamctrl_e_geom.
 */
/* already defined in service servo validation */



/* --- Function set_velocity -------------------------------------------- */

/** Validation codel wamctrl_check_geom of function set_velocity.
 *
 * Returns genom_ok.
 * Throws wamctrl_e_geom.
 */
/* already defined in service servo validation */



/* --- Activity set_wo_zero --------------------------------------------- */

/** Validation codel wamctrl_check_geom of activity set_wo_zero.
 *
 * Returns genom_ok.
 * Throws wamctrl_e_geom.
 */
/* already defined in service servo validation */



/* --- Activity log ----------------------------------------------------- */

/** Validation codel wamctrl_log_open of activity log.
 *
 * Returns genom_ok.
 * Throws wamctrl_e_sys.
 */
genom_event
wamctrl_log_open(const char path[64], uint32_t decimation,
              wamctrl_log_s **log, const genom_context self)
{
  int fd;

  fd = open(path, O_WRONLY|O_APPEND|O_CREAT|O_TRUNC, 0666);
  if (fd < 0) return wamctrl_e_sys_error(path, self);

  if ((*log)->fd >= 0)
    close((*log)->fd);
  if ((*log)->req.aio_fildes >= 0) {
    if ((*log)->pending)
      while (aio_error(&(*log)->req) == EINPROGRESS)
        /* empty body */;
  }
  (*log)->fd = fd;
  (*log)->req.aio_fildes = -1;
  (*log)->pending = false;
  (*log)->skipped = false;
  (*log)->decimation = decimation < 1 ? 1 : decimation;
  (*log)->missed = 0;
  (*log)->total = 0;

  return genom_ok;
}


/* --- Attribute set_saturation_weights --------------------------------- */

/** Codel wamctrl_reset_controller of attribute set_saturation_weights.
 *
 * Returns genom_ok.
 * Throws wamctrl_e_inval.
 */
genom_event
wamctrl_reset_controller(const wamctrl_ids_body_s *body,
                      const wamctrl_ids_servo_s *servo,
                      const genom_context self)
{
  wamctrl_controller_init(body, servo);
  return genom_ok;
}


/* --- Attribute set_wlimit --------------------------------------------- */

/** Codel wamctrl_set_wlimit of attribute set_wlimit.
 *
 * Returns genom_ok.
 */
genom_event
wamctrl_set_wlimit(wamctrl_ids_body_s *body, const genom_context self)
{
  (void)self;

  /* order bounds correctly */
  if (body->wmin > body->wmax) {
    double w = body->wmin; body->wmin = body->wmax; body->wmax = w;
  }

  wamctrl_wrench_bounds(
    body->G, body->wmin, body->wmax, body->wrench_min, body->wrench_max);
  return genom_ok;
}

/** Codel wamctrl_reset_controller of attribute set_wlimit.
 *
 * Returns genom_ok.
 */
/* already defined in service set_saturation_weights */



/* --- Attribute set_geom ----------------------------------------------- */

/** Codel wamctrl_set_geom of attribute set_geom.
 *
 * Returns genom_ok.
 */
genom_event
wamctrl_set_geom(wamctrl_ids_body_s *body, const genom_context self)
{
  double w;
  int i, j;

  wamctrl_invert_G(body->G, body->iG);
  wamctrl_wrench_bounds(
    body->G, body->wmin, body->wmax, body->wrench_min, body->wrench_max);

  /* count number of rotors from the iG matrix */
  for (i = or_rotorcraft_max_rotors - 1; i >= 0; i--) {
    w = 0.;
    for(j = 0; j < 6; j++) w += body->iG[i * 6 + j] * body->iG[i * 6 + j];
    if (w > 1e-6) break;
  }
  body->rotors = i + 1;
  body->init = true;
  return genom_ok;
}

/** Codel wamctrl_reset_controller of attribute set_geom.
 *
 * Returns genom_ok.
 */
/* already defined in service set_saturation_weights */



/* --- Function set_gtmrp_geom ------------------------------------------ */

/** Codel wamctrl_set_gtmrp_geom of function set_gtmrp_geom.
 *
 * Returns genom_ok.
 * Throws wamctrl_e_inval.
 */
genom_event
wamctrl_set_gtmrp_geom(uint16_t rotors, double cx, double cy, double cz,
                    double armlen, double mass, double mbodyw,
                    double mbodyh, double mmotor, double rx, double ry,
                    int16_t rz, double cf, double ct,
                    wamctrl_ids_body_s *body, const genom_context self)
{
  genom_event e;

  e = wamctrl_gtmrp_allocmatrix(
    rotors, cx, cy, cz,
    armlen, rx * M_PI/180., ry * M_PI/180., rz, cf, ct, body->G, self);
  if (e) return e;

  e = wamctrl_set_geom(body, self);
  if (e) return e;
  
  e = wamctrl_inertia(
    rotors, armlen, mass, mbodyw, mbodyh, mmotor, body->J, self);
  if (e) return e;

  body->mass = mass;
  body->rotors = rotors;
  body->init = true;
  return genom_ok;
}

/** Codel wamctrl_reset_controller of function set_gtmrp_geom.
 *
 * Returns genom_ok.
 * Throws wamctrl_e_inval.
 */
/* already defined in service set_saturation_weights */


/* --- Function get_reference ------------------------------------------- */

/** Codel wamctrl_get_reference of function get_reference.
 *
 * Returns genom_ok.
 */
genom_event
wamctrl_get_reference(const or_rigid_body_state *internal,
                   or_rigid_body_state *reference,
                   const genom_context self)
{
  *reference = *internal;

  /* explicitly use all fields to return data */
  reference->pos._present = true;
  reference->att._present = true;
  reference->vel._present = true;
  reference->avel._present = true;
  reference->acc._present = true;

  return genom_ok;
}


/* --- Function set_state ----------------------------------------------- */

/** Codel wamctrl_set_state of function set_state.
 *
 * Returns genom_ok.
 * Throws wamctrl_e_geom.
 */
genom_event
wamctrl_set_state(const or_t3d_pos *pos, const or_t3d_att *att,
               const or_t3d_vel *vel, const or_t3d_avel *avel,
               const or_t3d_acc *acc, or_rigid_body_state *reference,
               const genom_context self)
{
  struct timeval tv;
  (void)self; /* -Wunused-parameter */

  gettimeofday(&tv, NULL);
  reference->ts.sec = tv.tv_sec;
  reference->ts.nsec = tv.tv_usec * 1000.;
  reference->intrinsic = false;

  if (isnan(pos->x))
    reference->pos._present = false;
  else {
    reference->pos._present = true;
    reference->pos._value = *pos;
  }

  if (isnan(att->qw))
    reference->att._present = false;
  else {
    reference->att._present = true;
    reference->att._value = *att;
  }

  if (isnan(vel->vx))
    reference->vel._present = false;
  else {
    reference->vel._present = true;
    reference->vel._value = *vel;
  }

  if (isnan(avel->wx))
    reference->avel._present = false;
  else {
    reference->avel._present = true;
    reference->avel._value = *avel;
  }

  if (isnan(acc->ax))
    reference->acc._present = false;
  else {
    reference->acc._present = true;
    reference->acc._value = *acc;
  }

  return genom_ok;
}


/* --- Function set_position -------------------------------------------- */

/** Codel wamctrl_set_position of function set_position.
 *
 * Returns genom_ok.
 * Throws wamctrl_e_geom.
 */
genom_event
wamctrl_set_position(double x, double y, double z, double yaw,
                  or_rigid_body_state *reference,
                  const genom_context self)
{
  struct timeval tv;
  (void)self; /* -Wunused-parameter */

  gettimeofday(&tv, NULL);
  reference->ts.sec = tv.tv_sec;
  reference->ts.nsec = tv.tv_usec * 1000.;
  reference->intrinsic = false;

  reference->pos._present = true;
  reference->pos._value.x = x;
  reference->pos._value.y = y;
  reference->pos._value.z = z;

  reference->att._present = true;
  reference->att._value.qw = cos(yaw/2.);
  reference->att._value.qx = 0.;
  reference->att._value.qy = 0.;
  reference->att._value.qz = sin(yaw/2.);

  reference->vel._present = true;
  reference->vel._value.vx = 0.;
  reference->vel._value.vy = 0.;
  reference->vel._value.vz = 0.;

  reference->avel._present = true;
  reference->avel._value.wx = 0.;
  reference->avel._value.wy = 0.;
  reference->avel._value.wz = 0.;

  reference->acc._present = true;
  reference->acc._value.ax = 0.;
  reference->acc._value.ay = 0.;
  reference->acc._value.az = 0.;

  return genom_ok;
}


/* --- Function set_velocity -------------------------------------------- */

/** Codel wamctrl_set_velocity of function set_velocity.
 *
 * Returns genom_ok.
 * Throws wamctrl_e_geom.
 */
genom_event
wamctrl_set_velocity(double x, double y, double z, double yaw,
                  or_rigid_body_state *reference,
                  const genom_context self)
{
  struct timeval tv;
  (void)self; /* -Wunused-parameter */

  gettimeofday(&tv, NULL);
  reference->ts.sec = tv.tv_sec;
  reference->ts.nsec = tv.tv_usec * 1000.;
  reference->intrinsic = false;

  reference->pos._present = false;
  reference->att._present = false;

  reference->vel._present = true;
  reference->vel._value.vx = x;
  reference->vel._value.vy = y;
  reference->vel._value.vz = z;

  reference->avel._present = true;
  reference->avel._value.wx = 0.;
  reference->avel._value.wy = 0.;
  reference->avel._value.wz = yaw;

  reference->acc._present = false;
  return genom_ok;
}


/* --- Function stop ---------------------------------------------------- */

/** Codel wamctrl_servo_stop of function stop.
 *
 * Returns genom_ok.
 */
genom_event
wamctrl_servo_stop(or_rigid_body_state *reference,
                const genom_context self)
{
  struct timeval tv;
  (void)self; /* -Wunused-parameter */

  gettimeofday(&tv, NULL);
  reference->ts.sec = tv.tv_sec;
  reference->ts.nsec = tv.tv_usec * 1000.;
  reference->intrinsic = false;

  reference->pos._present = true;
  reference->att._present = true;

  reference->vel._present = true;
  reference->vel._value.vx = 0.;
  reference->vel._value.vy = 0.;
  reference->vel._value.vz = 0.;

  reference->avel._present = true;
  reference->avel._value.wx = 0.;
  reference->avel._value.wy = 0.;
  reference->avel._value.wz = 0.;

  reference->acc._present = true;
  reference->acc._value.ax = 0.;
  reference->acc._value.ay = 0.;
  reference->acc._value.az = 0.;

  reference->aacc._present = false;
  reference->jerk._present = false;
  reference->snap._present = false;

  return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel wamctrl_log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
wamctrl_log_stop(wamctrl_log_s **log, const genom_context self)
{
  (void)self; /* -Wunused-parameter */

  if (*log && (*log)->req.aio_fildes >= 0)
    close((*log)->req.aio_fildes);
  (*log)->fd = (*log)->req.aio_fildes = -1;

  return genom_ok;
}


/* --- Function log_info ------------------------------------------------ */

/** Codel wamctrl_log_info of function log_info.
 *
 * Returns genom_ok.
 */
genom_event
wamctrl_log_info(const wamctrl_log_s *log, uint32_t *miss, uint32_t *total,
              const genom_context self)
{
  *miss = *total = 0;
  if (log) {
    *miss = log->missed;
    *total = log->total;
  }
  return genom_ok;
}

/* --- Task start: IDS initialization -------------------------- */

genom_event
init_ids(char **urdf_path, const genom_context self)
{
  *urdf_path = strdup("");
  return wamctrl_ether;
}


/* --- Function set -------------------------------------------- */

genom_event
set_string(const char *value, char **urdf_path,
           const genom_context self)
{
  if (*urdf_path) free(*urdf_path);

  *urdf_path = strdup(value);

  return genom_ok;
}


/* --- Function get -------------------------------------------- */

genom_event
get_string(const char *urdf_path, char **value,
           const genom_context self)
{
  *value = strdup(urdf_path);

  return genom_ok;
}


/* --- Helper: Write joint efforts to output port --------------------- */

void
wamctrl_write_joint_efforts(or_joint_input *joint_data,
                                   const double *efforts, size_t nj,
                                   const struct timeval *tv)
{
  size_t i, maxj;

  if (!joint_data) return;

  joint_data->ts.sec = tv->tv_sec;
  joint_data->ts.nsec = tv->tv_usec * 1000;
  joint_data->position._present = false;
  joint_data->velocity._present = false;
  joint_data->effort._present = true;

  maxj = sizeof(joint_data->effort._value._buffer) /
         sizeof(joint_data->effort._value._buffer[0]);
  if (nj > maxj) nj = maxj;
  if (nj > wamctrl_MAX_JOINTS) nj = wamctrl_MAX_JOINTS;

  joint_data->effort._value._length = nj;
  for (i = 0; i < nj; i++) {
    joint_data->effort._value._buffer[i] = efforts ? efforts[i] : 0.0;
  }
}


/* --- Helper: Disable wholebody controller ------------------------------ */

genom_event
wamctrl_disable_wholebody(wamctrl_ids_wholebody_s *wholebody,
                                 const genom_context self)
{
  (void)self;

  if (wholebody) {
    wholebody->init = false;
    /* Reset integrator when switching modes */
    wamctrl_controller_reset_integrator();
  }
  return genom_ok;
}
