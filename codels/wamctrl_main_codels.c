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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "wamctrl_c_types.h"
#include "codels.h"


/* --- Task main -------------------------------------------------------- */

/** Codel wamctrl_main_start of task main.
 *
 * Triggered by wamctrl_start.
 * Yields to wamctrl_init.
 */
genom_event
wamctrl_main_start(wamctrl_ids *ids, const wamctrl_rotor_input *rotor_input,
                const wamctrl_joint_input *joint_input,
                const genom_context self)
{
  (void)joint_input;

  ids->body = (wamctrl_ids_body_s){
    .wmax = 100., .wmin = 16.,
    .init = false
  };

  ids->servo = (wamctrl_ids_servo_s){
    .sat = { .x = 0.10, .v = 0.1, .ix = 0.10 },
    .satweight = {
      .enable = true,
      .thrust = 10.0, .tilt = 100.0, .head = 1.0
    },
    .gain = {
      .Kpxy = 14., .Kvxy = 7., .Kpz = 20., .Kvz = 10.,
      .Kqxy = 2.3, .Kwxy = .23, .Kqz = .2, .Kwz = .02,
      .Kixy = 0., .Kiz = 0.
    },

    .att_mode = wamctrl_tilt_prioritized,

    .ramp = 3,
    .scale = 0.,

    .emerg = {
      .descent = .1,
      .dx = 0.05 * 0.05 /9.,
      .dq = 5. * 5. * M_PI*M_PI/180./180./9.,
      .dv = 0.2 * 0.2 /9.,
      .dw = 20. * 20. * M_PI*M_PI/180./180./9.
    }
  };

  ids->reference = (or_rigid_body_state){
    .pos._present = false,
    .att._present = false, .att._value = { .qw = 1. },
    .vel._present = false,
    .avel._present = false,
    .acc._present = false,
    .aacc._present = false,
    .jerk._present = false,
    .snap._present = false
  };


  /* init controller */
  wamctrl_controller_init(&ids->body, &ids->servo);

  /* init logging */
  ids->log = malloc(sizeof(*ids->log));
  if (!ids->log) abort();
  *ids->log = (wamctrl_log_s){
    .fd = -1,
    .req = {
      .aio_fildes = -1,
      .aio_offset = 0,
      .aio_buf = ids->log->buffer,
      .aio_nbytes = 0,
      .aio_reqprio = 0,
      .aio_sigevent = { .sigev_notify = SIGEV_NONE },
      .aio_lio_opcode = LIO_NOP
    },
    .pending = false, .skipped = false,
    .decimation = 1, .missed = 0, .total = 0
  };

  /* Initialize wholebody controller */
  ids->wholebody = (wamctrl_ids_wholebody_s){
    .nj = 0,
    .Kp_base = {10, 10, 10, 10, 10, 10},
    .Kd_base = {5, 5, 5, 5, 5, 5},
    .Kp_joint = {1, 1, 0, 0, 0, 0, 0, 0},
    .Kd_joint = {0.3, 0.3, 0, 0, 0, 0, 0, 0},
    .qd_base = {0, 0, 1, 0, 0, 0, 1},  /* hover at z=1, identity quat */
    .qd_joint = {0, 0, 0, 0, 0, 0, 0, 0},
    .init = false
  };

  /* Initialize pinocchio pointer to NULL (allocated in load_urdf) */
  ids->pinocchio = NULL;

  return wamctrl_init;
}


/** Codel wamctrl_main_init of task main.
 *
 * Triggered by wamctrl_init.
 * Yields to wamctrl_pause_init, wamctrl_pause_control.
 */
genom_event
wamctrl_main_init(or_rigid_body_state *reference,
               const wamctrl_ids_body_s *body, const wamctrl_state *state,
               const wamctrl_joints *joints,
               const wamctrl_rotor_measure *rotor_measure,
               const wamctrl_rotor_input *rotor_input,
               const wamctrl_joint_input *joint_input,
               const wamctrl_wrench_measure *wrench_measure,
               const genom_context self)
{
  or_pose_estimator_state *state_data;
  const or_joint_state *joints_data;
  or_rotorcraft_input *input_data;
  or_joint_input *joint_data = joint_input->data(self);
  struct timeval tv;
  int i;
  size_t j, nj, maxj, nlog;

  state_data = state->data(self);

  gettimeofday(&tv, NULL);

  /* output zero (minimal) velocity */
  input_data = rotor_input->data(self);
  if (!input_data) return wamctrl_pause_init;

  input_data->ts.sec = tv.tv_sec;
  input_data->ts.nsec = tv.tv_usec * 1000;
  input_data->control = or_rotorcraft_velocity;

  input_data->desired._length = body->rotors;
  for(i = 0; i < input_data->desired._length; i++)
    input_data->desired._buffer[i] = 0.;

  rotor_input->write(self); // writes a zero-velocity command during startup/pause

  /* also keep joint port alive during init with zero efforts */
  wamctrl_write_joint_efforts(joint_data, NULL, wamctrl_MAX_JOINTS, &tv);
  if (joint_data) joint_input->write(self);

  /* wait for geometry */
  if (!body->init) return wamctrl_pause_init;

  /* update measured wrench */
  if (!state->read(self))
    wamctrl_main_measure(body, state, rotor_measure, wrench_measure, self);

  /* switch to servo mode upon reception of the first valid position or
   * velocity. Ensure that state has valid pos/att and initialize any empty
   * pos/att reference to it. */
  if (!reference->pos._present && !reference->vel._present)
    return wamctrl_pause_init;

  if (state_data && state_data->pos._present && state_data->att._present) {
    if (!reference->pos._present)
      reference->pos._value = state_data->pos._value;
    if (!reference->att._present)
      reference->att._value = state_data->att._value;

    return wamctrl_pause_control;
  }

  return wamctrl_pause_init;
}


/** Codel wamctrl_main_control of task main.
 *
 * Triggered by wamctrl_control.
 * Yields to wamctrl_measure, wamctrl_emergency.
 */
genom_event
wamctrl_main_control(const wamctrl_ids_body_s *body, wamctrl_ids_servo_s *servo,
                  const wamctrl_state *state,
                  const wamctrl_wrench_measure *wrench_measure,
                  const wamctrl_joints *joints,
                  const wamctrl_ids_wholebody_s *wholebody,
                  wamctrl_pinocchio_s **pinocchio,
                  or_rigid_body_state *reference, wamctrl_log_s **log,
                  const wamctrl_rotor_input *rotor_input,
                  const wamctrl_joint_input *joint_input,
                  const genom_context self)
{
  or_pose_estimator_state *state_data = NULL;
  or_wrench_estimator_state *wrench_data = wrench_measure->data(self);
  or_rotorcraft_input *input_data = rotor_input->data(self);
  or_joint_input *joint_data = joint_input->data(self);
  const or_joint_state *joints_data = NULL;
  struct timeval tv;
  size_t i;
  int e;
  static bool wb_fallback_warned = false;

  gettimeofday(&tv, NULL);

  /* Defensive NULL check for rotor input */
  if (!input_data) {
    warnx("rotor input unavailable");
    return wamctrl_emergency;
  }

  /* read current state */
  if (state->read(self) || !(state_data = state->data(self))) {
    warnx("state unavailable");
    warnx("emergency descent");
    return wamctrl_emergency;
  }

  /* check state */
  e = wamctrl_state_check(tv, servo, state_data);
  if (e) {
    if (e & wamctrl_ETS) warnx("obsolete state");
    if (e & wamctrl_EPOS) warnx("uncertain position");
    if (e & wamctrl_EATT) warnx("uncertain orientation");
    if (e & wamctrl_EVEL) warnx("uncertain velocity");
    if (e & wamctrl_EAVEL) warnx("uncertain angular velocity");
    warnx("emergency descent");
    return wamctrl_emergency;
  }

  /* check reference */
  wamctrl_reference_check(tv, reference);

  /* Read joint state */
  if (joints->read(self) == genom_ok)
    joints_data = joints->data(self);

  /* Try wholebody controller if initialized */
  if (wholebody->init && wamctrl_pinocchio_is_loaded(*pinocchio)) {
    double q[7 + 8];   /* max 7 base + 8 joints */
    double v[6 + 8];   /* max 6 base + 8 joints */
    double tau[6 + 8];
    int ctrl_nj = wholebody->nj;

    /* Build current configuration q = [x,y,z,qx,qy,qz,qw,q1,...,qnj] */
    q[0] = state_data->pos._value.x;
    q[1] = state_data->pos._value.y;
    q[2] = state_data->pos._value.z;
    q[3] = state_data->att._value.qx;
    q[4] = state_data->att._value.qy;
    q[5] = state_data->att._value.qz;
    q[6] = state_data->att._value.qw;

    /* Velocities from state (same as Python - no frame conversion) */
    v[0] = state_data->vel._value.vx;
    v[1] = state_data->vel._value.vy;
    v[2] = state_data->vel._value.vz;
    v[3] = state_data->avel._value.wx;
    v[4] = state_data->avel._value.wy;
    v[5] = state_data->avel._value.wz;

    /* Joint state from dynamixel */
    if (joints_data && joints_data->position._length > 0 && ctrl_nj > 0) {
      for (i = 0; i < (size_t)ctrl_nj && i < joints_data->position._length; i++) {
        q[7 + i] = joints_data->position._buffer[i];
        v[6 + i] = (i < joints_data->velocity._length)
                   ? joints_data->velocity._buffer[i] : 0.0;
      }
    } else {
      /* No joint data - use zeros */
      for (i = 0; i < (size_t)ctrl_nj; i++) {
        q[7 + i] = 0.0;
        v[6 + i] = 0.0;
      }
    }

    /* Call whole-body controller */
    if (wamctrl_wholebody_controller(*pinocchio, wholebody, q, v, tau) == 0) {
      double tau_base[6];

      /* tau from Pinocchio is already in body frame; iG expects body frame wrench */
      for (i = 0; i < 6; i++) tau_base[i] = tau[i];

      /* Base wrench to rotor velocities: u = iG @ tau_base */
      for (i = 0; i < body->rotors; i++) {
        double u = 0;
        int j;
        for (j = 0; j < 6; j++) {
          u += body->iG[i * 6 + j] * tau_base[j];
        }
        /* Clip to propeller velocity squared limits */
        if (u < wamctrl_PROP_VEL_MIN_SQ) u = wamctrl_PROP_VEL_MIN_SQ;
        if (u > wamctrl_PROP_VEL_MAX_SQ) u = wamctrl_PROP_VEL_MAX_SQ;
        input_data->desired._buffer[i] = sqrt(u);
      }
      input_data->desired._length = body->rotors;

      /* Joint torques - build effort array and use helper */
      {
        double efforts[wamctrl_MAX_JOINTS];
        int ctrl_nj_local = wholebody->nj;
        for (i = 0; i < wamctrl_MAX_JOINTS; i++) {
          efforts[i] = (i < (size_t)ctrl_nj_local) ? tau[6 + i] : 0.0;
        }
        wamctrl_write_joint_efforts(joint_data, efforts, wamctrl_MAX_JOINTS, &tv);
        if (joint_data) joint_input->write(self);
      }

      /* output */
      input_data->ts = state_data->ts;
      input_data->control = or_rotorcraft_velocity;
      /* Skip ramp for wholebody controller - it already has proper control law */
      servo->scale = 1.;
      rotor_input->write(self);
      wb_fallback_warned = false;  /* Reset fallback warning on success */
      return wamctrl_measure;
    }
    /* Fallback to legacy controller if wholebody failed */
    if (!wb_fallback_warned) {
      warnx("wholebody controller failed, falling back to legacy");
      wb_fallback_warned = true;
      /* Reset integrator when switching to legacy mode */
      wamctrl_controller_reset_integrator();
    }
  }

  /* Legacy position controller */
  wamctrl_controller(body, servo, state_data, reference, wrench_data,
                  *log, &input_data->desired);

  input_data->ts = state_data->ts;
  if (servo->scale < 1.) {
    for(i = 0; i < input_data->desired._length; i++)
      input_data->desired._buffer[i] *= servo->scale;

    servo->scale += 1e-3 * wamctrl_control_period_ms / servo->ramp;
  }

  /* Legacy controller joint output: small holding torque for first 2 joints */
  {
    double efforts[wamctrl_MAX_JOINTS] = {1.0, 1.0, 0., 0., 0., 0., 0., 0.};
    wamctrl_write_joint_efforts(joint_data, efforts, wamctrl_MAX_JOINTS, &tv);
    if (joint_data) joint_input->write(self);
  }

  rotor_input->write(self); // publishes the controller result
  return wamctrl_measure;
}


/** Codel wamctrl_main_measure of task main.
 *
 * Triggered by wamctrl_measure.
 * Yields to wamctrl_pause_control.
 */
genom_event
wamctrl_main_measure(const wamctrl_ids_body_s *body, const wamctrl_state *state,
                  const wamctrl_rotor_measure *rotor_measure,
                  const wamctrl_wrench_measure *wrench_measure,
                  const genom_context self)
{
  const or_pose_estimator_state *state_data;
  const or_rotorcraft_output *rotor_data;
  or_wrench_estimator_state *wrench_data;
  double wprop[or_rotorcraft_max_rotors];
  double wrench[6];
  struct timeval tv;
  double now;
  size_t i;

  wrench_data = wrench_measure->data(self);
  if (!wrench_data) return wamctrl_pause_control;

  gettimeofday(&tv, NULL);
  now = tv.tv_sec + 1e-6 * tv.tv_usec;

  *wrench_data = (or_wrench_estimator_state){
    .ts = { .sec = tv.tv_sec, .nsec = 1000 * tv.tv_usec },
    .intrinsic = false,
    .force = { ._present = false },
    .force_cov = { ._present = false },
    .torque = { ._present = false },
    .torque_cov = { ._present = false }
  };

  /* current state (already read by control codel) */
  if (!(state_data = state->data(self)))
    goto output;

  /* current propeller speed */
  if (rotor_measure->read(self) || !(rotor_data = rotor_measure->data(self)))
    goto output;

  for(i = 0; i < rotor_data->rotor._length; i++) {
    if (now > 0.1 +
        rotor_data->rotor._buffer[i].ts.sec +
        1e-9 * rotor_data->rotor._buffer[i].ts.nsec)
      goto output;

    if (rotor_data->rotor._buffer[i].spinning)
      wprop[i] = rotor_data->rotor._buffer[i].velocity;
    else
      wprop[i] = 0.;
  }
  for(;i < or_rotorcraft_max_rotors; i++)
    wprop[i] = 0.;

  /* wrench */
  if (wamctrl_wrench(body, state_data, wprop, wrench))
    goto output;

  *wrench_data = (or_wrench_estimator_state){
    .ts = { .sec = tv.tv_sec, .nsec = 1000 * tv.tv_usec },
    .intrinsic = false,
    .force = {
      ._present = true,
      ._value = { .x = wrench[0], .y = wrench[1], .z = wrench[2] }
    },
    .force_cov = { ._present = false },
    .torque = {
      ._present = true,
      ._value = { .x = wrench[3], .y = wrench[4], .z = wrench[5] }
    },
    .torque_cov = { ._present = false }
  };

output:
  wrench_measure->write(self);

  return wamctrl_pause_control;
}


/** Codel wamctrl_main_emergency of task main.
 *
 * Triggered by wamctrl_emergency.
 * Yields to wamctrl_pause_emergency, wamctrl_control.
 */
genom_event
wamctrl_main_emergency(const wamctrl_ids_body_s *body,
                    wamctrl_ids_servo_s *servo, const wamctrl_state *state,
                    or_rigid_body_state *reference, wamctrl_log_s **log,
                    const wamctrl_rotor_input *rotor_input,
                    const wamctrl_joint_input *joint_input,
                    const genom_context self)
{
  static const or_wrench_estimator_state wrench_data = {
    .force._present = false, .torque._present = false
  };

  or_pose_estimator_state *state_data = NULL;
  or_rotorcraft_input *input_data = rotor_input->data(self);
  or_joint_input *joint_data = joint_input->data(self);
  struct timeval tv;
  size_t i;
  int e;

  gettimeofday(&tv, NULL);

  /* read current state */
  if (state->read(self) || !(state_data = state->data(self))) {
    state_data = &(or_pose_estimator_state){
      /* data without any 'present' field except current timestamp for logs */
      .ts.sec = tv.tv_sec, .ts.nsec = 1000 * tv.tv_usec
    };
  }

  /* check state */
  e = wamctrl_state_check(tv, servo, state_data);
  if (!e) {
    warnx("recovered from emergency");
    return wamctrl_control;
  }

  /* check reference */
  wamctrl_reference_check(tv, reference);

  /* position controller */
  wamctrl_controller(body, servo, state_data, reference, &wrench_data,
                  *log, &input_data->desired);

  /* output */
  input_data->ts = state_data->ts;
  if (servo->scale < 1.) {
    for(i = 0; i < input_data->desired._length; i++)
      input_data->desired._buffer[i] *= servo->scale;

    servo->scale += 1e-3 * wamctrl_control_period_ms / servo->ramp;
  }

  /* also publish joint commands during emergency - higher torque for joint 0,1 */
  {
    double efforts[wamctrl_MAX_JOINTS] = {10.0, 10.0, 0., 0., 0., 0., 0., 0.};
    wamctrl_write_joint_efforts(joint_data, efforts, wamctrl_MAX_JOINTS, &tv);
    if (joint_data) joint_input->write(self);
  }

  rotor_input->write(self); // sends the emergency-mode command
  return wamctrl_pause_emergency;
}


/** Codel wamctrl_main_stop of task main.
 *
 * Triggered by wamctrl_stop.
 * Yields to wamctrl_ether.
 */
genom_event
wamctrl_main_stop(const wamctrl_rotor_input *rotor_input,
               const wamctrl_joint_input *joint_input,
               const genom_context self)
{
  or_rotorcraft_input *input_data;
  or_joint_input *joint_data = joint_input->data(self);
  struct timeval tv;
  int i;

  input_data = rotor_input->data(self);
  if (!input_data) return wamctrl_ether;

  gettimeofday(&tv, NULL);
  input_data->ts.sec = tv.tv_sec;
  input_data->ts.nsec = tv.tv_usec * 1000;
  input_data->control = or_rotorcraft_velocity;

  for(i = 0; i < input_data->desired._length; i++)
    input_data->desired._buffer[i] = 0.;

  /* zero joint efforts on stop */
  wamctrl_write_joint_efforts(joint_data, NULL, wamctrl_MAX_JOINTS, &tv);
  if (joint_data) joint_input->write(self);

  rotor_input->write(self); // sends a zero-velocity command on stop
  return wamctrl_ether;
}


/* --- Activity servo --------------------------------------------------- */

/** Codel wamctrl_servo_main of activity servo.
 *
 * Triggered by wamctrl_start.
 * Yields to wamctrl_pause_start, wamctrl_ether.
 * Throws wamctrl_e_input, wamctrl_e_geom.
 */
genom_event
wamctrl_servo_main(const wamctrl_reference *in,
                or_rigid_body_state *reference,
                const genom_context self)
{
  const or_rigid_body_state *ref_data;

  if (in->read(self)) return wamctrl_e_input(self);
  ref_data = in->data(self);
  if (!ref_data) return wamctrl_e_input(self);

  /* check if timestamps have changed */
  if (reference->ts.nsec == ref_data->ts.nsec &&
      reference->ts.sec == ref_data->ts.sec) return wamctrl_pause_start;

  /* keep old value for missing fields for wamctrl_reference_check */
#define copy_if_present(dst, src, f)                                    \
  do {                                                                  \
    dst->f._present = src->f._present;                                  \
    if (src->f._present) dst->f._value = src->f._value;                 \
  } while(0)

  copy_if_present(reference, ref_data, pos);
  copy_if_present(reference, ref_data, att);
  copy_if_present(reference, ref_data, vel);
  copy_if_present(reference, ref_data, avel);
  copy_if_present(reference, ref_data, acc);
  copy_if_present(reference, ref_data, aacc);
  copy_if_present(reference, ref_data, jerk);
  copy_if_present(reference, ref_data, snap);
#undef copy_if_present

  reference->ts = ref_data->ts;
  reference->intrinsic = ref_data->intrinsic;
  return wamctrl_pause_start;
}


/* --- Activity set_current_position ------------------------------------ */

/** Codel wamctrl_set_current_position of activity set_current_position.
 *
 * Triggered by wamctrl_start.
 * Yields to wamctrl_ether.
 * Throws wamctrl_e_input, wamctrl_e_geom.
 */
genom_event
wamctrl_set_current_position(const wamctrl_state *state,
                          or_rigid_body_state *reference,
                          const genom_context self)
{
  const or_pose_estimator_state *state_data;
  double qw, qx, qy, qz;
  double yaw;

  if (state->read(self)) return wamctrl_e_input(self);
  state_data = state->data(self);
  if (!state_data) return wamctrl_e_input(self);
  if (!state_data->pos._present) return wamctrl_e_input(self);
  if (!state_data->att._present) return wamctrl_e_input(self);

  qw = state_data->att._value.qw;
  qx = state_data->att._value.qx;
  qy = state_data->att._value.qy;
  qz = state_data->att._value.qz;
  yaw = atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz));

  reference->ts = state_data->ts;
  reference->intrinsic = state_data->intrinsic;

  reference->pos._present = true;
  reference->pos._value.x = state_data->pos._value.x;
  reference->pos._value.y = state_data->pos._value.y;
  reference->pos._value.z = state_data->pos._value.z;

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

  reference->aacc._present = false;
  reference->jerk._present = false;
  reference->snap._present = false;

  return wamctrl_ether;
}


/* --- Activity log ----------------------------------------------------- */

/** Codel wamctrl_log_header of activity log.
 *
 * Triggered by wamctrl_start.
 * Yields to wamctrl_ether.
 * Throws wamctrl_e_sys.
 */
genom_event
wamctrl_log_header(const wamctrl_ids_servo_s *servo, wamctrl_log_s **log,
                const genom_context self)
{
  int s;

  /* log header with some config info */
  s = dprintf(
    (*log)->fd, "# logged on %s#\n" /* note that ctime(3) has a \n */,
    ctime(&(time_t){ time(NULL) }));
  if (s < 0) goto err;

  s = dprintf((*log)->fd, wamctrl_log_header_fmt "\n");
  if (s < 0) goto err;

  /* enable async writes */
  (*log)->req.aio_fildes = (*log)->fd;

  return wamctrl_ether;
err:
  wamctrl_log_stop(log, self);
  return wamctrl_e_sys_error("log", self);
}
