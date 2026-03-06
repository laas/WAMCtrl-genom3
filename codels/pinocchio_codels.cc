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

#include <err.h>
#include <cstdlib>
#include <cmath>
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <Eigen/Dense>

#include "wamctrl_c_types.h"
#include "codels.h"

/* C-safe accessor for pinocchio loaded state */
extern "C" int
wamctrl_pinocchio_is_loaded(const wamctrl_pinocchio_s *pinocchio)
{
  return (pinocchio && pinocchio->loaded) ? 1 : 0;
}

extern "C" genom_event
wamctrl_load_urdf(const char *urdf_path,
                         wamctrl_pinocchio_s **pinocchio,
                         wamctrl_ids_wholebody_s *wholebody,
                         const genom_context self)
{
  namespace pin = pinocchio;

  try {
    /* Allocate struct if needed */
    if (*pinocchio == NULL) {
      *pinocchio = new wamctrl_pinocchio_s;
      (*pinocchio)->model = NULL;
      (*pinocchio)->data = NULL;
      (*pinocchio)->loaded = false;
    }

    /* Clean up previous model */
    if ((*pinocchio)->model) delete (*pinocchio)->model;
    if ((*pinocchio)->data) delete (*pinocchio)->data;

    /* Load new model with free-flyer base */
    (*pinocchio)->model = new pin::Model();
    pin::urdf::buildModel(urdf_path,
                          pin::JointModelFreeFlyer(),
                          *(*pinocchio)->model);
    (*pinocchio)->data = new pin::Data(*(*pinocchio)->model);
    (*pinocchio)->loaded = true;

    /* Pre-allocate vectors to avoid allocation in control loop */
    const int nv = (*pinocchio)->model->nv;
    const int nq = (*pinocchio)->model->nq;
    (*pinocchio)->qd_cache.resize(nq);
    (*pinocchio)->error_cache.resize(nv);
    (*pinocchio)->Kp_cache.resize(nv);
    (*pinocchio)->Kd_cache.resize(nv);
    (*pinocchio)->tau_cache.resize(nv);

    /* Set number of joints (nv - 6 base DOFs) */
    int nj = (*pinocchio)->model->nv - 6;
    if (nj < 0) nj = 0;
    if (nj > 8) nj = 8;
    wholebody->nj = nj;
    wholebody->init = true;

    warnx("Loaded URDF: %s (nq=%d, nv=%d, nj=%d)",
          urdf_path,
          (int)(*pinocchio)->model->nq,
          (int)(*pinocchio)->model->nv,
          nj);
    return genom_ok;

  } catch (const std::exception &e) {
    warnx("Failed to load URDF: %s", e.what());
    if (*pinocchio) (*pinocchio)->loaded = false;
    wholebody->init = false;
    return wamctrl_e_sys_error("load_urdf", self);
  }
}

extern "C" genom_event
wamctrl_set_wholebody_gains(const double Kp_base[6],
                                   const double Kd_base[6],
                                   const double Kp_joint[8],
                                   const double Kd_joint[8],
                                   wamctrl_ids_wholebody_s *wholebody,
                                   const genom_context self)
{
  (void)self;

  for (int i = 0; i < 6; i++) {
    wholebody->Kp_base[i] = Kp_base[i];
    wholebody->Kd_base[i] = Kd_base[i];
  }
  for (int i = 0; i < 8; i++) {
    wholebody->Kp_joint[i] = Kp_joint[i];
    wholebody->Kd_joint[i] = Kd_joint[i];
  }

  warnx("Wholebody gains set: Kp_base=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f], Kd_base=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
        wholebody->Kp_base[0], wholebody->Kp_base[1], wholebody->Kp_base[2],
        wholebody->Kp_base[3], wholebody->Kp_base[4], wholebody->Kp_base[5],
        wholebody->Kd_base[0], wholebody->Kd_base[1], wholebody->Kd_base[2],
        wholebody->Kd_base[3], wholebody->Kd_base[4], wholebody->Kd_base[5]);

  return genom_ok;
}

extern "C" genom_event
wamctrl_set_config(double x, double y, double z,
                          double qx, double qy, double qz, double qw,
                          const double qd[8],
                          wamctrl_ids_wholebody_s *wholebody,
                          or_rigid_body_state *reference,
                          const genom_context self)
{
  (void)self;

  /* Normalize quaternion */
  double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
  if (norm < 1e-6) norm = 1.0;

  wholebody->qd_base[0] = x;
  wholebody->qd_base[1] = y;
  wholebody->qd_base[2] = z;
  wholebody->qd_base[3] = qx / norm;
  wholebody->qd_base[4] = qy / norm;
  wholebody->qd_base[5] = qz / norm;
  wholebody->qd_base[6] = qw / norm;

  for (int i = 0; i < 8; i++) {
    wholebody->qd_joint[i] = qd[i];
  }

  /* Set reference to allow transition from init to control */
  reference->pos._present = true;
  reference->pos._value.x = x;
  reference->pos._value.y = y;
  reference->pos._value.z = z;

  warnx("Config set: pos=[%.2f,%.2f,%.2f], quat=[%.3f,%.3f,%.3f,%.3f], joints=[%.2f,%.2f,...]",
        wholebody->qd_base[0], wholebody->qd_base[1], wholebody->qd_base[2],
        wholebody->qd_base[3], wholebody->qd_base[4], wholebody->qd_base[5], wholebody->qd_base[6],
        wholebody->qd_joint[0], wholebody->qd_joint[1]);

  return genom_ok;
}

/*
 * wamctrl_wholebody_controller - Compute whole-body PD + gravity compensation torques
 *
 * Control law: τ = g(q) + Kp·e - Kd·q̇
 *
 * With joint limit handling: if joint is at limit, torque is clamped to
 * prevent pushing further into the limit.
 *
 * Inputs:
 *   pinocchio      - model/data (with pre-allocated caches)
 *   wholebody      - gains and desired config
 *   q_in[]         - current config [x,y,z,qx,qy,qz,qw,q1,...,qnj] (7+nj)
 *   v_in[]         - current velocity [vx,vy,vz,wx,wy,wz,dq1,...,dqnj] (6+nj)
 *
 * Outputs:
 *   tau_out[]      - torques [fx,fy,fz,tx,ty,tz,tau1,...,taunj] (6+nj)
 *
 * Returns 0 on success, -1 if not initialized
 */
int wamctrl_wholebody_controller(
    wamctrl_pinocchio_s *pinocchio,
    const wamctrl_ids_wholebody_s *wb,
    const double q_in[],
    const double v_in[],
    double tau_out[])
{
  namespace pin = pinocchio;
  static const double JOINT_LIMIT_EPS = 0.01;  /* radians from limit */

  if (!pinocchio || !pinocchio->loaded || !wb->init)
    return -1;

  const pin::Model &model = *pinocchio->model;
  pin::Data &data = *pinocchio->data;
  const int nj = wb->nj;
  const int nv = model.nv;  /* 6 + nj */
  const int nq = model.nq;  /* 7 + nj */

  /* Map inputs to Eigen */
  Eigen::Map<const Eigen::VectorXd> q(q_in, nq);
  Eigen::Map<const Eigen::VectorXd> v(v_in, nv);

  /* Use pre-allocated vectors */
  Eigen::VectorXd &qd = pinocchio->qd_cache;
  Eigen::VectorXd &e = pinocchio->error_cache;
  Eigen::VectorXd &Kp = pinocchio->Kp_cache;
  Eigen::VectorXd &Kd = pinocchio->Kd_cache;
  Eigen::VectorXd &tau = pinocchio->tau_cache;

  /* Build desired configuration qd */
  qd.segment<3>(0) = Eigen::Map<const Eigen::Vector3d>(wb->qd_base);      /* pos */
  qd.segment<4>(3) = Eigen::Map<const Eigen::Vector4d>(wb->qd_base + 3);  /* quat */
  for (int i = 0; i < nj; i++)
    qd(7 + i) = wb->qd_joint[i];

  /* Compute configuration error using Pinocchio's difference */
  e = pin::difference(model, q, qd);

  /* Compute gravity compensation only (crba() not needed for PD+g control) */
  pin::computeGeneralizedGravity(model, data, q);
  const Eigen::VectorXd &g = data.g;

  /* Build Kp and Kd vectors */
  for (int i = 0; i < 6; i++) {
    Kp(i) = wb->Kp_base[i];
    Kd(i) = wb->Kd_base[i];
  }
  for (int i = 0; i < nj; i++) {
    Kp(6 + i) = wb->Kp_joint[i];
    Kd(6 + i) = wb->Kd_joint[i];
  }

  /* Control law: tau = g + Kp * e - Kd * v */
  tau = g + Kp.asDiagonal() * e - Kd.asDiagonal() * v;

  /* Joint limit handling: clamp torques to prevent pushing into limits */
  for (int i = 0; i < nj; i++) {
    const int qi = 7 + i;  /* joint position index in q */
    const int vi = 6 + i;  /* joint velocity index in v/tau */

    /* Check upper limit */
    if (q(qi) >= model.upperPositionLimit(qi) - JOINT_LIMIT_EPS) {
      if (tau(vi) > 0.0) tau(vi) = 0.0;  /* Don't push further up */
    }
    /* Check lower limit */
    if (q(qi) <= model.lowerPositionLimit(qi) + JOINT_LIMIT_EPS) {
      if (tau(vi) < 0.0) tau(vi) = 0.0;  /* Don't push further down */
    }
  }

  /* Copy output */
  Eigen::Map<Eigen::VectorXd>(tau_out, nv) = tau;

  return 0;
}
