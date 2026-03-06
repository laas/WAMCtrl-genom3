/*
 * Copyright (c) 2016-2018,2021-2024 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                           Johanata Brayan on Tue Mar 05 2026
 */
#include "acwamctrl.h"

#include <aio.h>
#include <err.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "proxsuite/proxqp/dense/dense.hpp"

#include "codels.h"


/*
 * --- wamctrl_controller_init ------------------------------------------------
 *
 */

static proxsuite::proxqp::dense::QP<double> wamctrl_wrenchsat(4, 0, 0);

/* Flag to request integrator reset on next controller call */
static bool wamctrl_integrator_reset_requested = false;

void
wamctrl_controller_init(const wamctrl_ids_body_s *body,
                     const wamctrl_ids_servo_s *servo)
{
  using namespace proxsuite::proxqp;
  using namespace Eigen;

  /* The saturation algorithm finds a vector x s.t. the total wrench is within
   * the propellers limits. */
  wamctrl_wrenchsat =
    dense::QP<double>(4, 0, body->rotors, HessianType::Diagonal);

  wamctrl_wrenchsat.settings.eps_abs = 1e-3;
  wamctrl_wrenchsat.settings.initial_guess =
    InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
  wamctrl_wrenchsat.settings.verbose = false;

  /* minimize Σki.(xi - 1)², with appropriate ki weighting coefficients */
  const Vector4d g(
    - servo->satweight.thrust,
    - servo->satweight.tilt,
    - servo->satweight.tilt,
    - servo->satweight.head);
  const Matrix4d H = (-g).asDiagonal();

  /* lower/upper bounds for squared propellers velocities */
  VectorXd l(body->rotors), u(body->rotors);
  l.array() = body->wmin * std::fabs(body->wmin);
  u.array() = body->wmax * std::fabs(body->wmax);

  /* initialize with a fake problem first (average vertical thrust, no torque)
   * so that internal work vectors are computed. In the controller, C is
   * updated with real data. */
  const Map<
    const Matrix<double, Dynamic, 6, RowMajor> > iG(body->iG, body->rotors, 6);
  MatrixXd C(body->rotors, 4);
  C.setZero();
  C.col(0).noalias() =
    iG.col(2) * (body->wrench_min[2] + body->wrench_max[2])/2.;

  wamctrl_wrenchsat.init(H, g, proxsuite::nullopt, proxsuite::nullopt, C, l, u);
}


/*
 * --- wamctrl_controller_reset_integrator -----------------------------------
 *
 * Reset position integrator. Call this when switching control modes
 * (e.g., from wholebody to legacy controller) to avoid transients.
 */

void
wamctrl_controller_reset_integrator(void)
{
  wamctrl_integrator_reset_requested = true;
}


/*
 * --- wamctrl_controller -----------------------------------------------------
 *
 * Implements the controller described in:
 *
 * [1] T. Lee, M. Leoky and N. H. McClamroch, "Geometric tracking control of a
 * quadrotor UAV on SE(3)", 49th IEEE Conference on Decision and Control
 * (CDC), Atlanta, GA, 2010, pp. 5420-5425.
 *
 * Differential flatness equations and derivations can be found here:
 *
 * [2] M. Faessler, A. Franchi and D. Scaramuzza. "Differential Flatness of
 * Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed
 * Trajectories". IEEE Robotics and Automation Letters, Vol. 3., N°2,
 * pp. 620-626, Apr. 2018.
 */

int
wamctrl_controller(const wamctrl_ids_body_s *body,
                const wamctrl_ids_servo_s *servo,
                const or_pose_estimator_state *state,
                const or_rigid_body_state *reference,
                const or_wrench_estimator_state *mwrench,
                wamctrl_log_s *log,
                or_rotorcraft_rotor_control *wprop)
{
  using namespace Eigen;

  Matrix3d Rd;
  Quaternion<double> qd;
  Vector3d xd, vd, wd, ad, jd;

  Matrix3d R;
  Quaternion<double> q;
  Vector3d x, v, w;

  Vector3d ex, ev, eR, ew;
  static Vector3d Iex = Vector3d::Zero();
  static bool Iex_initialized = false;

  /* Initialize or reset integrator */
  if (!Iex_initialized || wamctrl_integrator_reset_requested) {
    Iex.setZero();
    Iex_initialized = true;
    wamctrl_integrator_reset_requested = false;
  }

  double c;
  Vector3d f;
  Matrix<double, 6, 1> wrench;
  Map< Matrix<double, or_rotorcraft_max_rotors, 1> > wprop_(wprop->_buffer);

  size_t i;


  /* geometry */
  const Map<
    const Matrix<double, or_rotorcraft_max_rotors, 6, RowMajor>
    > iG_(body->iG);


  /* gains */
  const Array3d Kp(servo->gain.Kpxy, servo->gain.Kpxy, servo->gain.Kpz);
  const Array3d Ki(servo->gain.Kixy, servo->gain.Kixy, servo->gain.Kiz);
  const Array3d Kv(servo->gain.Kvxy, servo->gain.Kvxy, servo->gain.Kvz);
  const Array3d Kq(servo->gain.Kqxy, servo->gain.Kqxy, servo->gain.Kqz);
  const Array3d Kw(servo->gain.Kwxy, servo->gain.Kwxy, servo->gain.Kwz);


  /* reference state - values are always valid due to wamctrl_reference_check() */
  xd <<
    reference->pos._value.x, reference->pos._value.y, reference->pos._value.z;
  qd.coeffs() <<
    reference->att._value.qx,
    reference->att._value.qy,
    reference->att._value.qz,
    reference->att._value.qw;

  vd <<
    reference->vel._value.vx,
    reference->vel._value.vy,
    reference->vel._value.vz;
  wd <<
    reference->avel._value.wx,
    reference->avel._value.wy,
    reference->avel._value.wz;

  ad <<
    reference->acc._value.ax,
    reference->acc._value.ay,
    reference->acc._value.az;

  jd <<
    reference->jerk._value.jx,
    reference->jerk._value.jy,
    reference->jerk._value.jz;

  if (reference->intrinsic) {
    vd = qd * vd;
    wd = qd * wd;
    ad = qd * ad;
    jd = qd * jd;
  }


  /* current state */
  if (state->pos._present) {
    x << state->pos._value.x, state->pos._value.y, state->pos._value.z;
  } else {
    x = xd;
    ad = Eigen::Vector3d(0, 0, - servo->emerg.descent);
    Iex << 0., 0., 0.;
  }

  if (state->att._present) {
    q.coeffs() <<
      state->att._value.qx, state->att._value.qy, state->att._value.qz,
      state->att._value.qw;
  } else {
    q = qd;
  }
  R = q.matrix();

  if (state->vel._present) {
    v << state->vel._value.vx, state->vel._value.vy, state->vel._value.vz;
  } else {
    v = vd;
  }

  if (state->avel._present) {
    w << state->avel._value.wx, state->avel._value.wy, state->avel._value.wz;
  } else {
    w = wd;
  }

  /* reference acceleration */
  ad = Vector3d(0, 0, wamctrl_GRAVITY) + ad;
  c = ad.norm();

  /* reference orientation
   *
   * Z is parallel to 'ad' and X is aligned with the reference heading.
   * This is the reference body frame Rd = [ xB yB zB ] used in [2].
   *
   * In case ad is 0 (free fall), any orientation is acceptable as explained in
   * [2], Appendix A, section 4. It is expected to be a transcient situation
   * so just use zB from current state.
   */
  c > 1e-4 ? Rd.col(2) = ad / c : Rd.col(2) = R.col(2);
  Rd.col(1) = Rd.col(2).cross(qd.matrix().col(0)).normalized();
  Rd.col(0) = Rd.col(1).cross(Rd.col(2));

  /* reference angular velocity
   *
   * This uses equations derived in [2], with all drag-related coefficient
   * neglected. This leads to the following computations, using notations from
   * the article.
   *
   * Rd rotation matrix can always be written as the composition of 3
   * rotations, so that:
   *                     [ c1.c2    c1.s2.s3 - c3.s1    s1.s3 + c1.c3.s2 ]
   * Rd = [ xB yB zB ] = [ c2.s1    c1.c3 + s1.s2.s3    c3.s1.s2 - c1.s2 ]
   *                     [  -s2          c2.s3               c2.c3       ]
   * where 1, 2, 3 are yaw, pitch, roll intrinsic Euler angles and c and s
   * represent cos and sin (e.g. c1 = cos(yaw)).
   *
   * Then, xC corresponding to the heading direction is the normalized
   * projection of xB on the x,y world plane and yC is xC rotated by π/2:
   *   |c2|.xC = [  c1.c2    c2.s1    0 ]ᵀ
   *   |c2|.yC = [ -c2.s1    c1.c2    0 ]ᵀ
   *
   * So we have the following properties:
   *   |c2|.xCᵀ.xB  =  c2²  =  (1 - s2²)
   *   |c2|.yCᵀ.zB  =  -c2.s3
   *   |c2|.||yCᵀ × zB||  =  |c2.c3|
   *
   * So finally:
   *   ωx = -yBᵀ.j / c
   *   ωy =  xBᵀ.j / c
   *   ωz = ((1 - s2²).dψ/dt - c2.s3.ωy) / |c2.c3|
   *
   * with dψ/dt = wd(2) (world frame) as fed by the 'reference' state.
   */
  wd(0) = - Rd.col(1).dot(jd) / (c > 1e-4 ? c : 1e-4);
  wd(1) =   Rd.col(0).dot(jd) / (c > 1e-4 ? c : 1e-4);
  wd(2) =
    fabs(Rd(2,2)) > 1e-4 ?
    ((1.0 - Rd(2,0)*Rd(2,0))*wd(2) - Rd(2,1)*wd(1)) / fabs(Rd(2,2))
    : 0.0;

  wd = Rd * wd;	/* back to world frame */


  /* position error */
  ex = x - xd;
  for(i = 0; i < 3; i++)
    if (fabs(ex(i)) > servo->sat.x) ex(i) = copysign(servo->sat.x, ex(i));

  Iex += ex * wamctrl_control_period_ms/1000.;
  for(i = 0; i < 3; i++)
    if (fabs(Iex(i)) > servo->sat.ix) Iex(i) = copysign(servo->sat.ix, Iex(i));

  /* velocity error */
  ev = v - vd;
  for(i = 0; i < 3; i++)
    if (fabs(ev(i)) > servo->sat.v) ev(i) = copysign(servo->sat.v, ev(i));


  /* desired thrust */
  f = - Kp * ex.array() - Kv * ev.array() - Ki * Iex.array() +
      body->mass * ad.array();

  /* desired orientation
   * This reuses the xB axis computed earlier in Rd.col(0) */
  Rd.col(2) = f.normalized();
  Rd.col(1) = Rd.col(2).cross(Rd.col(0)).normalized();
  Rd.col(0) = Rd.col(1).cross(Rd.col(2));


  /* orientation error */
  switch (servo->att_mode) {
    default: /* full attitude */ {
      Matrix3d E(0.5 * (Rd.transpose()*R - R.transpose()*Rd));

      eR <<
        (E(2, 1) - E(1, 2))/2.,
        (E(0, 2) - E(2, 0))/2.,
        (E(1, 0) - E(0, 1))/2.;
      break;
    }

    case wamctrl_tilt_prioritized:
      /* D. Brescianini and R. D’Andrea, “Tilt-Prioritized Quadrocopter
       * Attitude Control”, IEEE Transactions on Control Systems Technology,
       * vol. 28, no. 2, pp. 376–387, Mar. 2020. */

      /* attitude error in body frame s.t. q.qE = qd */
      Quaternion<double> qE(q.conjugate() * Quaternion<double>(Rd));

      /* reduced attitude and yaw error merged in a single vector */
      double n = hypot(qE.w(), qE.z());

      eR <<
        (qE.w() * qE.x() - qE.y() * qE.z()) / n,
        (qE.w() * qE.y() + qE.x() * qE.z()) / n,
        qE.z() / n;
      if (qE.w() < 0.) eR(2) = -eR(2);

      /* opposite and factor two to match sign and linearization of default
       * controller error so that gains are compatible */
      eR = - 2 * eR;
      break;
  }

  /* angular velocity error */
  ew.noalias() = R.transpose() * (w - wd);


  /* wrench - XXX assumes vertical thrust in body frame */
  wrench.block<3, 1>(0, 0) << 0., 0., f.dot(R.col(2));
  wrench.block<3, 1>(3, 0) = - Kq * eR.array() - Kw * ew.array();

  /* absolute wrench limitation */
  for(i = 0; i < 6; i++) {
    if (wrench(i) < body->wrench_min[i]) wrench(i) = body->wrench_min[i];
    if (wrench(i) > body->wrench_max[i]) wrench(i) = body->wrench_max[i];
  }

  /* wrench saturation: ensure wmin² <= G¯¹.wrench <= wmax² */
  wprop->_length = body->rotors;
  wprop_.noalias() = iG_ * wrench;
  if (servo->satweight.enable &&
      (wprop_.array().head(body->rotors) > body->wmax * std::fabs(body->wmax) ||
       wprop_.array().head(body->rotors) < body->wmin * std::fabs(body->wmin))
      .any()) {
    using namespace proxsuite::proxqp;

    Matrix<double, Dynamic, 4> C(body->rotors, 4);
    Matrix<double, 6, 4> kw;

    kw <<
      wrench(0), 0.,        0.,        0., /* scale thrust by x1 (only z) */
      wrench(1), 0.,        0.,        0.,
      wrench(2), 0.,        0.,        0.,
      0.,        wrench(3), 0.,        0., /* scale x,y,z torque by x2,x3,x4 */
      0.,        0.,        wrench(4), 0.,
      0.,        0.,        0.,        wrench(5);
    C.noalias() = (iG_ * kw).topRows(body->rotors);

    wamctrl_wrenchsat.update(
      proxsuite::nullopt, proxsuite::nullopt,
      proxsuite::nullopt, proxsuite::nullopt,
      C, proxsuite::nullopt, proxsuite::nullopt);
    wamctrl_wrenchsat.solve();
    if (wamctrl_wrenchsat.results.info.status == QPSolverOutput::PROXQP_SOLVED)
      wprop_.head(body->rotors).noalias() = C * wamctrl_wrenchsat.results.x;
    else
      wamctrl_wrenchsat.results.x.setZero();
  } else
    wamctrl_wrenchsat.results.x.setOnes();

  /* output: signed square root and extra clipping to really enforce limits in
   * case the saturation is disabled did not find a solution */
  wprop_.head(body->rotors) =
    wprop_.head(body->rotors).unaryExpr([body](double w2) {
      double w = std::copysign(std::sqrt(std::fabs(w2)), w2);
      return w < body->wmin ? body->wmin : w > body->wmax ? body->wmax : w;
    });


  /* logging */
  if (log->req.aio_fildes >= 0) {
    log->total++;
    if (log->total % log->decimation == 0) {
      if (log->pending) {
        if (aio_error(&log->req) != EINPROGRESS) {
          log->pending = false;
          if (aio_return(&log->req) <= 0) {
            warn("log");
            close(log->req.aio_fildes);
            log->req.aio_fildes = -1;
          }
        } else {
          log->skipped = true;
          log->missed++;
        }
      }
    }

    if (log->req.aio_fildes >= 0 && !log->pending) {
      double yaw = atan2(Rd(1,0), Rd(0,0));
      double pitch = asin(-Rd(2,0));
      double roll = atan2(Rd(2,1), Rd(2,2));

      /* new saturated wrench */
      wrench = Map<
        const Matrix<double, 6, or_rotorcraft_max_rotors, RowMajor>
        >(body->G) * wprop_.
               unaryExpr([body](double w) { return w * std::fabs(w); });

      log->req.aio_nbytes = snprintf(
        log->buffer, sizeof(log->buffer),
        "%s" wamctrl_log_fmt " %d %g %g %g %g %g %g\n",
        log->skipped ? "\n" : "",
        state->ts.sec, state->ts.nsec,
        state->ts.sec - reference->ts.sec +
        (state->ts.nsec - reference->ts.nsec)*1e-9,
        wrench(0), wrench(1), wrench(2), wrench(3), wrench(4), wrench(5),
        mwrench->force._present ? mwrench->force._value.x : nan(""),
        mwrench->force._present ? mwrench->force._value.y : nan(""),
        mwrench->force._present ? mwrench->force._value.z : nan(""),
        mwrench->torque._present ? mwrench->torque._value.x : nan(""),
        mwrench->torque._present ? mwrench->torque._value.y : nan(""),
        mwrench->torque._present ? mwrench->torque._value.z : nan(""),
        xd(0), xd(1), xd(2), roll, pitch, yaw,
        vd(0), vd(1), vd(2), wd(0), wd(1), wd(2),
        ad(0), ad(1), ad(2),
        ex(0), ex(1), ex(2), ev(0), ev(1), ev(2),
        eR(0), eR(1), eR(2), ew(0), ew(1), ew(2),
        wamctrl_wrenchsat.results.x(0), wamctrl_wrenchsat.results.x(1),
        wamctrl_wrenchsat.results.x(2), wamctrl_wrenchsat.results.x(3),
        /* Wholebody fields: wb_active=0 for legacy controller */
        0,    /* wb_active */
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0  /* wb_tau (not applicable) */
      );

      if (aio_write(&log->req)) {
        warn("log");
        close(log->req.aio_fildes);
        log->req.aio_fildes = -1;
      } else
        log->pending = true;

      log->skipped = false;
    }
  }

  return 0;
}


/*
 * --- wamctrl_state_check ----------------------------------------------------
 *
 * Return non-zero in case of emergency
 */

int
wamctrl_state_check(const struct timeval now,
                 const wamctrl_ids_servo_s *servo,
                 or_pose_estimator_state *state)
{
  int e = wamctrl_EOK;

  /* check state */
  if (now.tv_sec + 1e-6 * now.tv_usec >
      wamctrl_STATE_TIMEOUT_S + state->ts.sec + 1e-9 * state->ts.nsec) {
    state->pos._present = false;
    state->att._present = false;
    state->vel._present = false;
    state->avel._present = false;
    return wamctrl_ETS;
  }

  if (!state->pos._present
      || std::isnan(state->pos._value.x)
      || !state->pos_cov._present
      || state->pos_cov._value.cov[0] > servo->emerg.dx
      || state->pos_cov._value.cov[2] > servo->emerg.dx
      || state->pos_cov._value.cov[5] > servo->emerg.dx) {
    state->pos._present = false;
    e |= wamctrl_EPOS;
  }

  if (!state->att._present
      || std::isnan(state->att._value.qw) ||
      !state->att_cov._present ||
      state->att_cov._value.cov[0] > servo->emerg.dq ||
      state->att_cov._value.cov[2] > servo->emerg.dq ||
      state->att_cov._value.cov[5] > servo->emerg.dq ||
      state->att_cov._value.cov[9] > servo->emerg.dq) {
    state->att._present = false;
    e |= wamctrl_EATT;
  }

  if (!state->vel._present
      || std::isnan(state->vel._value.vx)
      || !state->vel_cov._present
      || state->vel_cov._value.cov[0] > servo->emerg.dv
      || state->vel_cov._value.cov[2] > servo->emerg.dv
      || state->vel_cov._value.cov[5] > servo->emerg.dv) {
    state->vel._present = false;
    e |= wamctrl_EVEL;
  }

  if (!state->avel._present
      || std::isnan(state->avel._value.wx)
      || !state->avel_cov._present
      || state->avel_cov._value.cov[0] > servo->emerg.dw
      || state->avel_cov._value.cov[2] > servo->emerg.dw
      || state->avel_cov._value.cov[5] > servo->emerg.dw) {
    state->avel._present = false;
    e |= wamctrl_EAVEL;
  }

  return e;
}


/*
 * --- wamctrl_reference_check ------------------------------------------------
 *
 * Update missing fields of the reference by integrating other fields
 */

void
wamctrl_reference_check(const struct timeval now,
                     or_rigid_body_state *reference)
{
  static const double dt = wamctrl_control_period_ms / 1000.;
  static const double dt2 = dt * dt;
  static const double dt2_2 = dt2 / 2.;

  or_t3d_pos *p = &reference->pos._value;
  or_t3d_att *q = &reference->att._value;
  or_t3d_vel *v = &reference->vel._value;
  or_t3d_avel *w = &reference->avel._value;
  or_t3d_acc *a = &reference->acc._value;
  or_t3d_jerk *j = &reference->jerk._value;

  /* deal with obsolete reference */
  if (now.tv_sec + 1e-6 * now.tv_usec >
      wamctrl_STATE_TIMEOUT_S + reference->ts.sec + 1e-9 * reference->ts.nsec) {
    reference->vel._present = true;
    v->vx = v->vy = v->vz = 0.;

    reference->avel._present = true;
    w->wx = w->wy = w->wz = 0.;

    reference->acc._present = true;
    a->ax = a->ay = a->az = 0.;

    reference->jerk._present = true;
    j->jx = j->jy = j->jz = 0.;
  }

  if (!reference->pos._present) {
    /* use previous pos and integrate vel, acc */
    Eigen::Vector3d dp(dt * v->vx + dt2_2 * a->ax,
                       dt * v->vy + dt2_2 * a->ay,
                       dt * v->vz + dt2_2 * a->az);
    if (reference->intrinsic)
      dp = Eigen::Quaternion<double>(q->qw, q->qx, q->qy, q->qz) * dp;

    p->x += dp(0);
    p->y += dp(1);
    p->z += dp(2);
  }

  if (!reference->vel._present) {
    /* use previous vel and integrate acc */
    v->vx += dt * a->ax;
    v->vy += dt * a->ay;
    v->vz += dt * a->az;
  }

  if (!reference->acc._present) {
    /* reset */
    a->ax = a->ay = a->az = 0.;
  }

  if (!reference->jerk._present) {
    /* reset */
    j->jx = j->jy = j->jz = 0.;
  }

  if (!reference->att._present) {
    /* use previous att and integrate avel */
    double a2 = dt2 * (w->wx * w->wx + w->wy * w->wy + w->wz * w->wz);

    if (a2 < 1e-1 /* otherwise do nothing, too high */) {
      Eigen::Quaternion<double> Q(q->qw, q->qx, q->qy, q->qz), dq;
      Eigen::Vector3d W(w->wx, w->wy, w->wz);
      if (reference->intrinsic)
        W = Q * W;

      dq.w() = 1 - a2/8; /* cos(a/2) */
      dq.vec() = dt * (0.5 - a2/48) /* dt * sin(a/2)/a */ * W;
      Q = dq * Q;

      q->qw = Q.w(); q->qx = Q.x(); q->qy = Q.y(); q->qz = Q.z();
    }
  }

  if (!reference->avel._present) {
    /* reset */
    w->wx = w->wy = w->wz = 0.;
  }
}


/*
 * --- wamctrl_wrench ---------------------------------------------------------
 *
 * Compute measured total wrench
 */

int
wamctrl_wrench(const wamctrl_ids_body_s *body,
            const or_pose_estimator_state *state,
            const double wprop[or_rotorcraft_max_rotors],
            double wrench[6])
{
  using namespace Eigen;

  Quaternion<double> q;
  Map< const Array<double, or_rotorcraft_max_rotors, 1> >wprop_(wprop);
  Map< Matrix<double, 6, 1> >wrench_(wrench);

  Map< const Matrix<double,
                    6, or_rotorcraft_max_rotors, RowMajor> > G(body->G);

  /* current state - XXX do something if state not present / uncertain */
  if (state->att._present && !std::isnan(state->att._value.qw))
    q.coeffs() <<
      state->att._value.qx, state->att._value.qy, state->att._value.qz,
      state->att._value.qw;
  else
    q = Quaternion<double>::Identity();

  wrench_ = G * wprop_.square().matrix();
  wrench_.block<3, 1>(0, 0) = q * wrench_.block<3, 1>(0, 0);
  wrench_.block<3, 1>(3, 0) = q * wrench_.block<3, 1>(3, 0);

  return 0;
}
