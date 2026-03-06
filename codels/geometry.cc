/*
 * Copyright (c) 2022,2024 LAAS/CNRS
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
 *                                           Anthony Mallet on Mon Jun 13 2022
 */
#include <cmath>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "codels.h"

/* --- wamctrl_gtmrp_allocmatrix ---------------------------------------------- */

genom_event
wamctrl_gtmrp_allocmatrix(int rotors,
                       double cx, double cy, double cz,
                       double armlen, double rx, double ry, double rz,
                       double cf, double ct,
                       double G[6 * or_rotorcraft_max_rotors],
                       const genom_context self)
{
  using namespace Eigen;

  Map< Matrix<double, 6, or_rotorcraft_max_rotors, RowMajor> > G_(G);
  Vector3d z, p;
  int i, sign;

  if (rotors < 0 || rotors > or_rotorcraft_max_rotors) {
    wamctrl_e_inval_detail d = { "bad number of rotors" };
    return wamctrl_e_inval(&d, self);
  }

  for(i = 0, sign = 1; i < rotors; i++, sign = -sign) {
    /* thrust direction */
    z =
      (AngleAxisd(2 * i * M_PI / rotors, Vector3d::UnitZ())
       * AngleAxisd(sign * rx, Vector3d::UnitX())
       * AngleAxisd(ry, Vector3d::UnitY())).matrix().col(2);

    /* thrust position */
    p =
      armlen *
      AngleAxisd(2 * i * M_PI / rotors, Vector3d::UnitZ()).matrix().col(0)
      + Vector3d(cx, cy, cz);

    /* wrench */
    G_.col(i) <<
      cf * z,
      cf * p.cross(z) - sign * rz * ct * z;
  }

  for(; i < or_rotorcraft_max_rotors; i++)
    G_.col(i) << Vector3d::Zero(), Vector3d::Zero();

  return genom_ok;
}


/* --- wamctrl_invert_G ------------------------------------------------------- */

void
wamctrl_invert_G(const double G[6 * or_rotorcraft_max_rotors],
              double iG[or_rotorcraft_max_rotors * 6])
{
  using namespace Eigen;

  Map< const Matrix<double, 6, or_rotorcraft_max_rotors, RowMajor> > G_(G);
  Map< Matrix<double, or_rotorcraft_max_rotors, 6, RowMajor> > iG_(iG);

  iG_ = G_.
    jacobiSvd(ComputeFullU | ComputeFullV).
    solve(Matrix<double, 6, 6>::Identity());
}


/* --- wamctrl_Gw2 ------------------------------------------------------------ */

void
wamctrl_Gw2(const double G[6 * or_rotorcraft_max_rotors], const double w,
         double f[6])
{
  using namespace Eigen;

  Map< const Matrix<double, 6, or_rotorcraft_max_rotors, RowMajor> > G_(G);
  Map< Matrix<double, 6, 1> > f_(f);

  f_ = G_ * Matrix<double, or_rotorcraft_max_rotors, 1>::Constant(w * w);
}


/* --- wamctrl_wrench_bounds -------------------------------------------------- */

void
wamctrl_wrench_bounds(const double G[6 * or_rotorcraft_max_rotors],
                   const double wmin, const double wmax,
                   double fmin[6], double fmax[6])
{
  using namespace Eigen;

  Map< const Matrix<double, 6, or_rotorcraft_max_rotors, RowMajor> > G_(G);
  const double w2min = std::copysign(wmin * wmin, wmin);
  const double w2max = std::copysign(wmax * wmax, wmax);
  int i;

  /* Using the i-th line of G, build a vector w² with wmin or wmax as the
   * velocity of the j-th propeller, depending on the sign of G(i,j).
   * G(i,:) * w² will give the min/max achievable force or torque. */
  for(i = 0; i < 6; i++) {
    fmin[i] = G_.row(i) * G_.row(i).unaryExpr([=](double g) {
      return g >= 0. ? w2min : w2max;
    }).transpose();
    fmax[i] = G_.row(i) * G_.row(i).unaryExpr([=](double g) {
      return g >= 0. ? w2max : w2min;
    }).transpose();
  }
}


/* --- wamctrl_inertia -------------------------------------------------------- */

genom_event
wamctrl_inertia(int rotors, double armlen,
             double mass, double mbodyw, double mbodyh, double mmotor,
             double J[3 * 3], const genom_context self)
{
  using namespace Eigen;

  Map<Matrix3d> J_(J);
  double bmass, izz;

  bmass = mass - rotors * mmotor;
  if (bmass <= 0.) {
    wamctrl_e_inval_detail d = { "total motor mass greater than body mass" };
    return wamctrl_e_inval(&d, self);
  }

  /* main body (rectangular cuboid) */
  J_ = Vector3d(
    1/12. * bmass * (mbodyw * mbodyw + mbodyh * mbodyh),
    1/12. * bmass * (mbodyw * mbodyw + mbodyh * mbodyh),
    1/6. * bmass * mbodyw * mbodyw).asDiagonal();

  /* motors (circular loop) */
  izz = rotors * mmotor * armlen * armlen;
  J_ += Vector3d(
    izz / 2.,
    izz / 2.,
    izz).asDiagonal();

  return genom_ok;
}


/* --- wamctrl_scale_inertia -------------------------------------------------- */

genom_event
wamctrl_scale_inertia(double s, double J[3 * 3], const genom_context self)
{
  using namespace Eigen;

  Map<Matrix3d> J_(J);

  J_ *= s;
  return genom_ok;
}
