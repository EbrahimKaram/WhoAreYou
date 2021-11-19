/// \file Path.cpp
/// \brief Step and path generator for a single path motor
///
/// \copyright Written over 2014-2018 by Garth Zeglin <garthz@cmu.edu>.  To the
/// extent possible under law, the author has dedicated all copyright and
/// related and neighboring rights to this software to the public domain
/// worldwide.  This software is distributed without any warranty.  You should
/// have received a copy of the CC0 Public Domain Dedication along with this
/// software.  If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#include "Path.h"

//================================================================
Path::Path()
{
  q    = 0.0;
  //We want to set the velocity to 200 for x
  qd   = 0;
  qdd  = 0.0;
  q_d  = 0.0;
  qd_d = 0.0;
  t    = 0;
  //Below changed by Bob for 0.5 Hz. it was 1 Hz before
  k    = 4*M_PI*M_PI;   // 1 Hz; freq = (1/2*pi) * sqrt(k/m); k = (freq*2*pi)^2
  b    = 1.0;           // damping (would be 2*sqrt(k) for critical damping)
  qd_max  = 3500.0;     // typical physical limit for 4x microstepping
  qdd_max = 35000.0;
}

//================================================================
// Path integration running from main event loop.
void Path::pollForInterval(unsigned long interval)
{
  float dt = 1e-6 * interval;

  // calcuate the derivatives
  float qdd = k * (q_d - q) + b * (qd_d - qd);

  // clamp the acceleration within range for safety
  qdd = constrain(qdd, -qdd_max, qdd_max);

  // integrate one time step
  q  += qd  * dt;
  qd += qdd * dt;
  q_d += qd_d * dt;  // integrate the target velocity into the target position
  t += dt;

  // clamp the model velocity within range for safety
  qd = constrain(qd, -qd_max, qd_max);
}
//================================================================
