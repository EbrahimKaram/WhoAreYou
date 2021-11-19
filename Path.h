/// \file Path.h
/// \brief Path generator for a single motor.
///
/// \copyright Written 2018 by Garth Zeglin <garthz@cmu.edu>.  To the extent
/// possible under law, the author has dedicated all copyright and related and
/// neighboring rights to this software to the public domain worldwide.  This
/// software is distributed without any warranty.  You should have received a
/// copy of the CC0 Public Domain Dedication along with this software.  If not,
/// see <http://creativecommons.org/publicdomain/zero/1.0/>.
///
/// \details This class implements several smooth path generators intended for
/// generating gestural motions on a single motor channel.  It assumes a
/// separate controller manages the step generator of closed-loop control of the
/// physical hardware.

#ifndef __PATH_H_INCLUDED__
#define __PATH_H_INCLUDED__

#include <math.h>

// ================================================================
class Path {

private:
  float q;    	   ///< current model position, in dimensionless units (e.g. step or encoder counts)
  float qd;   	   ///< current model velocity in units/sec
  float qdd;  	   ///< current model acceleration, in units/sec/sec
  float q_d;  	   ///< current model target position in dimensionless units
  float qd_d;  	   ///< current model target velocity in dimensionless units/sec
  float t;    	   ///< elapsed model time, in seconds
  float k;    	   ///< proportional feedback gain, in (units/sec/sec)/(units), which is (1/sec^2)
  float b;    	   ///< derivative feedback gain, in (units/sec/sec)/(units/sec), which is (1/sec)
  float qd_max;    ///< maximum allowable speed in units/sec
  float qdd_max;   ///< maximum allowable acceleration in units/sec/sec

public:

  /// Main constructor.
  Path();

  /// Path-integration polling function to be called as often as possible,
  /// typically from the main event loop.  The interval argument is the duration
  /// in microseconds since the last call.
  void pollForInterval(unsigned long interval);

  /// Add a signed offset to the target position.  The units are dimensionless
  /// 'steps'.  If using a microstepping driver, these may be less than a
  /// physical motor step.
  void incrementTarget(long offset) { q_d += offset; }

  /// Set the absolute target position in dimensionless units.
  void setTarget(long position) { q_d = position; }

  /// Set the target velocity in dimensionless units/second.
  void setVelocity(long velocity) { qd_d = velocity; }

  /// Return the current position in dimensionless units.
  long currentPosition(void) { return (long) q; }

  /// Return the current velocity in units/second.
  long currentVelocity(void) { return (long) qd; }

  /// Configure the second-order model gains.
  void setPDgains(float k_new, float b_new) { k = k_new; b = b_new; }

  /// Convenience function to set second order model gains in terms of natural frequency and damping ratio.
  /// The frequency is in Hz, the damping ratio is 1.0 at critical damping.
  void setFreqDamping(float freq, float damping) {
    k = freq * freq * 4 * M_PI * M_PI;
    b = 2 * sqrtf(k) * damping;
  }

  /// Configure the velocity and acceleration limits.
  void setLimits(float qdmax, float qddmax) { qd_max = qdmax; qdd_max = qddmax; }
};

#endif //__PATH_H_INCLUDED__
