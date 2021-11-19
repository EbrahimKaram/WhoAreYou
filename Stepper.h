/// \file Stepper.h
/// \brief Step generator for a single stepper motor.
///
/// \copyright Written over 2014-2018 by Garth Zeglin <garthz@cmu.edu>.  To the
/// extent possible under law, the author has dedicated all copyright and
/// related and neighboring rights to this software to the public domain
/// worldwide.  This software is distributed without any warranty.  You should
/// have received a copy of the CC0 Public Domain Dedication along with this
/// software.  If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
///
/// \details This class implements fast constant-velocity stepping.  It is
/// possible to use this directly, but the overall sketch pairs this with a
/// interpolating path generator which frequently updates the position and
/// velocity setpoints.

#ifndef __STEPPER_H_INCLUDED__
#define __STEPPER_H_INCLUDED__

#include <stdint.h>

/// An instance of this class manages generation of step and direction signals
/// for one stepper motor.
class Stepper {

private:
  /****************************************************************/
  // The following instance variables may only be modified from a non-interrupt
  // context, i.e., not within poll().

  /// the I/O pins for this channel designated using the Arduino convention
  uint8_t step_pin, dir_pin;

  /// the target position in dimensionless step counts
  long target;

  /// the interval in microseconds between steps
  unsigned long step_interval;

  /****************************************************************/
  // The following instance variables may be modified within poll() from an interrupt context.

  /// the current position in dimensionless step counts
  long position;

  /// the time elapsed in microseconds since the last step occurred
  unsigned long elapsed;

  /****************************************************************/
public:

  /// Main constructor.  The arguments are the pin numbers for the step and
  /// direction outputs. Note: this does not initialize the underlying hardware.
  Stepper(uint8_t step_pin, uint8_t dir_pin);

  /// Step-generator polling function to be called as often as possible.  This
  /// is typically called from a timer interrupt.  The interval argument is the
  /// duration in microseconds since the last call.
  void pollForInterval(unsigned long interval);

  /// Add a signed offset to the target position.  The units are dimensionless
  /// 'steps'.  If using a microstepping driver, these may be less than a
  /// physical motor step.
  void incrementTarget(long offset) { target += offset; }

  /// Set the absolute target position.
  void setTarget(long position) { target = position; }

  /// Return the current position in dimensionless 'steps'.
  long currentPosition(void) { return position; }

  /// Set a constant speed in steps/second.  Note that the value must be
  /// non-zero and positive.  The maximum rate available is a function of the
  /// polling rate.
  void setSpeed(int speed) {
    // (1000000 microseconds/second) / (steps/second) = (microseconds/step)
    if (speed > 0) {
      step_interval = 1000000 / speed;
      if (step_interval == 0) step_interval = 1;
    }
  }

};

#endif //__STEPPER_H_INCLUDED__
