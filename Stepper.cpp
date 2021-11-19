/// \file Stepper.cpp
/// \brief Step and path generator for a single stepper motor
///
/// \copyright Written over 2014-2018 by Garth Zeglin <garthz@cmu.edu>.  To the
/// extent possible under law, the author has dedicated all copyright and
/// related and neighboring rights to this software to the public domain
/// worldwide.  This software is distributed without any warranty.  You should
/// have received a copy of the CC0 Public Domain Dedication along with this
/// software.  If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.

#include "Stepper.h"
#include <Arduino.h>

Stepper::Stepper(uint8_t _step_pin, uint8_t _dir_pin)
{
  step_pin = _step_pin;
  dir_pin  = _dir_pin;
  position = 0;
  target   = 0;
  elapsed  = 0;

  step_interval = 200;  // 200 microseconds = 5000 steps/sec
}
//================================================================
// Step generator running on fast timer interrupt.
void Stepper::pollForInterval(unsigned long interval)
{
  // Accumulated the time elapsed since the step.
  elapsed += interval;

  if (elapsed >= step_interval) {
    // reset the timer according to the target interval to produce a correct an
    // average rate even if extra time has passed
    elapsed -= step_interval;

    // check whether to emit a step
    if (position != target) {

      // always set the direction to match the sign of motion
      digitalWrite(dir_pin, (position < target) ? HIGH : LOW);

      // emit a step
      digitalWrite(step_pin, HIGH);

      // update the position count
      if (position < target) position++;
      else position--;
      digitalWrite(step_pin, LOW);
    }
  }
}
//================================================================
