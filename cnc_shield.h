/// \file cnc_shield.h
/// \brief Hardware I/O definitions for the CNC Arduino Shield.
///
/// \copyright Written over 2014-2018 by Garth Zeglin <garthz@cmu.edu>.  To the
/// extent possible under law, the author has dedicated all copyright and
/// related and neighboring rights to this software to the public domain
/// worldwide.  This software is distributed without any warranty.  You should
/// have received a copy of the CC0 Public Domain Dedication along with this
/// software.  If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.

#ifndef __CNC_SHIELD_H_INCLUDED__
#define __CNC_SHIELD_H_INCLUDED__

// The following specify the essential digital I/O pin assignments for the CNC
// Shield using the standard Arduino pin numbering scheme.

/// Output pin assignments to control the A4988 stepper motor drivers.  The step
/// inputs are triggered on a rising edge, with a minimum 1 microsecond HIGH and
/// LOW pulse widths.

/// If the fourth A axis stepper is used, jumpers are installed to connect A-DIR
/// to D13 (also SpinDir) and A-STEP to D12 (also SpinEnable).


#define X_AXIS_STEP_PIN 2
#define Y_AXIS_STEP_PIN 3
#define Z_AXIS_STEP_PIN 4
#define A_AXIS_STEP_PIN 12     // requires an optional jumper

#define X_AXIS_DIR_PIN 5
#define Y_AXIS_DIR_PIN 6
#define Z_AXIS_DIR_PIN 7
#define A_AXIS_DIR_PIN 13      // requires an optional jumper

#define STEPPER_ENABLE_PIN 8  // active-low (i.e. LOW turns on the drivers)

/// Active-low input pins designated for limit stops.
#define X_LIMIT_PIN 9
#define Y_LIMIT_PIN 10
#define Z_LIMIT_PIN 11

/// Optional spindle control output pins.
#define SPINDLE_ENABLE_PIN 12
#define SPINDLE_DIR_PIN 13  // N.B. this usually is also the onboard LED.

#endif // __CNC_SHIELD_H_INCLUDED__
