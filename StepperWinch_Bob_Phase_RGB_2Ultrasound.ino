/// \file StepperWinch.ino
/// \brief Driver for four-channel stepper-motor capstan winch system.
///
/// \copyright Written over 2014-2018 by Garth Zeglin <garthz@cmu.edu>.  To the
/// extent possible under law, the author has dedicated all copyright and
/// related and neighboring rights to this software to the public domain
/// worldwide.  This software is distributed without any warranty.  You should
/// have received a copy of the CC0 Public Domain Dedication along with this
/// software.  If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
///
/// \details This sketch is designed to support the expressive gestural motion
/// of a kinetic fabric sculpture rather than perform precision trajectory
/// tracking as would be typical for CNC applications.  The communicates over
/// the serial port to receive motion commands and report status.  The protocol
/// is event-driven and implements parameterized gestural motions; it is
/// patterned after MIDI or OSC conventions.  This sketch assumes the following:
///
/// 1. The TimerOne library is installed in the Arduino IDE.
/// 2. Four A4988 stepper motor drivers are connected following the CNC Shield pin conventions.
/// 3. If using a CNC Shield board, A-axis jumpers are installed to connect A-DIR to D13 (also SpinDir) and A-STEP to D12 (also SpinEnable)..
/// 4. The serial communication baud rate is 115200 bps.
///
/// Typically, for 200 step/rev (1.8 deg) stepper motors the drivers are
/// configured for 1/4 step microstepping (MS2 pulled high).  However, the
/// protocol commands use integer step units so the code does not depend on
/// this.

// ================================================================
// Import the TimerOne library to support timer interrupt processing.
#include "TimerOne.h"

// Include the other modules from this sketch.
#include "cnc_shield.h"
#include "Stepper.h"
#include "Path.h"

// ================================================================
// Communication protocol.

// The message protocol is based on plain-text commands sent as keywords and
// values in a line of text.  One line is one message.  All the input message
// formats begin with a string naming a specific command or destination followed
// by one or two arguments.  The output formats are similar but include more
// general debugging output with a variable number of tokens.

// ----------------------------------------------------------------
// The following global messages are not channel-specific.

// Command	Arguments		Meaning
// ping                                 query whether the server is running
// version				query the identity of the sketch
// srate        <value>                 set the status reporting interval in milliseconds
// enable       <value>                 enable or disable all driver outputs, value is 0 or non-zero

// ----------------------------------------------------------------
// The following messages include a token representing the flag set specifying
// the affected axes.  The flag set should include one or more single-letter
// channel specifiers (regex form: "[xyza]+").  Note that the flag set cannot be
// empty.

// --------------------------------

// Absolute move. There should be an integer target value corresponding to each
// included channel; each controller target is set to the specified position.
// The motion is not coordinated; different channels may finish at different
// times.  Note that this command will enable all drivers.

//   a <flags> <offset>+
//
// Examples:
//   a xyza 100 120 -200 -50	move the axes to the specified locations
//   a x 50			move the X axis to +50

// --------------------------------

// Relative move. There should be an offset value corresponding to each included
// channel; each controller target is incremented by the specified amount.  The
// motion is not coordinated; different channels may finish at different times.
// Note that this command will enable all drivers.
//
//   d <flags> <offset>+
//
// Examples:
//   d xyza 10 10 -10 -10	move all axes ten steps (two forward, two backward)
//   d x 50			move the X axis fifty steps forward

// --------------------------------

// Set velocity. There should be an integer velocity value corresponding to each
// included channel; each controller target velocity is set to the amount
// specified in units/sec.  Note that this command will enable all drivers.
//
//   v <flags> <value>+
//
// Examples:
//   v xyza 10 10 -10 -10	set all axes to drift 10 steps per second (two forward, two backward)
//   v x 500			set the X axis to constantly move forward at roughly half speed

// --------------------------------
// Set second-order gains.  The same dynamic parameters are applied to all included channels.
//   g <flags> <frequency (Hz)> <damping-ratio>
//
// Examples:
//   g xyza 2.0 1.0		set all channels to 1 Hz natural frequency with critical damping
//   g xyza 0.1 0.5		set all channels to 0.1 Hz natural frequency and underdamping

// --------------------------------
// Set velocity and acceleration limits.  The same dynamic parameters are applied to all included channels.
//   l <flags> <maximum velocity (steps/sec)> <maximum acceleration (steps/sec/sec)>
//
// Examples:
//   l xyza 4000 40000		set all channels to 4000 steps/sec and 40000 steps/sec/sec

// ----------------------------------------------------------------
// This program generates the following messages:

// Command	Arguments		Meaning
// awake                                initialization has completed or ping was received
// txyza        <usec> <x> <y> <z> <a>  Arduino clock time in microseconds, followed by absolute step position
// dbg		<value-or-token>+	debugging message to print for user
// id		<tokens>+		tokens identifying the specific sketch

// ================================================================
// Global variables and constants.

// The baud rate is the number of bits per second transmitted over the serial port.
#define BAUD_RATE 115200
// if we are doing 24 frames a second need to set and reset every 21 ms

// Interval in microseconds between status messages. 10^6 microseconds in a second
static unsigned long status_poll_interval = 200000; // 5 Hz message rate to start

/// Control objects for the stepper channels.  The declaration statically
/// initializes the global state objects for the channels.  Note that this does
/// not initialize the hardware; that is performed in setup().
static Stepper x_axis(X_AXIS_STEP_PIN, X_AXIS_DIR_PIN);
static Stepper y_axis(Y_AXIS_STEP_PIN, Y_AXIS_DIR_PIN);
static Stepper z_axis(Z_AXIS_STEP_PIN, Z_AXIS_DIR_PIN);
static Stepper a_axis(A_AXIS_STEP_PIN, A_AXIS_DIR_PIN);

volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
volatile bool newPulseDurationAvailable = false;

// we want to start with the color white
static int color[] = {255, 255, 255};
/// Path generator object for each channel.
static Path x_path, y_path, z_path, a_path;

/// The timestamp in microseconds for the last polling cycle, used to compute
/// the exact interval between stepper motor updates.
static unsigned long last_interrupt_clock = 0;

/// Identification string.
static const char version_string[] = "id StepperWinch " __DATE__;

// ================================================================
/// Enable or disable the stepper motor drivers.  The output is active-low,
/// so this inverts the sense.
static inline void set_driver_enable(int value)
{
  digitalWrite(STEPPER_ENABLE_PIN, (value != 0) ? LOW : HIGH);
}

// ================================================================
/// Interrupt handler to update all the stepper motor channels.  Note that this
/// is called from a timer interrupt context, so it should take as little time as
/// feasible and cannot use serial I/O (i.e. no debugging messages).
void stepper_output_interrupt(void)
{
  // read the clock
  unsigned long now = micros();

  // Compute the time elapsed since the last poll.  This will correctly handle wrapround of
  // the 32-bit long time value given the properties of twos-complement arithmetic.
  unsigned long interval = now - last_interrupt_clock;
  last_interrupt_clock = now;

  // Update all the stepper channels. This may emit step signals or simply
  // update the timing and state variables.
  x_axis.pollForInterval(interval);
  y_axis.pollForInterval(interval);
  z_axis.pollForInterval(interval);
  a_axis.pollForInterval(interval);
}

// ================================================================
/// Polling function called from the main event loop to update the path model
/// and update the step generators.
void path_poll(unsigned long interval)
{
  x_path.pollForInterval(interval);
  y_path.pollForInterval(interval);
  z_path.pollForInterval(interval);
  a_path.pollForInterval(interval);

  // update the step generator for new targets
  x_axis.setTarget(x_path.currentPosition());
  x_axis.setSpeed(abs(x_path.currentVelocity()));

  y_axis.setTarget(y_path.currentPosition());
  y_axis.setSpeed(abs(y_path.currentVelocity()));

  z_axis.setTarget(z_path.currentPosition());
  z_axis.setSpeed(abs(z_path.currentVelocity()));

  a_axis.setTarget(a_path.currentPosition());
  a_axis.setSpeed(abs(a_path.currentVelocity()));
}
// ================================================================
/// Return a Path object or NULL for each flag in the flag token.  As a side effect, updates
/// the source pointer, leaving it at the terminating null.
static Path *path_flag_iterator(char **tokenptr)
{
  char flag = **tokenptr;
  if (flag == 0)
    return NULL;
  else
  {
    (*tokenptr) += 1;
    switch (flag)
    {
    case 'x':
      return &x_path;
    case 'y':
      return &y_path;
    case 'z':
      return &z_path;
    case 'a':
      return &a_path;
    default:
      return NULL;
    }
  }
}

// ================================================================
/// Process an input message.  Unrecognized commands are silently ignored.
///   argc - number of argument tokens
///   argv - array of pointers to strings, one per token

static void parse_input_message(int argc, char *argv[])
{
  if (argc == 0)
    return;

  // Interpret the first token as a command symbol.
  char *command = argv[0];

  if (string_equal(command, "enable"))
  {
    if (argc > 1)
      set_driver_enable(atoi(argv[1]));
  }
  else if (string_equal(command, "a"))
  {
    if (argc > 2)
    {
      set_driver_enable(1);
      char *flags = argv[1];
      int channel = 0;
      while (*flags)
      {
        Path *p = path_flag_iterator(&flags);
        if (p)
        {
          if (argc > (channel + 2))
          {
            p->setTarget(atol(argv[channel + 2]));
            channel++;
          }
        }
      }
    }
  }
  else if (string_equal(command, "d"))
  {
    if (argc > 2)
    {
      set_driver_enable(1);
      char *flags = argv[1];
      int channel = 0;
      while (*flags)
      {
        Path *p = path_flag_iterator(&flags);
        if (p)
        {
          if (argc > (channel + 2))
          {
            p->incrementTarget(atol(argv[channel + 2]));
            channel++;
          }
        }
      }
    }
  }
  else if (string_equal(command, "v"))
  {
    if (argc > 2)
    {
      set_driver_enable(1);
      char *flags = argv[1];
      int channel = 0;
      while (*flags)
      {
        Path *p = path_flag_iterator(&flags);
        if (p)
        {
          if (argc > (channel + 2))
          {
            p->setVelocity(atol(argv[channel + 2]));
            channel++;
          }
        }
      }
    }
  }
  else if (string_equal(command, "g"))
  {
    if (argc > 3)
    {
      char *flags = argv[1];
      float frequency = atof(argv[2]);
      float damping_ratio = atof(argv[3]);
      while (*flags)
      {
        Path *p = path_flag_iterator(&flags);
        if (p)
          p->setFreqDamping(frequency, damping_ratio);
      }
    }
  }
  else if (string_equal(command, "l"))
  {
    if (argc > 3)
    {
      char *flags = argv[1];
      float qdmax = atof(argv[2]);
      float qddmax = atof(argv[3]);
      while (*flags)
      {
        Path *p = path_flag_iterator(&flags);
        if (p)
          p->setLimits(qdmax, qddmax);
      }
    }
  }
  else if (string_equal(command, "version"))
  {
    send_message(version_string);
  }
  else if (string_equal(command, "ping"))
  {
    send_message("awake");
  }
  else if (string_equal(command, "srate"))
  {
    if (argc > 1)
    {
      long value = atol(argv[1]);
      // set the reporting interval (milliseconds -> microseconds)
      if (value > 0)
        status_poll_interval = 1000 * value;
      else
        send_debug_message("invalid srate value");
    }
  }
  // Added By Bob for the colors setting
  // Need to send the color in terms of RGB setting
  else if (string_equal(command, "color"))
  {
    if (argc > 3)
    {
      int red = atof(argv[1]);
      int green = atof(argv[2]);
      int blue = atof(argv[3]);
      if ((red >= 0) && (red <= 255))
      {
        color[0] = red;
      }
      if ((green >= 0) && (green <= 255))
      {
        color[1] = green;
      }
      if ((blue >= 0) && (blue <= 255))
      {
        color[2] = blue;
      }

      SetColor(color);
    }
  }
}

/****************************************************************/
/// Polling function to send status reports at periodic intervals.
static void status_poll(unsigned long interval)
{
  // static variables are intialized only once
  //https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/static/
  // Static Variables are like global variables that are
  static long timer = 0;
  timer -= interval;

  if (timer < 0)
  {
    timer += status_poll_interval;

    // send a time and position reading
    long clock = micros();
    long x = x_axis.currentPosition();
    long y = y_axis.currentPosition();
    long z = z_axis.currentPosition();
    long a = a_axis.currentPosition();
    send_message("txyza", clock, x, y, z, a);
  }
}

#define ECHO_PRESENCE A4 //SDA
#define TRIG_PRESENCE A5 // SCL
static bool start = false;
#define WAIT_ITERATIONS 6
static void sensePresence(unsigned long interval)
{
  static long timer = 0;
  //every 5 seconds get a reading. every 50 seconds if no one is there
  static int wait_for_interations = WAIT_ITERATIONS;
  timer -= interval;
  long duration; // variable for the duration of sound wave travel
  int distance;  // variable for the distance measurement
  if (timer < 0)
  {
    //if we want to do this every second. This is in microsecond. so times 1 million
    //https://stackoverflow.com/questions/14220217/representing-big-numbers-in-source-code-for-readability#
    timer += 5L * 1000000UL;

    digitalWrite(TRIG_PRESENCE, LOW);
    delayMicroseconds(1);
    // Sets the TRIG_PRESENCE HIGH (ACTIVE) for 10 microseconds
    digitalWrite(TRIG_PRESENCE, HIGH);
    // this was 5
    delayMicroseconds(5);
    digitalWrite(TRIG_PRESENCE, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    // TODO: pulse in is blocking we have to do an interrupt
    //https://roboticsbackend.com/arduino-pulsein-with-interrupts/#:~:text=The%20problem%20with%20pulseIn(),can%20also%20be%20quite%20big.

    duration = pulseIn(ECHO_PRESENCE, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    // Displays the distance on the Serial Monitor
    if ((!start) && (distance < 60))
    {
      start = true;
      wait_for_interations = WAIT_ITERATIONS;
    }
    else
    {
      wait_for_interations--;
      if (wait_for_interations < 1)
      {
        start = false;
        wait_for_interations = WAIT_ITERATIONS;
      }
    }
    if (Serial)
    {
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
    }
  }
}

#define ECHO_COLOR 3  // this is pin 3 on the arduino that would alter the system
#define TRIG_COLOR 12 // This is D12 Spindle

static void changeColor(unsigned long interval)
{
  static long timer = 0;
  timer -= interval;
  long duration; // variable for the duration of sound wave travel
  int distance;  // variable for the distance measurement

  if (timer < 0)
  {
    // We want to do this every second
    timer = 1L * 1000000UL;
    digitalWrite(TRIG_COLOR, HIGH);
    delayMicroseconds(2);
    digitalWrite(TRIG_COLOR, LOW);
  }

  if (newPulseDurationAvailable)
  {
    newPulseDurationAvailable = false;
    unsigned long pulseDuration = pulseInTimeEnd - pulseInTimeBegin;
    distance = pulseDuration * 0.034 / 2;
    if (Serial)
    {
      Serial.print("Color Senor Read: ");
      Serial.print(distance);
      Serial.println(" cm");
    }
    int red[3] = {255, 0, 0};
    int green[3] = {0, 255, 0};
    int blue[3] = {0, 0, 255};
    // this should only change when coloring is set
    if (start)
    {
      if (distance < 60)
      {
        SetColorVariable(red);

//        setColor(red);
      }
      else if (distance > 60)
      {
//        setColor(blue);
        SetColorVariable(blue);
      }
//      else
//      {
////        setColor(green);
//        SetColorVariable(green);
//      }
    }
  }
}
static void toggleDevice()
{
  static bool machine_on = false;
  if (start && !machine_on)
  {
    set_driver_enable(1);
    x_path.setVelocity(200);
    x_axis.setSpeed(200);
    machine_on = true;
  }
  else if (!start && machine_on)
  {
    set_driver_enable(1);
    x_path.setVelocity(0);
    x_axis.setSpeed(0);
    machine_on = false;
  }
}

/****************************************************************/
/**** Standard entry points for Arduino system ******************/
/****************************************************************/

/// Standard Arduino initialization function to configure the system.\
// RGB
#define RED_LIGHT 11  // Z end STOP
#define GREEN_LIGHT 9 // X end STOP
#define BLUE_LIGHT 10 // Y end STOP

void setup(void)
{
  // set up the CNC Shield I/O
  digitalWrite(STEPPER_ENABLE_PIN, HIGH); // initialize drivers in disabled state
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);

  pinMode(X_AXIS_STEP_PIN, OUTPUT);

  // We wont using the Y axis Step simply because it will be used to get the interrupts
  // pinMode(Y_AXIS_STEP_PIN, OUTPUT);

  pinMode(Z_AXIS_STEP_PIN, OUTPUT);
  pinMode(A_AXIS_STEP_PIN, OUTPUT);

  // For the presence UltraSound Sensor
  pinMode(TRIG_PRESENCE, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(ECHO_PRESENCE, INPUT);

  // for the color change UltraSound Sensor
  pinMode(TRIG_COLOR, OUTPUT);
  pinMode(ECHO_COLOR, INPUT);

  attachInterrupt(digitalPinToInterrupt(ECHO_COLOR),
                  buttonPinInterrupt,
                  CHANGE);

  //We need to attach an interrupt on the ECHO pin
  // Reference: https://roboticsbackend.com/arduino-pulsein-with-interrupts/#:~:text=The%20problem%20with%20pulseIn(),can%20also%20be%20quite%20big.
  // The Echo signal needs to go from low to high and then high to low. The length of it being high is the distance we want
  // https://www.tindie.com/products/upgradeindustries/hc-sr05-hy-srf05-precision-ultrasonic-sensor/
  //TODO: send the trigger every 5 seconds and wait for the response

  // for the RGB Light
  pinMode(RED_LIGHT, OUTPUT);
  digitalWrite(RED_LIGHT, true);
  pinMode(BLUE_LIGHT, OUTPUT);
  digitalWrite(BLUE_LIGHT, true);
  pinMode(GREEN_LIGHT, OUTPUT);
  digitalWrite(GREEN_LIGHT, true);

  pinMode(X_AXIS_DIR_PIN, OUTPUT);
  pinMode(Y_AXIS_DIR_PIN, OUTPUT);
  pinMode(Z_AXIS_DIR_PIN, OUTPUT);
  pinMode(A_AXIS_DIR_PIN, OUTPUT);

#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
#endif

  // initialize the Serial port
  Serial.begin(BAUD_RATE);

  // set up the timer1 interrupt and attach it to the stepper motor controls
  last_interrupt_clock = micros();
  Timer1.initialize(100); // 100 microsecond intervals, e.g. 10kHz
  Timer1.attachInterrupt(stepper_output_interrupt);

  //We might want to do this to enable and disable the zeotrope
  set_driver_enable(1);
  x_path.setFreqDamping(0.5, 1);
  // x_path.setVelocity(200);
  // x_axis.setSpeed(200);

  // send a wakeup message
  send_message("awake");

  //It would be best to set the natural frequency and speed here somehow
}

void buttonPinInterrupt()
{
  if (digitalRead(ECHO_COLOR) == HIGH)
  {
    // start measuring
    pulseInTimeBegin = micros();
  }
  else
  {
    // stop measuring
    pulseInTimeEnd = micros();
    newPulseDurationAvailable = true;
  }
}

/****************************************************************/
/// Standard Arduino polling function to handle all I/O and periodic processing.
/// This loop should never be allowed to stall or block so that all tasks can be
/// constantly serviced.
static unsigned long last_event_loop = 0;

unsigned long last_strobe_trigger = 0;
bool strobe = false;

#define STEPS_PER_REVOLUTION 200
//the below is in microteps
// then we have the number of frames per circle. which is 12. we divide 200/12= 16.666 microsteps
#define THICKNESS_OF_FRAME 3

//TODO: need to add a color setter in the serial read would allow for some fun settings

//TODO: turn on and off based on the Ultrasound reader
// TODO: change color based on the reading from another ultrasound reader
void loop(void)
{

  // read the clock
  unsigned long now = micros();

  // Compute the time elapsed since the last polling cycle.  This will correctly handle wrapround of
  // the 32-bit long time value given the properties of twos-complement arithmetic.
  unsigned long interval = now - last_event_loop;
  last_event_loop = now;

  // We shouldn't read from Serial if serial is not even connected
  if (Serial)
  {
    serial_input_poll();
    status_poll(interval);
  }

  path_poll(interval);
  sensePresence(interval);
  toggleDevice();
  changeColor(interval);

  // We are using the x position of the servo motor.

  unsigned long position_x = x_axis.currentPosition();

  unsigned long offset = position_x % STEPS_PER_REVOLUTION;
  // in order to have some kind of resoluton, we will multiply by 100
  offset = offset * 100;

  if ((offset % 1666) <= (THICKNESS_OF_FRAME * 100))
  {
    if (!strobe)
    {

      //Please note that color is a globl variable
      SetColor(color);
      strobe = true;
    }
  }

  else
  {
    if (strobe)
    {
      strobe = false;
      //This needs to turn off the light
      digitalWrite(RED_LIGHT, false);
      digitalWrite(BLUE_LIGHT, false);
      digitalWrite(GREEN_LIGHT, false);
    }
  }

  // other polled tasks can go here
  // The CNC Shield
  // Reference: https://forum.arduino.cc/t/x-1000l-what-does-the-l-do/45843
  // This way I'm not capped at 32,767 which is the default values for ints

  // CNC arduino Shield
  // https://courses.ideate.cmu.edu/16-375/f2018/text/resrc/cnc-shield.html
}

//RGB color setter
void SetColor(int rgb[])
{
  const uint8_t RGB_Pins[3] = {RED_LIGHT, GREEN_LIGHT, BLUE_LIGHT};

  for (int i = 0; i < 3; i++)
  {
    // don't need to set the value
    if (rgb[i] >= 0)
    {
      analogWrite(RGB_Pins[i], rgb[i]);

      // TODO: don't know iff
      //      color[i] = rgb[i];
    }
  }
}

void SetColorVariable(int colorToSet[])
{
  for (int i = 0; i < 3; i++)
  {
    color[i] = colorToSet[i];
  }
}
/****************************************************************/
/****************************************************************/
