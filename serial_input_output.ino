/// \file CNCShieldServer/serial_input_output.ino
/// \brief I/O routines to manage line-oriented ASCII commands and status.
///
/// \copyright Written over 2014-2018 by Garth Zeglin <garthz@cmu.edu>.  To the
/// extent possible under law, the author has dedicated all copyright and
/// related and neighboring rights to this software to the public domain
/// worldwide.  This software is distributed without any warranty.  You should
/// have received a copy of the CC0 Public Domain Dedication along with this
/// software.  If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.

// The Arduino IDE combines all .ino files into one compilation unit, so this file will
// exist in the same global namespace as the main .ino file.

// N.B. the function serial_input_poll() directly calls parse_input_message()
// when a complete input line has been received; this function must be provided
// elsewhere in the sketch.

/****************************************************************/

// The maximum message line length.
#define MAX_LINE_LENGTH 80

// The maximum number of tokens in a single message.
#define MAX_TOKENS 10

/****************************************************************/
/**** Utility functions *****************************************/
/****************************************************************/

/// Send a single debugging string to the console.
static void send_debug_message( const char *str )
{
  Serial.print("dbg ");
  Serial.println( str );
}


/****************************************************************/
/// Send a zero-argument message back to the host.
static void send_message( const char *command )
{
  Serial.println( command );
}

/****************************************************************/
/// Send a five-argument message back to the host.
static void send_message( const char *command, long value1, long value2, long value3, long value4, long value5 )
{
  Serial.print( command );
  Serial.print( " " );
  Serial.print( value1 );
  Serial.print( " " );
  Serial.print( value2 );
  Serial.print( " " );
  Serial.print( value3 );
  Serial.print( " " );
  Serial.print( value4 );
  Serial.print( " " );
  Serial.println( value5 );
}

/****************************************************************/
/// Wrapper on strcmp for clarity of code.  Returns true if strings are
/// identical.
static int string_equal( char *str1, const char str2[]) 
{
  return !strcmp(str1, str2);
}

/****************************************************************/
/// Polling function to process messages arriving over the serial port.  Each
/// iteration through this polling function processes at most one character.  It
/// records the input message line into a buffer while simultaneously dividing it
/// into 'tokens' delimited by whitespace.  Each token is a string of
/// non-whitespace characters, and might represent either a symbol or an integer.
/// Once a message is complete, parse_input_message() is called.

static void serial_input_poll(void)
{
  static char input_buffer[ MAX_LINE_LENGTH ];   // buffer for input characters
  static char *argv[MAX_TOKENS];                 // buffer for pointers to tokens
  static int chars_in_buffer = 0;  // counter for characters in buffer
  static int chars_in_token = 0;   // counter for characters in current partially-received token (the 'open' token)
  static int argc = 0;             // counter for tokens in argv
  static int error = 0;            // flag for any error condition in the current message

  // Check if at least one byte is available on the serial input.
  if (Serial.available()) {
    int input = Serial.read();

    // If the input is a whitespace character, end any currently open token.
    if ( isspace(input) ) {
      if ( !error && chars_in_token > 0) {
	if (chars_in_buffer == MAX_LINE_LENGTH) error = 1;
	else {
	  input_buffer[chars_in_buffer++] = 0;  // end the current token
	  argc++;                               // increase the argument count
	  chars_in_token = 0;                   // reset the token state
	}
      }

      // If the whitespace input is an end-of-line character, then pass the message buffer along for interpretation.
      if (input == '\r' || input == '\n') {

	// if the message included too many tokens or too many characters, report an error
	if (error) send_debug_message("excessive input error");

	// else process any complete message
	else if (argc > 0) parse_input_message( argc, argv ); 

	// reset the full input state
	error = chars_in_token = chars_in_buffer = argc = 0;                     
      }
    }

    // Else the input is a character to store in the buffer at the end of the current token.
    else {
      // if beginning a new token
      if (chars_in_token == 0) {

	// if the token array is full, set an error state
	if (argc == MAX_TOKENS) error = 1;

	// otherwise save a pointer to the start of the token
	else argv[ argc ] = &input_buffer[chars_in_buffer];
      }

      // the save the input and update the counters
      if (!error) {
	if (chars_in_buffer == MAX_LINE_LENGTH) error = 1;
	else {
	  input_buffer[chars_in_buffer++] = input;
	  chars_in_token++;
	}
      }
    }
  }
}
/****************************************************************/
