/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
/* Copyright (c) 2011-2013 BEEVC - Electronic Systems   */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/


#include	<string.h>
#include <stdbool.h>

#include	"serial.h"
#include	"sermsg.h"

#include	"gcode_parse.h"
#include	"gcode_process.h"
#include        "machine.h"
#include        "config.h"

GCODE_COMMAND next_target;

static uint8_t last_field = 0;

#define crc(a, b)		(a ^ b)

static struct {
  uint8_t sign; 
  int exponent;
  } read_digit;
static double value;

// accept the next character and process it
void gcode_parse_char(uint8_t c);

// uses the global variable next_target.N
void request_resend(void);


void gcode_parse_init(void)
{

  next_target.target.feed_rate = config.homing_feedrate_z;
}


/*
	utility functions
*/

#if 0
static int32_t	decfloat_to_int(decfloat *df, int32_t multiplicand, int32_t denominator)
{
  int64_t	r = df->mantissa;
  uint8_t	e = df->exponent;

  // e=1 means we've seen a decimal point but no digits after it, and e=2 means we've seen a decimal point with one digit so it's too high by one if not zero
  if (e)
  {
    e--;
  }

  // scale factors
  if (multiplicand != 1)
  {
    r *= multiplicand;
  }
  if (denominator != 1)
  {
    r /= denominator;
  }

  // sign
  if (df->sign)
  {
    r = -r;
  }

  // exponent- try to keep divides to a minimum for common (small) values at expense of slightly more code
  while (e >= 5)
  {
    r /= 100000;
    e -= 5;
  }

  if (e == 1)
  {
    r /= 10;
  }
  else if (e == 2)
  {
    r /= 100;
  }
  else if (e == 3)
  {
    r /= 1000;
  }
  else if (e == 4)
  {
    r /= 10000;
  }

  return r;
}
#endif

double power (double x, int exp)
{  
  double result = 1.0;
  while (exp--)
  {
    result = result * x;
  }

  return result;
}

double inch_to_mm (double inches)
{  
  return inches * 25.4;
}

/*
	public functions
*/

eParseResult gcode_parse_line (tLineBuffer *pLine) 
{
    int j;
    eParseResult result = PR_OK;

    for (j=0; j < pLine->len; j++){
        gcode_parse_char (pLine->data [j]);
    }

    // process
    result = process_gcode_command();

    // reset variables
    next_target.seen_X = next_target.seen_Y = next_target.seen_Z = \
    next_target.seen_E = next_target.seen_F = next_target.seen_S = \
    next_target.seen_P = next_target.seen_N = next_target.seen_M = \
    next_target.seen_N = next_target.seen_B = next_target.seen_A = \
    next_target.seen_T = next_target.seen_U = next_target.seen_V = \
    next_target.seen_checksum = next_target.seen_semi_comment = \
    next_target.seen_parens_comment = next_target.checksum_read = \
    next_target.checksum_calculated = next_target.seen_D = \
    next_target.seen_W = next_target.seen_L = 0;
    next_target.chpos = 0;
    last_field = 0;
    read_digit.sign = read_digit.exponent = 0;
    value = 0;

    // dont assume a G1 by default
    next_target.seen_G = 0;
    next_target.G = 0;
    next_target.A = 0;

    if (next_target.option_relative)
    {
      next_target.target.x = next_target.target.y = next_target.target.z = 0.0;
      next_target.target.e = 0.0;
    }

    return result;
}

/****************************************************************************
*                                                                           *
* Character Received - add it to our command                                *
*                                                                           *
****************************************************************************/
void gcode_parse_char(uint8_t c) 
{
  uint8_t save_ch;

  #ifdef ASTERISK_IN_CHECKSUM_INCLUDED
  if (next_target.seen_checksum == 0)
  {
    next_target.checksum_calculated = crc(next_target.checksum_calculated, c);
  }
  #endif

  save_ch = c;

  // uppercase
  if (c >= 'a' && c <= 'z')
  {
    c &= ~32;
  }

  // process previous field
  if (last_field)
  {
    // check if we're seeing a new field or end of line
    // any character will start a new field, even invalid/unknown ones
    if ((c >= 'A' && c <= 'Z') || c == '*' || (c == 10) || (c == 13))
    {
      // before using value, apply the sign
      if (read_digit.sign)
      {
        value = -value;
      }

      switch (last_field)
      {
        case 'G':
        next_target.G = value;
        break;

        case 'M':
        next_target.M = value;
        // this is a bit hacky since string parameters don't fit in general G code syntax
        // NB: filename MUST start with a letter and MUST NOT contain spaces
        // letters will also be converted to uppercase
        if ((next_target.M == 23) || (next_target.M == 30) || (next_target.M == 639))
        {
          next_target.getting_string = 1;
        }
        break;

        case 'X':
        if (next_target.option_inches)
        {
          next_target.target.x = inch_to_mm(value);
        }
        else
        {
          next_target.target.x = value;
        }
        break;

        case 'Y':
        if (next_target.option_inches)
        {
          next_target.target.y = inch_to_mm(value);
        }
        else
        {
          next_target.target.y = value;
        }
        break;

        case 'Z':
        if (next_target.option_inches)
        {
          next_target.target.z = inch_to_mm(value);
        }
        else
        {
          next_target.target.z = value;
        }
        break;

        case 'E':
        if (next_target.option_inches)
        {
          next_target.target.e = inch_to_mm(value);
        }
        else
        {
          next_target.target.e = value;
        }
        break;

        case 'F':
        if (next_target.option_inches)
        {
          next_target.target.feed_rate = inch_to_mm(value);
        }
        else
        {
          next_target.target.feed_rate = value;
        }
        break;

        case 'S':
        next_target.S = value;
        break;

        case 'P':
        // if this is dwell, multiply by 1000 to convert seconds to milliseconds
        if (next_target.G == 4)
        {
          next_target.P = value * 1000.0;
        }
        else
        {
          next_target.P = value;
        }
        break;

        case 'N':
        next_target.N = value;
        break;

        case 'A':
        next_target.A = value;
        break;

        case 'D':
        next_target.D = value;
        break;

        case 'L':
        next_target.L = value;
        break;

        case 'T':
        config.kp = value;
        break;

        case 'U':
        config.ki = (value/1000);
        break;

        case 'V':
        config.kd = value;
        break;

        case 'W':
        next_target.W = value;
        break;

        case '*':
        next_target.checksum_read = value;
        break;

        case ';':
        case '^':
        next_target.seen_semi_comment = 1;
        break;
      }

      // reset for next field
      last_field = 0;
      read_digit.sign = read_digit.exponent = 0;
      value = 0;
    }
  }

  if (next_target.getting_string)
  {
    if ((c == 10) || (c == 13) || ( c == ' ')  || ( c == '*'))
    {
      next_target.getting_string = 0;
    }
    else
    {
      if (next_target.chpos < sizeof(next_target.filename))
      {
        next_target.filename [next_target.chpos++] = c;
        next_target.filename [next_target.chpos] = 0;
      }
    }      
  }

  // skip comments, filenames
  if (next_target.seen_semi_comment == 0 && next_target.seen_parens_comment == 0 && next_target.getting_string == 0)
  {
    // new field?
    if ((c >= 'A' && c <= 'Z') || c == '*')
    {
      last_field = c;
    }

    // process character
    switch (c)
    {
      // each currently known command is either G or M, so preserve previous G/M unless a new one has appeared
      // FIXME: same for T command
      case 'G':
      next_target.seen_G = 1;
      next_target.seen_M = 0;
      next_target.M = 0;
      break;

      case 'M':
      next_target.seen_M = 1;
      next_target.seen_G = 0;
      next_target.G = 0;
      break;

      case 'X':
      next_target.seen_X = 1;
      break;

      case 'Y':
      next_target.seen_Y = 1;
      break;

      case 'Z':
      next_target.seen_Z = 1;
      break;

      case 'E':
      next_target.seen_E = 1;
      break;

      case 'F':
      next_target.seen_F = 1;
      break;

      case 'S':
      next_target.seen_S = 1;
      break;

      case 'P':
      next_target.seen_P = 1;
      break;

      case 'N':
      next_target.seen_N = 1;
      break;

      case 'A':
      next_target.seen_A = 1;
      break;

      case 'L':
      next_target.seen_L = 1;
      break;

      case 'D':
      next_target.seen_D = 1;
      break;

      case 'B':
      next_target.seen_B = 1;
      break;

      case 'T':
      next_target.seen_T = 1;
      break;

      case 'U':
      next_target.seen_U = 1;
      break;

      case 'V':
      next_target.seen_V = 1;
      break;

      case 'W':
      next_target.seen_W = 1;
      break;

      case '*':
      next_target.seen_checksum = 1;
      break;

      // comments
      case ';':
      case '^':
      next_target.seen_semi_comment = 1;
      break;

      case '(':
      next_target.seen_parens_comment = 1;
      break;

      // now for some numeracy
      case '-':
      read_digit.sign = 1;
      // force sign to be at start of number, so 1-2 = -2 instead of -12
      read_digit.exponent = 0;
      break;

      case '.':
      if (read_digit.exponent == 0)
      {
        read_digit.exponent = 1;
      }
      break;

      default:
      // can't do ranges in switch..case, so process actual digits here
      if (c >= '0' && c <= '9')
      {
        if (read_digit.exponent == 0)
        {
          value = value * 10 + (c - '0');
        }
        else
        {
          value += (double)(c - '0') / power(10, read_digit.exponent);
        }

        if (read_digit.exponent)
        {
          read_digit.exponent++;
        }
      }
    }
  }
  else if (next_target.seen_parens_comment == 1 && c == ')')
  {
    next_target.seen_parens_comment = 0; // recognize stuff after a (comment)
  }

  #ifndef ASTERISK_IN_CHECKSUM_INCLUDED
  if (next_target.seen_checksum == 0)
  {
    next_target.checksum_calculated = crc(next_target.checksum_calculated, save_ch);
  }
  #endif
}

/****************************************************************************
*                                                                           *
* Request a resend of the current line - used from various places.          *
*                                                                           *
* Relies on the global variable next_target.N being valid.                  *
*                                                                           *
****************************************************************************/
void request_resend(void)
{
  serial_writestr("rs ");
  serwrite_uint8(next_target.N);
  serial_writestr("\r\n");
}
