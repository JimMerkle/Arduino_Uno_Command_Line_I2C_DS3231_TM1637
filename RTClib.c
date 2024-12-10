/**************************************************************************/
// RTClib.c
// RTC library for Arduino, written by JeeLab, updated / modified by Adafruit
// Supports multiple RTC modules as well as functions to convert from
// Unix time (32-bit number of seconds) to RTC (hours, minutes, seconds).
// https://github.com/adafruit/RTClib/blob/master/src/RTClib.cpp
// This library has been modified to work with "C".  (No C++ support required)
/**************************************************************************/
#include "stdint.h"
#include "RTClib.h"

/**************************************************************************/
// utility code
/**************************************************************************/

/**
  Number of days in each month, from January to November. December is not
  needed. Omitting it avoids an incompatibility with Paul Stoffregen's Time
  library. C.f. https://github.com/adafruit/RTClib/issues/114
*/
const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/**************************************************************************/
/*!
    @brief  Given a date, return number of days since 2000/01/01,
            valid for 2000--2099
    @param y Year
    @param m Month
    @param d Day
    @return Number of days
*/
/**************************************************************************/
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
  if (y >= 2000U)
    y -= 2000U;
  uint16_t days = d;
  for (uint8_t i = 1; i < m; ++i)
    days += daysInMonth[i - 1];
  if (m > 2 && y % 4 == 0)
    ++days;
  return days + 365 * y + (y + 3) / 4 - 1;
}

/**************************************************************************/
/*!
    @brief  Given a number of days, hours, minutes, and seconds, return the
   total seconds
    @param days Days
    @param h Hours
    @param m Minutes
    @param s Seconds
    @return Number of seconds total
*/
/**************************************************************************/
static uint32_t time2ulong(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
  return ((days * 24UL + h) * 60 + m) * 60 + s;
}

/**************************************************************************/
/*!
    @brief
    [Unix time](https://en.wikipedia.org/wiki/Unix_time).

    This builds a DateTime from an integer specifying the number of seconds
    elapsed since the epoch: 1970-01-01 00:00:00. This number is analogous
    to Unix time, with two small differences:

     - The Unix epoch is specified to be at 00:00:00
       [UTC](https://en.wikipedia.org/wiki/Coordinated_Universal_Time),
       whereas this class has no notion of time zones. The epoch used in
       this class is then at 00:00:00 on whatever time zone the user chooses
       to use, ignoring changes in DST.

     - Unix time is conventionally represented with signed numbers, whereas
       this constructor takes an unsigned argument. Because of this, it does
       _not_ suffer from the
       [year 2038 problem](https://en.wikipedia.org/wiki/Year_2038_problem).

    If called without argument, it returns the earliest time representable
    by this class: 2000-01-01 00:00:00.

    @see The `unixtime()` method is the converse of this constructor.

    @param t Time elapsed in seconds since 1970-01-01 00:00:00.
*/
/**************************************************************************/
void unix2rtc(DATE_TIME * dt, uint32_t t){
  t -= SECONDS_FROM_1970_TO_2000; // bring to 2000 timestamp from 1970

  dt->ss = t % 60;
  t /= 60;
  dt->mm = t % 60;
  t /= 60;
  dt->hh = t % 24;
  uint16_t days = t / 24;
  uint8_t leap;
  for (dt->yOff = 0;; ++dt->yOff) {
    leap = dt->yOff % 4 == 0;
    if (days < 365U + leap)
      break;
    days -= 365 + leap;
  }
  for (dt->m = 1; dt->m < 12; ++dt->m) {
    uint8_t daysPerMonth = daysInMonth[dt->m - 1];
    if (leap && dt->m == 2)
      ++daysPerMonth;
    if (days < daysPerMonth)
      break;
    days -= daysPerMonth;
  }
  dt->d = days + 1;
}

/**************************************************************************/
/*!
    @brief  Return the day of the week.
    @return Day of week as an integer from 0 (Sunday) to 6 (Saturday).
*/
/**************************************************************************/
uint8_t dayOfTheWeek(DATE_TIME * dt) {
  uint16_t day = date2days(dt->yOff, dt->m, dt->d);
  return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

/**************************************************************************/
/*!
    @brief  Return Unix time: seconds since 1 Jan 1970.

    @see unix2rtc(DATE_TIME * dt, uint32_t t);

    @return Number of seconds since 1970-01-01 00:00:00.
*/
/**************************************************************************/
uint32_t rtc2unix(DATE_TIME * dt) {
  uint32_t t;
  uint16_t days = date2days(dt->yOff, dt->m, dt->d);
  t = time2ulong(days, dt->hh, dt->mm, dt->ss);
  t += SECONDS_FROM_1970_TO_2000; // seconds from 1970 to 2000

  return t;
}

/**************************************************************************/
/*!
    @brief  Convert the DateTime to seconds since 1 Jan 2000
    @return Number of seconds since 2000-01-01 00:00:00.
*/
/**************************************************************************/
uint32_t rtc2seconds(DATE_TIME * dt) {
  uint32_t t;
  uint16_t days = date2days(dt->yOff, dt->m, dt->d);
  t = time2ulong(days, dt->hh, dt->mm, dt->ss);
  return t;
}

/**************************************************************************/
/*!
    @brief  Convert a string containing two digits to uint8_t, e.g. "09" returns
   9
    @param p Pointer to a string containing two digits
*/
/**************************************************************************/
static uint8_t conv2d(const char *p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

/**************************************************************************/
/*!
    @brief  Generate the build time into DATE_TIME structure.

    This function expects its parameters to be strings in the format
    generated by the compiler's preprocessor macros `__DATE__` and
    `__TIME__`. Usage:

    ```
    buildTime(DATE_TIME *dt, __DATE__, __TIME__);
    ```

    @param date Date string, e.g. "Apr 16 2020".
    @param time Time string, e.g. "18:34:56".
*/
/**************************************************************************/
void buildTime(DATE_TIME *dt, const char *date, const char *time) {
  dt->yOff = conv2d(date + 9);
  // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
  switch (date[0]) {
  case 'J':
	dt->m = (date[1] == 'a') ? 1 : ((date[2] == 'n') ? 6 : 7);
    break;
  case 'F':
	dt->m = 2;
    break;
  case 'A':
	dt->m = date[2] == 'r' ? 4 : 8;
    break;
  case 'M':
	dt->m = date[2] == 'r' ? 3 : 5;
    break;
  case 'S':
	dt->m = 9;
    break;
  case 'O':
	dt->m = 10;
    break;
  case 'N':
	dt->m = 11;
    break;
  case 'D':
	dt->m = 12;
    break;
  }
  dt->d = conv2d(date + 4);
  dt->hh = conv2d(time);
  dt->mm = conv2d(time + 3);
  dt->ss = conv2d(time + 6);
}

