/**************************************************************************/
// RTClib.h
// RTC library for Arduino, written by JeeLab, updated / modified by Adafruit
// Supports multiple RTC modules as well as functions to convert from
// Unix time (32-bit number of seconds) to RTC (hours, minutes, seconds).
// https://github.com/adafruit/RTClib/blob/master/src/RTClib.cpp
/**************************************************************************/
#ifndef __RTCLIB_H__
#define __RTCLIB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/** Constants */
#define SECONDS_PER_DAY 86400L ///< 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000                                              \
  946684800 ///< Unixtime for 2000-01-01 00:00:00, useful for initialization

#ifndef DATE_TIME_
// Structure to hold/record time read from to written to an external RTC module (DS3231)
typedef struct {
	uint8_t yOff; ///< Year offset from 2000
	uint8_t m;    ///< Month 1-12
	uint8_t d;    ///< Day 1-31
	uint8_t hh;   ///< Hours 0-23
	uint8_t mm;   ///< Minutes 0-59
	uint8_t ss;   ///< Seconds 0-59
}DATE_TIME;
#define DATE_TIME_
#endif

void unix2rtc(DATE_TIME * dt, uint32_t t);
uint8_t dayOfTheWeek(DATE_TIME * dt);
uint32_t rtc2unix(DATE_TIME * dt);
uint32_t rtc2seconds(DATE_TIME * dt);
void buildTime(DATE_TIME *dt, const char *date, const char *time);

#ifdef __cplusplus
}
#endif

#endif // __RTCLIB_H__
