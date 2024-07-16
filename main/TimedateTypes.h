/**
 * @file TimedateTypes.h
 * @author Samuel Yow
 * @brief  Common struct accessible by the PCF driver and the ws2812 driver (in progress)
 */

#ifndef TIMEDATE_TYPES
#define TIMEDATE_TYPES

#include <stdint.h>

typedef struct
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;

    uint8_t day;
    uint8_t weekday;

    uint8_t month;
    uint32_t year;
}Datetime;

#endif //TIMEDATE_TYPES