#ifndef _ULTIL_H_
#define _ULTIL_H_

#include "Arduino.h"
#include "math.h"

namespace GNSS_RTK_ROVER
{
    inline double degToRad(double deg) {
        return (deg * M_PI)/180;
    }

    inline double ftToM(double ft) {
        return ft * 0.3048;
    }
}

#endif