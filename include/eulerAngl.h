#ifndef _EULERANGL_H_
#define _EULERANGL_H_

#include "Arduino.h"

namespace GNSS_RTK_ROVER
{
    class EAProv
    {
        public:
        virtual double getYaw() = 0;
        virtual double getPitch() = 0;
        virtual double getRoll() = 0;
    };
}

#endif