#ifndef _CORRECTION_H_
  #define _CORRECTION_H_

#include "Arduino.h"

#define FT_IN_DEGREE 5280

namespace GNSS_RTK_ROVER
{
    // GPS Correction Provider
    class CorProv
    {
        public:
        virtual double getLatCorr() = 0;
        virtual double getLonCorr() = 0;
        virtual double getAltCorr() = 0;
    };

    // Correction Provider based on Euler Angles
    class EACorProv : public CorProv
    {
        public:
        // unitHeight: distance from the tip of the bar to the antenna
        // eaProv: euler angles provider
        EACorProv(double unitHeight, EAProv* eaProv) : m_unitHeight(unitHeight), m_eaProv(eaProv), m_correction({0}) {};
        void calculateCorrection();

        double getLatCorr() { return m_correction[0]; };
        double getLonCorr() { return m_correction[1]; };
        double getAltCorr() { return m_correction[2]; };

        private:
        EAProv* m_eaProv;
        double m_unitHeight;
        double m_correction[3]; // lat -> 0, lon -> 1, alt -> 2
    };

}

#endif
