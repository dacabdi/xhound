#include "eulerAngl.h"
#include "util.h"
#include <math.h>

#include "correction.h"


namespace GNSS_RTK_ROVER
{
    void EACorProv::calculateCorrection()
    {
        auto yaw = degToRad((-1) * (m_eaProv->getYaw() + 6.58));
        auto roll = degToRad((-1) * (m_eaProv->getRoll()));
        auto pitch = degToRad(m_eaProv->getPitch() - 180);

        m_correction[0] = (m_unitHeight * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)))/FT_IN_DEGREE; // lat
        m_correction[1] = (-1 * m_unitHeight * (sin(roll) * cos(yaw) - cos(roll) * sin(yaw) * sin(pitch)))/FT_IN_DEGREE; // lon
        m_correction[2] = ftToM(m_unitHeight) * cos(roll) * cos(pitch); // alt
    }
}