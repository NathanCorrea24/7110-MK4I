#include "Systems/CANCoder.h"
#include <wpi/numbers>

using namespace hb;

CANcode::CANcode(const int ID, const double offset) : CANCoder(ID), m_offset(offset), m_ID(ID) {
  ConfigSensorInitializationStrategy(BootToAbsolutePosition);
  double calc = GetAbsolutePosition();
  printf("CANCoder: %i, calc %5.2f, reading %5.2f\n", ID, (calc - 180.0), calc);
}

void CANcode::SetDistancePerPulse(const double DPP) const {}

double CANcode::Get() {
  // double calc = ((GetAbsolutePosition() - m_offset)/360.0) * 2.0 * wpi::numbers::pi - wpi::numbers::pi;
  // if(m_ID == 0) printf("%d: %5.2f\n", m_ID, calc);
  // return calc; 
  double rv = GetAbsolutePosition() - m_offset;

  // rv -= 180.0;
    
  if (rv < 0.0) rv += 360.0;

  if (rv > 360.0) rv -= 360.0;

  double a = (wpi::numbers::pi * 2 * (rv / 360.0)) - (wpi::numbers::pi);

  // if (m_ID == 0) printf("%d: %5.2f\n", m_ID, a);

  return -a;
}