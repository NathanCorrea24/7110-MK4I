#pragma once

#include <rev/CANSparkMax.h>

namespace hb {
  
  class NeoMotor : public rev::CANSparkMax, public rev::SparkMaxRelativeEncoder {

    public:
       /**
        * @brief Create a neo motor and use its encoder 
        * 
        * @param ID Motor ID
        * @param Type Motor Type
        */
      explicit NeoMotor(const int ID, rev::CANSparkMax::MotorType Type);
      
      /**
       * @brief This function is not neccessary and should 
       * be removed from the project as soon as possible
       * 
       * 
       * @param DPP distance per pulse
       */
      void SetDistancePerPulse(double DPP) const;
      
      /**
       * @brief Get the rate of the motor adjusted for the MK4I
       * 
       * @return Rate of the Motor in MPS
       */
      double GetRate();

    private:
      const int m_ID;
  };
} // namespace hb
