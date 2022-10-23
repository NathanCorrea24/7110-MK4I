#pragma once

#include <ctre/phoenix.h>

namespace hb {
  
  class CANcode : public CANCoder {
  
    public:
      
      /**
       * @brief Create a CANCoder Object
       * 
       * @param ID CANId 
       * @param offset Offset for CANCoder
       */
      explicit CANcode(const int id, const double offset);

      /**
       * @brief Should be removed as soon as possible
       * 
       * @param DPP Distance Per Pulse
       */
      void SetDistancePerPulse(const double DPP) const;

      /**
       * @brief Returns the position of the CANCoder in radians
       * 
       * @return The position of the CANCoder in Radians
       */
      double Get();
    
    private:

      const double m_offset;
      const int m_ID;
  };
} // namespace hb