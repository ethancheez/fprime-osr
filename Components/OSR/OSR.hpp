// ======================================================================
// \title  OSR.hpp
// \author ethanchee
// \brief  hpp file for OSR component implementation class
// ======================================================================

#ifndef OSR_HPP
#define OSR_HPP

#include "Components/OSR/OSRComponentAc.hpp"
#include <cmath>

namespace OsrModule {

  class OSR :
    public OSRComponentBase
  {

    struct Dimension3D
    {
      F32 x;
      F32 y;
      F32 z;
    };

    struct Twist
    {
      Dimension3D linear;
      Dimension3D angular;
    };

    struct RoverDimensions
    {
      F32 d1;
      F32 d2;
      F32 d3;
      F32 d4;
    };

    struct RoverCornerPositions
    {
      F32 left_front_pos;
      F32 left_back_pos;
      F32 right_front_pos;
      F32 right_back_pos;
    };

    struct RoverVelocities
    {
      F32 left_front_vel;
      F32 left_middle_vel;
      F32 left_back_vel;
      F32 right_front_vel;
      F32 right_middle_vel;
      F32 right_back_vel;
    };
    
    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object OSR
      //!
      OSR(
          const char *const compName /*!< The component name*/
      );

      //! Destroy object OSR
      //!
      ~OSR();

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations
      // ----------------------------------------------------------------------

      //! Implementation for CMD_CB command handler
      //! Respond to an incoming Twist command in one of two ways depending on the mode (intuitive)
      void CMD_CB_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          F32 twist_linear_x, 
          F32 twist_linear_y, 
          F32 twist_linear_z, 
          F32 twist_angular_x, 
          F32 twist_angular_y, 
          F32 twist_angular_z, 
          bool intuitive 
      );

    PRIVATE:

      F32 twist_to_turning_radius(Twist twist, bool clip = true, bool intuitive_mode = false);
      F32 angle_to_turning_radius(F32 angle);
      RoverCornerPositions calculate_corner_positions(F32 radius);
      RoverVelocities calculate_drive_velocities(F32 speed, F32 curr_radius);
      bool corner_cmd_threshold(RoverCornerPositions corner_pos);

      U32 velocity2qpps(F32 velocity, F32 ticks_per_rev, F32 gear_ratio);

      void setDriveVelocities(RoverVelocities vel);
      void setCornerPositions(RoverCornerPositions pos);

      // Rover config
      RoverDimensions rover_dimensions;
      F32 min_radius;
      F32 max_radius;
      F32 no_cmd_thresh;
      F32 wheel_radius;
      F32 max_velocity;

    };

} // end namespace OsrModule

#endif
