// ======================================================================
// \title  OSR.cpp
// \author ethanchee
// \brief  cpp file for OSR component implementation class
// ======================================================================


#include <Components/OSR/OSR.hpp>
#include <Components/Roboclaw/Roboclaw.hpp>
#include <FpConfig.hpp>
#include <limits>
#include <algorithm>

namespace OsrModule {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  OSR ::
    OSR(
        const char *const compName
    ) : OSRComponentBase(compName),
        rover_dimensions({0.184, 0.267, 0.267, 0.256}),
        min_radius(0.45),
        max_radius(6.4),
        no_cmd_thresh(0.05),
        wheel_radius(0.075),
        max_velocity(wheel_radius * 130 * 100 / 60 * 2 * 3.14159)
  {

  }

  OSR ::
    ~OSR()
  {

  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  /*
    Respond to an incoming Twist command in one of two ways depending on the mode (intuitive)

    The Mathematically correct mode (intuitive=False) means that 
      * when the linear velocity is zero, an angular velocity does not cause the corner motors to move
        (since simply steering the corners while standing still doesn't generate a twist)
      * when driving backwards, steering behaves opposite as what you intuitively might expect
        (this is to hold true to the commanded twist)
    Use this topic with a controller that generated velocities based on targets. When you're
    controlling the robot with a joystick or other manual input topic, consider using the 
    /cmd_vel_intuitive topic instead.

    The Intuitive mode (intuitive=True) means that sending a positive angular velocity (moving joystick left)
    will always make the corner wheels turn 'left' regardless of the linear velocity.

    :param intuitive: determines the mode
  */
  void OSR ::
    CMD_CB_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        F32 twist_linear_x,
        F32 twist_linear_y,
        F32 twist_linear_z,
        F32 twist_angular_x,
        F32 twist_angular_y,
        F32 twist_angular_z,
        bool intuitive
    )
  {
    Twist twist = {
      .linear = { twist_linear_x, twist_linear_y, twist_linear_z },
      .angular = { twist_angular_x, twist_angular_y, twist_angular_z }
    };

    F32 desired_turning_radius = this->twist_to_turning_radius(twist, true, intuitive);
    OSR::RoverCornerPositions corner_pos = this->calculate_corner_positions(desired_turning_radius);

    // if we're turning, calculate the max velocity the middle of the rover can go
    F32 max_vel = abs(desired_turning_radius) / (abs(desired_turning_radius) + this->rover_dimensions.d1) * this->max_velocity;
    if(std::isnan(max_vel))
      max_vel = this->max_velocity;
    F32 velocity = std::min(max_vel, twist.linear.x);

    OSR::RoverVelocities drive_vel = this->calculate_drive_velocities(velocity, desired_turning_radius);

    if(this->corner_cmd_threshold(corner_pos))
      this->setCornerPositions(corner_pos);

    this->setDriveVelocities(drive_vel);

    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  F32 OSR::twist_to_turning_radius(Twist twist, bool clip, bool intuitive_mode)
  {
    F32 radius;

    if(twist.angular.z == 0)
      return std::numeric_limits<F32>::infinity();

    if (intuitive_mode && twist.linear.x < 0)
        radius = twist.linear.x / -twist.angular.z;
    else
        radius = twist.linear.x / twist.angular.z;

    if(!clip)
      return radius;

    if(radius == 0)
    {
      if(intuitive_mode)
      {
        if(twist.angular.z == 0)
          return this->max_radius;
        else
          radius = this->min_radius * this->max_velocity / twist.angular.z;
      }
      else
        return this->max_radius;
    }

    if(radius > 0)
      radius = std::max(this->min_radius, std::min(this->max_radius, radius));
    else
      radius = std::max(-this->max_radius, std::min(-this->min_radius, radius));

    return radius;
  }

  F32 OSR::angle_to_turning_radius(F32 angle)
  {
    F32 div = tan(angle);

    if(div == 0)
      return std::numeric_limits<F32>::infinity();

    return this->rover_dimensions.d3 / div;
  }

  OSR::RoverCornerPositions OSR::calculate_corner_positions(F32 radius)
  {
    RoverCornerPositions pos = { 0, 0, 0, 0 };

    if (radius >= this->max_radius)
      return pos;  // assume straight

    F32 theta_front_closest = atan2(this->rover_dimensions.d3, abs(radius) - this->rover_dimensions.d1);
    F32 theta_front_farthest = atan2(this->rover_dimensions.d3, abs(radius) + this->rover_dimensions.d1);

    if(radius > 0)
    {
      pos.left_front_pos = -theta_front_closest;
      pos.left_back_pos = theta_front_closest;
      pos.right_back_pos = theta_front_farthest;
      pos.right_front_pos = -theta_front_farthest;
    }
    else
    {
      pos.left_front_pos = theta_front_farthest;
      pos.left_back_pos = -theta_front_farthest;
      pos.right_back_pos = -theta_front_closest;
      pos.right_front_pos = theta_front_closest;
    }

    return pos;
  }

  OSR::RoverVelocities OSR::calculate_drive_velocities(F32 speed, F32 curr_radius)
  {
    speed = std::max(-this->max_velocity, std::min(this->max_velocity, speed));

    RoverVelocities vel = { 0, 0, 0, 0, 0, 0 };

    if(speed == 0)
    {
      return vel;
    }
    else if(abs(curr_radius) >= this->max_radius)
    {
      F32 angular_vel = speed / this->wheel_radius;
      vel.left_front_vel = angular_vel;
      vel.left_middle_vel = angular_vel;
      vel.left_back_vel = angular_vel;
      vel.right_back_vel = angular_vel;
      vel.right_middle_vel = angular_vel;
      vel.right_front_vel = angular_vel;
      return vel;
    }
    else
    {
      // for the calculations, we assume positive radius (turn left) and adjust later
      F32 radius = abs(curr_radius);
      // the entire vehicle moves with the same angular velocity dictated by the desired speed,
      // around the radius of the turn. v = r * omega
      F32 angular_velocity_center = speed / radius;
      // calculate desired velocities of all centers of wheels. Corner wheels on the same side
      // move with the same velocity. v = r * omega again
      F32 vel_middle_closest = (radius - this->rover_dimensions.d4) * angular_velocity_center;
      F32 vel_corner_closest = hypot(radius - this->rover_dimensions.d1, this->rover_dimensions.d3) * angular_velocity_center;
      F32 vel_corner_farthest = hypot(radius + this->rover_dimensions.d1, this->rover_dimensions.d3) * angular_velocity_center;
      F32 vel_middle_farthest = (radius + this->rover_dimensions.d4) * angular_velocity_center;

      // now from these desired velocities, calculate the desired angular velocity of each wheel
      // v = r * omega again
      F32 ang_vel_middle_closest = vel_middle_closest / this->wheel_radius;
      F32 ang_vel_corner_closest = vel_corner_closest / this->wheel_radius;
      F32 ang_vel_corner_farthest = vel_corner_farthest / this->wheel_radius;
      F32 ang_vel_middle_farthest = vel_middle_farthest / this->wheel_radius;

      if(curr_radius > 0) // turning left
      {
        vel.left_front_vel = ang_vel_corner_closest;
        vel.left_back_vel = ang_vel_corner_closest;
        vel.left_middle_vel = ang_vel_middle_closest;
        vel.right_back_vel = ang_vel_corner_farthest;
        vel.right_front_vel = ang_vel_corner_farthest;
        vel.right_middle_vel = ang_vel_middle_farthest;
      }
      else // turning right
      {
        vel.left_front_vel = ang_vel_corner_farthest;
        vel.left_back_vel = ang_vel_corner_farthest;
        vel.left_middle_vel = ang_vel_middle_farthest;
        vel.right_back_vel = ang_vel_corner_closest;
        vel.right_front_vel = ang_vel_corner_closest;
        vel.right_middle_vel = ang_vel_middle_closest;
      }

      return vel;
    }
  }

  bool OSR::corner_cmd_threshold(RoverCornerPositions corner_pos)
  {
    I32 curr_positions[4];

    Fw::PolyType encoderPolyData;
    Svc::MeasurementStatus mstat;
    Fw::Time ts;
    void *data;
    OsrModule::MotorTlmData *encoderTlmData;

    this->getPolyDbVal_out(0, 131 - 0x80, mstat, ts, encoderPolyData);
    encoderPolyData.get(data);
    encoderTlmData = reinterpret_cast<OsrModule::MotorTlmData *>(data);
    curr_positions[0] = (*encoderTlmData)[0]; // Right front
    curr_positions[1] = (*encoderTlmData)[1]; // Right back
    this->getPolyDbVal_out(0, 132 - 0x80, mstat, ts, encoderPolyData);
    encoderPolyData.get(data);
    encoderTlmData = reinterpret_cast<OsrModule::MotorTlmData *>(data);
    curr_positions[2] = (*encoderTlmData)[0]; // Left back
    curr_positions[3] = (*encoderTlmData)[1]; // Left front

    if(abs(corner_pos.right_front_pos - curr_positions[0]) > this->no_cmd_thresh)
        return true;
    else if(abs(corner_pos.right_back_pos - curr_positions[1]) > this->no_cmd_thresh)
        return true;
    else if(abs(corner_pos.left_back_pos - curr_positions[2]) > this->no_cmd_thresh)
        return true;
    else if(abs(corner_pos.left_front_pos - curr_positions[3]) > this->no_cmd_thresh)
        return true;
    else
        return false;
  }

  void OSR::setDriveVelocities(RoverVelocities vel)
  {
    this->motorControlOut1_out(0, OsrModule::ROBOCLAW_CMD::M1SPEEDACCEL, OsrModule::MOTOR_SELECT::MOTOR1, vel.right_front_vel, 10000, 0, 0);
    this->motorControlOut1_out(0, OsrModule::ROBOCLAW_CMD::M2SPEEDACCEL, OsrModule::MOTOR_SELECT::MOTOR2, vel.right_middle_vel, 10000, 0, 0);
    this->motorControlOut2_out(0, OsrModule::ROBOCLAW_CMD::M1SPEEDACCEL, OsrModule::MOTOR_SELECT::MOTOR1, vel.right_back_vel, 10000, 0, 0);
    this->motorControlOut2_out(0, OsrModule::ROBOCLAW_CMD::M2SPEEDACCEL, OsrModule::MOTOR_SELECT::MOTOR2, vel.left_back_vel, 10000, 0, 0);
    this->motorControlOut3_out(0, OsrModule::ROBOCLAW_CMD::M1SPEEDACCEL, OsrModule::MOTOR_SELECT::MOTOR1, vel.left_middle_vel, 10000, 0, 0);
    this->motorControlOut3_out(0, OsrModule::ROBOCLAW_CMD::M2SPEEDACCEL, OsrModule::MOTOR_SELECT::MOTOR2, vel.left_front_vel, 10000, 0, 0);
  }

  void OSR::setCornerPositions(RoverCornerPositions pos)
  {
    this->motorControlOut4_out(0, OsrModule::ROBOCLAW_CMD::M1SPEEDACCELDECCELPOS, OsrModule::MOTOR_SELECT::MOTOR1, 1000, (pow(2, 15) - 1) * 0.8, 0, pos.right_front_pos);
    this->motorControlOut4_out(0, OsrModule::ROBOCLAW_CMD::M2SPEEDACCELDECCELPOS, OsrModule::MOTOR_SELECT::MOTOR2, 1000, (pow(2, 15) - 1) * 0.8, 0, pos.right_back_pos);
    this->motorControlOut5_out(0, OsrModule::ROBOCLAW_CMD::M1SPEEDACCELDECCELPOS, OsrModule::MOTOR_SELECT::MOTOR1, 1000, (pow(2, 15) - 1) * 0.8, 0, pos.left_back_pos);
    this->motorControlOut5_out(0, OsrModule::ROBOCLAW_CMD::M2SPEEDACCELDECCELPOS, OsrModule::MOTOR_SELECT::MOTOR2, 1000, (pow(2, 15) - 1) * 0.8, 0, pos.left_front_pos);
  }

} // end namespace OsrModule
