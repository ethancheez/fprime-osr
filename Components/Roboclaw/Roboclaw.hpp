// ======================================================================
// \title  Roboclaw.hpp
// \author ethanchee
// \brief  hpp file for Roboclaw component implementation class
// ======================================================================

#ifndef Roboclaw_HPP
#define Roboclaw_HPP

#include "Components/Roboclaw/RoboclawComponentAc.hpp"

namespace OsrModule {

  class Roboclaw :
    public RoboclawComponentBase
  {

    const NATIVE_INT_TYPE MAX_VELOCITY = 6000;
    const NATIVE_INT_TYPE MAX_DUTY_CYCLE = 32767;

    enum MOTOR {
      MOTOR1,
      MOTOR2
    };

    enum TLM_STATE_MACHINE {
      ENCODER,
      SPEED
    };

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object Roboclaw
      //!
      Roboclaw(
          const char *const compName /*!< The component name*/
      );

      //! Destroy object Roboclaw
      //!
      ~Roboclaw();

      void set_addr(U8 addr);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for comDataIn
      //!
      void comDataIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &recvBuffer, 
          const Drv::RecvStatus &recvStatus 
      );

      //! Handler implementation for motorControlIn
      //!
      void motorControlIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          const OsrModule::ROBOCLAW_CMD &cmd, 
          const OsrModule::MOTOR_SELECT &motor, 
          U8 speed_percentage, 
          U32 acceleration, 
          U32 distance 
      );

      //! Handler implementation for run
      //!
      void run_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations
      // ----------------------------------------------------------------------

      //! Implementation for MOVE_CONTINUOUS command handler
      //! Continuously move in a specified direction at a given speed
      void MOVE_CONTINUOUS_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          OsrModule::MOVE_DIRECTION direction, 
          U8 speed_percentage 
      );

      //! Implementation for MOVE_DISTANCE command handler
      //! Move in a specified direction with a given speed and distance
      void MOVE_DISTANCE_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          OsrModule::MOVE_DIRECTION direction, 
          U8 speed_percentage, 
          U32 distance 
      );

      //! Implementation for MOVE_ACCELERATED_CONTINUOUS command handler
      //! Continuously move in a specified direction with a given speed and acceleration
      void MOVE_ACCELERATED_CONTINUOUS_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          OsrModule::MOVE_DIRECTION direction, 
          U32 acceleration, 
          U8 speed_percentage 
      );

      //! Implementation for MOVE_ACCELERATED_DISTANCE command handler
      //! Move in a specified direction with a given speed, acceleration, and distance
      void MOVE_ACCELERATED_DISTANCE_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          OsrModule::MOVE_DIRECTION direction, 
          U32 acceleration, 
          U8 speed_percentage, 
          U32 distance 
      );

      //! Implementation for STOP command handler
      //! Stop all motors
      void STOP_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      //! Implementation for RESET_ENCODERS command handler
      //! Command to reset encoder telemetry values
      void RESET_ENCODERS_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

    PRIVATE:

      void setDutyCycle(OsrModule::Roboclaw::MOTOR motor, OsrModule::MOVE_DIRECTION direction, U8 speed_percentage);
      void setVelocity(OsrModule::Roboclaw::MOTOR motor, OsrModule::MOVE_DIRECTION direction, U8 speed_percentage);
      void setVelocityDistance(OsrModule::Roboclaw::MOTOR motor, OsrModule::MOVE_DIRECTION direction, U8 speed_percentage, U32 distance);
      void setAccelVelocity(OsrModule::Roboclaw::MOTOR motor, OsrModule::MOVE_DIRECTION direction, U32 accel, U8 speed_percentage);
      void setAccelVelocityDistance(OsrModule::Roboclaw::MOTOR motor, OsrModule::MOVE_DIRECTION direction, U32 accel, U8 speed_percentage, U32 distance);
      void setDutyCycleM1M2(OsrModule::MOVE_DIRECTION direction, U8 speed_percentage);
      void setVelocityM1M2(OsrModule::MOVE_DIRECTION direction, U8 speed_percentage);
      void setVelocityDistanceM1M2(OsrModule::MOVE_DIRECTION direction, U8 speed_percentage, U32 distance);
      void setAccelVelocityM1M2(OsrModule::MOVE_DIRECTION direction, U32 accel, U8 speed_percentage);
      void setAccelVelocityDistanceM1M2(OsrModule::MOVE_DIRECTION direction, U32 accel, U8 speed_percentage, U32 distance);

      void getEncoderValues();
      void getSpeedValues();
      void updateTlm(OsrModule::ROBOCLAW_CMD cmd, I32 ret1, I32 ret2);

      void fillBuffer8(U8 *buf, U8 val);
      void fillBuffer16(U8 *buf, U16 val);
      void fillBuffer32(U8 *buf, U32 val);
      void write(U8 address, U8 command, U8 *tx_data, NATIVE_INT_TYPE tx_length);
      U16 crc16(U8 *packet, NATIVE_INT_TYPE nBytes);

      NATIVE_INT_TYPE m_addr;
      OsrModule::ROBOCLAW_CMD curr_cmd;
      U8 tx_buffer[32];
      U8 rx_buffer[32];
      NATIVE_INT_TYPE rx_index;
      bool waitRecv;

      // Telemetry Data
      OsrModule::MotorTlmData encoderTlmData;
      OsrModule::MotorTlmData speedTlmData;
      TLM_STATE_MACHINE tlm_state;

    };

} // end namespace OsrModule

#endif
