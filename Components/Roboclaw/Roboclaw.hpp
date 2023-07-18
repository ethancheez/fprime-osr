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

    enum TLM_STATE_MACHINE {
      ENCODER,
      SPEED,
      POS_PID1,
      POS_PID2
    };

    struct MotorConfig
    {
      F32 ticks_per_rev;
      F32 gear_ratio;
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

      void configure(U8 addr, MotorConfig m1config, MotorConfig m2config);

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
          F32 velocity, 
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
          OsrModule::MOTOR_SELECT motor, 
          OsrModule::MOVE_DIRECTION direction, 
          U8 speed_percentage 
      );

      //! Implementation for MOVE_DISTANCE command handler
      //! Move in a specified direction with a given speed and distance
      void MOVE_DISTANCE_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          OsrModule::MOTOR_SELECT motor, 
          OsrModule::MOVE_DIRECTION direction, 
          U8 speed_percentage, 
          U32 distance 
      );

      //! Implementation for MOVE_ACCELERATED_CONTINUOUS command handler
      //! Continuously move in a specified direction with a given speed and acceleration
      void MOVE_ACCELERATED_CONTINUOUS_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          OsrModule::MOTOR_SELECT motor, 
          OsrModule::MOVE_DIRECTION direction, 
          U32 acceleration, 
          U8 speed_percentage 
      );

      //! Implementation for MOVE_ACCELERATED_DISTANCE command handler
      //! Move in a specified direction with a given speed, acceleration, and distance
      void MOVE_ACCELERATED_DISTANCE_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          OsrModule::MOTOR_SELECT motor, 
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

      void setVelocity(OsrModule::MOTOR_SELECT motor, I32 velocity);
      void setVelocityDistance(OsrModule::MOTOR_SELECT motor, I32 velocity, U32 distance);
      void setAccelVelocity(OsrModule::MOTOR_SELECT motor, U32 accel, I32 velocity);
      void setAccelVelocityDistance(OsrModule::MOTOR_SELECT motor, U32 accel, I32 velocity, U32 distance);
      void setVelocityAccelDeccelPosition(OsrModule::MOTOR_SELECT motor, U32 accel, U32 velocity, U32 deccel, U32 position);
      void setVelocityM1M2(I32 velocity);
      void setVelocityDistanceM1M2(I32 velocity, U32 distance);
      void setAccelVelocityM1M2(U32 accel, I32 velocity);
      void setAccelVelocityDistanceM1M2(U32 accel, I32 velocity, U32 distance);
      void setVelocityAccelDeccelPositionM1M2(U32 accel, U32 velocity, U32 deccel, U32 position);

      I32 velocity2qpps(F32 velocity, OsrModule::MOTOR_SELECT motor);

      void getEncoderValues();
      void getSpeedValues();
      void getPositionPIDs();
      void switchTlmState();

      void fillBuffer8(U8 *buf, U8 val);
      void fillBuffer16(U8 *buf, U16 val);
      void fillBuffer32(U8 *buf, U32 val);
      void write(U8 address, U8 command, U8 *tx_data, NATIVE_INT_TYPE tx_length);
      U16 crc16(U8 *packet, NATIVE_INT_TYPE nBytes);

      NATIVE_INT_TYPE m_addr;
      MotorConfig m1_config;
      MotorConfig m2_config;

      OsrModule::ROBOCLAW_CMD curr_cmd;
      U8 tx_buffer[33];
      U8 rx_buffer[33];
      NATIVE_INT_TYPE rx_index;
      bool waitRecv;

      // Telemetry Data
      OsrModule::MotorTlmData encoderTlmData;
      OsrModule::MotorTlmData speedTlmData;
      OsrModule::MotorPosPIDData posPidTlmData;
      TLM_STATE_MACHINE tlm_state;

    };

} // end namespace OsrModule

#endif
