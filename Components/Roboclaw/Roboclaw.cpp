// ======================================================================
// \title  Roboclaw.cpp
// \author ethanchee
// \brief  cpp file for Roboclaw component implementation class
// ======================================================================


#include <Components/Roboclaw/Roboclaw.hpp>
#include <FpConfig.hpp>
#include <Fw/Logger/Logger.hpp>

namespace OsrModule {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Roboclaw ::
    Roboclaw(
        const char *const compName
    ) : RoboclawComponentBase(compName),
        m_addr(0x80),
        rx_index(0),
        waitRecv(false),
        tlm_state(TLM_STATE_MACHINE::ENCODER)
  {

  }

  Roboclaw ::
    ~Roboclaw()
  {

  }

  void Roboclaw::configure(U8 addr, MotorConfig m1config, MotorConfig m2config)
  {
    this->m_addr = addr;
    this->m1_config = m1config;
    this->m2_config = m2config;

    this->write(m_addr, OsrModule::ROBOCLAW_CMD::RESETENC, nullptr, 0);
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Roboclaw ::
    comDataIn_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &recvBuffer,
        const Drv::RecvStatus &recvStatus
    )
  {
    I32 ret1 = 0;
    I32 ret2 = 0;

    memcpy(&rx_buffer[rx_index], recvBuffer.getData(), recvBuffer.getSize());
    rx_index += recvBuffer.getSize();

    switch(curr_cmd)
    {
      case OsrModule::ROBOCLAW_CMD::GETENCODERS:
      case OsrModule::ROBOCLAW_CMD::GETISPEEDS:
        if(rx_index == 10)
        {
          ret1 += rx_buffer[0] << 24;
          ret1 += rx_buffer[1] << 16;
          ret1 += rx_buffer[2] << 8;
          ret1 += rx_buffer[3];
          
          ret2 += rx_buffer[4] << 24;
          ret2 += rx_buffer[5] << 16;
          ret2 += rx_buffer[6] << 8;
          ret2 += rx_buffer[7];

          if(curr_cmd == OsrModule::ROBOCLAW_CMD::GETENCODERS)
          {
            this->encoderTlmData[0] = ret1;
            this->encoderTlmData[1] = ret2;
            this->tlmWrite_EncoderValues(encoderTlmData);
          }
          else
          {
            this->speedTlmData[0] = ret1;
            this->speedTlmData[1] = ret2;
            this->tlmWrite_SpeedValues(speedTlmData);
          }

          waitRecv = false;
        }
        break;
      case OsrModule::ROBOCLAW_CMD::READM1POSPID:
      case OsrModule::ROBOCLAW_CMD::READM2POSPID:
          U32 kp, ki, kd, kiMax, deadZone, min, max;

          if(rx_index == 30)
          {
            kp += rx_buffer[0] << 24;
            kp += rx_buffer[1] << 16;
            kp += rx_buffer[2] << 8;
            kp += rx_buffer[3];

            ki += rx_buffer[4] << 24;
            ki += rx_buffer[5] << 16;
            ki += rx_buffer[6] << 8;
            ki += rx_buffer[7];

            kd += rx_buffer[8] << 24;
            kd += rx_buffer[9] << 16;
            kd += rx_buffer[10] << 8;
            kd += rx_buffer[11];

            kiMax += rx_buffer[12] << 24;
            kiMax += rx_buffer[13] << 16;
            kiMax += rx_buffer[14] << 8;
            kiMax += rx_buffer[15];

            deadZone += rx_buffer[16] << 24;
            deadZone += rx_buffer[17] << 16;
            deadZone += rx_buffer[18] << 8;
            deadZone += rx_buffer[19];

            min += rx_buffer[20] << 24;
            min += rx_buffer[21] << 16;
            min += rx_buffer[22] << 8;
            min += rx_buffer[23];

            max += rx_buffer[24] << 24;
            max += rx_buffer[25] << 16;
            max += rx_buffer[26] << 8;
            max += rx_buffer[27];

            OsrModule::PosPidData posPidData = { kp, ki, kd, kiMax, deadZone, min, max };

            if(curr_cmd == OsrModule::ROBOCLAW_CMD::READM1POSPID)
            {
              this->posPidTlmData[0] = posPidData;
            }
            else
            {
              this->posPidTlmData[1] = posPidData;
            }
            this->tlmWrite_PosPIDValues(posPidTlmData);

            waitRecv = false;
          }
         break;
      default:
        break;
    }

    // Fw::Logger::logMsg("Recv Len: %d\n", recvBuffer.getSize());

    if(!waitRecv)
    {
      switchTlmState();
      rx_index = 0;
    }

    this->deallocate_out(0, recvBuffer);
  }

  void Roboclaw ::
    motorControlIn_handler(
        const NATIVE_INT_TYPE portNum,
        const OsrModule::ROBOCLAW_CMD &cmd,
        const OsrModule::MOTOR_SELECT &motor,
        F32 velocity,
        U32 acceleration,
        U32 distance
    )
  {
    I32 vel = this->velocity2qpps(velocity, motor);

    switch(cmd)
    {
      case OsrModule::ROBOCLAW_CMD::M1SPEED:
      case OsrModule::ROBOCLAW_CMD::M2SPEED:
        this->setVelocity(motor, vel);
        break;
    }
  }

  void Roboclaw ::
    run_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    this->getEncoderValues();
    this->getSpeedValues();
    this->getPositionPIDs();
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void Roboclaw ::
    MOVE_CONTINUOUS_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        OsrModule::MOTOR_SELECT motor,
        OsrModule::MOVE_DIRECTION direction,
        U8 speed_percentage
    )
  {
    speed_percentage = (speed_percentage > 100) ? 100 : speed_percentage;
    I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);

    if(direction == OsrModule::MOVE_DIRECTION::BACKWARD)
      velocity *= -1;
    else if(direction == OsrModule::MOVE_DIRECTION::STOP)
      velocity = 0;

    if(motor == OsrModule::MOTOR_SELECT::BOTH)
      this->setVelocityM1M2(velocity);
    else
      this->setVelocity(motor, velocity);

    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void Roboclaw ::
    MOVE_DISTANCE_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        OsrModule::MOTOR_SELECT motor,
        OsrModule::MOVE_DIRECTION direction,
        U8 speed_percentage,
        U32 distance
    )
  {
    speed_percentage = (speed_percentage > 100) ? 100 : speed_percentage;
    I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);

    if(direction == OsrModule::MOVE_DIRECTION::BACKWARD)
    {
      velocity *= -1;
    }
    else if(direction == OsrModule::MOVE_DIRECTION::STOP)
    {
      velocity = 0;
      distance = 0;
    }

    if(motor == OsrModule::MOTOR_SELECT::BOTH)
      this->setVelocityDistanceM1M2(velocity, distance);
    else
      this->setVelocityDistance(motor, velocity, distance);

    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void Roboclaw ::
    MOVE_ACCELERATED_CONTINUOUS_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        OsrModule::MOTOR_SELECT motor,
        OsrModule::MOVE_DIRECTION direction,
        U32 acceleration,
        U8 speed_percentage
    )
  {
    speed_percentage = (speed_percentage > 100) ? 100 : speed_percentage;
    I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);

    if(direction == OsrModule::MOVE_DIRECTION::BACKWARD)
      velocity *= -1;
    else if(direction == OsrModule::MOVE_DIRECTION::STOP)
      velocity = 0;

    if(motor == OsrModule::MOTOR_SELECT::BOTH)
      this->setAccelVelocityM1M2(acceleration, velocity);
    else
      this->setAccelVelocity(motor, acceleration, velocity);

    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void Roboclaw ::
    MOVE_ACCELERATED_DISTANCE_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        OsrModule::MOTOR_SELECT motor,
        OsrModule::MOVE_DIRECTION direction,
        U32 acceleration,
        U8 speed_percentage,
        U32 distance
    )
  {
    speed_percentage = (speed_percentage > 100) ? 100 : speed_percentage;
    I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);

    if(direction == OsrModule::MOVE_DIRECTION::BACKWARD)
    {
      velocity *= -1;
    }
    else if(direction == OsrModule::MOVE_DIRECTION::STOP)
    {
      velocity = 0;
      distance = 0;
    }

    if(motor == OsrModule::MOTOR_SELECT::BOTH)
      this->setAccelVelocityDistanceM1M2(acceleration, velocity, distance);
    else
      this->setAccelVelocityDistance(motor, acceleration, velocity, distance);

    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void Roboclaw ::
    STOP_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    this->setVelocityM1M2(0);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void Roboclaw ::
    RESET_ENCODERS_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    this->write(m_addr, OsrModule::ROBOCLAW_CMD::RESETENC, nullptr, 0);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

} // end namespace OsrModule
