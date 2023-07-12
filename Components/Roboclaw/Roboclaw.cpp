// ======================================================================
// \title  Roboclaw.cpp
// \author ethanchee
// \brief  cpp file for Roboclaw component implementation class
// ======================================================================


#include <Components/Roboclaw/Roboclaw.hpp>
#include <FpConfig.hpp>
#include <Fw/Logger/Logger.hpp>

namespace Components {

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

  void Roboclaw::set_addr(U8 addr)
  {
    this->m_addr = addr;
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
      case RoboclawModule::ROBOCLAW_CMD::GETENCODERS:
      case RoboclawModule::ROBOCLAW_CMD::GETISPEEDS:
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

          waitRecv = false;
        }
        break;
      default:
        break;
    }

    // Fw::Logger::logMsg("Recv Len: %d\n", recvBuffer.getSize());

    if(!waitRecv)
    {
      updateTlm(curr_cmd, ret1, ret2);
      rx_index = 0;
    }

    this->deallocate_out(0, recvBuffer);
  }

  void Roboclaw ::
    motorControlIn_handler(
        const NATIVE_INT_TYPE portNum,
        const RoboclawModule::ROBOCLAW_CMD &cmd,
        const RoboclawModule::MOTOR_SELECT &motor,
        U8 speed_percentage,
        U32 acceleration,
        U32 distance
    )
  {
    // TODO
  }

  void Roboclaw ::
    run_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    this->getEncoderValues();
    this->getSpeedValues();
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void Roboclaw ::
    MOVE_CONTINUOUS_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        RoboclawModule::MOVE_DIRECTION direction,
        U8 speed_percentage
    )
  {
    speed_percentage = (speed_percentage > 100) ? 100 : speed_percentage;
    this->setVelocityM1M2(direction, speed_percentage);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void Roboclaw ::
    MOVE_DISTANCE_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        RoboclawModule::MOVE_DIRECTION direction,
        U8 speed_percentage,
        U32 distance
    )
  {
    speed_percentage = (speed_percentage > 100) ? 100 : speed_percentage;
    this->setVelocityDistanceM1M2(direction, speed_percentage, distance);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void Roboclaw ::
    MOVE_ACCELERATED_CONTINUOUS_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        RoboclawModule::MOVE_DIRECTION direction,
        U32 acceleration,
        U8 speed_percentage
    )
  {
    speed_percentage = (speed_percentage > 100) ? 100 : speed_percentage;
    this->setAccelVelocityM1M2(direction, acceleration, speed_percentage);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void Roboclaw ::
    MOVE_ACCELERATED_DISTANCE_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        RoboclawModule::MOVE_DIRECTION direction,
        U32 acceleration,
        U8 speed_percentage,
        U32 distance
    )
  {
    speed_percentage = (speed_percentage > 100) ? 100 : speed_percentage;
    this->setAccelVelocityDistanceM1M2(direction, acceleration, speed_percentage, distance);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void Roboclaw ::
    STOP_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    this->setVelocityM1M2(RoboclawModule::MOVE_DIRECTION::STOP, 0);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void Roboclaw ::
    RESET_ENCODERS_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    this->write(m_addr, RoboclawModule::ROBOCLAW_CMD::RESETENC, nullptr, 0);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

} // end namespace Components
