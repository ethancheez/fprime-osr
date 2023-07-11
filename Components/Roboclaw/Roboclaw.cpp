// ======================================================================
// \title  Roboclaw.cpp
// \author ethanchee
// \brief  cpp file for Roboclaw component implementation class
// ======================================================================


#include <Components/Roboclaw/Roboclaw.hpp>
#include <FpConfig.hpp>

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
      case GETENCODERS:
      case GETISPEEDS:
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

    if(!waitRecv)
    {
      updateTlm(curr_cmd, ret1, ret2);
      rx_index = 0;
    }

    this->deallocate_out(0, recvBuffer);
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
    MOVE_DIRECTION_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        Components::ROBOCLAW_MOVE_DIRECTION direction,
        U8 speed_percentage
    )
  {
    speed_percentage = (speed_percentage > 100) ? 100 : speed_percentage;
    I32 duty_cycle = 32767 * ((F32)speed_percentage / 100);

    switch(direction)
    {
      case Components::ROBOCLAW_MOVE_DIRECTION::BACKWARD:
        duty_cycle *= -1;
        break;
      case Components::ROBOCLAW_MOVE_DIRECTION::STOP:
        duty_cycle = 0;
        break;
      default:
        break;
    }

    tx_buffer[0] = (U8) ((duty_cycle >> 8) & 0xFF);
    tx_buffer[1] = (U8) (duty_cycle & 0xFF);

    tx_buffer[2] = (U8) ((duty_cycle >> 8) & 0xFF);
    tx_buffer[3] = (U8) (duty_cycle & 0xFF);
    
    this->write(m_addr, MIXEDDUTY, tx_buffer, 4);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void Roboclaw ::
    RESET_ENCODERS_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    this->write(m_addr, RESETENC, nullptr, 0);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

} // end namespace Components
