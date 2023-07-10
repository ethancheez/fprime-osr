// ======================================================================
// \title  Roboclaw.cpp
// \author ethanchee
// \brief  cpp file for Roboclaw component implementation class
// ======================================================================


#include <Components/Roboclaw/Roboclaw.hpp>
#include <FpConfig.hpp>
#include <Fw/Logger/Logger.hpp>

#include <unistd.h>

namespace Components {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Roboclaw ::
    Roboclaw(
        const char *const compName
    ) : RoboclawComponentBase(compName)
  {

  }

  Roboclaw ::
    ~Roboclaw()
  {

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
    this->lock.lock();

    memcpy(&rx_buffer[rx_index], recvBuffer.getData(), recvBuffer.getSize());
    rx_index += recvBuffer.getSize();

    switch(curr_cmd)
    {
      case Roboclaw::CMD::GETM1ENC:
      case Roboclaw::CMD::GETM2ENC:
      case Roboclaw::CMD::GETM1SPEED:
      case Roboclaw::CMD::GETM2SPEED:
      case Roboclaw::CMD::GETERROR:
        if(rx_index == 7)
        {
          ret1 += rx_buffer[0] << 24;
          ret1 += rx_buffer[1] << 16;
          ret1 += rx_buffer[2] << 8;
          ret1 += rx_buffer[3];

          recvComplete = true;
        }
        break;
      case Roboclaw::CMD::GETPWMS:
      case Roboclaw::CMD::GETCURRENTS:
        if(rx_index == 6)
        {
          ret1 += rx_buffer[0] << 8;
          ret1 += rx_buffer[1];
          ret2 += rx_buffer[2] << 8;
          ret2 += rx_buffer[3];
          
          recvComplete = true;
        }
        break;
      case Roboclaw::CMD::GETCONFIG:
        if(rx_index == 4)
        {
          ret1 += rx_buffer[0] << 8;
          ret1 += rx_buffer[1];
          
          recvComplete = true;
        }
        break;
      case Roboclaw::CMD::GETVERSION:
        break;
      case Roboclaw::CMD::GETM1MAXCURRENT:
      case Roboclaw::CMD::GETM2MAXCURRENT:
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
          
          recvComplete = true;
        }
        break;
      case Roboclaw::CMD::GETDUTYCYCLE:
        
        break;
    }

    if(recvComplete)
    {
      Fw::Logger::logMsg("RET1: %d\tRET2: %d\n", ret1, ret2);

      reset_outputs();
    }

    this->lock.unLock();
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void Roboclaw ::
    GET_DATA_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        U8 addr,
        Components::ROBOCLAW_GET_DATA_CMDS cmd
    )
  {
    curr_cmd = (Roboclaw::CMD)((NATIVE_INT_TYPE)cmd);
    this->write(addr, cmd, nullptr, 0);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void Roboclaw ::
    SET_MOTORS_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        U8 addr,
        Components::ROBOCLAW_SET_MOTORS_CMDS cmd,
        U32 motor1,
        U32 motor2
    )
  {
    curr_cmd = (Roboclaw::CMD)((NATIVE_INT_TYPE)cmd);
    U8 tx_size = 0;
    
    switch(curr_cmd)
    {
      case Roboclaw::CMD::MIXEDDUTY:
        tx_buffer[0] = (U8) ((motor1 >> 8) & 0xFF);
        tx_buffer[1] = (U8) (motor1 & 0xFF);

        tx_buffer[2] = (U8) ((motor2 >> 8) & 0xFF);
        tx_buffer[3] = (U8) (motor2 & 0xFF);

        tx_size = 4;
        break;
      case Roboclaw::CMD::MIXEDSPEED:
        tx_buffer[0] = (U8) ((motor1 >> 24) & 0xFF);
        tx_buffer[1] = (U8) ((motor1 >> 16) & 0xFF);
        tx_buffer[2] = (U8) ((motor1 >> 8) & 0xFF);
        tx_buffer[3] = (U8) (motor1 & 0xFF);

        tx_buffer[4] = (U8) ((motor2 >> 24) & 0xFF);
        tx_buffer[5] = (U8) ((motor2 >> 16) & 0xFF);
        tx_buffer[6] = (U8) ((motor2 >> 8) & 0xFF);
        tx_buffer[7] = (U8) (motor2 & 0xFF);

        tx_size = 8;
        break;
    }

    this->write(addr, cmd, tx_buffer, tx_size);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void Roboclaw ::
    SET_SINGLE_MOTOR_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        U8 addr,
        Components::ROBOCLAW_SET_SINGLE_MOTOR cmd,
        U32 val
    )
  {
    curr_cmd = (Roboclaw::CMD)((NATIVE_INT_TYPE)cmd);
    U8 tx_size = 0;

    switch(curr_cmd)
    {
      case Roboclaw::CMD::M1FORWARD:
      case Roboclaw::CMD::M2FORWARD:
        tx_buffer[0] = (U8) ((val >> 8) & 0xFF);
        tx_buffer[1] = (U8) (val & 0xFF);
        
        tx_size = 1;
        break;
    }

    this->write(addr, cmd, tx_buffer, tx_size);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }


  // ----------------------------------------------------------------------
  // Helpers
  // ----------------------------------------------------------------------

  void Roboclaw::reset_outputs()
  {
    ret1 = 0;
    ret2 = 0;
    rx_index = 0;
    recvComplete = false;
  }

  void Roboclaw::write(U8 address, U8 command, U8 *tx_data, NATIVE_INT_TYPE tx_length) 
  {
    this->lock.lock();

    Fw::Buffer buf = this->allocate_out(0, tx_length + 4);
    U8 *packet = buf.getData();

    // Header
    packet[0] = address;
    packet[1] = command;

    // Data
    if (tx_length > 0 && tx_data != nullptr)
    {
      memcpy(&packet[2], tx_data, tx_length);
    }

    // CRC
    U16 crc = crc16(&packet[0], tx_length + 2);
    packet[tx_length + 2] = (U8) ((crc >> 8) & 0xFF);
    packet[tx_length + 2 + 1] = (U8) (crc & 0xFF);

    this->comDataOut_out(0, buf);

    this->lock.unLock();
  }

  U16 Roboclaw::crc16(U8 *packet, NATIVE_INT_TYPE nBytes) {
    U16 crc = 0;
    for (NATIVE_INT_TYPE byte = 0; byte < nBytes; byte++) {
      crc = crc ^ ((U16) packet[byte] << 8);
      for (U8 bit = 0; bit < 8; bit++) {
        if (crc & 0x8000)
          crc = (crc << 1) ^ 0x1021;
        else
          crc = crc << 1;
      }
    }
    return crc;
  }

} // end namespace Components
