// ======================================================================
// \title  RoboclawImpl.cpp
// \author ethanchee
// \brief  cpp file for Roboclaw component implementation class
// ======================================================================

#include <Components/Roboclaw/Roboclaw.hpp>

namespace Components {

    void Roboclaw::getEncoderValues()
    {
        if(waitRecv || (not this->tlm_state == TLM_STATE_MACHINE::ENCODER))
            return;

        this->curr_cmd = GETENCODERS;
        this->write(m_addr, GETENCODERS, nullptr, 0);
        this->waitRecv = true;
    }

    void Roboclaw::getSpeedValues()
    {
        if(waitRecv || (not this->tlm_state == TLM_STATE_MACHINE::SPEED))
            return;

        this->curr_cmd = GETISPEEDS;
        this->write(m_addr, GETISPEEDS, nullptr, 0);
        this->waitRecv = true;
    }

    void Roboclaw::updateTlm(Roboclaw::CMD cmd, I32 ret1, I32 ret2)
    {
        switch(cmd)
        {
            case GETENCODERS:
                this->encoderTlmData[0] = ret1;
                this->encoderTlmData[1] = ret2;
                this->tlmWrite_EncoderValues(encoderTlmData);
                this->tlm_state = TLM_STATE_MACHINE::SPEED;
                break;
            case GETISPEEDS:
                this->speedTlmData[0] = ret1;
                this->speedTlmData[1] = ret2;
                this->tlmWrite_SpeedValues(speedTlmData);
                this->tlm_state = TLM_STATE_MACHINE::ENCODER;
            default:
                break;
        }
    }

    void Roboclaw::write(U8 address, U8 command, U8 *tx_data, NATIVE_INT_TYPE tx_length) 
    {
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
    }

    U16 Roboclaw::crc16(U8 *packet, NATIVE_INT_TYPE nBytes)
    {
        U16 crc = 0;
        for (NATIVE_INT_TYPE byte = 0; byte < nBytes; byte++)
        {
            crc = crc ^ ((U16) packet[byte] << 8);
            for (U8 bit = 0; bit < 8; bit++)
            {
                if (crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc = crc << 1;
            }
        }
        return crc;
    }

}
