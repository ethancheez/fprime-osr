// ======================================================================
// \title  RoboclawImpl.cpp
// \author ethanchee
// \brief  cpp file for Roboclaw component implementation class
// ======================================================================

#include <Components/Roboclaw/Roboclaw.hpp>
#include <FpConfig.hpp>

namespace Components {

    void Roboclaw::setDutyCycleM1M2(Components::ROBOCLAW_MOVE_DIRECTION direction, U8 speed_percentage)
    {
        I32 duty_cycle = MAX_DUTY_CYCLE * ((F32)speed_percentage / 100);

        if(direction == Components::ROBOCLAW_MOVE_DIRECTION::BACKWARD)
            duty_cycle *= -1;
        else if(direction == Components::ROBOCLAW_MOVE_DIRECTION::STOP)
            duty_cycle = 0;

        // Motor 1 Duty Cycle
        tx_buffer[0] = (U8) ((duty_cycle >> 8) & 0xFF);
        tx_buffer[1] = (U8) (duty_cycle & 0xFF);
        // Motor 2 Duty Cycle
        tx_buffer[2] = (U8) ((duty_cycle >> 8) & 0xFF);
        tx_buffer[3] = (U8) (duty_cycle & 0xFF);
        
        this->write(m_addr, MIXEDDUTY, tx_buffer, 4);
    }

    void Roboclaw::setVelocityM1M2(Components::ROBOCLAW_MOVE_DIRECTION direction, U8 speed_percentage)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);

        if(direction == Components::ROBOCLAW_MOVE_DIRECTION::BACKWARD)
            velocity *= -1;
        else if(direction == Components::ROBOCLAW_MOVE_DIRECTION::STOP)
            velocity = 0;

        // Motor 1 Velocity
        tx_buffer[0] = (U8) ((velocity >> 24) & 0xFF);
        tx_buffer[1] = (U8) ((velocity >> 16) & 0xFF);
        tx_buffer[2] = (U8) ((velocity >> 8) & 0xFF);
        tx_buffer[3] = (U8) (velocity & 0xFF);
        // Motor 2 Velocity
        tx_buffer[4] = (U8) ((velocity >> 24) & 0xFF);
        tx_buffer[5] = (U8) ((velocity >> 16) & 0xFF);
        tx_buffer[6] = (U8) ((velocity >> 8) & 0xFF);
        tx_buffer[7] = (U8) (velocity & 0xFF);
        
        this->write(m_addr, MIXEDSPEED, tx_buffer, 8);
    }

    void Roboclaw::setVelocityDistanceM1M2(Components::ROBOCLAW_MOVE_DIRECTION direction, U8 speed_percentage, U32 distance)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == Components::ROBOCLAW_MOVE_DIRECTION::BACKWARD)
        {
            velocity *= -1;
        }
        else if(direction == Components::ROBOCLAW_MOVE_DIRECTION::STOP)
        {
            velocity = 0;
            distance = 0;
        }

        // Motor 1 Velocity
        tx_buffer[0] = (U8) ((velocity >> 24) & 0xFF);
        tx_buffer[1] = (U8) ((velocity >> 16) & 0xFF);
        tx_buffer[2] = (U8) ((velocity >> 8) & 0xFF);
        tx_buffer[3] = (U8) (velocity & 0xFF);
        // Motor 1 Distance
        tx_buffer[4] = (U8) ((distance >> 24) & 0xFF);
        tx_buffer[5] = (U8) ((distance >> 16) & 0xFF);
        tx_buffer[6] = (U8) ((distance >> 8) & 0xFF);
        tx_buffer[7] = (U8) (distance & 0xFF);
        // Motor 2 Velocity
        tx_buffer[8] = (U8) ((velocity >> 24) & 0xFF);
        tx_buffer[9] = (U8) ((velocity >> 16) & 0xFF);
        tx_buffer[10] = (U8) ((velocity >> 8) & 0xFF);
        tx_buffer[11] = (U8) (velocity & 0xFF);
        // Motor 2 Distance
        tx_buffer[12] = (U8) ((distance >> 24) & 0xFF);
        tx_buffer[13] = (U8) ((distance >> 16) & 0xFF);
        tx_buffer[14] = (U8) ((distance >> 8) & 0xFF);
        tx_buffer[15] = (U8) (distance & 0xFF);
        // Flag
        tx_buffer[16] = (U8) (flag & 0xFF);

        this->write(m_addr, MIXEDSPEEDDIST, tx_buffer, 17);
    }

    void Roboclaw::setAccelVelocityM1M2(Components::ROBOCLAW_MOVE_DIRECTION direction, U32 accel, U8 speed_percentage)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == Components::ROBOCLAW_MOVE_DIRECTION::BACKWARD)
            velocity *= -1;
        else if(direction == Components::ROBOCLAW_MOVE_DIRECTION::STOP)
            velocity = 0;

        // Acceleration
        tx_buffer[0] = (U8) ((accel >> 24) & 0xFF);
        tx_buffer[1] = (U8) ((accel >> 16) & 0xFF);
        tx_buffer[2] = (U8) ((accel >> 8) & 0xFF);
        tx_buffer[3] = (U8) (accel & 0xFF);
        // Motor 1 Velocity
        tx_buffer[4] = (U8) ((velocity >> 24) & 0xFF);
        tx_buffer[5] = (U8) ((velocity >> 16) & 0xFF);
        tx_buffer[6] = (U8) ((velocity >> 8) & 0xFF);
        tx_buffer[7] = (U8) (velocity & 0xFF);
        // Motor 2 Velocity
        tx_buffer[8] = (U8) ((velocity >> 24) & 0xFF);
        tx_buffer[9] = (U8) ((velocity >> 16) & 0xFF);
        tx_buffer[10] = (U8) ((velocity >> 8) & 0xFF);
        tx_buffer[11] = (U8) (velocity & 0xFF);

        this->write(m_addr, MIXEDSPEEDACCEL, tx_buffer, 12);
    }

    void Roboclaw::setAccelVelocityDistanceM1M2(Components::ROBOCLAW_MOVE_DIRECTION direction, U32 accel, U8 speed_percentage, U32 distance)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == Components::ROBOCLAW_MOVE_DIRECTION::BACKWARD)
        {
            velocity *= -1;
        }
        else if(direction == Components::ROBOCLAW_MOVE_DIRECTION::STOP)
        {
            velocity = 0;
            distance = 0;
        }

        // Acceleration
        tx_buffer[0] = (U8) ((accel >> 24) & 0xFF);
        tx_buffer[1] = (U8) ((accel >> 16) & 0xFF);
        tx_buffer[2] = (U8) ((accel >> 8) & 0xFF);
        tx_buffer[3] = (U8) (accel & 0xFF);
        // Motor 1 Velocity
        tx_buffer[4] = (U8) ((velocity >> 24) & 0xFF);
        tx_buffer[5] = (U8) ((velocity >> 16) & 0xFF);
        tx_buffer[6] = (U8) ((velocity >> 8) & 0xFF);
        tx_buffer[7] = (U8) (velocity & 0xFF);
        // Motor 1 Distance
        tx_buffer[8] = (U8) ((distance >> 24) & 0xFF);
        tx_buffer[9] = (U8) ((distance >> 16) & 0xFF);
        tx_buffer[10] = (U8) ((distance >> 8) & 0xFF);
        tx_buffer[11] = (U8) (distance & 0xFF);
        // Motor 2 Velocity
        tx_buffer[12] = (U8) ((velocity >> 24) & 0xFF);
        tx_buffer[13] = (U8) ((velocity >> 16) & 0xFF);
        tx_buffer[14] = (U8) ((velocity >> 8) & 0xFF);
        tx_buffer[15] = (U8) (velocity & 0xFF);
        // Motor 2 Distance
        tx_buffer[16] = (U8) ((distance >> 24) & 0xFF);
        tx_buffer[17] = (U8) ((distance >> 16) & 0xFF);
        tx_buffer[18] = (U8) ((distance >> 8) & 0xFF);
        tx_buffer[19] = (U8) (distance & 0xFF);
        // Flag
        tx_buffer[20] = (U8) (flag & 0xFF);

        this->write(m_addr, MIXEDSPEEDACCELDIST, tx_buffer, 21);
    }

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
                break;
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
