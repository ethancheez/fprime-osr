// ======================================================================
// \title  RoboclawImpl.cpp
// \author ethanchee
// \brief  cpp file for Roboclaw component implementation class
// ======================================================================

#include <Components/Roboclaw/Roboclaw.hpp>
#include <FpConfig.hpp>

namespace Components {

    void Roboclaw::setDutyCycle(Components::Roboclaw::MOTOR motor, RoboclawModule::MOVE_DIRECTION direction, U8 speed_percentage)
    {
        I32 duty_cycle = MAX_DUTY_CYCLE * ((F32)speed_percentage / 100);

        if(direction == RoboclawModule::MOVE_DIRECTION::BACKWARD)
            duty_cycle *= -1;
        else if(direction == RoboclawModule::MOVE_DIRECTION::STOP)
            duty_cycle = 0;

        fillBuffer16(&tx_buffer[0], duty_cycle);    // Motor Duty Cycle
        
        RoboclawModule::ROBOCLAW_CMD cmd = (motor == Components::Roboclaw::MOTOR::MOTOR1) ? RoboclawModule::ROBOCLAW_CMD::M1DUTY : RoboclawModule::ROBOCLAW_CMD::M2DUTY;
        this->write(m_addr, cmd, tx_buffer, 2);
    }

    void Roboclaw::setDutyCycleM1M2(RoboclawModule::MOVE_DIRECTION direction, U8 speed_percentage)
    {
        I32 duty_cycle = MAX_DUTY_CYCLE * ((F32)speed_percentage / 100);

        if(direction == RoboclawModule::MOVE_DIRECTION::BACKWARD)
            duty_cycle *= -1;
        else if(direction == RoboclawModule::MOVE_DIRECTION::STOP)
            duty_cycle = 0;

        fillBuffer16(&tx_buffer[0], duty_cycle);    // Motor 1 Duty Cycle
        fillBuffer16(&tx_buffer[2], duty_cycle);    // Motor 2 Duty Cycle
        
        this->write(m_addr, RoboclawModule::ROBOCLAW_CMD::MIXEDDUTY, tx_buffer, 4);
    }

    void Roboclaw::setVelocity(Components::Roboclaw::MOTOR motor, RoboclawModule::MOVE_DIRECTION direction, U8 speed_percentage)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);

        if(direction == RoboclawModule::MOVE_DIRECTION::BACKWARD)
            velocity *= -1;
        else if(direction == RoboclawModule::MOVE_DIRECTION::STOP)
            velocity = 0;

        fillBuffer32(&tx_buffer[0], velocity);  // Motor Velocity
        
        RoboclawModule::ROBOCLAW_CMD cmd = (motor == Components::Roboclaw::MOTOR::MOTOR1) ? RoboclawModule::ROBOCLAW_CMD::M1SPEED : RoboclawModule::ROBOCLAW_CMD::M2SPEED;
        this->write(m_addr, cmd, tx_buffer, 4);
    }

    void Roboclaw::setVelocityM1M2(RoboclawModule::MOVE_DIRECTION direction, U8 speed_percentage)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);

        if(direction == RoboclawModule::MOVE_DIRECTION::BACKWARD)
            velocity *= -1;
        else if(direction == RoboclawModule::MOVE_DIRECTION::STOP)
            velocity = 0;

        fillBuffer32(&tx_buffer[0], velocity);  // Motor 1 Velocity
        fillBuffer32(&tx_buffer[4], velocity);  // Motor 2 Velocity
        
        this->write(m_addr, RoboclawModule::ROBOCLAW_CMD::MIXEDSPEED, tx_buffer, 8);
    }

    void Roboclaw::setVelocityDistance(Components::Roboclaw::MOTOR motor, RoboclawModule::MOVE_DIRECTION direction, U8 speed_percentage, U32 distance)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == RoboclawModule::MOVE_DIRECTION::BACKWARD)
        {
            velocity *= -1;
        }
        else if(direction == RoboclawModule::MOVE_DIRECTION::STOP)
        {
            velocity = 0;
            distance = 0;
        }

        fillBuffer32(&tx_buffer[0], velocity);  // Motor Velocity
        fillBuffer32(&tx_buffer[4], distance);  // Motor Distance
        fillBuffer8(&tx_buffer[8], flag);       // Flag

        RoboclawModule::ROBOCLAW_CMD cmd = (motor == Components::Roboclaw::MOTOR::MOTOR1) ? RoboclawModule::ROBOCLAW_CMD::M1SPEEDDIST : RoboclawModule::ROBOCLAW_CMD::M2SPEEDDIST;
        this->write(m_addr, cmd, tx_buffer, 9);
    }

    void Roboclaw::setVelocityDistanceM1M2(RoboclawModule::MOVE_DIRECTION direction, U8 speed_percentage, U32 distance)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == RoboclawModule::MOVE_DIRECTION::BACKWARD)
        {
            velocity *= -1;
        }
        else if(direction == RoboclawModule::MOVE_DIRECTION::STOP)
        {
            velocity = 0;
            distance = 0;
        }

        fillBuffer32(&tx_buffer[0], velocity);  // Motor 1 Velocity
        fillBuffer32(&tx_buffer[4], distance);  // Motor 1 Distance
        fillBuffer32(&tx_buffer[8], velocity);  // Motor 2 Velocity
        fillBuffer32(&tx_buffer[12], distance); // Motor 2 Distance
        fillBuffer8(&tx_buffer[16], flag);      // Flag

        this->write(m_addr, RoboclawModule::ROBOCLAW_CMD::MIXEDSPEEDDIST, tx_buffer, 17);
    }

    void Roboclaw::setAccelVelocity(Components::Roboclaw::MOTOR motor, RoboclawModule::MOVE_DIRECTION direction, U32 accel, U8 speed_percentage)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == RoboclawModule::MOVE_DIRECTION::BACKWARD)
            velocity *= -1;
        else if(direction == RoboclawModule::MOVE_DIRECTION::STOP)
            velocity = 0;
        
        fillBuffer32(&tx_buffer[0], accel);     // Acceleration
        fillBuffer32(&tx_buffer[4], velocity);  // Motor Velocity

        RoboclawModule::ROBOCLAW_CMD cmd = (motor == Components::Roboclaw::MOTOR::MOTOR1) ? RoboclawModule::ROBOCLAW_CMD::M1SPEEDACCEL : RoboclawModule::ROBOCLAW_CMD::M2SPEEDACCEL;
        this->write(m_addr, RoboclawModule::ROBOCLAW_CMD::M1SPEEDACCEL, tx_buffer, 8);
    }

    void Roboclaw::setAccelVelocityM1M2(RoboclawModule::MOVE_DIRECTION direction, U32 accel, U8 speed_percentage)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == RoboclawModule::MOVE_DIRECTION::BACKWARD)
            velocity *= -1;
        else if(direction == RoboclawModule::MOVE_DIRECTION::STOP)
            velocity = 0;
        
        fillBuffer32(&tx_buffer[0], accel);     // Acceleration
        fillBuffer32(&tx_buffer[4], velocity);  // Motor 1 Velocity
        fillBuffer32(&tx_buffer[8], velocity);  // Motor 2 Velocity

        this->write(m_addr, RoboclawModule::ROBOCLAW_CMD::MIXEDSPEEDACCEL, tx_buffer, 12);
    }

    void Roboclaw::setAccelVelocityDistance(Components::Roboclaw::MOTOR motor, RoboclawModule::MOVE_DIRECTION direction, U32 accel, U8 speed_percentage, U32 distance)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == RoboclawModule::MOVE_DIRECTION::BACKWARD)
        {
            velocity *= -1;
        }
        else if(direction == RoboclawModule::MOVE_DIRECTION::STOP)
        {
            velocity = 0;
            distance = 0;
        }
        
        fillBuffer32(&tx_buffer[0], accel);     // Acceleration
        fillBuffer32(&tx_buffer[4], velocity);  // Motor Velocity 
        fillBuffer32(&tx_buffer[8], distance);  // Motor Distance
        fillBuffer8(&tx_buffer[12], flag);      // Flag

        RoboclawModule::ROBOCLAW_CMD cmd = (motor == Components::Roboclaw::MOTOR::MOTOR1) ? RoboclawModule::ROBOCLAW_CMD::M1SPEEDACCELDIST : RoboclawModule::ROBOCLAW_CMD::M2SPEEDACCELDIST;
        this->write(m_addr, cmd, tx_buffer, 13);
    }

    void Roboclaw::setAccelVelocityDistanceM1M2(RoboclawModule::MOVE_DIRECTION direction, U32 accel, U8 speed_percentage, U32 distance)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == RoboclawModule::MOVE_DIRECTION::BACKWARD)
        {
            velocity *= -1;
        }
        else if(direction == RoboclawModule::MOVE_DIRECTION::STOP)
        {
            velocity = 0;
            distance = 0;
        }
        
        fillBuffer32(&tx_buffer[0], accel);     // Acceleration
        fillBuffer32(&tx_buffer[4], velocity);  // Motor 1 Velocity 
        fillBuffer32(&tx_buffer[8], distance);  // Motor 1 Distance
        fillBuffer32(&tx_buffer[12], velocity); // Motor 2 Velocity
        fillBuffer32(&tx_buffer[16], distance); // Motor 2 Distance
        fillBuffer8(&tx_buffer[20], flag);      // Flag

        this->write(m_addr, RoboclawModule::ROBOCLAW_CMD::MIXEDSPEEDACCELDIST, tx_buffer, 21);
    }

    void Roboclaw::getEncoderValues()
    {
        if(waitRecv || (not this->tlm_state == TLM_STATE_MACHINE::ENCODER))
            return;

        this->curr_cmd = RoboclawModule::ROBOCLAW_CMD::GETENCODERS;
        this->write(m_addr, RoboclawModule::ROBOCLAW_CMD::GETENCODERS, nullptr, 0);
        this->waitRecv = true;
    }

    void Roboclaw::getSpeedValues()
    {
        if(waitRecv || (not this->tlm_state == TLM_STATE_MACHINE::SPEED))
            return;

        this->curr_cmd = RoboclawModule::ROBOCLAW_CMD::GETISPEEDS;
        this->write(m_addr, RoboclawModule::ROBOCLAW_CMD::GETISPEEDS, nullptr, 0);
        this->waitRecv = true;
    }

    void Roboclaw::updateTlm(RoboclawModule::ROBOCLAW_CMD cmd, I32 ret1, I32 ret2)
    {
        switch(cmd)
        {
            case RoboclawModule::ROBOCLAW_CMD::GETENCODERS:
                this->encoderTlmData[0] = ret1;
                this->encoderTlmData[1] = ret2;
                this->tlmWrite_EncoderValues(encoderTlmData);
                this->tlm_state = TLM_STATE_MACHINE::SPEED;
                break;
            case RoboclawModule::ROBOCLAW_CMD::GETISPEEDS:
                this->speedTlmData[0] = ret1;
                this->speedTlmData[1] = ret2;
                this->tlmWrite_SpeedValues(speedTlmData);
                this->tlm_state = TLM_STATE_MACHINE::ENCODER;
                break;
            default:
                break;
        }
    }

    void Roboclaw::fillBuffer8(U8 *buf, U8 val)
    {
        *(buf) = (U8) (val & 0xFF);
    }

    void Roboclaw::fillBuffer16(U8 *buf, U16 val)
    {
        *(buf) = (U8) ((val >> 8) & 0xFF);
        *(buf + 1) = (U8) (val & 0xFF);
    }

    void Roboclaw::fillBuffer32(U8 *buf, U32 val)
    {
        *(buf) = (U8) ((val >> 24) & 0xFF);
        *(buf + 1) = (U8) ((val >> 16) & 0xFF);
        *(buf + 2) = (U8) ((val >> 8) & 0xFF);
        *(buf + 3) = (U8) (val & 0xFF);
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
