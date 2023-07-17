// ======================================================================
// \title  RoboclawImpl.cpp
// \author ethanchee
// \brief  cpp file for Roboclaw component implementation class
// ======================================================================

#include <Components/Roboclaw/Roboclaw.hpp>
#include <FpConfig.hpp>

namespace OsrModule {

    void Roboclaw::setDutyCycle(OsrModule::MOTOR_SELECT motor, OsrModule::MOVE_DIRECTION direction, U8 speed_percentage)
    {
        I32 duty_cycle = MAX_DUTY_CYCLE * ((F32)speed_percentage / 100);

        if(direction == OsrModule::MOVE_DIRECTION::BACKWARD)
            duty_cycle *= -1;
        else if(direction == OsrModule::MOVE_DIRECTION::STOP)
            duty_cycle = 0;

        fillBuffer16(&tx_buffer[0], duty_cycle);    // Motor Duty Cycle
        
        OsrModule::ROBOCLAW_CMD cmd = (motor == OsrModule::MOTOR_SELECT::MOTOR1) ? OsrModule::ROBOCLAW_CMD::M1DUTY : OsrModule::ROBOCLAW_CMD::M2DUTY;
        this->write(m_addr, cmd, tx_buffer, 2);
    }

    void Roboclaw::setDutyCycleM1M2(OsrModule::MOVE_DIRECTION direction, U8 speed_percentage)
    {
        I32 duty_cycle = MAX_DUTY_CYCLE * ((F32)speed_percentage / 100);

        if(direction == OsrModule::MOVE_DIRECTION::BACKWARD)
            duty_cycle *= -1;
        else if(direction == OsrModule::MOVE_DIRECTION::STOP)
            duty_cycle = 0;

        fillBuffer16(&tx_buffer[0], duty_cycle);    // Motor 1 Duty Cycle
        fillBuffer16(&tx_buffer[2], duty_cycle);    // Motor 2 Duty Cycle
        
        this->write(m_addr, OsrModule::ROBOCLAW_CMD::MIXEDDUTY, tx_buffer, 4);
    }

    void Roboclaw::setVelocity(OsrModule::MOTOR_SELECT motor, OsrModule::MOVE_DIRECTION direction, U8 speed_percentage)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);

        if(direction == OsrModule::MOVE_DIRECTION::BACKWARD)
            velocity *= -1;
        else if(direction == OsrModule::MOVE_DIRECTION::STOP)
            velocity = 0;

        fillBuffer32(&tx_buffer[0], velocity);  // Motor Velocity
        
        OsrModule::ROBOCLAW_CMD cmd = (motor == OsrModule::MOTOR_SELECT::MOTOR1) ? OsrModule::ROBOCLAW_CMD::M1SPEED : OsrModule::ROBOCLAW_CMD::M2SPEED;
        this->write(m_addr, cmd, tx_buffer, 4);
    }

    void Roboclaw::setVelocityM1M2(OsrModule::MOVE_DIRECTION direction, U8 speed_percentage)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);

        if(direction == OsrModule::MOVE_DIRECTION::BACKWARD)
            velocity *= -1;
        else if(direction == OsrModule::MOVE_DIRECTION::STOP)
            velocity = 0;

        fillBuffer32(&tx_buffer[0], velocity);  // Motor 1 Velocity
        fillBuffer32(&tx_buffer[4], velocity);  // Motor 2 Velocity
        
        this->write(m_addr, OsrModule::ROBOCLAW_CMD::MIXEDSPEED, tx_buffer, 8);
    }

    void Roboclaw::setVelocityDistance(OsrModule::MOTOR_SELECT motor, OsrModule::MOVE_DIRECTION direction, U8 speed_percentage, U32 distance)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == OsrModule::MOVE_DIRECTION::BACKWARD)
        {
            velocity *= -1;
        }
        else if(direction == OsrModule::MOVE_DIRECTION::STOP)
        {
            velocity = 0;
            distance = 0;
        }

        fillBuffer32(&tx_buffer[0], velocity);  // Motor Velocity
        fillBuffer32(&tx_buffer[4], distance);  // Motor Distance
        fillBuffer8(&tx_buffer[8], flag);       // Flag

        OsrModule::ROBOCLAW_CMD cmd = (motor == OsrModule::MOTOR_SELECT::MOTOR1) ? OsrModule::ROBOCLAW_CMD::M1SPEEDDIST : OsrModule::ROBOCLAW_CMD::M2SPEEDDIST;
        this->write(m_addr, cmd, tx_buffer, 9);
    }

    void Roboclaw::setVelocityDistanceM1M2(OsrModule::MOVE_DIRECTION direction, U8 speed_percentage, U32 distance)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == OsrModule::MOVE_DIRECTION::BACKWARD)
        {
            velocity *= -1;
        }
        else if(direction == OsrModule::MOVE_DIRECTION::STOP)
        {
            velocity = 0;
            distance = 0;
        }

        fillBuffer32(&tx_buffer[0], velocity);  // Motor 1 Velocity
        fillBuffer32(&tx_buffer[4], distance);  // Motor 1 Distance
        fillBuffer32(&tx_buffer[8], velocity);  // Motor 2 Velocity
        fillBuffer32(&tx_buffer[12], distance); // Motor 2 Distance
        fillBuffer8(&tx_buffer[16], flag);      // Flag

        this->write(m_addr, OsrModule::ROBOCLAW_CMD::MIXEDSPEEDDIST, tx_buffer, 17);
    }

    void Roboclaw::setAccelVelocity(OsrModule::MOTOR_SELECT motor, OsrModule::MOVE_DIRECTION direction, U32 accel, U8 speed_percentage)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == OsrModule::MOVE_DIRECTION::BACKWARD)
            velocity *= -1;
        else if(direction == OsrModule::MOVE_DIRECTION::STOP)
            velocity = 0;
        
        fillBuffer32(&tx_buffer[0], accel);     // Acceleration
        fillBuffer32(&tx_buffer[4], velocity);  // Motor Velocity

        OsrModule::ROBOCLAW_CMD cmd = (motor == OsrModule::MOTOR_SELECT::MOTOR1) ? OsrModule::ROBOCLAW_CMD::M1SPEEDACCEL : OsrModule::ROBOCLAW_CMD::M2SPEEDACCEL;
        this->write(m_addr, cmd, tx_buffer, 8);
    }

    void Roboclaw::setAccelVelocityM1M2(OsrModule::MOVE_DIRECTION direction, U32 accel, U8 speed_percentage)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == OsrModule::MOVE_DIRECTION::BACKWARD)
            velocity *= -1;
        else if(direction == OsrModule::MOVE_DIRECTION::STOP)
            velocity = 0;
        
        fillBuffer32(&tx_buffer[0], accel);     // Acceleration
        fillBuffer32(&tx_buffer[4], velocity);  // Motor 1 Velocity
        fillBuffer32(&tx_buffer[8], velocity);  // Motor 2 Velocity

        this->write(m_addr, OsrModule::ROBOCLAW_CMD::MIXEDSPEEDACCEL, tx_buffer, 12);
    }

    void Roboclaw::setAccelVelocityDistance(OsrModule::MOTOR_SELECT motor, OsrModule::MOVE_DIRECTION direction, U32 accel, U8 speed_percentage, U32 distance)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == OsrModule::MOVE_DIRECTION::BACKWARD)
        {
            velocity *= -1;
        }
        else if(direction == OsrModule::MOVE_DIRECTION::STOP)
        {
            velocity = 0;
            distance = 0;
        }
        
        fillBuffer32(&tx_buffer[0], accel);     // Acceleration
        fillBuffer32(&tx_buffer[4], velocity);  // Motor Velocity 
        fillBuffer32(&tx_buffer[8], distance);  // Motor Distance
        fillBuffer8(&tx_buffer[12], flag);      // Flag

        OsrModule::ROBOCLAW_CMD cmd = (motor == OsrModule::MOTOR_SELECT::MOTOR1) ? OsrModule::ROBOCLAW_CMD::M1SPEEDACCELDIST : OsrModule::ROBOCLAW_CMD::M2SPEEDACCELDIST;
        this->write(m_addr, cmd, tx_buffer, 13);
    }

    void Roboclaw::setAccelVelocityDistanceM1M2(OsrModule::MOVE_DIRECTION direction, U32 accel, U8 speed_percentage, U32 distance)
    {
        I32 velocity = MAX_VELOCITY * ((F32)speed_percentage / 100);
        U8 flag = 1;

        if(direction == OsrModule::MOVE_DIRECTION::BACKWARD)
        {
            velocity *= -1;
        }
        else if(direction == OsrModule::MOVE_DIRECTION::STOP)
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

        this->write(m_addr, OsrModule::ROBOCLAW_CMD::MIXEDSPEEDACCELDIST, tx_buffer, 21);
    }

    void Roboclaw::getEncoderValues()
    {
        if(waitRecv || (not this->tlm_state == TLM_STATE_MACHINE::ENCODER))
            return;

        this->curr_cmd = OsrModule::ROBOCLAW_CMD::GETENCODERS;
        this->write(m_addr, OsrModule::ROBOCLAW_CMD::GETENCODERS, nullptr, 0);
        this->waitRecv = true;
    }

    void Roboclaw::getSpeedValues()
    {
        if(waitRecv || (not this->tlm_state == TLM_STATE_MACHINE::SPEED))
            return;

        this->curr_cmd = OsrModule::ROBOCLAW_CMD::GETISPEEDS;
        this->write(m_addr, OsrModule::ROBOCLAW_CMD::GETISPEEDS, nullptr, 0);
        this->waitRecv = true;
    }

    void Roboclaw::updateTlm(OsrModule::ROBOCLAW_CMD cmd, I32 ret1, I32 ret2)
    {
        switch(cmd)
        {
            case OsrModule::ROBOCLAW_CMD::GETENCODERS:
                this->encoderTlmData[0] = ret1;
                this->encoderTlmData[1] = ret2;
                this->tlmWrite_EncoderValues(encoderTlmData);
                this->tlm_state = TLM_STATE_MACHINE::SPEED;
                break;
            case OsrModule::ROBOCLAW_CMD::GETISPEEDS:
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
