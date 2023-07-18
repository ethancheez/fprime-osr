// ======================================================================
// \title  RoboclawImpl.cpp
// \author ethanchee
// \brief  cpp file for Roboclaw component implementation class
// ======================================================================

#include <Components/Roboclaw/Roboclaw.hpp>
#include <FpConfig.hpp>
#include <Fw/Logger/Logger.hpp>

namespace OsrModule {

    void Roboclaw::setVelocity(OsrModule::MOTOR_SELECT motor, I32 velocity)
    {
        fillBuffer32(&tx_buffer[0], velocity);  // Motor Velocity
        
        OsrModule::ROBOCLAW_CMD cmd = (motor == OsrModule::MOTOR_SELECT::MOTOR1) ? OsrModule::ROBOCLAW_CMD::M1SPEED : OsrModule::ROBOCLAW_CMD::M2SPEED;
        this->write(m_addr, cmd, tx_buffer, 4);
    }

    void Roboclaw::setVelocityM1M2(I32 velocity)
    {
        fillBuffer32(&tx_buffer[0], velocity);  // Motor 1 Velocity
        fillBuffer32(&tx_buffer[4], velocity);  // Motor 2 Velocity
        
        this->write(m_addr, OsrModule::ROBOCLAW_CMD::MIXEDSPEED, tx_buffer, 8);
    }

    void Roboclaw::setVelocityDistance(OsrModule::MOTOR_SELECT motor, I32 velocity, U32 distance)
    {
        U8 flag = 1;

        fillBuffer32(&tx_buffer[0], velocity);  // Motor Velocity
        fillBuffer32(&tx_buffer[4], distance);  // Motor Distance
        fillBuffer8(&tx_buffer[8], flag);       // Flag

        OsrModule::ROBOCLAW_CMD cmd = (motor == OsrModule::MOTOR_SELECT::MOTOR1) ? OsrModule::ROBOCLAW_CMD::M1SPEEDDIST : OsrModule::ROBOCLAW_CMD::M2SPEEDDIST;
        this->write(m_addr, cmd, tx_buffer, 9);
    }

    void Roboclaw::setVelocityDistanceM1M2(I32 velocity, U32 distance)
    {
        U8 flag = 1;

        fillBuffer32(&tx_buffer[0], velocity);  // Motor 1 Velocity
        fillBuffer32(&tx_buffer[4], distance);  // Motor 1 Distance
        fillBuffer32(&tx_buffer[8], velocity);  // Motor 2 Velocity
        fillBuffer32(&tx_buffer[12], distance); // Motor 2 Distance
        fillBuffer8(&tx_buffer[16], flag);      // Flag

        this->write(m_addr, OsrModule::ROBOCLAW_CMD::MIXEDSPEEDDIST, tx_buffer, 17);
    }

    void Roboclaw::setAccelVelocity(OsrModule::MOTOR_SELECT motor, U32 accel, I32 velocity)
    {
        U8 flag = 1;
        
        fillBuffer32(&tx_buffer[0], accel);     // Acceleration
        fillBuffer32(&tx_buffer[4], velocity);  // Motor Velocity

        OsrModule::ROBOCLAW_CMD cmd = (motor == OsrModule::MOTOR_SELECT::MOTOR1) ? OsrModule::ROBOCLAW_CMD::M1SPEEDACCEL : OsrModule::ROBOCLAW_CMD::M2SPEEDACCEL;
        this->write(m_addr, cmd, tx_buffer, 8);
    }

    void Roboclaw::setAccelVelocityM1M2(U32 accel, I32 velocity)
    {
        U8 flag = 1;
        
        fillBuffer32(&tx_buffer[0], accel);     // Acceleration
        fillBuffer32(&tx_buffer[4], velocity);  // Motor 1 Velocity
        fillBuffer32(&tx_buffer[8], velocity);  // Motor 2 Velocity

        this->write(m_addr, OsrModule::ROBOCLAW_CMD::MIXEDSPEEDACCEL, tx_buffer, 12);
    }

    void Roboclaw::setAccelVelocityDistance(OsrModule::MOTOR_SELECT motor, U32 accel, I32 velocity, U32 distance)
    {
        U8 flag = 1;
        
        fillBuffer32(&tx_buffer[0], accel);     // Acceleration
        fillBuffer32(&tx_buffer[4], velocity);  // Motor Velocity 
        fillBuffer32(&tx_buffer[8], distance);  // Motor Distance
        fillBuffer8(&tx_buffer[12], flag);      // Flag

        OsrModule::ROBOCLAW_CMD cmd = (motor == OsrModule::MOTOR_SELECT::MOTOR1) ? OsrModule::ROBOCLAW_CMD::M1SPEEDACCELDIST : OsrModule::ROBOCLAW_CMD::M2SPEEDACCELDIST;
        this->write(m_addr, cmd, tx_buffer, 13);
    }

    void Roboclaw::setAccelVelocityDistanceM1M2(U32 accel, I32 velocity, U32 distance)
    {
        U8 flag = 1;
        
        fillBuffer32(&tx_buffer[0], accel);     // Acceleration
        fillBuffer32(&tx_buffer[4], velocity);  // Motor 1 Velocity 
        fillBuffer32(&tx_buffer[8], distance);  // Motor 1 Distance
        fillBuffer32(&tx_buffer[12], velocity); // Motor 2 Velocity
        fillBuffer32(&tx_buffer[16], distance); // Motor 2 Distance
        fillBuffer8(&tx_buffer[20], flag);      // Flag

        this->write(m_addr, OsrModule::ROBOCLAW_CMD::MIXEDSPEEDACCELDIST, tx_buffer, 21);
    }

    void Roboclaw::setVelocityAccelDeccelPosition(OsrModule::MOTOR_SELECT motor, U32 accel, U32 velocity, U32 deccel, U32 position)
    {
        U8 flag = 1;

        fillBuffer32(&tx_buffer[0], accel);     // Acceleration
        fillBuffer32(&tx_buffer[4], velocity);  // Motor velocity
        fillBuffer32(&tx_buffer[8], deccel);    // Motor deceleration
        fillBuffer32(&tx_buffer[12], position); // Motor position
        fillBuffer8(&tx_buffer[16], flag);      // Flag

        OsrModule::ROBOCLAW_CMD cmd = (motor == OsrModule::MOTOR_SELECT::MOTOR1) ? OsrModule::ROBOCLAW_CMD::M1SPEEDACCELDECCELPOS : OsrModule::ROBOCLAW_CMD::M2SPEEDACCELDECCELPOS;
        this->write(m_addr, cmd, tx_buffer, 17);
    }

    void Roboclaw::setVelocityAccelDeccelPositionM1M2(U32 accel, U32 velocity, U32 deccel, U32 position)
    {
        U8 flag = 1;

        fillBuffer32(&tx_buffer[0], accel);     // Motor 1 Acceleration
        fillBuffer32(&tx_buffer[4], velocity);  // Motor 1 velocity
        fillBuffer32(&tx_buffer[8], deccel);    // Motor 1 deceleration
        fillBuffer32(&tx_buffer[12], position); // Motor 1 position
        fillBuffer32(&tx_buffer[16], accel);    // Motor 2 Acceleration
        fillBuffer32(&tx_buffer[20], velocity); // Motor 2 velocity
        fillBuffer32(&tx_buffer[24], deccel);   // Motor 2 deceleration
        fillBuffer32(&tx_buffer[28], position); // Motor 2 position
        fillBuffer8(&tx_buffer[32], flag);      // Flag

        this->write(m_addr, OsrModule::ROBOCLAW_CMD::MIXEDSPEEDACCELDECCELPOS, tx_buffer, 33);
    }

    I32 Roboclaw::velocity2qpps(F32 velocity, OsrModule::MOTOR_SELECT motor)
    {
        F32 gear_ratio = (motor == OsrModule::MOTOR_SELECT::MOTOR1) ? m1_config.gear_ratio : m2_config.gear_ratio;
        F32 ticks_per_rev = (motor == OsrModule::MOTOR_SELECT::MOTOR1) ? m1_config.ticks_per_rev : m2_config.ticks_per_rev;
        return velocity * gear_ratio * ticks_per_rev / (2 * 3.1415926535897);
    }

    void Roboclaw::getEncoderValues()
    {
        if(waitRecv || (this->tlm_state != TLM_STATE_MACHINE::ENCODER))
            return;

        this->curr_cmd = OsrModule::ROBOCLAW_CMD::GETENCODERS;
        this->write(m_addr, curr_cmd, nullptr, 0);
        this->waitRecv = true;
    }

    void Roboclaw::getSpeedValues()
    {
        if(waitRecv || (this->tlm_state != TLM_STATE_MACHINE::SPEED))
            return;

        this->curr_cmd = OsrModule::ROBOCLAW_CMD::GETISPEEDS;
        this->write(m_addr, curr_cmd, nullptr, 0);
        this->waitRecv = true;
    }

    void Roboclaw::getPositionPIDs()
    {
        if(waitRecv || ((this->tlm_state != TLM_STATE_MACHINE::POS_PID1) && (this->tlm_state != TLM_STATE_MACHINE::POS_PID2)))
            return;

        if(this->tlm_state == TLM_STATE_MACHINE::POS_PID1)
        {
            this->curr_cmd = OsrModule::ROBOCLAW_CMD::READM1POSPID;
        }
        else
        {
            this->curr_cmd = OsrModule::ROBOCLAW_CMD::READM2POSPID;
        }

        this->write(m_addr, curr_cmd, nullptr, 0);
        this->waitRecv = true;
    }

    void Roboclaw::switchTlmState()
    {
        switch(this->tlm_state)
        {
            case TLM_STATE_MACHINE::ENCODER:
                this->tlm_state = TLM_STATE_MACHINE::SPEED;
                break;
            case TLM_STATE_MACHINE::SPEED:
                this->tlm_state = TLM_STATE_MACHINE::POS_PID1;
                break;
            case TLM_STATE_MACHINE::POS_PID1:
                this->tlm_state = TLM_STATE_MACHINE::POS_PID2;
                break;
            case TLM_STATE_MACHINE::POS_PID2:
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
