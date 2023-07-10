module Components {

    enum ROBOCLAW_GET_DATA_CMDS {
        GET_M1_ENCODER = 16,
        GET_M2_ENCODER = 17,
        GET_M1_SPEED = 18,
        GET_M2_SPEED = 19,
        GET_VERSION = 21,
        GET_PWMS = 48,
        GET_CURRENTS = 49,
        GET_STATUS = 90,
        GET_CONFIG = 99,
        GET_M1_CURRENT_LIMITS = 135,
        GET_M2_CURRENT_LIMITS = 136
    };

    enum ROBOCLAW_SET_SINGLE_MOTOR {
        M1_FORWARD = 0,
        M1_BACKWARD = 1
    }

    enum ROBOCLAW_SET_MOTORS_CMDS {
        SET_DUTY = 34,
        SET_SPEED = 37
    }

    @ Roboclaw Component
    passive component Roboclaw {

        sync command GET_DATA(addr: U8, cmd: ROBOCLAW_GET_DATA_CMDS)

        sync command SET_MOTORS(addr: U8, cmd: ROBOCLAW_SET_MOTORS_CMDS, motor1: U32, motor2: U32)

        sync command SET_SINGLE_MOTOR(addr: U8, cmd: ROBOCLAW_SET_SINGLE_MOTOR, val: U32)

        @ Data coming in
        sync input port comDataIn: Drv.ByteStreamRecv

        @ Data passing back out
        output port comDataOut: Drv.ByteStreamSend

        @ Allows for deallocation of buffers
        output port deallocate: Fw.BufferSend

        @ Allows for allocation of buffers
        output port allocate: Fw.BufferGet

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        @ Port for sending command registrations
        command reg port cmdRegOut

        @ Port for receiving commands
        command recv port cmdIn

        @ Port for sending command responses
        command resp port cmdResponseOut

        @ Port for sending textual representation of events
        text event port logTextOut

        @ Port for sending events to downlink
        event port logOut

        @ Port for sending telemetry channels to downlink
        telemetry port tlmOut

    }
}