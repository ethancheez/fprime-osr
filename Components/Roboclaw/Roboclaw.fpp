module Components {

    enum ROBOCLAW_MOVE_DIRECTION {
        FORWARD,
        BACKWARD,
        STOP
    }

    array MotorTlmData = [2] I32

    @ Roboclaw Component
    passive component Roboclaw {

        @ Command to move the motors attached to the Roboclaw forward or backward, given a speed percentage from 0 to 100
        sync command MOVE_DIRECTION(direction: ROBOCLAW_MOVE_DIRECTION, speed_percentage: U8)

        @ Command to reset encoder telemetry values
        sync command RESET_ENCODERS

        @ Data coming in
        sync input port comDataIn: Drv.ByteStreamRecv

        @ Data passing back out
        output port comDataOut: Drv.ByteStreamSend

        @ Allows for deallocation of buffers
        output port deallocate: Fw.BufferSend

        @ Allows for allocation of buffers
        output port allocate: Fw.BufferGet

        @ Port receiving calls from the rate group
        sync input port run: Svc.Sched

        ###############################################################################
        # Telemetry
        ###############################################################################

        telemetry EncoderValues: MotorTlmData

        telemetry SpeedValues: MotorTlmData

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