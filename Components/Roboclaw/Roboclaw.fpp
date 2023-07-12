module Components {

    array MotorTlmData = [2] I32

    @ Roboclaw Component
    passive component Roboclaw {

        @ Port for receiving motor commands
        sync input port motorControlIn: RoboclawModule.motorControl

        @ Continuously move in a specified direction at a given speed
        sync command MOVE_CONTINUOUS(direction: RoboclawModule.MOVE_DIRECTION, speed_percentage: U8)

        @ Move in a specified direction with a given speed and distance
        sync command MOVE_DISTANCE(direction: RoboclawModule.MOVE_DIRECTION, speed_percentage: U8, distance: U32)

        @ Continuously move in a specified direction with a given speed and acceleration
        sync command MOVE_ACCELERATED_CONTINUOUS(direction: RoboclawModule.MOVE_DIRECTION, acceleration: U32, speed_percentage: U8)

        @ Move in a specified direction with a given speed, acceleration, and distance
        sync command MOVE_ACCELERATED_DISTANCE(direction: RoboclawModule.MOVE_DIRECTION, acceleration: U32, speed_percentage: U8, distance: U32)

        @ Stop all motors
        sync command STOP

        @ Reset encoder telemetry values
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