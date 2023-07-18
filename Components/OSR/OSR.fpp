module OsrModule {
    @ Component for the Open Source Rover
    passive component OSR {

        output port motorControlOut1: OsrModule.motorControl
        
        output port motorControlOut2: OsrModule.motorControl
        
        output port motorControlOut3: OsrModule.motorControl

        output port motorControlOut4: OsrModule.motorControl

        output port motorControlOut5: OsrModule.motorControl

        output port getPolyDbVal: Svc.Poly

        ###############################################################################
        # Commands
        ###############################################################################

        @ Respond to an incoming Twist command in one of two ways depending on the mode (intuitive)
        sync command CMD_CB(
            twist_linear_x: F32,
            twist_linear_y: F32,
            twist_linear_z: F32,
            twist_angular_x: F32,
            twist_angular_y: F32,
            twist_angular_z: F32,
            intuitive: bool
        )

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