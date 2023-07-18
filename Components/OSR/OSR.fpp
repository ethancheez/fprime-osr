module OsrModule {
    @ Component for the Open Source Rover
    passive component OSR {

        output port motorControlOut1: OsrModule.motorControl
        
        output port motorControlOut2: OsrModule.motorControl
        
        output port motorControlOut3: OsrModule.motorControl

        output port motorControlOut4: OsrModule.motorControl

        output port motorControlOut5: OsrModule.motorControl

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

        ##############################################################################
        #### Uncomment the following examples to start customizing your component ####
        ##############################################################################

        # @ Example async command
        # async command COMMAND_NAME(param_name: U32)

        # @ Example telemetry counter
        # telemetry ExampleCounter: U64

        # @ Example event
        # event ExampleStateEvent(example_state: Fw.On) severity activity high id 0 format "State set to {}"

        # @ Example port: receiving calls from the rate group
        # sync input port run: Svc.Sched

        # @ Example parameter
        # param PARAMETER_NAME: U32

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

        @ Port to return the value of a parameter
        param get port prmGetOut

        @Port to set the value of a parameter
        param set port prmSetOut

    }
}