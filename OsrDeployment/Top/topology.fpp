module OsrDeployment {

  # ----------------------------------------------------------------------
  # Symbolic constants for port numbers
  # ----------------------------------------------------------------------

    enum Ports_RateGroups {
      rateGroup1
      rateGroup2
      rateGroup3
    }

  topology OsrDeployment {

    # ----------------------------------------------------------------------
    # Instances used in the topology
    # ----------------------------------------------------------------------

    instance $health
    instance blockDrv
    instance tlmSend
    instance cmdDisp
    instance cmdSeq
    instance comDriver
    instance comQueue
    instance comStub
    instance deframer
    instance eventLogger
    instance fatalAdapter
    instance fatalHandler
    instance fileDownlink
    instance fileManager
    instance fileUplink
    instance bufferManager
    instance framer
    instance linuxTime
    instance polyDb
    instance prmDb
    instance rateGroup1
    instance rateGroup2
    instance rateGroup3
    instance rateGroupDriver
    instance textLogger
    instance systemResources

    # Roboclaw
    instance osr
    instance roboclaw1
    instance roboclaw2
    instance roboclaw3
    instance roboclaw4
    instance roboclaw5
    instance roboclawCommDriver

    # ----------------------------------------------------------------------
    # Pattern graph specifiers
    # ----------------------------------------------------------------------

    command connections instance cmdDisp

    event connections instance eventLogger

    param connections instance prmDb

    telemetry connections instance tlmSend

    text event connections instance textLogger

    time connections instance linuxTime

    health connections instance $health

    # ----------------------------------------------------------------------
    # Direct graph specifiers
    # ----------------------------------------------------------------------

    connections Downlink {

      eventLogger.PktSend -> comQueue.comQueueIn[0]
      tlmSend.PktSend -> comQueue.comQueueIn[1]
      fileDownlink.bufferSendOut -> comQueue.buffQueueIn[0]

      comQueue.comQueueSend -> framer.comIn
      comQueue.buffQueueSend -> framer.bufferIn

      framer.framedAllocate -> bufferManager.bufferGetCallee
      framer.framedOut -> comStub.comDataIn
      framer.bufferDeallocate -> fileDownlink.bufferReturn

      comDriver.deallocate -> bufferManager.bufferSendIn
      comDriver.ready -> comStub.drvConnected

      comStub.comStatus -> framer.comStatusIn
      framer.comStatusOut -> comQueue.comStatusIn
      comStub.drvDataOut -> comDriver.send

    }

    connections FaultProtection {
      eventLogger.FatalAnnounce -> fatalHandler.FatalReceive
    }

    connections RateGroups {
      # Block driver
      blockDrv.CycleOut -> rateGroupDriver.CycleIn

      # Rate group 1
      rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup1] -> rateGroup1.CycleIn
      rateGroup1.RateGroupMemberOut[0] -> tlmSend.Run
      rateGroup1.RateGroupMemberOut[1] -> fileDownlink.Run
      rateGroup1.RateGroupMemberOut[2] -> systemResources.run
      rateGroup1.RateGroupMemberOut[3] -> roboclaw1.run
      rateGroup1.RateGroupMemberOut[4] -> roboclaw2.run
      rateGroup1.RateGroupMemberOut[5] -> roboclaw3.run
      rateGroup1.RateGroupMemberOut[6] -> roboclaw4.run
      rateGroup1.RateGroupMemberOut[7] -> roboclaw5.run

      # Rate group 2
      rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup2] -> rateGroup2.CycleIn
      rateGroup2.RateGroupMemberOut[0] -> cmdSeq.schedIn

      # Rate group 3
      rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup3] -> rateGroup3.CycleIn
      rateGroup3.RateGroupMemberOut[0] -> $health.Run
      rateGroup3.RateGroupMemberOut[1] -> blockDrv.Sched
      rateGroup3.RateGroupMemberOut[2] -> bufferManager.schedIn
    }

    connections Sequencer {
      cmdSeq.comCmdOut -> cmdDisp.seqCmdBuff
      cmdDisp.seqCmdStatus -> cmdSeq.cmdResponseIn
    }

    connections Uplink {

      comDriver.allocate -> bufferManager.bufferGetCallee
      comDriver.$recv -> comStub.drvDataIn
      comStub.comDataOut -> deframer.framedIn

      deframer.framedDeallocate -> bufferManager.bufferSendIn
      deframer.comOut -> cmdDisp.seqCmdBuff

      cmdDisp.seqCmdStatus -> deframer.cmdResponseIn

      deframer.bufferAllocate -> bufferManager.bufferGetCallee
      deframer.bufferOut -> fileUplink.bufferSendIn
      deframer.bufferDeallocate -> bufferManager.bufferSendIn
      fileUplink.bufferSendOut -> bufferManager.bufferSendIn
    }

    connections Roboclaw {
      roboclawCommDriver.allocate -> bufferManager.bufferGetCallee
      roboclawCommDriver.$recv -> roboclaw1.comDataIn     # TODO: Change to serial hub
      roboclaw1.deallocate -> bufferManager.bufferSendIn  # TODO: Change to serial hub

      roboclaw1.allocate -> bufferManager.bufferGetCallee
      roboclaw1.comDataOut -> roboclawCommDriver.send
      roboclaw2.allocate -> bufferManager.bufferGetCallee
      roboclaw2.comDataOut -> roboclawCommDriver.send
      roboclaw3.allocate -> bufferManager.bufferGetCallee
      roboclaw3.comDataOut -> roboclawCommDriver.send
      roboclaw4.allocate -> bufferManager.bufferGetCallee
      roboclaw4.comDataOut -> roboclawCommDriver.send
      roboclaw5.allocate -> bufferManager.bufferGetCallee
      roboclaw5.comDataOut -> roboclawCommDriver.send

      roboclawCommDriver.deallocate -> bufferManager.bufferSendIn

      osr.motorControlOut1 -> roboclaw1.motorControlIn
      osr.motorControlOut2 -> roboclaw2.motorControlIn
      osr.motorControlOut3 -> roboclaw3.motorControlIn
      osr.motorControlOut4 -> roboclaw4.motorControlIn
      osr.motorControlOut5 -> roboclaw5.motorControlIn

      osr.getPolyDbVal -> polyDb.getValue
      roboclaw1.setPolyDbVal -> polyDb.setValue
      roboclaw2.setPolyDbVal -> polyDb.setValue
      roboclaw3.setPolyDbVal -> polyDb.setValue
      roboclaw4.setPolyDbVal -> polyDb.setValue
      roboclaw5.setPolyDbVal -> polyDb.setValue
    }

  }

}
