// ======================================================================
// \title  OSR.cpp
// \author ethanchee
// \brief  cpp file for OSR component implementation class
// ======================================================================


#include <Components/OSR/OSR.hpp>
#include <FpConfig.hpp>

namespace OsrModule {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  OSR ::
    OSR(
        const char *const compName
    ) : OSRComponentBase(compName)
  {

  }

  OSR ::
    ~OSR()
  {

  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void OSR ::
    TODO_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    // TODO
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

} // end namespace OsrModule
