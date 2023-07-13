// ======================================================================
// \title  OSR.hpp
// \author ethanchee
// \brief  hpp file for OSR component implementation class
// ======================================================================

#ifndef OSR_HPP
#define OSR_HPP

#include "Components/OSR/OSRComponentAc.hpp"

namespace OsrModule {

  class OSR :
    public OSRComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object OSR
      //!
      OSR(
          const char *const compName /*!< The component name*/
      );

      //! Destroy object OSR
      //!
      ~OSR();

      PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations
      // ----------------------------------------------------------------------

      //! Implementation for TODO command handler
      //! 
      void TODO_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );


    };

} // end namespace OsrModule

#endif
