// ======================================================================
// \title  OSR.hpp
// \author ethanchee
// \brief  hpp file for OSR component implementation class
// ======================================================================

#ifndef OSR_HPP
#define OSR_HPP

#include "Components/OSR/OSRComponentAc.hpp"

namespace Components {

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


    };

} // end namespace Components

#endif
