//
//  EcLinkLayerI8254.hpp
//  am2b
//
//  Created by Felix Sygulla on 2015-08-05.
//  Copyright 2015 Chair of Applied Mechanics, TUM
//  https://www.amm.mw.tum.de/
//

#ifndef ECLINKLAYERI8254_HPP_74676772
#define ECLINKLAYERI8254_HPP_74676772

#define HWL_EC_ETHERNET_CONTROLLER_ID   2     // network port

#include<EcLink.h>

#include <iface_prio.hpp>
#include "BusException.hpp"

namespace ec {

  /*! Provides EcLinkLayerPolicy for Intel I8254x network controller family */
  class EcLinkLayerI8254 {

  public:
  
    //! Constructor, initialize memory and interface
    EcLinkLayerI8254() {
    
      // allocate memory for the link adapter:
      m_paramsAdapter = (EC_T_LINK_PARMS_I8254X*) OsMalloc(sizeof(EC_T_LINK_PARMS_I8254X));
      if (m_paramsAdapter == 0) {
        // OUT OF MEMORY
        throw BusException("EC-LinkParms: Out of Memory!");
      }
    
      // init with zeros
      OsMemset(m_paramsAdapter, 0, sizeof(EC_T_LINK_PARMS_I8254X));
      OsMemset(&m_paramsAdapter->linkParms, 0, sizeof(EC_T_LINK_PARMS));
    
    }
  
    //! Destructor, deinitialize memory
    ~EcLinkLayerI8254() {
    
      // free memory
      if (m_paramsAdapter != 0) {
        OsFree(m_paramsAdapter);
      }
    
    }
  
    /*! Create and return LinkLayer Parameters for I8254X */
    EC_T_LINK_PARMS* getLinkParams() {

      m_paramsAdapter->linkParms.dwSignature = EC_LINK_PARMS_SIGNATURE_I8254X;
      m_paramsAdapter->linkParms.dwSize = sizeof(EC_T_LINK_PARMS_I8254X);
      OsStrncpy(m_paramsAdapter->linkParms.szDriverIdent, EC_LINK_PARMS_IDENT_I8254X, MAX_DRIVER_IDENT_LEN-1);
      m_paramsAdapter->linkParms.dwInstance = HWL_EC_ETHERNET_CONTROLLER_ID;
      m_paramsAdapter->linkParms.eLinkMode = EcLinkMode_POLLING;
      m_paramsAdapter->linkParms.dwIstPriority = PRIO_HWIO();

      return &m_paramsAdapter->linkParms;
    }
  
  private:
  
    EC_T_LINK_PARMS_I8254X*   m_paramsAdapter;
  
  };

}

#endif /* end of include guard: ECLINKLAYERI8254_HPP_74676772 */
