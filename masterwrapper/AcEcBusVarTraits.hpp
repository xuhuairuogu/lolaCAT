//
//  AcEcBusVarTraits.hpp
//  am2b
//
//  Contains all information about how to link the generalized BusVar implementation
//  with the Acontis EtherCAT Master
//
//  Created by Felix Sygulla on 2015-08-19.
//  Copyright 2015 Chair of Applied Mechanics, TUM
//  https://www.amm.mw.tum.de/
//

#ifndef ACECBUSVARTRAITS_HPP_555DA5B2
#define ACECBUSVARTRAITS_HPP_555DA5B2

#include <typeinfo>
#include <EcType.h>
#include "BusException.hpp"

namespace ec {

  /*! AcEc BusVar injection class */
  class BusVarTraits {
  
  public:
    
    /*! for PDO only: offset to the data area*/
    int               m_offset;
    
    /*! for SDO only: Pointer to the mailbox object */
    EC_T_MBXTFER*     m_tferObj = 0;
    
    /*! for SDO only: Slave id */
    EC_T_DWORD        m_slaveId;
    
    /*! for SDO only: object id*/
    EC_T_WORD         m_objId;
    
    /*! for SDO only: subindex */
    EC_T_BYTE         m_subIdx;
  };
  
}

/*! BusVarType depends on BusVarTraits and the following code depends on
   BusVarType. This resolves the double dependency.
 */
#include "BusVarType.hpp"

namespace ec {
  
  /*! Return true if Bus Type matches given EcType */
  const bool isOfBusType(const BusVarType* var, EC_T_WORD ecatType) {

    /* Arrays are defined as DEFTYPE_NULL */
    if (var->isArray()) {
      return (ecatType == DEFTYPE_NULL);
    }
  
    /* Standard variables */
    if (*(var->getTypeId()) == typeid(bool)) {
    
      return (ecatType == DEFTYPE_BOOLEAN);
    
    } else if (*(var->getTypeId()) == typeid(int8_t)) {
    
      return (ecatType == DEFTYPE_INTEGER8);
    
    } else if (*(var->getTypeId()) == typeid(uint8_t)) {

      return (ecatType == DEFTYPE_UNSIGNED8);
    
    } else if (*(var->getTypeId()) == typeid(int16_t)) {
    
      return (ecatType == DEFTYPE_INTEGER16);
    
    } else if (*(var->getTypeId()) == typeid(uint16_t)) {
    
      return (ecatType == DEFTYPE_UNSIGNED16);
    
    } else if (*(var->getTypeId()) == typeid(int32_t)) {
    
      return (ecatType == DEFTYPE_INTEGER32);
    
    } else if (*(var->getTypeId()) == typeid(uint32_t)) {
    
      return (ecatType == DEFTYPE_UNSIGNED32);
    
    } else if (*(var->getTypeId()) == typeid(int64_t)) {
    
      return (ecatType == DEFTYPE_INTEGER64);
    
    } else if (*(var->getTypeId()) == typeid(uint64_t)) {
    
      return (ecatType == DEFTYPE_UNSIGNED64);
    
    } else if (*(var->getTypeId()) == typeid(float)) {
    
      return (ecatType == DEFTYPE_REAL32);
    
    } else if (*(var->getTypeId()) == typeid(double)) {
    
      return (ecatType == DEFTYPE_REAL64);
    
    } else {
    
      throw BusException("Tried to match unknown BusVarType in AcEcBusVarTraits!\n");
    
    }
  
  }

}

#endif /* end of include guard: ACECBUSVARTYPE_HPP_555DA5B2 */
