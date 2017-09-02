//
//  AcEcFixedSlaveInstanceMapper.hpp
//  am2b
//
//  Created by Felix Sygulla on 2015-08-05.
//  Copyright 2015 Chair of Applied Mechanics, TUM
//  https://www.amm.mw.tum.de/
//

#ifndef ACECFIXEDSLAVEINSTANCEMAPPER_HPP_6B6D3FB3
#define ACECFIXEDSLAVEINSTANCEMAPPER_HPP_6B6D3FB3

#include <string>
#include <AtEthercat.h>

namespace ec {

  /*! Implements SlaveInstanceMapperPolicy with hard-coded assignment between ethercat slave and device instance for the acontis master */
  class AcEcFixedSlaveInstanceMapper {
  
  public:
  
    //! Sets the slave name and station address for this BusSlave instance
    void attachSlave(const std::string& name, EC_T_WORD stationAddress) {
      m_slaveName = name;
      m_stationAddress = stationAddress;
  
    }
  
    /*! Assembles a var name with the slave name to a fully qualified identifier */
    std::string getFullIdentifier(const std::string& varName) {
      std::string str;
    
      str.append(m_slaveName);
      str.append(".");
      str.append(varName);
      return str;
    }
  
    /*! Returns the station address */
    EC_T_WORD getStationAddress() {
      return m_stationAddress;
    }
  
    /*! Returns the slave id */
    EC_T_DWORD getSlaveID() {
    
      // try to update slave ID if invalid
      if (m_slaveID == INVALID_SLAVE_ID) {
        m_slaveID = ecatGetSlaveId(this->getStationAddress());
      }
    
      return m_slaveID;
    }
  
    /*! Returns the slave name */
    std::string getName() {
      return m_slaveName;
    }
  
  protected:
  
    /* Part of the PolicyInterface */
  
    //! Slave name
    std::string   m_slaveName;
  
    //! Station address
    EC_T_WORD     m_stationAddress = 0;
  
    //! AcEc Slave ID
    EC_T_DWORD    m_slaveID = INVALID_SLAVE_ID;

  };

}

#endif /* end of include guard: ACECFIXEDSLAVEINSTANCEMAPPER_HPP_6B6D3FB3 */