//
//  AcEcMasterNotifications_impl.hpp
//  am2b
//
//  Created by Felix Sygulla on 2015-08-05.
//  Copyright 2015 Chair of Applied Mechanics, TUM
//  https://www.amm.mw.tum.de/
//

/*
 Implementation to handle AcEcMaster notifications. This is directly included in the AcEcMaster template class
 and defines the notify(...) method called directly by the internal master callback.
 
*/


/*! notify member function called by the ethercat master after bus state change or bus error */
void notify(EC_T_DWORD dwCode, EC_T_NOTIFYPARMS* pParms) {
  
  /* Cast the pParms data to different representations */
  
  //! General notification struct
  EC_T_NOTIFICATION_DESC*         pDesc             = (EC_T_NOTIFICATION_DESC*) pParms->pbyInBuf;
 
  //! Error notification struct
  EC_T_ERROR_NOTIFICATION_DESC*   pErrorDesc        = (EC_T_ERROR_NOTIFICATION_DESC*) pParms->pbyInBuf;
  
  //! Mailbox transfer notification struct
  EC_T_MBXTFER* pMboxTransfer = (EC_T_MBXTFER*) pParms->pbyInBuf;
  
  
  switch(dwCode) {

    //! Master state change
    case EC_NOTIFY_STATECHANGED:
    {
      EC_T_STATECHANGE* pStateChangeParms = (EC_T_STATECHANGE*) pParms->pbyInBuf;
      pmsgMaster("Bus state change from %s to %s\n", ecatStateToStr(pStateChangeParms->oldState), ecatStateToStr(pStateChangeParms->newState));
    }
    break;
    
    //! Ethernet Cable connected
    case EC_NOTIFY_ETH_LINK_CONNECTED:
      pdbgMaster("Ethernet cable connected.\n");
    break;
    
    //! scan bus status
    case EC_NOTIFY_SB_STATUS:
    
      if (pDesc->desc.ScanBusNtfyDesc.dwResultCode == EC_E_NOERROR) {
        pdbgMaster("Bus scan found %d slaves\n", pDesc->desc.ScanBusNtfyDesc.dwSlaveCount);
      } else {
        LOG_EC_WARNING("Error during bus scan!", pDesc->desc.ScanBusNtfyDesc.dwResultCode);
      }
    
    break;
    
    //! Distributed Clocks slave init status
    case EC_NOTIFY_DC_STATUS:
    {
      /* Initialization of DC Instance finished when this notification is called */
      EC_T_DWORD dwStatus  = EC_GETDWORD((pParms->pbyInBuf));

      if (EC_E_NOERROR != dwStatus)
      {
        LOG_EC_ERROR("DC Initialization failed!", dwStatus);
      }
    
    }
    break;
    
    //! Distributed Clocks slave sync status
    case EC_NOTIFY_DC_SLV_SYNC:

      if (pDesc->desc.SyncNtfyDesc.IsInSync) {
        // FIXME: achName and stationaddress may not be defined in this case (manual unclear)
        pdbgMaster("DC slave status change for %s (%d): In Sync\n", pDesc->desc.SyncNtfyDesc.SlaveProp.achName, pDesc->desc.SyncNtfyDesc.SlaveProp.wStationAddress);
        
      } else {
        
        pwrnMaster("DC slave status change for %s (%d): Out Of Sync\n", pDesc->desc.SyncNtfyDesc.SlaveProp.achName, pDesc->desc.SyncNtfyDesc.SlaveProp.wStationAddress);
        
      }
    
    break;

    //! Distributed Clocks master sync status
    case EC_NOTIFY_DCM_SYNC:
    
      // check status of the clock
      if (pDesc->desc.DcmInSyncDesc.IsInSync) {
        pdbgMaster("DCM master status change: In Sync, CtlErrorAvg: %d\n", pDesc->desc.DcmInSyncDesc.nCtlErrorNsecAvg);

      } else {
        pwrnMaster("DCM master status change: Out Of Sync, CtlErrorCur: %d\n", pDesc->desc.DcmInSyncDesc.nCtlErrorNsecCur);
      }
    
    break;
    
    //! Slave state change
    case EC_NOTIFY_SLAVE_STATECHANGED:
      
      pdbgMaster("Slave state change for %s (%d) to %s.\n", 
            pDesc->desc.SlaveStateChangedDesc.SlaveProp.achName, 
            pDesc->desc.SlaveStateChangedDesc.SlaveProp.wStationAddress,
            ecatStateToStr(pDesc->desc.SlaveStateChangedDesc.newState));
            
    break;
    
    //! Collects state change information for multiple slaves
    //! Disabled by default
    case EC_NOTIFY_SLAVES_STATECHANGED:
      perrMaster("EC_NOTIFY_SLAVES_STATECHANGED. Not implemented\n");
    break;
    
    //! Response to raw EtherCAT command
    case EC_NOTIFY_RAWCMD_DONE:
      perrMaster("EC_NOTIFY_RAWMD_DONE. Not implemented.\n");
    break;
    
    //! Slave presence status changed
    case EC_NOTIFY_SLAVE_PRESENCE:
    
      if (pDesc->desc.SlavePresenceDesc.bPresent) {
        pmsgMaster("Slave with address %d added to the bus!\n", pDesc->desc.SlavePresenceDesc.wStationAddress);
        
      } else {
        EC_FAULT; // Fault reaction
        perrMaster("Slave with address %d removed from the bus!\n", pDesc->desc.SlavePresenceDesc.wStationAddress);
      }
    
    break;
    
    //! Collects slave presence status
    //! Disabled by default
    case EC_NOTIFY_SLAVES_PRESENCE:
      perrMaster("EC_NOTIFY_SLAVES_PRESENCE. Not implemented\n");
    break;

    //! Mailbox transfer completion
    case EC_NOTIFY_MBOXRCV:
    {
        
      EC_T_MBXTFER* pmbox = (EC_T_MBXTFER*) pParms->pbyInBuf;

      switch (pmbox->eMbxTferType) {
        
        /* Asynchronous SDOs */
        case eMbxTferType_COE_SDO_DOWNLOAD:
        case eMbxTferType_COE_SDO_UPLOAD:
        {
          
          bool bFoundTferObj = false;
          
          // Check which registered variable is linked
          for (std::vector<BusVarType*>::iterator it = m_variablesSDO.begin() ; it != m_variablesSDO.end(); ++it) {
    
            // check for pointer equality
            if ((*it)->m_tferObj == pmbox) {
              
              bFoundTferObj = true;
      
              // scoped lock
              std::lock_guard<std::timed_mutex> lock((*it)->getMutex());
      
              // update transfer in progress flag
              (*it)->m_SDOTransferInProgress = false;
              
              
              // Error during transfer?
              if ( (*it)->m_tferObj->dwErrorCode != EC_E_NOERROR) {
                
                (*it)->m_SDOTransferFailed = true;
                
                EC_T_SLAVE_PROP slaveProp;
                ecatGetSlaveProp((*it)->m_slaveId, &slaveProp);
                
                if ((*it)->m_tferObj->eMbxTferType == eMbxTferType_COE_SDO_DOWNLOAD) {
                  perrMaster("Error during asynchronous SDO Download (%d) to %s, objIndex=0x%x, subIdx=0x%x: %s\n", pmbox->dwTferId, slaveProp.achName, (*it)->m_objId, (*it)->m_subIdx, ecatGetText((*it)->m_tferObj->dwErrorCode));
                } else if ((*it)->m_tferObj->eMbxTferType == eMbxTferType_COE_SDO_UPLOAD) {
                  perrMaster("Error during asynchronous SDO Upload (%d) from %s, objIndex=0x%x, subIdx=0x%x: %s\n", pmbox->dwTferId, slaveProp.achName, (*it)->m_objId, (*it)->m_subIdx, ecatGetText((*it)->m_tferObj->dwErrorCode));
                } else {
                  pwrnMaster("Error during asynchronous SDO transfer (%d) from/to %s with type=%d, objIndex=0x%x, subIdx=0x%x: %s\n", pmbox->dwTferId, slaveProp.achName, (*it)->m_tferObj->eMbxTferType, (*it)->m_objId, (*it)->m_subIdx, ecatGetText((*it)->m_tferObj->dwErrorCode));
                }
                
              } else if ((*it)->m_tferObj->eTferStatus == eMbxTferStatus_TferDone) {
                
                // if the transfer was successful, update the corresponding flag
                (*it)->m_SDOTransferDone = true;

#ifdef HWL_EC_VERBOSE
                EC_T_SLAVE_PROP slaveProp;
                ecatGetSlaveProp((*it)->m_slaveId, &slaveProp);
              
                if ((*it)->m_tferObj->eMbxTferType == eMbxTferType_COE_SDO_DOWNLOAD) {
                  pdbgMaster("Completed asynchronous SDO Download (%d) to %s, objIndex=0x%x, subIdx=0x%x\n", pmbox->dwTferId, slaveProp.achName, (*it)->m_objId, (*it)->m_subIdx);
                } else if ((*it)->m_tferObj->eMbxTferType == eMbxTferType_COE_SDO_UPLOAD) {
                  pdbgMaster("Completed asynchronous SDO Upload (%d) from %s, objIndex=0x%x, subIdx=0x%x\n", pmbox->dwTferId, slaveProp.achName, (*it)->m_objId, (*it)->m_subIdx);
                } else {
                  pdbgMaster("Completed asynchronous SDO transfer (%d) from/to %s, type=%d, objIndex=0x%x, subIdx=0x%x\n", pmbox->dwTferId, slaveProp.achName, (*it)->m_tferObj->eMbxTferType, (*it)->m_objId, (*it)->m_subIdx);
                }
#endif
                
              }

              break;  // for
      
            }
          }
          
          // check if the mailbox object was found in the linked sdo objects
          if (!bFoundTferObj) {
            //FIXME: Do synchronous SDOs trigger this output? If not -> omit detection or output an error
            pdbgMaster("Mailbox transfer completion for unknown SDO transfer object (%d).\n", pmbox->dwTferId);
          }
          
          
        }
        break;  // case
    
        //! CoE Emergency object
        case eMbxTferType_COE_EMERGENCY:
        {
          EC_FAULT; // Fault reaction
          
          if (pmbox->eTferStatus != eMbxTferStatus_TferDone) {
            perrMaster("Error during transmission of CoE Emergency Object!\n");
            break;
          }
          
          // check if a slave registered for this object
          for (std::vector<BusVarType*>::iterator it = m_variablesSDO.begin() ; it != m_variablesSDO.end(); ++it) {
            
            if ((*it)->m_offset == BUSVAR_COE_EMERGENCY && (*it)->m_objId == pmbox->MbxData.CoE_Emergency.wStationAddress) {
              // This is the corresponding bus var
              
              // scoped lock
              std::lock_guard<std::timed_mutex> lock((*it)->getMutex());
              
              (*it)->m_SDOTransferDone = true;
              
              // copy data
              memcpy((char*)(*it)->getPointer(), (void*) &pmbox->MbxData.CoE_Emergency.wErrorCode, sizeof(EC_T_WORD));
              memcpy((char*)(*it)->getPointer()+sizeof(EC_T_WORD), (void*) &pmbox->MbxData.CoE_Emergency.byErrorRegister, sizeof(EC_T_BYTE));
              memcpy((char*)(*it)->getPointer()+sizeof(EC_T_WORD)+sizeof(EC_T_BYTE), (void*) pmbox->MbxData.CoE_Emergency.abyData, 5*sizeof(EC_T_BYTE));
              return;
            }
            
          }
          
          perrMaster("Received unknown CoE Emergency Object for station addr %d!\n", 
                     pmbox->MbxData.CoE_Emergency.wStationAddress);
        }
        break;
        
#ifdef HWL_EC_VERBOSE
        //! Other MBOX object
        default:

          pdbgMaster("Mailbox Transfer Completion (MBOXRCV), status = %d, tferId=%d, type=%d\n", pMboxTransfer->eTferStatus, pMboxTransfer->dwTferId, pMboxTransfer->eMbxTferType);
#endif
      }

    }
    break;

    //! scan bus mismatch
    case EC_NOTIFY_SB_MISMATCH:
    //! duplicate hot-connect group
    case EC_NOTIFY_SB_DUPLICATE_HC_NODE:
    {
      
      perrMaster("Scan bus configuration mismatch.\n\tParent slave address: %d\n\tUnexpected slave with address %d and serial nr %d\n\tMissing slave with address %d and serial nr %d\n", 
                  pDesc->desc.ScanBusMismatch.wPrevFixedAddress, 
                  pDesc->desc.ScanBusMismatch.wBusFixedAddress, 
                  pDesc->desc.ScanBusMismatch.dwBusSerialNo, 
                  pDesc->desc.ScanBusMismatch.wCfgFixedAddress,
                  pDesc->desc.ScanBusMismatch.dwCfgSerialNo);
      
    }
    break;
    
    //! Cyclic command working counter mismatch
    case EC_NOTIFY_CYCCMD_WKC_ERROR:
    {
      static LogRateLimiter wkclimiter(HWL_EC_MAX_MSG_PER_ERROR, HWL_EC_REDUCED_MSG_RATE);
      wkclimiter.count();
      
      if (wkclimiter.onLimit()) {
        pwrnMaster("Reached maximum number of messages for CYCCMD_WKC_ERROR. Reducing report rate...\n");
      }
      if (!wkclimiter.log()) {
        break;
      }
      
      pwrnMaster("Cyclic command working counter mismatch on addr %d by cmd 0x%02x, wkcSet=%d, wkcAct=%d\n",
                    pErrorDesc->desc.WkcErrDesc.dwAddr,
                    pErrorDesc->desc.WkcErrDesc.byCmd,
                    pErrorDesc->desc.WkcErrDesc.wWkcSet,
                    pErrorDesc->desc.WkcErrDesc.wWkcAct);
    }
    break;
    
    //! Master init command working counter mismatch
    case EC_NOTIFY_MASTER_INITCMD_WKC_ERROR:
    
      pwrnMaster("Master init command working counter mismatch on addr %d by cmd 0x%02x, wkcSet=%d, wkcAct=%d\n", 
                    pErrorDesc->desc.WkcErrDesc.dwAddr,
                    pErrorDesc->desc.WkcErrDesc.byCmd,
                    pErrorDesc->desc.WkcErrDesc.wWkcSet,
                    pErrorDesc->desc.WkcErrDesc.wWkcAct);
    
    break;
    
    //! Slave init command working counter mismatch
    case EC_NOTIFY_SLAVE_INITCMD_WKC_ERROR:
      pwrnMaster("Init command working counter mismatch for %s (%d) on addr %d by cmd 0x%02x, wkcSet=%d, wkcAct=%d\n",
                    pErrorDesc->desc.WkcErrDesc.SlaveProp.achName,
                    pErrorDesc->desc.WkcErrDesc.SlaveProp.wStationAddress,
                    pErrorDesc->desc.WkcErrDesc.dwAddr,
                    pErrorDesc->desc.WkcErrDesc.byCmd,
                    pErrorDesc->desc.WkcErrDesc.wWkcSet,
                    pErrorDesc->desc.WkcErrDesc.wWkcAct);
    break;

    //! EoE Mailbox working counter mismatch
    case EC_NOTIFY_EOE_MBXSND_WKC_ERROR:
      pwrnMaster("EoE Mailbox working counter mismatch for %s (%d), addr %d by cmd 0x%02x, wkcSet=%d, wkcAct=%d\n",
                    pErrorDesc->desc.WkcErrDesc.SlaveProp.achName,
                    pErrorDesc->desc.WkcErrDesc.SlaveProp.wStationAddress,
                    pErrorDesc->desc.WkcErrDesc.dwAddr,
                    pErrorDesc->desc.WkcErrDesc.byCmd,
                    pErrorDesc->desc.WkcErrDesc.wWkcSet,
                    pErrorDesc->desc.WkcErrDesc.wWkcAct);
    break;
  
    //! CoE Mailbox working counter mismatch
    case EC_NOTIFY_COE_MBXSND_WKC_ERROR:
      pwrnMaster("CoE Mailbox working counter mismatch for %s (%d), addr %d by cmd 0x%02x, wkcSet=%d, wkcAct=%d\n",
                    pErrorDesc->desc.WkcErrDesc.SlaveProp.achName,
                    pErrorDesc->desc.WkcErrDesc.SlaveProp.wStationAddress,
                    pErrorDesc->desc.WkcErrDesc.dwAddr,
                    pErrorDesc->desc.WkcErrDesc.byCmd,
                    pErrorDesc->desc.WkcErrDesc.wWkcSet,
                    pErrorDesc->desc.WkcErrDesc.wWkcAct);
    break;
  
    //! FoE Mailbox working counter mismatch
    case EC_NOTIFY_FOE_MBXSND_WKC_ERROR:
      pwrnMaster("FoE Mailbox working counter mismatch on addr %d by cmd 0x%02x, wkcSet=%d, wkcAct=%d\n", 
                    pErrorDesc->desc.WkcErrDesc.dwAddr,
                    pErrorDesc->desc.WkcErrDesc.byCmd,
                    pErrorDesc->desc.WkcErrDesc.wWkcSet,
                    pErrorDesc->desc.WkcErrDesc.wWkcAct);
    break;
  
    //! VoE Mailbox working counter mismatch
    case EC_NOTIFY_VOE_MBXSND_WKC_ERROR:
      pwrnMaster("VoE Mailbox working counter mismatch for %s (%d), addr %d by cmd 0x%02x, wkcSet=%d, wkcAct=%d\n",
                    pErrorDesc->desc.WkcErrDesc.SlaveProp.achName,
                    pErrorDesc->desc.WkcErrDesc.SlaveProp.wStationAddress,
                    pErrorDesc->desc.WkcErrDesc.dwAddr,
                    pErrorDesc->desc.WkcErrDesc.byCmd,
                    pErrorDesc->desc.WkcErrDesc.wWkcSet,
                    pErrorDesc->desc.WkcErrDesc.wWkcAct);
    break;

    //! Frame response error / unexpected frame received
    case EC_NOTIFY_FRAME_RESPONSE_ERROR:
    {
      static LogRateLimiter noframelimiter(HWL_EC_MAX_MSG_PER_ERROR, HWL_EC_REDUCED_MSG_RATE);
      noframelimiter.count();
      
      if (noframelimiter.onLimit()) {
        pwrnMaster("Reached maximum number of messages for FRAME_RESPONSE_ERROR. Reducing report rate...\n");
        //EC_FAULT;// Fault reaction
        //Frame response also handled in JobTask
      }
      if (!noframelimiter.log()) {  
        break;
      }
      
      // Switch for error types
      switch(pErrorDesc->desc.FrameRspErrDesc.EErrorType) {
                
        //! No Ethernet frame received (timeout, frame loss)
        case eRspErr_NO_RESPONSE:
          pwrnMaster("Missing response to %s frame!\n", pErrorDesc->desc.FrameRspErrDesc.bIsCyclicFrame ? "cyclic" : "acyclic");
        break;
        
        //! Wrong IDX value in the acyclic frame
        case eRspErr_WRONG_IDX:
          pwrnMaster("Wrong IDX value in acyclic frame, set: %d, act: %d\n", 
                      pErrorDesc->desc.FrameRspErrDesc.byEcCmdHeaderIdxSet, 
                      pErrorDesc->desc.FrameRspErrDesc.byEcCmdHeaderIdxAct);
        break;
        
        //! Unexpected frame was received
        case eRspErr_UNEXPECTED:
          pwrnMaster("Unexpected %s frame received!\n", pErrorDesc->desc.FrameRspErrDesc.bIsCyclicFrame ? "cyclic" : "acyclic");
        break;
        
        //! frame will be re-sent (timeout, frame loss)
        case eRspErr_FRAME_RETRY:
          pwrnMaster("Missing response to %s frame, retrying...\n", pErrorDesc->desc.FrameRspErrDesc.bIsCyclicFrame ? "cyclic" : "acyclic");
        break;
        
        //! all retry mechanisms fail to re-send acyclic frames
        case eRspErr_RETRY_FAIL:
          pwrnMaster("All retry mechanisms failed to re-send acyclic frame!\n");
        break;
        
      }
      
    }
    break;
    
    //! Slave init command reponse error
    case EC_NOTIFY_SLAVE_INITCMD_RESPONSE_ERROR:
    
      switch(pErrorDesc->desc.InitCmdErrDesc.EErrorType) {
      
        //! Timeout (no frame received)
        case eInitCmdErr_NO_RESPONSE:
          pwrnMaster("Init command response error (timeout) for %s (%d) during state change %s!\n",
                      pErrorDesc->desc.InitCmdErrDesc.SlaveProp.achName,
                      pErrorDesc->desc.InitCmdErrDesc.SlaveProp.wStationAddress,
                      pErrorDesc->desc.InitCmdErrDesc.achStateChangeName);
        break;
        
        case eInitCmdErr_VALIDATION_ERR:
          pwrnMaster("Init command response error (invalid response) for %s (%d) during state change %s!\n",
                      pErrorDesc->desc.InitCmdErrDesc.SlaveProp.achName,
                      pErrorDesc->desc.InitCmdErrDesc.SlaveProp.wStationAddress,
                      pErrorDesc->desc.InitCmdErrDesc.achStateChangeName);
        break;
        
        case eInitCmdErr_FAILED:
          pwrnMaster("Init command response error (cmd failed) for %s (%d) during state change %s!\n",
                      pErrorDesc->desc.InitCmdErrDesc.SlaveProp.achName,
                      pErrorDesc->desc.InitCmdErrDesc.SlaveProp.wStationAddress,
                      pErrorDesc->desc.InitCmdErrDesc.achStateChangeName);
        break;
      
      }
    
    break;

    //! Master init command response error
    case EC_NOTIFY_MASTER_INITCMD_RESPONSE_ERROR:
      switch(pErrorDesc->desc.InitCmdErrDesc.EErrorType) {
    
        //! Timeout (no frame received)
        case eInitCmdErr_NO_RESPONSE:
          pwrnMaster("Master init command response error (timeout) during state change %s!\n", pErrorDesc->desc.InitCmdErrDesc.achStateChangeName);
        break;
      
        case eInitCmdErr_VALIDATION_ERR:
          pwrnMaster("Master init command response error (invalid response) during state change %s!\n", pErrorDesc->desc.InitCmdErrDesc.achStateChangeName);
        break;
    
      }
    break;
    
    //! Mailbox slave init command response error
    case EC_NOTIFY_MBSLAVE_INITCMD_TIMEOUT:
    
      pwrnMaster("Mailbox init command timout for %s (%d) during state change %s!\n",
                  pErrorDesc->desc.InitCmdErrDesc.SlaveProp.achName,
                  pErrorDesc->desc.InitCmdErrDesc.SlaveProp.wStationAddress,
                  pErrorDesc->desc.InitCmdErrDesc.achStateChangeName);
    break;
    
    //! Not all slaves are in OPERATIONAL state
    case EC_NOTIFY_NOT_ALL_DEVICES_OPERATIONAL:
    {
      EC_FAULT;// Fault reaction

      static LogRateLimiter oplimiter(HWL_EC_MAX_MSG_PER_ERROR, HWL_EC_REDUCED_MSG_RATE);
      
      if (m_allDevsInOperationalState) {
        // if this is the first time this error occurs,
        // reset the counter
        oplimiter.reset();
      }
      
      oplimiter.count();
      m_allDevsInOperationalState = false;
      
      if (oplimiter.onLimit()) {
        pwrnMaster("Reached maximum number of messages for NOT_ALL_DEVICES_OPERATIONAL. Reducing report rate...\n");
      }
      if (!oplimiter.log()) {
        break;
      }
      
      perrMaster("Not all slaves are in OPERATIONAL state!\n");
      
    }  
    break;

    //! Ethernet cable disconnected
    case EC_NOTIFY_ETH_LINK_NOT_CONNECTED:
    {
      EC_FAULT;// Fault reaction
      
      static LogRateLimiter ethlimiter(HWL_EC_MAX_MSG_PER_ERROR, HWL_EC_REDUCED_MSG_RATE);
      ethlimiter.count();
      
      if (ethlimiter.onLimit()) {
        pwrnMaster("Reached maximum number of messages for ETH_LINK_NOT_CONNECTED. Reducing report rate...\n");
      }
      if (!ethlimiter.log()) {
        break;
      }
      
      perrMaster("Ethernet link cable disconnected!\n");
      
    }
    break;

    //! slave error status
    case EC_NOTIFY_STATUS_SLAVE_ERROR:
    {
      // will be executed before each EC_NOTIFY_SLAVE_ERROR_STATUS_INFO, but without detailed information on the error status
      EC_FAULT;// Fault reaction
      
      static LogRateLimiter slavelimiter(HWL_EC_MAX_MSG_PER_ERROR, 5);
      slavelimiter.count();
      
      if (slavelimiter.onLimit()) {
        pwrnMaster("Reached maximum number of messages for STATUS_SLAVE_ERROR. Reducing report rate...\n");
      }
      if(!slavelimiter.log()) {
        break;
      }
      
      perrMaster("A slave reported an error status.\n");
    }
    break;
    
  
    case EC_NOTIFY_SLAVE_ERROR_STATUS_INFO:
      pwrnMaster("%s (%d) reported an error with status %d, status code %d\n",
                  pErrorDesc->desc.SlaveErrInfoDesc.SlaveProp.achName,
                  pErrorDesc->desc.SlaveErrInfoDesc.SlaveProp.wStationAddress,
                  pErrorDesc->desc.SlaveErrInfoDesc.wStatus,
                  pErrorDesc->desc.SlaveErrInfoDesc.wStatusCode);
    break;


    case EC_NOTIFY_SLAVE_NOT_ADDRESSABLE:
      pwrnMaster("%s (%d) not addressable.\n", pErrorDesc->desc.WkcErrDesc.SlaveProp.achName, pErrorDesc->desc.WkcErrDesc.SlaveProp.wStationAddress);
    break;
    
    
#ifdef INCLUDE_SOE_SUPPORT
    //! SoE Mailbox working counter mismatch
    case EC_NOTIFY_SOE_MBXSND_WKC_ERROR:
      pwrnMaster("EoE Mailbox working counter mismatch for %s (%d), addr %d by cmd 0x%02x, wkcSet=%d, wkcAct=%d\n",
                  pErrorDesc->desc.WkcErrDesc.SlaveProp.achName,
                  pErrorDesc->desc.WkcErrDesc.SlaveProp.wStationAddress,
                  pErrorDesc->desc.WkcErrDesc.dwAddr,
                  pErrorDesc->desc.WkcErrDesc.byCmd,
                  pErrorDesc->desc.WkcErrDesc.wWkcSet,
                  pErrorDesc->desc.WkcErrDesc.wWkcAct);
    break;
    
    //! SoE Write error
    case EC_NOTIFY_SOE_WRITE_ERROR:
      pwrnMaster("SoE mailbox write error for %s (%d) during state change %s!\n",
                  pErrorDesc->desc.InitCmdErrDesc.SlaveProp.achName,
                  pErrorDesc->desc.InitCmdErrDesc.SlaveProp.wStationAddress,
                  pErrorDesc->desc.InitCmdErrDesc.achStateChangeName);
    break;
    
#endif
    
    //! CoE SDO abort during init command
    case EC_NOTIFY_MBSLAVE_COE_SDO_ABORT:
      pwrnMaster("CoE SDO abort during init command for %s (%d), res=%d, objIdx=0x%02x, subIdx=0x%02x\n",
                  pErrorDesc->desc.SdoAbortDesc.SlaveProp.achName,
                  pErrorDesc->desc.SdoAbortDesc.SlaveProp.wStationAddress,
                  pErrorDesc->desc.SdoAbortDesc.dwErrorCode,
                  pErrorDesc->desc.SdoAbortDesc.wObjIndex,
                  pErrorDesc->desc.SdoAbortDesc.bySubIndex);
    break;
      
    //! API-client registration dropped
    case EC_NOTIFY_CLIENTREGISTRATION_DROPPED:
      EC_FAULT;
      /* client registration was dropped because ecatConfigureMaster was called by another thread
         this should never happen with our software design. */
      perrMaster("Internal Error. Client registration dropped!\n");
      break;
    
#ifdef INCLUDE_FOE_SUPPORT
    //! FoE Mailbox sent error message
    case EC_NOTIFY_FOE_MBSLAVE_ERROR:
      pwrnMaster("FoE mailbox sent error message for %s (%d), %d, %s!\n",
                  pErrorDesc->desc.FoeErrorDesc.SlaveProp.achName,
                  pErrorDesc->desc.FoeErrorDesc.SlaveProp.wStationAddress,
                  pErrorDesc->desc.FoeErrorDesc.dwErrorCode,
                  pErrorDesc->desc.FoeErrorDesc.achErrorString);

    break;
#endif
    
    //! PDI watchdog error
    case EC_NOTIFY_PDIWATCHDOG:
      EC_FAULT;// Fault reaction
      perrMaster("PDI watchdog error on %s (%d)\n",
                  pErrorDesc->desc.PdiWatchdogDesc.SlaveProp.achName,
                  pErrorDesc->desc.PdiWatchdogDesc.SlaveProp.wStationAddress);
      break;
        
    //! Slave not supported - not in latest documentation
    case EC_NOTIFY_SLAVE_NOTSUPPORTED:
      pwrnMaster("Slave not supported!\n");
    break;
    
    //! Slave is in unexpected state
    case EC_NOTIFY_SLAVE_UNEXPECTED_STATE:
      if (this->isValidState(pErrorDesc->desc.SlaveUnexpectedStateDesc.expState)) {
        EC_FAULT;// Fault reaction
      }
      perrMaster("%s (%d) in unexpected state, cur=%s, exp=%s\n",
                  pErrorDesc->desc.SlaveUnexpectedStateDesc.SlaveProp.achName,
                  pErrorDesc->desc.SlaveUnexpectedStateDesc.SlaveProp.wStationAddress,
                  ecatStateToStr(pErrorDesc->desc.SlaveUnexpectedStateDesc.curState),
                  ecatStateToStr(pErrorDesc->desc.SlaveUnexpectedStateDesc.expState));
    break;
    
    //! All slaves back in OPERATIONAL
    case EC_NOTIFY_ALL_DEVICES_OPERATIONAL:
      pmsgMaster("All slaves (back) in OPERATIONAL state.\n");
      m_allDevsInOperationalState = true;
    break;

    //! EEPROM checksum error
    case EC_NOTIFY_EEPROM_CHECKSUM_ERROR:
      pwrnMaster("EEPROM checksum error on %s (%d)\n",
                  pErrorDesc->desc.EEPROMChecksumErrorDesc.SlaveProp.achName,
                  pErrorDesc->desc.EEPROMChecksumErrorDesc.SlaveProp.wStationAddress);
    break;
      
    //! Cable swapping detected
    case EC_NOTIFY_LINE_CROSSED:
      EC_FAULT; // fatal error
      perrMaster("Ethernet line crossed on %s (%d)! Port 0 must lead to the master!\n",
                  pDesc->desc.CrossedLineDesc.SlaveProp.achName,
                  pDesc->desc.CrossedLineDesc.SlaveProp.wStationAddress);
    break;

    
    //! Cabling error or junction redundancy
    case EC_NOTIFY_JUNCTION_RED_CHANGE:
      EC_FAULT; // fatal error
      perrMaster("Cabling error or junction redundancy at %s (%d), line break=%d\n",
                  pErrorDesc->desc.JunctionRedChangeDesc.SlaveProp.achName,
                  pErrorDesc->desc.JunctionRedChangeDesc.SlaveProp.wStationAddress,
                  pErrorDesc->desc.JunctionRedChangeDesc.bLineBreak);
    break;

    //! Collects EC_NOTIFY_SLAVE_UNEXPECTED_STATE
    //! Disabled by default
    case EC_NOTIFY_SLAVES_UNEXPECTED_STATE:   
      pwrnMaster("EC_NOTIFY_SLAVES_UNEXPECTED_STATE. Not implemented!\n");
    break;

    //! Collects EC_NOTIFY_SLAVE_ERROR_STATUS
    //! Disabled by default
    case EC_NOTIFY_SLAVES_ERROR_STATUS:
      pwrnMaster("EC_NOTIFY_SLAVES_ERROR_STATUS. Not implemented!\n");
    break;
    
#ifdef INCLUDE_HOTCONNECT    
    
    //! Detection of Hot connect groups
    case EC_NOTIFY_HC_DETECTADDGROUPS:
    case EC_NOTIFY_HC_PROBEALLGROUPS:
      pmsgMaster("Hot-connect group detection finished\n");
    break;

    //! Hot connect topology change
    case EC_NOTIFY_HC_TOPOCHGDONE:
      pmsgMaster("Hot-connect topology change done!\n");
    break;
    
    
#endif /* INCLUDE_HOTCONNECT */

    default:
      EC_FAULT;// Fault reaction
      perrMaster("Call to notify with unknown code %d\n", dwCode);
    
  }

}
