//
//  AcEcMaster_impl.hpp
//  am2b
//
//  Created by Felix Sygulla on 2016-03-23.
//  Copyright 2015 Chair of Applied Mechanics, TUM
//  https://www.amm.mw.tum.de/
//


// =============
// = init =
// =============
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > void AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::init(unsigned int busCycleTimeUs, bool enableDC, 
                                                                                                                                        bool enableOnlineDiagnosis, bool logDCStatus) {

  if (m_initialized) {
    perrMaster("init() on EtherCAT stack already called!\n");
    return;
  }
  
  m_logDCStatus = logDCStatus;
  if(m_logDCStatus)
    m_logbuf.start_log();

  // Expands to an error message in case the fault reaction is disabled...
  EC_FAULT_WARN;

  // set bus cycle time
  m_busCycleTimeUs = busCycleTimeUs;
  m_enableDC = enableDC;

  /* Init Remote API Server? */
  if (enableOnlineDiagnosis) {

    ATEMRAS_T_SRVPARMS RasConfig;
    OsMemset(&RasConfig, 0, sizeof(ATEMRAS_T_SRVPARMS));

    RasConfig.oAddr.dwAddr    = 0;    /* INADDR_ANY */
    RasConfig.wPort           = 6000; // default port
    RasConfig.dwCycleTime     = HWL_EC_RAS_REMOTE_CYCLE_TIME;
    RasConfig.dwWDTOLimit     = 10 / HWL_EC_RAS_REMOTE_CYCLE_TIME;  // WD Timeout after 10 secs
    RasConfig.dwReConTOLimit  = 6000; // reconnect timeout after 6000 cycles + 10 secs
    RasConfig.dwMasterPrio    = HWL_EC_RAS_MAIN_THREAD_PRIO;
    RasConfig.dwClientPrio    = HWL_EC_RAS_MAIN_THREAD_PRIO;
    RasConfig.dwConcNotifyAmount = 100;                  /* for the first pre-allocate 100 Notification spaces */
    RasConfig.dwMbxNotifyAmount  = 50;                   /* for the first pre-allocate 50 Notification spaces */
    RasConfig.dwMbxUsrNotifySize = 3000;                 /* 3K user space for Mailbox Notifications */
    RasConfig.dwCycErrInterval   = 500;                  /* span between to consecutive cyclic notifications of same type */

    // Start Ras Server
    m_lastRes = emRasSrvStart(RasConfig, &m_RasHandle);
    if (m_lastRes != EC_E_NOERROR) {
      LOG_EC_ERROR("Cannot initialize RaS Online Diagnosis Server!", m_lastRes);
    } else {
      m_rasStarted = true;
    }

    pmsgMaster("Started Online Diagnosis Server\n");
  }
  

#if HWL_EC_MAIN_THREAD_CPU != -1
  // Set thread affinity if enabled
  // This assures the QNX high resolution timer (CPU stamp)
  // doesn't jump because of cpu hopping. Furthermore, execution should be better-timed.
  EC_T_CPUSET cpuSet;
  EC_T_BOOL boolRes;
  EC_CPUSET_ZERO(cpuSet);
  EC_CPUSET_SET(cpuSet, HWL_EC_MAIN_THREAD_CPU);
  boolRes = OsSetThreadAffinity(EC_NULL, cpuSet);
  if (!boolRes) {
    perrMaster("Error setting thread affinity in main thread, invalid CPU index!\n");
  }
#endif

  /* configuration of the master */
  EC_T_INIT_MASTER_PARMS masterConfig;

  OsMemset(&masterConfig, 0, sizeof(EC_T_INIT_MASTER_PARMS));

  masterConfig.dwSignature                = ATECAT_SIGNATURE;
  masterConfig.dwSize                     = sizeof(EC_T_INIT_MASTER_PARMS);
  masterConfig.pLinkParms                 = m_linkLayer.getLinkParams();
  masterConfig.pLinkParmsRed              = EC_NULL;
  masterConfig.dwBusCycleTimeUsec         = busCycleTimeUs;
  masterConfig.dwMaxBusSlaves             = HWL_EC_MAX_NUM_SLAVES;
  masterConfig.dwMaxQueuedEthFrames       = 100;
  masterConfig.dwMaxSlaveCmdPerFrame      = 32;
  masterConfig.dwMaxSentQueuedFramesPerCycle = 1;
  masterConfig.dwEcatCmdMaxRetries        = 5;
  masterConfig.dwEoETimeout               = 1000;
  masterConfig.dwFoEBusyTimeout           = 250;
  masterConfig.dwLogLevel                 = HWL_EC_INT_VERBOSE_LEVEL;
  masterConfig.pfLogMsgCallBack           = hwlLogMsg;

  // Init Master
  m_lastRes = ecatInitMaster(&masterConfig);
  if (m_lastRes != EC_E_NOERROR) {
    LOG_EC_ERROR("Cannot initialize EtherCAT Master!", m_lastRes);
    throw BusException("Error initializing EtherCAT Master!");
  }



  // Create timing event
  m_timingEvent = OsCreateEvent();
  if (m_timingEvent == NULL) {
    perrMaster("Could not create timing event!\n");
    this->shutdown();
    throw BusException("Error creating timing event for thread synchronization!");
    return;
  }

  // Create timing task thread
  m_timingThread = OsCreateThread((EC_T_CHAR*) "tEcTimingTask", 
                                      AcEcTimingTaskWrapper<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>, 
                                      HWL_EC_TIMING_THREAD_PRIO,
                                      HWL_EC_JOB_THREAD_STACKSIZE, (void*) this);

  pmsgMaster("Started timing task thread\n");

  // wait for the thread to be started                            
  m_Timer.Start(2000);  // 2s timeout
  while(!m_Timer.IsElapsed() && !m_timingThreadRunning) {
    OsSleep(10);
  }
  if (!m_timingThreadRunning) {
    perrMaster("Could not start timing task thread!\n");
    this->shutdown();
    throw BusException("Error starting EtherCAT timing task thread!");
    return;
  }
  m_Timer.Stop();

  pmsgMaster("Timing task thread running\n");

  // create new data event
  m_newRXDataEvent = OsCreateEvent();
  if (m_newRXDataEvent == NULL) {
    perrMaster("Could not create newRXData event!\n");
    this->shutdown();
    throw BusException("Error creating newData event for waitForBusRXData() synchronization!");
    return;
  }
  
  // create new data event
  m_newTXDataEvent = OsCreateEvent();
  if (m_newTXDataEvent == NULL) {
    perrMaster("Could not create newTXData event!\n");
    this->shutdown();
    throw BusException("Error creating newData event for waitForBusTXData() synchronization!");
    return;
  }

  // create the jobtask thread
  m_jobThread = OsCreateThread((EC_T_CHAR*) "tEcJobTask", 
                                      AcEcJobTaskWrapper<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>, 
  #if !(defined EC_VERSION_GO32)
                                      HWL_EC_JOB_THREAD_PRIO,
  #else
                                      masterConfig.dwBusCycleTimeUsec,
  #endif
                                      HWL_EC_JOB_THREAD_STACKSIZE, (void*) this);

  pmsgMaster("Started job task thread\n");

  // wait for the thread to be started                            
  m_Timer.Start(2000);  // 2s timeout
  while(!m_Timer.IsElapsed() && !m_jobThreadRunning) {
    OsSleep(10);
  }
  if (!m_jobThreadRunning) {
    perrMaster("Could not start job task thread!\n");
    this->shutdown();
    throw BusException("Error starting EtherCAT job task thread!");
    return;
  }
  m_Timer.Stop();
  pmsgMaster("Job task thread running\n");

  m_initialized = true;

}


// =============
// = shutdown =
// =============
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > void AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::shutdown() {

  pmsgMaster("Terminating...\n");

  //stop loggin
  m_logbuf.stop_log();

  // switch to init
  switchStateSync(eEcatState_INIT);

  pmsgMaster("Switched bus state to INIT\n");

  // Deregister if the master has been configured
  if (m_configured) {
    

    // unregister
    m_lastRes = ecatUnregisterClient(m_client.dwClntId);
    if (m_lastRes != EC_E_NOERROR) {
      LOG_EC_ERROR("Cannot unregister EtherCAT Master", m_lastRes);
    }

    m_configured = false;
    pmsgMaster("Unregistered client\n");
  }

  // do not execute if the ECMaster has already been
  // deinitialized
  if (!m_initialized) {
    return;
  }
  
  m_timingThreadShutdown = true;
  
  // wait for thread to stop
  m_Timer.Start(2000);
  while(!m_Timer.IsElapsed() && m_timingThreadRunning) {
    OsSleep(10);
  }
  if (m_timingThread != NULL) {
  
    OsDeleteThreadHandle(m_timingThread);
    m_timingThread = 0;
  }
  
  OsDeleteEvent(m_timingEvent);
  m_timingEvent = 0;
  
  pmsgMaster("Stopped timing task thread\n");

  // stop jobtask thread
  m_jobThreadShutdown = true;

  // wait for thread to stop
  m_Timer.Start(2000);
  while(!m_Timer.IsElapsed() && m_jobThreadRunning) {
    OsSleep(10);
  }
  if (m_jobThread != NULL) {
  
    OsDeleteThreadHandle(m_jobThread);
    m_jobThread = 0;
  }
  
  OsDeleteEvent(m_newRXDataEvent);
  m_newRXDataEvent = 0;
  OsDeleteEvent(m_newTXDataEvent);
  m_newTXDataEvent = 0;

  pmsgMaster("Stopped job task thread\n");

  // Did we start the RaS Server?
  if (m_rasStarted) {
    printf("Stopping Remote API Server...\n");
    m_lastRes = emRasSrvStop(m_RasHandle, 2000);
    if (m_lastRes != EC_E_NOERROR) {
      LOG_EC_ERROR("Cannot stop Remote API server!", m_lastRes);
    } else {
      pmsgMaster("...done\n");
      m_rasStarted = false;
    }
  }

  

  // delete mailbox transfer objects
  for (std::vector<BusVarType*>::iterator it = m_variablesSDO.begin() ; it != m_variablesSDO.end(); ++it) {
    
    // CoE emergency objects don't have a separate mailbox object
    if ((*it)->m_offset != BUSVAR_COE_EMERGENCY) {
      ecatMbxTferDelete((*it)->m_tferObj);
    }
  }
  // clear list
  m_variablesSDO.clear();

  pmsgMaster("Deleted Mailbox Transfer Objects\n");

  // deinit master
  m_lastRes = ecatDeinitMaster();
  if (m_lastRes != EC_E_NOERROR) {
    LOG_EC_ERROR("Cannot deinitialize EtherCAT Master", m_lastRes);
  }

  pmsgMaster("Deinitialized EtherCAT Master\n");
  m_initialized = false;

}


// =============
// = configure =
// =============
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > EC_T_DWORD AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::configure(const std::string& eniFile) {

  // configure the etherCAT Master with the given eni file
  m_lastRes = ecatConfigureMaster(eCnfType_Filename, (unsigned char*) eniFile.c_str(), eniFile.length()+1);
  if (m_lastRes != EC_E_NOERROR) {
    LOG_EC_ERROR("Cannot configure EtherCAT Master!", m_lastRes);
    this->shutdown();
    throw BusException("Error configuring EtherCAT Master!");
    return m_lastRes;
  }

  pmsgMaster("Configuration successful\n");

  // (Re)Configure distributed clocks?
  if (m_enableDC) {
  
    
  
    // Configure DC
    EC_T_DC_CONFIGURE   DCconfig;
    OsMemset(&DCconfig, 0, sizeof(EC_T_DC_CONFIGURE));
    DCconfig.dwTimeout = 12000;   // initialization timeout in msecs
    DCconfig.dwDevLimit = HWL_EC_DC_DEVIATION_LIMIT;
    DCconfig.dwSettleTime = HWL_EC_DC_SETTLE_TIME;
    DCconfig.dwTotalBurstLength = HWL_EC_DC_BURST_LENGTH;
    DCconfig.dwBurstBulk = HWL_EC_DC_BURST_BULK;
  
    if (m_busCycleTimeUs < 1000 ) {
      // Reduce bulk at high bus speeds
      DCconfig.dwBurstBulk = HWL_EC_DC_BURST_BULK/2;
    }
  
    // configure DCs
    m_lastRes = ecatDcConfigure(&DCconfig);
    if (m_lastRes != EC_E_NOERROR) {
      LOG_EC_ERROR("Cannot configure Distributed Clocks!", m_lastRes);
      return m_lastRes;
    }
  
    // Configure DCM (Master synchronization)
    // with BusShift mode
    EC_T_DCM_CONFIG DCMConfig;
    OsMemset(&DCMConfig, 0, sizeof(EC_T_DCM_CONFIG));
    DCMConfig.eMode = eDcmMode_BusShift;

    /* Set value is the time difference between the cyclic frame send time and the DC base on bus (SYNC0 if shift is zero) */
    DCMConfig.u.BusShift.nCtlSetVal = ((m_busCycleTimeUs*2)/3)*1000; // 66% of the bus cycle time
    DCMConfig.u.BusShift.dwInSyncLimit = (m_busCycleTimeUs*1000)/10; // 10% of the bus cycle time for InSync monitoring limit
    DCMConfig.u.BusShift.dwInSyncSettleTime = HWL_EC_DCM_SETTLE_TIME;
    DCMConfig.u.BusShift.bLogEnabled = m_logDCStatus;  // Store logging information if true

    // init DCM
    m_lastRes = ecatDcmConfigure(&DCMConfig, 0);
    if (m_lastRes != EC_E_NOERROR) {
    
      if (m_lastRes == EC_E_FEATURE_DISABLED) {
        perrMaster("Cannot configure DCM Mode! Is your eni-file configured properly for BusShift Mode?\n");
        return m_lastRes;
      } else {
    
        LOG_EC_ERROR("Cannot configure DCM (Master Synchronization)!", m_lastRes);
        return m_lastRes;
      }
    }
  
    pmsgMaster("Configured Distributed Clocks\n");
    pmsgMaster("Bus Cycle Time (us): %i\n", m_busCycleTimeUs);
    pmsgMaster("DCM In-Sync Window (ns): %i\n", DCMConfig.u.BusShift.dwInSyncLimit);
    pmsgMaster("DC In-Sync Window (ns): %i\n", (int) std::pow(2.0, (float)DCconfig.dwDevLimit));
    
  
  }

  // register this client to the master
  m_lastRes = ecatRegisterClient(AcEcNotifyWrapper<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>, (void*) this, &m_client);
  if (m_lastRes != EC_E_NOERROR) {
    LOG_EC_ERROR("Cannot register client!", m_lastRes);
    this->shutdown();
    throw BusException("Error registering EtherCAT Master client!");
    return m_lastRes;
  }
  
  // master is configured
  m_configured = true;
  
  pmsgMaster("Registered Client\n");

  // Link the BusTime variable
  EC_T_PROCESS_VAR_INFO varInfo;
  m_lastRes = ecatFindInpVarByName((EC_T_CHAR*) "Inputs.BusTime", &varInfo);
  if (m_lastRes != EC_E_NOERROR) {
    LOG_EC_ERROR("\nError finding BusTime input variable! There may be no slave with DC-Support or just one slave?", m_lastRes);
    return EC_E_NOERROR;
  }
  
  if (m_busTime.getSize() != (unsigned int) varInfo.nBitSize) {
    perrMaster("Internal Error - BusTime variable size does not match!\n");
    this->shutdown();
    throw BusException("Internal Error while linking BusTime variable!");
    return m_lastRes;
  }
  
  if (!isOfBusType(&m_busTime, varInfo.wDataType)) {
    perrMaster("Internal Error - BusTime variable type does not match!\n");
    this->shutdown();
    throw BusException("Internal Error while linking BusTime variable!");
    return m_lastRes;
  }
  
  m_busTime.m_offset = varInfo.nBitOffs;
  m_variablesInputPDO.push_back(&m_busTime);

  pmsgMaster("Linked BusTime variable\n");

  return m_lastRes;

}

// ==============
// = linkPDOVar =
// ==============
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > bool AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::linkPDOVar(BusSlave<SlaveInstanceMapperPolicy>* const slave, const std::string& varName, 
                    BusVarType* ptr) {
  
  EC_T_PROCESS_VAR_INFO varInfo;

  /* Retrieve the full Identifier from the slave instance: */
  std::string fullName = slave->getFullIdentifier(varName);

  // check if PDO var
  if (!ptr->isPDO()) {
    perrMaster("Can not link SDO var with linkPDOVar(), %s\n", fullName.c_str());
    EC_FAULT; // fatal error
    return true;
  }

  // get variable data from the master
  if (ptr->isOutput()) {

    // output var
    m_lastRes = ecatFindOutpVarByName(const_cast<char*>(fullName.c_str()), &varInfo);
    if (m_lastRes != EC_E_NOERROR) {
      perrMaster("Error linking bus variable %s\n", fullName.c_str());
      LOG_EC_ERROR("Error finding slave output variable in config!", m_lastRes);   
      EC_FAULT; // fatal error
      return true; 
    }


  } else {

    // input var
    m_lastRes = ecatFindInpVarByName(const_cast<char*>(fullName.c_str()), &varInfo);
    if (m_lastRes != EC_E_NOERROR) {
      perrMaster("Error linking bus variable %s\n", fullName.c_str());
      LOG_EC_ERROR("Error finding slave input variable in config!", m_lastRes);   
      EC_FAULT; // fatal error
      return true; 
    }

  }

  // check configuration...
  if (ptr->getSize() != (unsigned int) varInfo.nBitSize) {
    perrMaster("Error linking bus variable %s\n", fullName.c_str());
    perrMaster("Variable size mismatch in linkPDOVar()!\n"
                 "size of bus variable instance: %d\n"
                 "size read from config file: %d\n",
                 ptr->getSize(),varInfo.nBitSize);
    EC_FAULT; // fatal error
    return true;
  }

  if (!isOfBusType(ptr, varInfo.wDataType)) {
    perrMaster("Error linking bus variable %s\n", fullName.c_str());
    perrMaster("Variable type mismatch in linkPDOVar()!\n"
                 "Type of bus variable: %s\n"
                 "EcType from config file: %d\n",
                 ptr->getTypeId()->name(), varInfo.wDataType);
    EC_FAULT; // fatal error
    return true;
  }

  // and store the offset in the PDO map
  ptr->m_offset = varInfo.nBitOffs;

#ifdef HWL_EC_VERBOSE
  pdbgMaster("Linked PDO variable '%s'\n", fullName.c_str());
#endif

  // Statistics
  m_numLinkedPDOVars++;
  m_byteSizePDOMap += ptr->getSize() / 8;

  // call parent
  return BusMaster<SlaveInstanceMapperPolicy>::linkPDOVar(slave, varName, ptr);

}

// ==============
// = linkSDOVar =
// ==============
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > bool AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::linkSDOVar(BusSlave<SlaveInstanceMapperPolicy>* const slave, const int& objIndex, 
                    const char& objSubIndex, BusVarType* ptr) {
                      
  // check if SDO var
  if (ptr->isPDO()) {
    perrMaster("Can not link PDO var with linkSDOVar() for %s, objIndex: 0x%x, subIdx: 0x%x\n", slave->getName().c_str(), objIndex, objSubIndex);
    EC_FAULT; // fatal error
    return true;
  }

  // Check size
  if (ptr->getSize() == 0) {
    perrMaster("Can not create Mailbox object of size zero for %s, objIndex: 0x%x, subIdx: 0x%x\n", slave->getName().c_str(), objIndex, objSubIndex);
    EC_FAULT; // fatal error
    return true;
  }

  // In the case this is meant to link against a CoE emergency object
  if (ptr->m_offset == BUSVAR_COE_EMERGENCY) {

    // A CoE emergency obj is 8 bytes long
    if (ptr->getSize() != 8*8) {
      perrMaster("Size mismatch. Can not link SDO CoE Emergency Object to BusVar for %s!\n", slave->getName().c_str());
      EC_FAULT; // fatal error
      return true;
    }

    if (ptr->isOutput()) {
      perrMaster("Internal Error. BusOutput as Emergency SDO object!\n");
      EC_FAULT; // fatal error
      return true;
    }

    // store the slave's station address in the objId variable
    ptr->m_objId = slave->getStationAddress();

#ifdef HWL_EC_VERBOSE
    pdbgMaster("Linked SDO CoE Emergency Object for '%s'\n", slave->getName().c_str());
#endif

    // just register the slave
    return BusMaster<SlaveInstanceMapperPolicy>::linkSDOVar(slave, objIndex, objSubIndex, ptr);
  }

  // Link "normal" SDO variable

  // create a mailbox transfer object for the size of the variable
  EC_T_MBXTFER_DESC mbxDesc;
  mbxDesc.dwMaxDataLen = (EC_T_DWORD) ptr->getSize();
  mbxDesc.pbyMbxTferDescData = (EC_T_BYTE*) ptr->getPointer();

  ptr->m_tferObj = ecatMbxTferCreate(&mbxDesc);

  if (ptr->m_tferObj == NULL) {
    perrMaster("Can not create Mailbox transfer object for %s, objIndex: 0x%x, subIdx: 0x%x\n", slave->getName().c_str(), objIndex, objSubIndex);
    EC_FAULT; // fatal error
    return true;
  }


  //! Set the client id
  ptr->m_tferObj->dwClntId = m_client.dwClntId;

  //! set the data len
  ptr->m_tferObj->dwDataLen = ptr->getSize();

  /* Save the arguments to the variable itself */
  ptr->m_objId = objIndex;
  ptr->m_subIdx = objSubIndex;
  ptr->m_slaveId = slave->getSlaveID();

  if (ptr->m_slaveId == INVALID_SLAVE_ID) {
    perrMaster("Error linking to SDO with objIndex 0x%x, subIdx 0x%x: Couldn't find slave %s\n", objIndex, objSubIndex, slave->getName().c_str());
    ecatMbxTferDelete(ptr->m_tferObj);
    ptr->m_tferObj = NULL;
    EC_FAULT; // fatal error
    return true;
  }

#ifdef HWL_EC_VERBOSE
  pdbgMaster("Linked SDO variable for '%s', objIndex: 0x%x, subIdx: 0x%x\n", slave->getName().c_str(), objIndex, objSubIndex);
#endif

  // Statistics
  m_numLinkedSDOVars++;
  m_byteSizeSDOMap += ptr->getSize() / 8;

  // Call parent
  return BusMaster<SlaveInstanceMapperPolicy>::linkSDOVar(slave, objIndex, objSubIndex, ptr);
}

// ================
// = asyncSendSDO =
// ================
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > bool AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::asyncSendSDO(BusSlave<SlaveInstanceMapperPolicy>* const slave, BusVarType* const ptr) {

  EC_T_DWORD res;
  
#ifdef DEBUG
  // check for null pointers
  if (ptr->m_tferObj == NULL) {
    perrMaster("Tried to send async SDO to %s for non-initialized SDO BusVar. Did you link the variable?\n", slave->getName().c_str());
  }
#endif

  // scoped lock
  std::lock_guard<std::timed_mutex> lock(ptr->getMutex());
  
  if (ptr->m_SDOTransferInProgress) {
    perrMaster("Error - asynchronous SDO transfer to %s, objIndex=0x%x, subIdx=0x%x already in progress!\n", slave->getName().c_str(), ptr->m_objId, ptr->m_subIdx);
    return true;
  }

  // set transfer in progress flag
  ptr->m_SDOTransferInProgress=true;
  ptr->m_SDOTransferDone = false;
  ptr->m_SDOTransferFailed = false;
  
  //! Set the transfer id
  ptr->m_tferObj->dwTferId = AcEcGlobalUIDCounter++;

  // reset the state of the transfer object
  ptr->m_tferObj->eTferStatus = eMbxTferStatus_Idle;

  // initiate the SDO download
  res = ecatCoeSdoDownloadReq(ptr->m_tferObj, ptr->m_slaveId, ptr->m_objId, ptr->m_subIdx, HWL_EC_SYNC_COE_TIMEOUT_MS, 0);

#ifdef HWL_EC_VERBOSE
  pdbgMaster("Requesting asynchronous SDO transfer (%d) to %s, objIndex=0x%x, subIdx=0x%x\n", ptr->m_tferObj->dwTferId, slave->getName().c_str(), ptr->m_objId, ptr->m_subIdx);
#endif

  if (res != EC_E_NOERROR) {
    LOG_EC_ERROR("Error during ecatCoeSdoDownloadReq", res);
    EC_FAULT; // fatal error
    return true;
  }

  return false;


}

// ===================
// = asyncReceiveSDO =
// ===================
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > bool AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::asyncReceiveSDO(BusSlave<SlaveInstanceMapperPolicy>* const slave, BusVarType* const ptr) {
    
  EC_T_DWORD res;
  
#ifdef DEBUG
  // check for null pointers
  if (ptr->m_tferObj == NULL) {
    perrMaster("Tried to receive async SDO from %s for non-initialized SDO BusVar. Did you link the variable?\n", slave->getName().c_str());
  }
#endif

  // scoped lock
  std::lock_guard<std::timed_mutex> lock(ptr->getMutex());
  
  if (ptr->m_SDOTransferInProgress) {
    perrMaster("Error - asynchronous SDO transfer from %s, objIndex=0x%x, subIdx=0x%x already in progress!\n", slave->getName().c_str(), ptr->m_objId, ptr->m_subIdx);
    return true;
  }

  // set transfer in progress flag
  ptr->m_SDOTransferInProgress=true;
  ptr->m_SDOTransferDone = false;
  ptr->m_SDOTransferFailed = false;

  //! Set the transfer id
  ptr->m_tferObj->dwTferId = AcEcGlobalUIDCounter++;

  // reset the state of the transfer object
  ptr->m_tferObj->eTferStatus = eMbxTferStatus_Idle;

  // initiate the SDO upload
  res = ecatCoeSdoUploadReq(ptr->m_tferObj, ptr->m_slaveId, ptr->m_objId, ptr->m_subIdx, HWL_EC_SYNC_COE_TIMEOUT_MS, 0);

#ifdef HWL_EC_VERBOSE
  pdbgMaster("Requesting asynchronous SDO transfer (%d) from %s, objIndex=0x%x, subIdx=0x%x\n", ptr->m_tferObj->dwTferId, slave->getName().c_str(), ptr->m_objId, ptr->m_subIdx);
#endif

  if (res != EC_E_NOERROR) {
    LOG_EC_ERROR("Error during ecatCoeSdoUploadReq", res);
    EC_FAULT; // fatal error
    return true;
  }

  return false;

}

// ===============
// = syncSendSDO =
// ===============
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > bool AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::syncSendSDO(BusSlave<SlaveInstanceMapperPolicy>* const slave, const int& objIndex, 
                      const char& objSubIndex, const void* const data, const int& dataLen) {
  
  if (data == NULL) {
    perrMaster("Tried to send sync SDO with data null-pointer..., slave %s, objIndex: 0x%x, subIdx: 0x%x\n", slave->getName().c_str(), objIndex, objSubIndex);
    EC_FAULT; // fatal error
    return true;
  }

#ifdef HWL_EC_VERBOSE
  pdbgMaster("Sending synchronous SDO to %s, objIndex=0x%x, subIdx=0x%x\n", slave->getName().c_str(), objIndex, objSubIndex);
#endif

  m_lastRes = ecatCoeSdoDownload(slave->getSlaveID(), objIndex, objSubIndex, (EC_T_BYTE*) data, dataLen, HWL_EC_SYNC_COE_TIMEOUT_MS, EC_NULL);
  if (m_lastRes != EC_E_NOERROR) {
    LOG_EC_ERROR("Error during synchronous SDO Download!", m_lastRes);   
    EC_FAULT; // fatal error
    return true; 
  }

  // no errors
  return false;
}

// ==================
// = syncReceiveSDO =
// ==================
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > bool AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::syncReceiveSDO(BusSlave<SlaveInstanceMapperPolicy>* const slave, const int& objIndex, 
                        const char& objSubIndex, void* const data, const int& dataLen, int* const outDataLen) {

  if (data == NULL) {
    perrMaster("Tried to receive sync SDO with data null-pointer..., slave %s, objIndex: 0x%x, subIdx: 0x%x\n", slave->getName().c_str(), objIndex, objSubIndex);
    EC_FAULT; // fatal error
    return true;
  }

#ifdef HWL_EC_VERBOSE
  pdbgMaster("Receiving synchronous SDO from %s, objIndex=0x%x, subIdx=0x%x\n", slave->getName().c_str(), objIndex, objSubIndex);
#endif

  EC_T_DWORD dataReceived;

  m_lastRes = ecatCoeSdoUpload(slave->getSlaveID(), objIndex, objSubIndex, (EC_T_BYTE*) data, dataLen, &dataReceived, HWL_EC_SYNC_COE_TIMEOUT_MS, EC_NULL);
  if (m_lastRes != EC_E_NOERROR) {
    LOG_EC_ERROR("Error during synchronous SDO Upload!", m_lastRes);
    EC_FAULT; // fatal error
    return true; 
  }

  /* Check for NULL pointer, otherwise set outDataLen */
  if (outDataLen != NULL) {
    *outDataLen = dataReceived;
  }

  // no errors
  return false;
}

// ============
// = getState =
// ============
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > BusState AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::getState() {

  m_curState = ecatGetMasterState();
  
  switch(m_curState) {
    
    case eEcatState_INIT:
    return BusState::INIT;
    
    case eEcatState_PREOP:
    return BusState::PREOP;
    
    case eEcatState_OP:
    return BusState::OP;
    
    case eEcatState_SAFEOP:
    return BusState::SAFEOP;
    
  }
  
  return BusState::UNKNOWN;
}

// =====================
// = setRequestedState =
// =====================
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > void AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::setRequestedState(const BusState& reqState, const bool& blocking) {
  
  m_reqState = eEcatState_UNKNOWN;
  
  switch(reqState) {
    
    case BusState::INIT:
      blocking ? switchStateSync(eEcatState_INIT) : switchStateASync(eEcatState_INIT);
      m_reqState = eEcatState_INIT;
    break;
    
    case BusState::PREOP:
      blocking ? switchStateSync(eEcatState_PREOP) : switchStateASync(eEcatState_PREOP);
      m_reqState = eEcatState_PREOP;
    break;
    
    case BusState::OP:
      blocking ? switchStateSync(eEcatState_OP) : switchStateASync(eEcatState_OP);
      m_reqState = eEcatState_OP;
    break;
    
    case BusState::SAFEOP:
      blocking ? switchStateSync(eEcatState_SAFEOP) : switchStateASync(eEcatState_SAFEOP);
      m_reqState = eEcatState_SAFEOP;
    break;
    
  }
  
}

// ==============
// = resetFault =
// ==============
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > void AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::resetFault() {

  if (!m_busRecoveryActive) {
    // Reset requested bus state
    switchStateASync(eEcatState_OP);
  }
  
  // Call parent class
  BusMaster<SlaveInstanceMapperPolicy>::resetFault();
  
}

// ===========
// = process =
// ===========
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > void AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::process() {
  
  typedef typename std::vector<BusSlave<SlaveInstanceMapperPolicy>*>::iterator SlaveIterator;
  m_curState = ecatGetMasterState();
  
  
  if (m_busRecoveryActive) {
    
    if (m_Timer.IsElapsed()) {
      m_Timer.Stop();
      m_busRecoveryActive = false;
    }
    
  } else if (!m_allDevsInOperationalState) {
    // Check the bus state and possible error recovery measures
    
    EC_T_DWORD connectedSlaves = ecatGetNumConnectedSlaves();
    
    // switch to SAFEOP in case a slave is missing
    if (m_curState == eEcatState_OP) {
      
      // State change
      switchStateASync(eEcatState_SAFEOP);
      m_Timer.Start(3000);  // Start 3 Seconds timeout
      m_busRecoveryActive = true;
      
    } else if (!m_fault && connectedSlaves == ecatGetNumConfiguredSlaves() && m_curState != eEcatState_INIT) {
      
      pwrnMaster("Bus Recovery - trying to set master into INIT state.\n");
      switchStateASync(eEcatState_INIT);
      m_Timer.Start(3000);  // Start 3 Seconds timeout
      m_busRecoveryActive = true;
      
    } else if (!m_fault && connectedSlaves == ecatGetNumConfiguredSlaves() && m_curState == eEcatState_INIT && m_reqState == eEcatState_OP) {
      
      // blocking...
      pwrnMaster("Bus Recovery - trying to set master into OP state.\n");
      switchStateASync(eEcatState_OP);
      m_Timer.Start(3000);  // Start 3 Seconds timeout
      m_busRecoveryActive = true;
      
    }
    
  }
  
  
  // process() and init() methods: trigger slaves only in SAFEOP and OP mode
  if (m_curState == eEcatState_OP || m_curState == eEcatState_SAFEOP) {
  
    // Call process() and initOP() on the slaves
    for (SlaveIterator it = m_slaves.begin(); it != m_slaves.end(); ++it) {
      
      if (m_curState != m_prevState && m_curState == eEcatState_OP) {
        // trigger initOp
        BusMaster<SlaveInstanceMapperPolicy>::initOpOnSlave(*it);
      }
      
      // call process on slave
      BusMaster<SlaveInstanceMapperPolicy>::processOnSlave(*it);

    }
  }
  
  // First initialization from UNKNOWN
  if (m_prevState == eEcatState_UNKNOWN && m_prevState != m_curState) {
    
    pmsgMaster("************************ Statistics ************************\n");
    pmsgMaster("Cyclic PDO variables count / total size: %i / %i bytes\n", 
                m_numLinkedPDOVars, m_byteSizePDOMap);
    pmsgMaster("Acyclic SDO variables count / total size: %i / %i bytes\n",
                m_numLinkedSDOVars, m_byteSizeSDOMap);
    pmsgMaster("************************************************************\n");

  }

  m_prevState = m_curState;
}



// =================
// = runTimingTask =
// =================
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > void AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::runTimingTask() {
  
  pthread_setname_np(pthread_self(),"ectimingtask");
  
  // Set thread affinity
  // This assures the QNX high resolution timer (CPU stamp)
  // doesn't jump because of cpu hopping. Furthermore, execution should be better-timed.
   EC_T_CPUSET cpuSet;
  EC_T_BOOL boolRes;
  EC_CPUSET_ZERO(cpuSet);
  EC_CPUSET_SET(cpuSet, HWL_EC_TIMING_THREAD_CPU);
  boolRes = OsSetThreadAffinity(EC_NULL, cpuSet);
  if (!boolRes) {
    perrMaster("Error setting thread affinity in timing task, invalid CPU index!\n");
  }
  
  // init setClockPeriod to change the tick length of the
  // scheduler
  struct _clockperiod oClockPeriod = {0};
  oClockPeriod.nsec = m_busCycleTimeUs * 1000;
  if(ClockPeriod(CLOCK_REALTIME, &oClockPeriod, EC_NULL, 0) == -1) {
      perrMaster("tEcTimingTask:: Cannot set the clock period! Error %i\n", errno);
      return;
  }
  
  // thread started and working
  m_timingThreadRunning = true;

  // Create timing events as long as the master runs
  while (!m_timingThreadShutdown) {
    
    /* wait for next cycle - wait time defined by ClockPeriod */
    OsSleep(1);
    
    // Trigger the job task thread
    OsSetEvent(m_timingEvent);
    
  }
  
  m_timingThreadRunning = false;
  
}

// ==============
// = runJobTask =
// ==============
template<class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > void AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>::runJobTask() {

  EC_T_DWORD  res;
  EC_T_BOOL   lastFrameOK = EC_FALSE;
  EC_T_INT    overloadCounter = 0;

  pthread_setname_np(pthread_self(),"ecjobtask");
  
  // thread started
  m_jobThreadRunning = true;


  // run cyclically
  while (!m_jobThreadShutdown) {

    // Synchronize with the timing thread
    OsWaitForEvent(m_timingEvent, EC_WAITINFINITE);

    trace_evt("ecjt-timing",4,__LINE__);

    // Synchronize external thread calls to waitForBus()
    OsSetEvent(m_newTXDataEvent);

    // process all receive frames
    res = ecatExecJob ( eUsrJob_ProcessAllRxFrames, &lastFrameOK);
    if (res != EC_E_NOERROR && res != EC_E_INVALIDSTATE && res != EC_E_LINK_DISCONNECTED) {
      LOG_EC_ERROR("Error during ProcessAllRxFrames!", res);
    }
    
    trace_evt("ecjt-procrx",4,__LINE__);

    // overload check
    if (EC_E_NOERROR == res) {
      
      if (!lastFrameOK) {
        
        overloadCounter += 10;
        m_overloadLogRateLimiter.count();

        if (m_overloadLogRateLimiter.onLimit()) {
          pwrnMaster("Reached maximum number of messages for system overload. Reducing report rate...\n");
        }
        if (m_overloadLogRateLimiter.log()) {
          pwrnMaster( "Warning: System overload: Cycle time too short or huge jitter!\n");
          trace_evt("ecjt-jitter",10,__LINE__);
        }

        // Fault in case more than 5 consecutive frames are lost
        if (overloadCounter >= 50) {
          perrMaster("Lost too many frames! Check cabling!\n");
          EC_FAULT; // fatal error
        }
      }
      else {
        // Decrement overload counter in case the last frame was received successfully
        if (overloadCounter > 0) {
          overloadCounter--;
      
          m_overloadLogRateLimiter.reset();
        }
      }
    }
    
    
    // Copy PDO input data to Input-Type Bus Vars

    // iterate over all linked variables
    // try to lock their mutex and copy the data 
    // if the mutex has been acquired
    for (std::vector<BusVarType*>::iterator it = m_variablesInputPDO.begin() ; it != m_variablesInputPDO.end(); ++it) {
  
      // try to lock the data area
      if ((*it)->getMutex().try_lock_for(std::chrono::microseconds(m_busCycleTimeUs/HWL_EC_TRY_LOCK_TIMEOUT_SCALE))) {
  
        if (isOfBusType((*it), DEFTYPE_BOOLEAN)) {
          // special handling for boolean type
          EC_T_BYTE tmp = 0;
          bool* tmpPtr;
          EC_GETBITS(ecatGetProcessImageInputPtr(), &tmp, (*it)->m_offset, 1);
          tmpPtr = (bool*)(*it)->getPointer();
  
          if (tmp) {
            *tmpPtr = true;
          } else {
            *tmpPtr = false;
          }
  
        } else {
  
          // copy input data to the memory area of the bus var
          EC_GETBITS(ecatGetProcessImageInputPtr(), (EC_T_BYTE*) (*it)->getPointer(), (*it)->m_offset, (*it)->getSize());
    
        }

        // unlock mutex
        (*it)->getMutex().unlock();
      }
  
    }
    
    trace_evt("ecjt-busvarsrx",4,__LINE__);

    // Readout the data from all clients and update the process data map
    //

    // iterate over all linked variables
    // try to lock their mutex and copy the data 
    // if the mutex has been acquired
    for (std::vector<BusVarType*>::iterator it = m_variablesOutputPDO.begin() ; it != m_variablesOutputPDO.end(); ++it) {
  
      // try to lock the data area
      if ((*it)->getMutex().try_lock_for(std::chrono::microseconds(m_busCycleTimeUs/HWL_EC_TRY_LOCK_TIMEOUT_SCALE))) {
  
        // copy the memory area
        EC_SETBITS(ecatGetProcessImageOutputPtr(), (EC_T_BYTE*) (*it)->getPointer(), (*it)->m_offset, (*it)->getSize());
      
        // unlock mutex
        (*it)->getMutex().unlock();
      }
  
    }

    trace_evt("ecjt-busvarstx",4,__LINE__);

    
    // Synchronize external thread calls to waitForBusRXData()
    OsSetEvent(m_newRXDataEvent);

    // Increase cycle counter
    m_cycleCounter++;
   
  
    // send all cyclic frames
    res = ecatExecJob(eUsrJob_SendAllCycFrames, EC_NULL);
    if (res != EC_E_NOERROR && res != EC_E_INVALIDSTATE && res != EC_E_LINK_DISCONNECTED) {
      LOG_EC_ERROR("Error during SendAllCycFrames!", res);
    }

    trace_evt("ecjt-cyclframessent",4,__LINE__);

    // administrative stuff
    res = ecatExecJob(eUsrJob_MasterTimer, EC_NULL);
    if (res != EC_E_NOERROR && res != EC_E_INVALIDSTATE) {
      LOG_EC_ERROR("Error during MasterTimer!", res);
    }

    trace_evt("ecjt-timerdone",4,__LINE__);

    // Log DCM data 
    if (m_logDCStatus) {
      //perr("before log dc status\n");
 
      char *logStr=0x0;
      ecatDcmGetLog(&logStr);

      if (logStr != 0x0)
        {
          strncpy(m_logStr.buf,logStr,XSTDIO_LINE_SZ);
          //pdbg(" DCMLog: %s\n", m_logStr.buf[0]);
          m_logbuf.push(m_logStr);
        }

    }

    // send acyclic frames
    res = ecatExecJob(eUsrJob_SendAcycFrames, EC_NULL);
    if (res != EC_E_NOERROR && res != EC_E_INVALIDSTATE && res != EC_E_LINK_DISCONNECTED) {
      LOG_EC_ERROR("Error during SendAcycFrames", res);
    }
  
    trace_evt("ecjt-acycldone",4,__LINE__);

#ifdef HWL_EC_DC_PRINT_STATUS
    ecatDcmShowStatus();
#endif

  }

  m_jobThreadRunning = false;

}
