//
//  AcEcMaster.hpp
//  am2b
//
//  Created by Felix Sygulla on 2015-08-05.
//  Copyright 2015 Chair of Applied Mechanics, TUM
//  https://www.amm.mw.tum.de/
// 
//

#ifndef ACECMASTER_HPP_439B4CBC
#define ACECMASTER_HPP_439B4CBC

#include <AtEthercat.h>
#include <AtEmRasSrv.h>
#include <EcTimer.h>
#include <string>
#include <exception>
#include <string.h>
#include <math.h>

#include <xstdio.h>
#include <xtrace.h>
#include <iface_prio.hpp>

#include "AcEcBusVarTraits.hpp"
#include "BusMaster.hpp"
#include "BusException.hpp"
#include "BusVar.hpp"
#include "LogRateLimiter.hpp"

#include <log_buf.hpp>

namespace ec {

  //! static variable to generate unique ids
  static EC_T_DWORD AcEcGlobalUIDCounter=1;

  /* Settings for the EtherCAT Master Stack */
  #define HWL_EC_MAX_NUM_SLAVES               35
  #define HWL_EC_TIMEOUT_STATE_CHANGE_MS      15000
  #define HWL_EC_TRY_LOCK_TIMEOUT_SCALE       100   //!< if the buscycletime is 1ms, lock timeout = 10us
  #define HWL_EC_SYNC_COE_TIMEOUT_MS          500   //!< Timeout for synchronuous CoE transfer
  
  /* Scheduling Settings */
  #define HWL_EC_TIMING_THREAD_PRIO           PRIO_EC_TIMING()
  #define HWL_EC_JOB_THREAD_PRIO              PRIO_EC_JOBTASK()
  #define HWL_EC_JOB_THREAD_STACKSIZE         0x4000
  #define HWL_EC_TIMING_THREAD_CPU            1     //!< CPU used for the timing task

  // It is important to run the main thread on a different CPU
  // (or to not use cpu affinity at all for the main thread)
  // Use -1 to disable cpu affinity (currently recommended)
  #define HWL_EC_MAIN_THREAD_CPU              -1     //!< CPU used for the calling thread

  //! Verbose state for BusMaster (Wrapper implementation)
  //! define to enable verbose output to cmd line
  //#define HWL_EC_VERBOSE

  //! Verbosity level for internal AcEcMaster logging. Default: EC_LOG_LEVEL_WARNING
  //! These messages are directly printed to stdout in the main thread. A lot of messages may slow down your application.
  #define HWL_EC_INT_VERBOSE_LEVEL            EC_LOG_LEVEL_WARNING        // Valid values are ..._CRITICAL, ..._ERROR, ..._WARNING, ..._INFO, ..._VERBOSE
  
  //! Maximum number of messages per error type until
  //! the message rate is reduced
  #define HWL_EC_MAX_MSG_PER_ERROR 10
  
  //! Reduced message rate. 10 = reports every 10th occurrence
  #define HWL_EC_REDUCED_MSG_RATE 2000

  /* Distributed Clocks */
  #undef HWL_EC_DC_PRINT_STATUS                    //!< debugging only, activate verbose info on console about distributed clocks
  #define HWL_EC_DC_DEVIATION_LIMIT           13      //!< 2^(x)-1 ns = max. allowed deviation of the clocks if "Sync Window Monitoring" activated in eni file. i.e. 13 -> 2^13-1 ns = 8191 ns = 8.1us
  #define HWL_EC_DC_SETTLE_TIME               3000    //!< DC settle time in ms.
  #define HWL_EC_DC_BURST_LENGTH              10000   //!< burst cycles (static drift compensation)
  #define HWL_EC_DC_BURST_BULK                12      //!< burst bulk (static drift compensation)

  #define HWL_EC_DCM_SETTLE_TIME              1500    //!< DCM settle time in ms

  /* RAS Server (Online Diagnosis) */
  #define HWL_EC_RAS_REMOTE_CYCLE_TIME        2     //!< ms, update time for RaS
  #define HWL_EC_RAS_MAIN_THREAD_PRIO         60

  /* Internal helper functions and macros */

  //! Get the description for an AcEc error depending on the return code and print as error
  #define LOG_EC_ERROR(X,E)       perr("%s, (Result = %s 0x%x)\n", X, ecatGetText(E), E)

  //! Get the description for an AcEc error depending on the return code and print as warning
  #define LOG_EC_WARNING(X,E)     pwrn("%s, (Result = %s 0x%x)\n", X, ecatGetText(E), E)

  /* Logging macros for the master */
  #define pmsgMaster(format, ...) pmsg("Master: " format, ##__VA_ARGS__)
  #define pwrnMaster(format, ...) pwrn("Master: " format, ##__VA_ARGS__)
  #define perrMaster(format, ...) perr("Master: " format, ##__VA_ARGS__)
  #define pdbgMaster(format, ...) pdbg("Master: " format, ##__VA_ARGS__)
  
  //! Disable EC_FAULT reaction in DEBUG configuration
  //! BE CAREFUL! This might be dangerous!
  #define HWL_EC_MASTER_DISABLE_FAULT_REACTION
  //#undef HWL_EC_MASTER_DISABLE_FAULT_REACTION

  //! Fatal error (fault) macro
  #ifdef DEBUG
    #ifdef HWL_EC_MASTER_DISABLE_FAULT_REACTION
      #define EC_FAULT perr("Master: Fault reaction disabled!\n");
      #define EC_FAULT_WARN perr("EtherCAT Master Fault Reaction DISABLED in code options! Be careful!\n")
    #else
      #define EC_FAULT if (!m_fault) {perr_ffl("Master: Fault reaction!\n"); m_fault=true; }
      #define EC_FAULT_WARN
    #endif
  #else
    #define EC_FAULT if (!m_fault) {pdbg_ffl("Master: Fault Reaction!\n");} m_fault=true;
    #define EC_FAULT_WARN
  #endif

  //! This is a wrapper for the internal AcEcMaster msgs reported via callback
  EC_T_BOOL hwlLogMsg(const EC_T_CHAR* szFormat, EC_T_VALIST vaArgs) {
  
    OsVprintf(szFormat, vaArgs);
    return EC_FALSE;
  }


  /*! Prototype for Master */
  template <class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy > class AcEcMaster;

  // =============================================================
  // = Internal member functions, which are friend to AcEcMaster =
  // =============================================================
  /*! wrapper for the ecatNotify function ptr to class member notify*/
  template <class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy> EC_T_DWORD AcEcNotifyWrapper(EC_T_DWORD dwCode, EC_T_NOTIFYPARMS* pParms) {

    // cast to correct class instance pointer
    AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>* thisPtr = static_cast<AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>* > (pParms->pCallerData);

    // call the member function to notify the Master instance itself
    thisPtr->notify(dwCode, pParms);
  
    return EC_E_NOERROR;
  }

  /*! wrapper for the thread-run function ptr to class member jobtask */
  template <class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy> void AcEcJobTaskWrapper(void* instance) {

    AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>* thisPtr = static_cast<AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>* > (instance);

    // call the member function
    thisPtr->runJobTask();
  }
  
  /*! wrapper for the thread-run function ptr to class member timingTask */
  template <class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy> void AcEcTimingTaskWrapper(void* instance) {

    AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>* thisPtr = static_cast<AcEcMaster<SlaveInstanceMapperPolicy, EcLinkLayerPolicy>* > (instance);

    // call the member function
    thisPtr->runTimingTask();
  }


  /*! Implements MasterAdapterInterface for Acontis EtherCAT Master Stack
   < SlaveInstanceMapperPolicy, EcLinkLayerPolicy > */
  template <class SlaveInstanceMapperPolicy, class EcLinkLayerPolicy >
  class AcEcMaster : public BusMaster<SlaveInstanceMapperPolicy> {
    using BusMaster<SlaveInstanceMapperPolicy>::m_variablesInputPDO;
    using BusMaster<SlaveInstanceMapperPolicy>::m_variablesOutputPDO;
    using BusMaster<SlaveInstanceMapperPolicy>::m_variablesSDO;
    using BusMaster<SlaveInstanceMapperPolicy>::m_fault;
    using BusMaster<SlaveInstanceMapperPolicy>::m_slaves;
    using BusMaster<SlaveInstanceMapperPolicy>::m_cycleCounter;

  public:
  
    //! Constructor
    AcEcMaster() : m_overloadLogRateLimiter(HWL_EC_MAX_MSG_PER_ERROR, HWL_EC_REDUCED_MSG_RATE) {
    }
    
    /*! Destructor */
    ~AcEcMaster() {
      this->shutdown();
    
      /* final OS layer cleanup */
      OsDeinit();
    
      pmsgMaster("Goodbye!\n");
    }
  
    /*! Initializes the EtherCAT Master stack.
      
        \param busCycleTimeUs Cycle time in microseconds
        \param enableDC Enable extended Distributed Clock configuration
        \param enableOnlineDiagnosis Start the remote diagnosis server
        \param logDCStatus Write Distributed Clocks information to file if true
  
        Throws an exception if initialization fails
    */
    void init(unsigned int busCycleTimeUs, bool enableDC = true, bool enableOnlineDiagnosis = false, bool logDCStatus = false);
  
    /*! Shut down the master, also called by the destructor */
    void shutdown();
  
    /*! Configure EtherCAT bus with specified eni file */
    EC_T_DWORD configure(const std::string& eniFile);
    
    
    /*! Returns the current state of the EtherCAT Bus */
    BusState getState();
      
      
    /*! Sets the requested state of the EtherCAT Bus.
        
        \param reqState Requested BusState
        \param blocking Switch synronously (blocking), defaults to true
    */
    void setRequestedState(const BusState& reqState, const bool& blocking=true);
    
    /*! Returns the current BusTime in nanoseconds. */
    uint32_t getBusTime() {
      return m_busTime;
    }
    /*! Returns the BusCycleTime in nanoseconds. */
    uint32_t getBusCycleTime() {
      return (uint32_t)(m_busCycleTimeUs*1e3);
    }
    
    /*! Blocks the current thread until a new EC cycle begins
    
        The method unblocks with the bus cycle time interval.
        Used to synchronize the Shared Memory interface loop with
        the bus thread.
    */
    void waitForBus() {
      
      // wait for the newdata event
      OsWaitForEvent(m_newTXDataEvent, EC_WAITINFINITE);
    }

    /*! Blocks the current thread until new RX data is available in Bus Vars
    
        The method unblocks with the bus cycle time interval.
        Used to synchronize the Shared Memory interface loop with
        the bus thread.
    */
    void waitForBusRXData() {
      
      // wait for the newdata event
      OsWaitForEvent(m_newRXDataEvent, EC_WAITINFINITE);
    }
    
    
    /*! Process function. Must be called cyclically from user application side.
        
        This is necessary, as not all EtherCAT stack functions may be called from inside the 
        job task.
    */
    void process();

    /*! Reset a fault on the master itself */
    void resetFault();
    
    /*! Returns the bus cycle time in microseconds. Overflows! */
    uint32_t getBusCycleTimeUs() {
      return m_busCycleTimeUs;
    }

  protected:
    
    /*! Virtual method implementation for linking to PDO variables
        This is called by the BusSlave methods to register a PDO variable*/
    bool linkPDOVar(BusSlave<SlaveInstanceMapperPolicy>* const slave, const std::string& varName, 
                    BusVarType* ptr);

    /*! Virtual method implementation for linking to SDO variables
        This is called by the BusSlave methods to register a SDO variable */
    bool linkSDOVar(BusSlave<SlaveInstanceMapperPolicy>* const slave, const int& objIndex, 
                    const char& objSubIndex, BusVarType* ptr);


    /*! Virtual method implementation for asynchronous SDO send (Download) 
        This is called by the BusSlave methods to send an asynchronous SDO*/
    bool asyncSendSDO(BusSlave<SlaveInstanceMapperPolicy>* const slave, BusVarType* const ptr);

    /*! Virtual method implementation for asynchronous SDO receive (Upload) 
        This is called by the BusSlave methods to receive an asynchronous SDO*/
    bool asyncReceiveSDO(BusSlave<SlaveInstanceMapperPolicy>* const slave, BusVarType* const ptr);

    /*! Virtual method implementation for synchronous SDO send (Download) 
        This is called by the BusSlave methods to send a synchronous SDO*/
    bool syncSendSDO(BusSlave<SlaveInstanceMapperPolicy>* const slave, const int& objIndex, 
                      const char& objSubIndex, const void* const data, const int& dataLen);

    /*! Virtual method implementation for synchronous SDO receive (Upload) 
        This is called by the BusSlave methods to receive a synchronous SDO*/
    bool syncReceiveSDO(BusSlave<SlaveInstanceMapperPolicy>* const slave, const int& objIndex, 
                        const char& objSubIndex, void* const data, const int& dataLen, int* const outDataLen);

  private:
    
    /*! Changes EtherCAT bus state synchronuously (blocking) */
    EC_T_DWORD switchStateSync(const EC_T_STATE& reqState) {
    
      m_lastRes = ecatSetMasterState(HWL_EC_TIMEOUT_STATE_CHANGE_MS, reqState);
      if (m_lastRes != EC_E_NOERROR) {
        EC_FAULT; // fatal error during runtime
        LOG_EC_ERROR("Could not change bus state!", m_lastRes);
        
        // This might be caused by the DCM sync
        EC_T_DWORD dwRes = 0;
        EC_T_DWORD dwStatus = 0;
        EC_T_INT   nDiffCur = 0, nDiffAvg = 0, nDiffMax = 0;
          
        dwRes = ecatDcmGetStatus(&dwStatus, &nDiffCur, &nDiffAvg, &nDiffMax);
        if (dwRes == EC_E_NOERROR) {
          if (dwStatus != EC_E_NOERROR) {
            perrMaster("DCM Status: %s (0x%08X)\n", ecatGetText(dwStatus), dwStatus);  
          }
        }
        else {
          LOG_EC_ERROR("Could not get DCM status!", dwRes);
        }
  
      }
      
      return m_lastRes;
    }
    
    /*! Changes EtherCAT bus state asynchronuously (non-blocking) */
    EC_T_DWORD switchStateASync(const EC_T_STATE& reqState) {
    
      m_lastRes = ecatSetMasterState(EC_NOWAIT, reqState);
      if (m_lastRes != EC_E_NOERROR) {
        EC_FAULT; // fatal error during runtime
        LOG_EC_ERROR("Could not change bus state!", m_lastRes);
        
        // This might be caused by the DCM sync
        EC_T_DWORD dwRes = 0;
        EC_T_DWORD dwStatus = 0;
        EC_T_INT   nDiffCur = 0, nDiffAvg = 0, nDiffMax = 0;
          
        dwRes = ecatDcmGetStatus(&dwStatus, &nDiffCur, &nDiffAvg, &nDiffMax);
        if (dwRes == EC_E_NOERROR) {
          if (dwStatus != EC_E_NOERROR) {
            perrMaster("DCM Status: %s (0x%08X)\n", ecatGetText(dwStatus), dwStatus);  
          }
        }
        else {
          LOG_EC_ERROR("Could not get DCM status!", dwRes);
        }
     
      }
      
      return m_lastRes;
    }

    /*! Returns true if the given EtherCAT state is valid */
    bool isValidState(const EC_T_STATE& ecstate) {
      return (ecstate == eEcatState_INIT || 
              ecstate == eEcatState_PREOP || 
              ecstate == eEcatState_SAFEOP || 
              ecstate == eEcatState_OP);
    }

    //! Here the AcEcMasterNotifications_impl.hpp code is included, which handles all bus notifications
    #include "AcEcMasterNotifications_impl.hpp"

    /*! EtherCAT timing task, highest priority.
        Used to synchronize job task
    */
    void runTimingTask();
    

    /*! EtherCAT job task, run with higher priority.
        Synchronized with the timing task (highest priority)
    */
    void runJobTask();

    /* befriend the wrapper functions for callbacks to the class members*/
    friend EC_T_DWORD AcEcNotifyWrapper<SlaveInstanceMapperPolicy, EcLinkLayerPolicy > (EC_T_DWORD dwCode, EC_T_NOTIFYPARMS* pParms);
    friend void AcEcJobTaskWrapper<SlaveInstanceMapperPolicy, EcLinkLayerPolicy > (void* instance);
    friend void AcEcTimingTaskWrapper<SlaveInstanceMapperPolicy, EcLinkLayerPolicy > (void* instance);

    /* Private Members */

    //! Result code for last operation
    EC_T_DWORD                      m_lastRes = 0;

    //! EtherCAT stack registered client
    EC_T_REGISTERRESULTS            m_client;

    //! Pointer to JobTask pthread
    void*                           m_jobThread = 0;
    
    //! Pointer to timing pthread
    void*                           m_timingThread = 0;
    
    //! Timing event for high-accuracy timing loop
    void*                           m_timingEvent = 0;
    
    //! newData event for waitForBusRXData() implementation
    void*                           m_newRXDataEvent = 0;
    
    //! newData event for sendDataReadyEvent() implementation
    void*                           m_newTXDataEvent = 0;
    
    //! Timer object used for timeouts
    CEcTimer                        m_Timer;

    //! Link Layer Description Object instanciated from template value
    EcLinkLayerPolicy               m_linkLayer;

    //! flag indicates if the etherCAT Master is initialized or has already been deinitialized
    volatile bool                   m_initialized = false;  // 

    //! flag indicates if the etherCAT MAster is configured or has already been deconfigured
    volatile bool                   m_configured = false;
  
    //! flag indicates if the RaS Server has been started
    bool                            m_rasStarted = false;
  
    //! Handle for the RaS Remote diagnosis server
    EC_T_PVOID                      m_RasHandle = 0;
  
    //! Flag indicates if internal DC configuration is used
    bool                            m_enableDC = false;
    
    //! Bus Variable used for the BusTime
    BusUInt32<BusInput>             m_busTime;
    
    //! Flag to indicate if bus recovery is in progress...
    bool                            m_busRecoveryActive = false;
    
    //! Flag to indicate if DC status logging is activated
    bool                            m_logDCStatus = false;
    
    //! String for DCM log
    PLine                           m_logStr;
    //! logbuffer for DCM log
    am2b::LogBuf<PLine>             m_logbuf{"masterdcm", 5000};

    /* The following members are also used within the jobtask thread.
       Be careful to avoid race conditions! */

    //! Indicates if the jobTask thread is running
    volatile bool                   m_jobThreadRunning = false;

    //! Signals shutdown of the jobTask thread
    volatile bool                   m_jobThreadShutdown = false;

    //! Indicates if the timing thread is running
    volatile bool                   m_timingThreadRunning = false;

    //! Signals shutdown of the timing thread
    volatile bool                   m_timingThreadShutdown = false;

    //! bus cycle time in microseconds
    EC_T_DWORD                      m_busCycleTimeUs = 0;

    //! Current Bus state (read in process())
    EC_T_STATE                      m_curState = eEcatState_UNKNOWN;

    //! previous Bus state (read in last call to process())
    EC_T_STATE                      m_prevState = eEcatState_UNKNOWN;

    //! Are all devices in OPERATIONAL state?
    volatile bool                   m_allDevsInOperationalState = true;

    //! requested bus state
    EC_T_STATE                      m_reqState = eEcatState_UNKNOWN;
    
    //! Log Rate Limiter overload
    LogRateLimiter                  m_overloadLogRateLimiter;

    /* Statistics variables */
    unsigned int                    m_numLinkedPDOVars = 0;
    unsigned int                    m_numLinkedSDOVars = 0;
    unsigned int                    m_byteSizePDOMap = 0;
    unsigned int                    m_byteSizeSDOMap = 0;

  };

#include "AcEcMaster_impl.hpp"
}

namespace am2b
{

  template<>
  void setup_log_columns<PLine>(LogBuf<PLine> *lb, const PLine *)
  {
    
    lb->add_vector_data(0, CHAR, "blub", XSTDIO_LINE_SZ);
    //lb->add_vector_data(offsetof(PLine,buf), CHAR, "blub", XSTDIO_LINE_SZ);
  };
}
  
#endif /* end of include guard: ACECMASTER_HPP_439B4CBC */
