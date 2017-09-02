//
//  test_elmo.cpp
//  am2b
//
//  Created by Felix Sygulla on 2015-08-07.
//  Copyright 2015 Chair of Applied Mechanics, TUM
//  https://www.amm.mw.tum.de/
//

// switch on elmo stm verbose mode
#define EL_STM_VERBOSE

// switch on ethercat master verbose mode
#define HWL_EC_VERBOSE

#include <iostream>
#include <string>
#include <csignal>

#include "AcEcMaster.hpp"
#include "EcLinkLayerI8254.hpp"

#include "AcEcFixedSlaveInstanceMapper.hpp"
#include "BusSlave.hpp"
#include "EL2004Device.hpp"
#include "ElmoGold.hpp"

#include <sys/mman.h>
#include <xstdio.h>
#include <progopt.hpp>

//#define TEST_HWL_AUTO_STOP

using namespace std;
using namespace am2b;
using namespace ec;

// SIG handler
// (ensure clean shutdown in SIG)
volatile bool hwl_ec_abort = false;
void handle_sigint(int) {
  hwl_ec_abort = true;
}

// test program for the ethercat master
int main (int argc, char *argv[]) {

  // Load whole program into memory for performance reasons
  if(-1 == mlockall(MCL_CURRENT|MCL_FUTURE)) {
    cout << "mlockall(): Error loading program into memory!" << endl;
    return EXIT_FAILURE;
  }

  ProgOpt opt(argv[0], "lola's low-level controller/ethercat driver.",
              argc,argv);
  opt.add(' ',"eni", true, "path to eni-xml file","../../../etc/ethercat/eni_ek_can_24elmos.xml");
  opt.add(' ',"use-dc",false,"use distributed clocks","1");
  opt.add('d',"diag", false, "enable remote diagnosis server", "0");
  opt.add(' ',"no-motor-motion",false, "disable any motor motion","0");
  opt.std_parse();

  string xml_file_name    = opt.val<string>("eni");
  bool use_dc             = opt.val<bool>("use-dc");
  bool use_ras            = opt.val<bool>("diag");
  bool no_motor_motion = opt.val<bool>("no-motor-motion");
  double sampling_period = _DT_CONT_;
  
  // Init the signal handler
  std::signal(SIGINT, handle_sigint);
  
  // create master instance
  AcEcMaster<AcEcFixedSlaveInstanceMapper, EcLinkLayerI8254 > master;
  master.init(1000, use_dc, use_ras);
  
  // configure master
  master.configure(xml_file_name);
  
  // create Elmo Device with homing based on absolute encoder
  ElmoGold<BusSlave<AcEcFixedSlaveInstanceMapper > > elmo(ElmoHomingType::ABS_ENCODER, 4000, 65535, no_motor_motion);
  
  // Attach the elmo to the specified bus slave and master
  elmo.attachSlave("Slave_zfr [Elmo Drive ]", 1012);
  elmo.setMaster(&master);

  // switch into operational mode (blocking)
  master.setRequestedState(BusState::OP);
  elmo.resetFault();
  
  
  
  // wait until in IDLE state
  while(!hwl_ec_abort) {
    
    OsSleep(10);
    
    // update the master and all slaves
    master.process();
    
    // wait until the elmo is in IDLE state (initialized)
    if (elmo.getState() == ElmoState::IDLE)
      break;
  }
  
  cout << "Press enter to execute homing and alignment on the drive (switch high voltage on before doing so)" << endl;
  cin.ignore();
  
  cout << "Executing homing on drive..." << endl;
  elmo.setRequestedState(ElmoState::HOMED);
  
  while (!hwl_ec_abort) {
    
    OsSleep(10);
    
    // update the master and all slaves
    master.process();
    
    if (elmo.preHoming()) {
      
      // TODO: calculate the homing offset here..
      // Set it using
      // elmo.setHomingOffset(...)
      
      // Acknowledge to continue with the homing process
      elmo.ackHoming();
      
    }
    
    // wait until the elmo is in HOMED state
    if (elmo.homingDone()) {
      break;
    }
  }
  
  cout << "Homing done..." << endl;
  
  
  // ask for permission to go into the operational state
  cout << "Press enter to set drive into the operational state (switch high voltage on before doing so)" << endl;
  cin.ignore();
  
  elmo.setRequestedState(ElmoState::OPERATIONAL);
  
  // Wait until elmo is in operational state
  while(!hwl_ec_abort) {
    
    // update the master and all slaves
    master.process();
    
    // wait until the elmo is in OPERATIONAL state
    // By default, the current position is held until
    // the first call to setDesiredPosition()
    if (elmo.getState() == ElmoState::OPERATIONAL)
      break;
  }
  
  int32_t pos = elmo.getPositionRaw();
  int32_t delta = 5;
  
  // wait until SIGINT is received
  unsigned long cnt = 0;
  while(!hwl_ec_abort) {

#ifdef TEST_HWL_AUTO_STOP
    if (cnt++ > 50) {
      break;
    }
#endif

    // synchronize this loop with the bus thread
    // In case this thread is slowed down, the
    // next update cycle is hit, i.e. BusVar values 
    // are going to change every frame, but the data is not
    // interpreted in this thread or via process().
    master.waitForBus();
    
    
    // increase desired position until 2000 counts, then decrease to -2000 counts
    pos += delta;
    
    if (pos > 2000) {
      pos = 2000;
      delta = -delta;
    }
    
    if (pos < -2000) {
      pos = -2000;
      delta = -delta;
    }
    
    elmo.setDesiredPositionRaw(pos);
    
    // Be sure to wait until all RX data has been processed
    master.waitForBusRXData();
    
    // update the master and all slaves
    master.process();

  }

  return 0;
}
