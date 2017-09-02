//
//  test_busonly.cpp
//  am2b
//
//  Created by Felix Sygulla on 2016-03-09.
//  Copyright 2015 Chair of Applied Mechanics, TUM
//  https://www.amm.mw.tum.de/
//

#include <iostream>
#include <string>
#include <csignal>

#include "AcEcMaster.hpp"
#include "EcLinkLayerI8254.hpp"

#include "AcEcFixedSlaveInstanceMapper.hpp"
#include "BusSlave.hpp"

#include <sys/mman.h>
#include <xstdio.h>
#include <progopt.hpp>

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
  opt.add(' ',"eni", true, "path to eni-xml file","eni.xml");
  opt.add(' ',"use-dc",false,"use distributed clocks","0");
  opt.add('d',"diag", false, "enable remote diagnosis server", "0");

  opt.std_parse();

  string xml_file_name    = opt.val<string>("eni");
  bool use_dc             = opt.val<bool>("use-dc");
  bool use_ras            = opt.val<bool>("diag");
  
  // Init the signal handler
  std::signal(SIGINT, handle_sigint);
  
  // create acontis master instance
  // Uses the link layer for i8254 network cards
  // Mapping between slaves on the bus and instances in code is done via slave names (fixed).
  AcEcMaster<AcEcFixedSlaveInstanceMapper, EcLinkLayerI8254 > master;
  master.init(1000, use_dc, use_ras); // 1000us bus cycle time
  
  // configure master with eni file
  master.configure(xml_file_name);
  
  // switch into operational mode
  master.setRequestedState(BusState::OP);
  
  // wait until SIGINT is received
  int i = 0;
  while(!hwl_ec_abort) {
    
    // synchronize this loop with the bus thread
    // Unblocks if new bus input data is ready
    master.waitForBus();
    
    //TODO: Update bus variables with target data here, i.e. here you would for example set new 
    // target positions for your joint controllers.
    
    // Be sure to wait until all RX data has been processed
    master.waitForBusRXData();
    
    // update the master and all slaves (process raw data, execute state machine logic)
    master.process();
    
    //TODO: Read sensor data from the slave instances here...
  }
  
  return 0;
}
