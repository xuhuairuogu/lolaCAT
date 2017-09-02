//
//  test_sdopdo.cpp
//  am2b
//
//  Created by Felix Sygulla on 2015-08-07.
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
#include "TestDevice.hpp"

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
  
  // create master instance
  AcEcMaster<AcEcFixedSlaveInstanceMapper, EcLinkLayerI8254 > master;
  master.init(1000, use_dc, use_ras);
  
  // configure master
  master.configure(xml_file_name);
  
  // create sample test device
  // Devices are always of type BusSlave.
  TestDevice<BusSlave< AcEcFixedSlaveInstanceMapper > >    slave(0x02);
  slave.attachSlave("Slave_1005 [Elmo Drive ]", 1005);  // Attach to bus slave with given name and address
  slave.setMaster(&master);                             // Set the master responsible for this slave
  
  // switch into operational mode
  master.setRequestedState(BusState::OP);
  
  
  // wait until SIGINT is received
  int i = 0;
  while(!hwl_ec_abort) {
    
    // synchronize this loop with the bus thread
    // Unblocks if new bus input data is ready
    master.waitForBus();
    
    // test synchronuous operation of SDOs
    if (i < 50) {
  
      // send and receive SDO packet synchronuously (test)
      slave.setByteVarSDOSync(i);
      printf("Synchronuous: SDO Download: %d, SDO Upload: %d \n", i, slave.getByteVarSDOSync());
      i++;
    
    }
    
    // Be sure to wait until all RX data has been processed
    master.waitForBusRXData();
    
    // update the master and all slaves (process raw data, execute state machine logic)
    master.process();
    
    if (i >= 50) {
      
      // test asynchronous operation of SDOs in both directions
      
      
      if (!slave.asyncReceiveInProgress()) {
        
        if (slave.newAsyncDataAvailable()) {
          
          // new data available -> print
          printf("Asynchronuous SDO Receive: %d\n", slave.getInputVar());
          
        }

      }
      
      // check if operation in progress?
      if (!slave.asyncSendInProgress()) {
        
        if (slave.doneAsyncDataTransfer()) {
          
          printf("Asynchronuous SDO Send completed, requesting async SDO receive!\n");
          
          // start receive transfer
          slave.receiveVarAsync();
          
        } else {
          
          if (i <= 120) {
            // start send transfer
            printf("Asynchronuous SDO Send: %d\n", i);
            slave.sendVarAsync(i);
            i++;
          }
          
        }
        
      }
      
    }

  }
  
  return 0;
}
