// -----------------------------------------------------------------------------
// console_main.cpp -- standalone executable entry for pcie_switch_console.
//
// SystemC supplies main(); this file only forwards to the model console API,
// which is compiled into libpcie_switch_model.a.
// -----------------------------------------------------------------------------
#include "pcie_switch_console.h"

extern "C" int sc_main(int argc, char** argv) {
    return pcie_tlm::pcie_switch_console_start(argc, argv);
}
