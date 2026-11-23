// -----------------------------------------------------------------------------
// pcie_switch_console.h -- interactive console for the PCIe switch TLM model.
//
// The console is part of the model (compiled into libpcie_switch_model.a).
// It runs a standalone SystemC fabric (root/upstream/crossbar + downstream
// port stubs) and lets the user inspect the configuration, query routing,
// run built-in testcases, and dump statistics. No HDL simulator is required.
//
// Entry point is idempotent: only the first call starts the command loop.
// It must be called from sc_main() before any sc_start() call.
// -----------------------------------------------------------------------------
#pragma once

namespace pcie_tlm {

// Start the interactive console.
//   argc/argv are reserved for future options (currently unused).
// Returns 0 on normal exit (EOF or "quit").
int pcie_switch_console_start(int argc = 0, char** argv = nullptr);

} // namespace pcie_tlm
