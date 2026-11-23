// -----------------------------------------------------------------------------
// Solution One - link power state machine (**declarations**) -- * T49-4: B8-B **ASPM (L0s/L1) + L1 substates**
//   Convention (spec: ASPM requires **both ends enabled**; L1 substates require both ends + CLKREQ):
//     - both ends = the link power face of two `CfgSpace`s (ext **0xF0..0xF8**; see the CfgSpace note);
//     - **enter when idle / exit on traffic** (traffic = the monotonic `activity()` counter:
//       change => back to L0);
//     - transition order: L0 --(idle >= `idle_l0s`)--> **L0s** --(idle >= `idle_l1`)--> **L1**
//       => inside L1 the substate depends on both ends' enables: **L1.2** (both ends l12 && CLKREQ) >
//       **L1.1** (both ends l11 && CLKREQ) > plain L1; NEVER: single-end enable => **no entry**
//       (`blocked_partial` explicitly accounted, not silent);
//     - return (any traffic) => **L0** + `exits` count; the first entry into each state is
//       counted once (`l0s/l1/l11/l12_entries`).
//   * Ownership: both link ends **belong to this TLM** (switch port + device/root port) => the state
//     machine lives in this model; real hardware corresponds to RTL link power negotiation =>
//     M6b cross-check (bridge-side cross-check item).
// -----------------------------------------------------------------------------
#pragma once
#include "pcie_cfg_space.h"

#include <systemc>
#include <cstdint>
#include <functional>

namespace pcie_tlm {

// State encoding (aligned with LinkSts[1:0]/[3]/[4])
enum class LinkPm : uint8_t { L0 = 0, L0S = 1, L1 = 2, L11 = 3, L12 = 4 };

struct LinkPmModule : sc_core::sc_module {
    sc_core::sc_in<bool> clk;
    CfgSpace* end_a = nullptr;                     // one end (switch port / device side; order does not matter)
    CfgSpace* end_b = nullptr;                     // the other end
    std::function<uint64_t()> activity;            // link activity monotonic counter (change => traffic present)
    int  idle_l0s = 2, idle_l1 = 16;               // idle thresholds (cycles)
    uint64_t  idle = 0, last_act = 0;
    LinkPm    state = LinkPm::L0;
    uint64_t  l0s_entries = 0, l1_entries = 0, l1_1_entries = 0, l1_2_entries = 0, exits = 0;
    uint64_t  blocked_partial = 0;                 // single-end enable => no entry (explicit accounting: one per missed entry)
    uint64_t  wake_exits = 0;                      // * T50-3: number of WAKE# forced wakeups (back to L0)
    // * T50-3: sideband WAKE# asserted => **immediately back to L0** (wakeup; NEVER waits for the idle threshold)
    void wake() {
        if (state != LinkPm::L0) { state = LinkPm::L0; ++exits; }
        ++wake_exits;
        idle = 0;
    }
    bool      ignore_one_side = false;             // testbench red mode: ignore the "both ends enabled" requirement

    SC_HAS_PROCESS(LinkPmModule);
    LinkPmModule(sc_core::sc_module_name nm) : sc_core::sc_module(nm), clk("clk") {
        SC_THREAD(run);
        sensitive << clk.pos();
    }
    // testbench criterion: state name
    static const char* name(LinkPm s) {
        switch (s) {
        case LinkPm::L0:  return "L0";
        case LinkPm::L0S: return "L0s";
        case LinkPm::L1:  return "L1";
        case LinkPm::L11: return "L1.1";
        default:          return "L1.2";
        }
    }
private:
    void run();
};

} // namespace pcie_tlm
