// -----------------------------------------------------------------------------
// Solution One - NTB function block (**declarations**), P2 -- non-transparent bridge
//   (translation windows / doorbell / scratchpad)
//   Source of the conventions: old line `tb_pcie_ntb` + `core/pcie_ntb.h`, ported
//   **item by item after reading**; WARNING the four pitfalls of the old line are **not
//   copied** here (confirmed by reading):
//     (1) the old line had a **one-way** A->B table (no ingress/egress pair) => * **B4-a
//         (2026-10-16) added**: each window = **a bidirectional window pair** {A side
//         [a_base, a_base+size) <=> B side [b_base, b_base+size)} --
//         **ingress** (src = -1, A/host side initiates) hits **a_base** => translate into
//         B-side mem (original path);
//         **egress** (src >= 0, B-side device initiates) hits **b_base** => **reverse
//         translate** A = a_base + (off - b_base) => **send upstream to A** (out.src = -1;
//         NEVER store into mem here -- real NTB semantics: pass through to the peer domain);
//         both offsets are relative to this block's `base` (model convention: both base
//         addresses are assigned by the platform/enumeration => only offsets are computed
//         inside the block);
//         doorbell / scratchpad are **shared by both directions** (the B side can also ring the
//         doorbell / read the scratchpad; event counting is global).
//     (2) the old doorbell was **not readable** => here: **readback = event count** (added);
//     (3) the old line had no interrupt => here still **count only + expose the line**
//         (interrupt = on demand);
//     (4) the old line **silently dropped** outside the window => here: **read => UR Cpl**
//         (NEVER silent) / write => drop + count;
//   Address convention (offset **relative to the NTB window base** `sw.ntb_base`; the old line
//   used absolute A addresses, here the base address is left to enumeration => offsets are used):
//     - offset **0x0000** = **doorbell** (write => count +1; read => return count)
//     - offset **[0x0200, 0x0220)** = **scratchpad 8 x 4B** (not translated; unwritten = 0)
//     - anything else (window hit) => **translate** B = b_base + (off - a_base) => store in
//       B-side memory (4096B)
//   Windows: x4 (`WIN_MAX`), **reverse-order hit = later configured wins** (old convention);
//   priority doorbell > scratchpad > window.
//   WARNING this block **does not parse anything beyond TLP semantics**: posted writes get no
//   response; read => CplD 4 bytes (NEVER the whole frame).
//   * Implementation = pcie_ntb.cpp.
// -----------------------------------------------------------------------------
#pragma once
#include "pcie_route.h"      // rt::* / PrivTlp (via pcie_sc_wire)
#include "pcie_sc_wire.h"

#include <cstdint>
#include <systemc>
#include <vector>

namespace pcie_tlm {

// NTB message = TLP + **source port** (-1 = UP/host side => response goes upstream; 0..N-1 => back to that DN port)
struct NtbMsg {
    int     src = -1;
    PrivTlp t;
};

// WARNING sc_fifo<T>'s print() requires operator<< (instantiated even if not printed => mandatory)
std::ostream& operator<<(std::ostream& os, const NtbMsg& m);

struct NtbModule : sc_core::sc_module {
    static const int    WIN_MAX = 4;
    static const size_t MEM_BYTES = 4096;
    static const int    SP_MAX = 8;
    static const uint64_t DOORBELL_OFF = 0x0000;
    static const uint64_t SP_OFF_BASE  = 0x0200;
    static const uint64_t SP_OFF_LIMIT = 0x0220;

    sc_core::sc_fifo_in<NtbMsg>  in;
    sc_core::sc_fifo_out<NtbMsg> out;

    struct Win { uint64_t a_base = 0, size = 0, b_base = 0; bool en = false; };
    Win      win[WIN_MAX];
    bool     enable = false;            // enabled by the 0xE8 register (or the testbench)
    uint64_t base = 0;                  // NTB window base (= mirror of sw.ntb_base; used to compute offsets inside the block)

    uint64_t events = 0;                // doorbell event count (reading the doorbell => returns this value)
    uint32_t sp[SP_MAX] = {0};          // scratchpad (unwritten = 0)
    std::vector<uint8_t> mem;           // B-side memory (translation destination)

    // Statistics (testbench criteria; NEVER silent)
    uint64_t win_rd = 0, win_wr = 0, doorbell_wr = 0, sp_rd = 0, sp_wr = 0,
             b_oob = 0, miss_wr = 0, miss_rd_ur = 0, doorbell_rd = 0;
    // -- * B4-a: egress (B-side) statistics (kept separate from ingress => the testbench can tell direction) --
    uint64_t win_rd_rev = 0, win_wr_rev = 0, rev_fwd = 0;
    uint64_t rev_last_addr = 0;        // A-side address after the most recent egress reverse translation (testbench criterion)

    SC_HAS_PROCESS(NtbModule);
    NtbModule(sc_core::sc_module_name nm);

    void set_window(int i, uint64_t a_base, uint64_t size, uint64_t b_base);   // testbench/platform configuration
    void clear_windows() { for (int i = 0; i < WIN_MAX; ++i) win[i] = Win{}; }

private:
    void run();
    void respond(int src, uint8_t status, const PrivTlp& req, const std::vector<uint8_t>& data);
};

} // namespace pcie_tlm
