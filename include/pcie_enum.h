// -----------------------------------------------------------------------------
// Option 1 - Enumeration program (**declarations**) - lightweight BIOS (no-OS; requirement (4) core)
//   Semantics (design Sec. 4.5 + databook S.5.1):
//     (1) Scan devices starting at bus 0 (host-visible: bus0 -> dev0 = USP, dev i+1 = DN_i);
//     (2) Read Type1 => allocate **sec/sub bus** (bridge header write = side effect => routing);
//     (3) Probe Type0 per bus => **BAR sizing (write all 1s, read back)** => allocate from pool => write Command.MSE;
//     (4) Read MSI Cap (**allocate IRQ number** => write MsgAddr/MsgData);
//     (5) Program each DN port memory window (0x20/0x22 => routing window) + program the USP window to cover the whole pool;
//     (6) Produce the **device tree** (in-memory: {bus,dev,fn,VID/DID,class,BAR x k,irq,sec/sub}).
//   * Transaction face = RC blocking cfg API (direct Cfg TLP; NO ECAM) => this unit must be called from the SC thread.
//   * Testbench fault injection (red modes): missing device / BAR pool not advancing (overlap) / no sec programming / MSE not set.
//   * Implementation file = pcie_enum.cpp.
// -----------------------------------------------------------------------------
#pragma once
#include "pcie_sc_upstream.h"

#include <cstdint>
#include <string>
#include <vector>

namespace pcie_tlm {

// -- Device tree entry -------------------------------------------------------
struct EnumDev {
    uint8_t  bus = 0, dev = 0, fn = 0;
    int      port = -1;                     // Ordinal (0 = USP; 1..N = DN order; EP shares its DN) --
                                            // used for every "per-device-order" allocation (vector/IRQ numbers)
    uint16_t vid = 0, did = 0;
    uint8_t  rev = 0, prog_if = 0, sub_class = 0, base_class = 0;
    bool     is_bridge = false;
    uint8_t  sec = 0, sub = 0;
    std::vector<uint64_t> bar_base, bar_size;
    std::vector<bool>     bar_is64;         // 64-bit flag per BAR (G7: determined from bit2 read back during size probing)
    std::vector<uint64_t> bar_io_size;      // Detected **IO BAR** size (B-2: NEVER allocate; ledger only)
    uint8_t  irq_line = 0;                  // IRQ number allocated by the lightweight BIOS
    uint32_t msi_addr = 0; uint16_t msi_data = 0;
    uint64_t bar_span() const { return bar_base.empty() ? 0 : (bar_size[0] ? bar_size[0] : 1); }
};

struct DeviceTree {
    std::vector<EnumDev> devs;
    uint8_t     next_bus = 1;
    bool        ok = false;
    std::string log;
    uint64_t    mc_base = 0, mc_stride = 0, bc_base = 0, bc_size = 0;   // Multicast/broadcast allocation results
    uint32_t    mc_mask = 0, bc_mask = 0;
    uint64_t    ntb_base = 0, ntb_size = 0;                              // * P2: NTB window allocation results
    // * P2: SR-IOV allocation results (PF VF BAR window; VF entries = {bus,dev,fn!=0} in the tree)
    uint64_t    vf_bar_base = 0, vf_bar_size = 0; int vf_num = 0; bool ari = false;
    uint64_t    vf_msix_enrolled = 0;                                    // * T48-3: number of VFs enrolled for VF-MSIX
    const EnumDev* find(uint8_t bus, uint8_t dev, uint8_t fn = 0) const {
        for (size_t i = 0; i < devs.size(); ++i)
            if (devs[i].bus == bus && devs[i].dev == dev && devs[i].fn == fn) return &devs[i];
        return nullptr;
    }
};

// -- Testbench fault injection (red modes; all default = normal) --------------
struct EnumFaults {
    int  skip_dev = -1;         // Skip **DN index on the internal bus** (0-based; drop device)
    bool overlap_bars = false;  // BAR allocation does not advance the pool (=> overlap)
    bool skip_sec = false;      // Do not program sec/sub (=> devices below cannot be probed)
    bool skip_mse = false;      // Do not set Command.MSE
    bool skip_msi = false;      // Do not allocate IRQ numbers
    bool skip_mc = false;       // Do not allocate multicast/broadcast addresses
    bool only_bar0 = false;     // Red mode (G7): probe/program BAR0 only (multi-BAR checks must fire - discriminative)
    bool miss_hi = false;       // Red mode (G7): do not read the upper half of a 64-bit pair => is64 misjudged (discriminative)
    bool ignore_io_bit = false; // Red mode (B-2): treat an IO BAR as a memory BAR (counterexample to NEVER-silent - discriminative)
    bool skip_pref_win = false; // Red mode (B-1): do not program the **prefetch window** => >=4G addresses are always dropped (discriminative)
    bool skip_ntb = false;      // * P2: do not allocate the NTB window (red mode => no NTB visible to the host)
    bool skip_sriov = false;    // * P2: do not configure SR-IOV (VF Enable not set => VFs unreachable; red mode)
    bool ari_late = false;      // * T48: red mode - ARI Capable Hierarchy is set in reverse order (NumVFs clamped to 7)
    bool skip_vf_msix = false;  // * T48-3: red mode - do not program/enable VF-MSIX (=> VF messages gated; discriminative)
};

// -- Enumerator ---------------------------------------------------------------
struct Enumerator {
    uint64_t pool_base = 0x10000000ULL;     // BAR address pool (memory window; 32-bit range)
    uint64_t pool_size = 0x02000000ULL;
    // -- Prefetch pool (B-1): dedicated to **64-bit prefetchable BARs** (>=4G range => program the **prefetch window** in 64-bit form) --
    uint64_t pref_pool_base = 0x200000000ULL;
    uint64_t pref_pool_size = 0x04000000ULL;
    uint64_t mc_base = 0xF0000000ULL, mc_stride = 0x00100000ULL;   // MC window (group index = (addr-base)/stride)
    uint64_t bc_base = 0xF1000000ULL, bc_size = 0x00100000ULL;     // BC window
    // -- MSI address window (M4): vector = addr - msi_base (entry self-address; enumeration allocates + registers RC IRQ table) --
    uint64_t msi_base = 0xFE000000ULL, msi_size = 0x00080000ULL;
    int      nport = 2;                     // Number of DN ports
    EnumFaults faults;
    uint64_t msix_enrolled = 0;             // * P2: number of devices with MSI-X enrolled this round (testbench check; NEVER silent)

    DeviceTree run(RcModule& rc);           // Blocking (call from the SC thread)
    // Cascade mode: depth-first recursive enumeration.  Type1 devices below a
    // downstream port are treated as another switch/bridge and scanned
    // recursively; bus numbers are allocated depth-first and every bridge's
    // memory window is programmed to cover its subtree.  The flat run() above
    // is unchanged (zero regression).
    DeviceTree run_tree(RcModule& rc, int max_depth = 6);
    // * T-2: upstream port's own MSI enable + register in the RC IRQ table (optional BIOS policy; NOT the default => existing testbenches unchanged)
    //   databook Sec. 4.12: the MSI interface exists **only on the upstream port** => upstream-port interrupts must also be configured (reqid = bus0/dev0)
    bool enroll_upstream_msi(RcModule& rc);

private:
    // Single source of truth for MSI allocation (shared by EPs and the upstream port; NEVER duplicated)
    bool msi_enroll(RcModule& rc, uint8_t bus, uint8_t dev, uint8_t fn, uint8_t line,
                    uint32_t& addr_out, uint16_t& data_out);
    // * P2: MSI-X (entries = storage inside the BAR => program entry by entry with **memory writes**; program first, then Enable; also clear MSI Enable)
    //   returns false = device has no MSI-X cap (caller falls back to MSI; NEVER silently do nothing)
    bool msix_enroll(RcModule& rc, uint8_t bus, uint8_t dev, uint8_t fn, uint8_t line,
                     const std::vector<uint64_t>& bar_base, uint32_t& addr_out, uint16_t& data_out);
    // * P2: SR-IOV enrollment (PF => probe/allocate VF BAR + NumVFs + **VF Enable** + per-VF enumerate/program MSE)
    //   NO SR-IOV cap => false (never silent); `m_lo/m_hi` are also extended to the VF window (the DN window must cover it)
    bool sriov_enroll(RcModule& rc, uint8_t bus, uint8_t dev, int port, DeviceTree& dt,
                      uint64_t& m_lo, uint64_t& m_hi);
    // * T48-3: VF-MSIX enrollment is **deferred** (entries = **memory writes** => must wait until windows (6)/(7) are programmed - same reason as MSI-X)
    struct PendVfMsix {
        uint8_t bus = 0, dev = 0; int port = 0;
        int vf_off = 0, vf_str = 1, nv = 0;
        uint64_t vbase = 0, vsz = 0;
    };
    std::vector<PendVfMsix> pend_vf_;
    void vf_msix_enroll(RcModule& rc, const PendVfMsix& p, DeviceTree& dt);
    uint64_t pool_next_ = 0;
    uint64_t msix_vec_cursor_ = 0x100;      // * T48-3: VF-MSIX vector cursor (NEVER collides with PF/MSI entry addresses)
    uint64_t pref_pool_next_ = 0;           // Prefetch pool cursor (B-1)
    bool     any_pref_ = false;             // Whether a prefetchable BAR was allocated this round (=> program the USP prefetch window on demand)
    uint8_t  dn_base_ = 1;                  // Downstream bus base (= USP.sec; DN_i => sec = dn_base + i)
    uint64_t alloc(uint64_t size, uint64_t gran);
    uint64_t alloc_pref(uint64_t size);     // Prefetch pool allocation (>=4G range)
    // -- Cascade (tree) enumeration helpers ---------------------------------
    DeviceTree* dt_tree_ = nullptr;         // tree under construction (run_tree only)
    uint8_t  next_bus_tree_ = 1;            // next free bus number (depth-first)
    int      max_depth_tree_ = 6;
    // Scan one bus; returns the number of devices found and accumulates the
    // allocated BAR range in [lo, hi) (hi <= lo when nothing was allocated).
    int  scan_bus_tree(RcModule& rc, uint8_t bus, int depth, int port_hint,
                       uint64_t& lo, uint64_t& hi);
    // Probe/allocate one Type0 device behind a bridge.
    bool probe_type0_tree(RcModule& rc, uint8_t bus, uint8_t dev, int port_hint,
                          EnumDev& e, uint64_t& lo, uint64_t& hi);
};

} // namespace pcie_tlm
