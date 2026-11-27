// -----------------------------------------------------------------------------
// Option 1 - SC skeleton - upstream side (**declarations**): RC + USP + crossbar (internal logic partition)
//   Design Sec. 2 topology: `RC - USP - [crossbar] - DSP0..(N-1) - EP0..(N-1)`;
//   Design Sec. 1 red line 2: the implementation may be a merged "upstream side" block, but (1) internal logic is partitioned
//     (2) config space sits at each level (RC has no Type0 / USP Type1 / DN Type1+dev) (3) enumeration walks level by level (filled in M2).
//   * This unit = M1 skeleton: transaction flow + window checks + routing decision + counters (NEVER silent);
//     config space / enumerator / MSI = M2/M4 (hook points left inline, placeholders marked).
// -----------------------------------------------------------------------------
#pragma once
#include "pcie_cfg_space.h"
#include "pcie_ntb.h"        // * P2: NTB functional block (message = TLP + source port)
#include "pcie_route.h"
#include "pcie_sc_wire.h"

#include <systemc>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <vector>

namespace pcie_tlm {

// -- RC interrupt receive (M4): IRQ table entry (the table is registered by the **enumerator** when allocating IRQ numbers) ----
//   vector = low 5 bits of the message (data[4:0]; per spec MSI data layout: bit4 = 32-vector mode, low 4 bits = vector number)
//   data   = **full value** of the data register for this entry (low bits = vector; written value must match => teeth for "wrong vector")
struct IrqEntry {
    uint16_t reqid = 0;                    // Requester BDF (EP)
    uint16_t vector = 0;                   // Vector number (low 5 bits significant)
    uint16_t data = 0;                     // Expected data register value (low 5 bits = vector)
    uint64_t addr = 0;                     // Expected MSI address (entry self-address)
    uint8_t  line = 0;                     // Allocated IRQ number (enumeration output)
    uint64_t hits = 0;                     // Hit count
};

// -- RC: ECAM master + enumerator + MSI receive (M1: task-level issue + return counts) ------------
struct RcModule : sc_core::sc_module {
    sc_core::sc_fifo_in<PrivTlp>  in;      // <- USP (return path: Cpl/MSI/errors)
    sc_core::sc_fifo_out<PrivTlp> out;     // -> USP
    // Identity/register face (used by enumerator M2; preset in M1)
    uint8_t  bus = 0, dev = 0, fn = 0;
    uint64_t ecam_base = 0, ecam_size = 0;         // M2: ECAM window
    // -- Interrupt receive (M4): MSI address window + IRQ table (table is NEVER hard-coded - filled by enumerator register_irq) --
    uint64_t irq_base = 0, irq_size = 0;           // MSI address window (allocated by enumeration; 0 => not configured)
    std::vector<IrqEntry>  irq;
    uint64_t irq_hits = 0, irq_miss = 0, irq_verr = 0;   // hits / address or vector mismatch / vector matches but data is wrong
    uint64_t poisoned_rx = 0, silent_late = 0;           // M4 negative: received poisoned request / Cpl arriving **after timeout**
    // -- * M6b-2d (D8): upstream **posted MemWr** bookkeeping (EP->root direction; model has no host memory => record only) ------
    //   Purpose: real-DUT testbench proves the >4B payload chain "EP TX -> swdn RX -> TLP -> TLM" (same direction as the user's vdk case);
    //   WARNING: only counts/records values, does **not** change existing semantics (previously such TLPs were ignored).
    uint64_t up_memwr_rx = 0;
    uint64_t up_memwr_addr_last = 0;
    uint16_t up_memwr_reqid_last = 0;
    uint64_t up_memwr_dwlen_last = 0;
    uint8_t  up_memwr_fbe_last = 0, up_memwr_lbe_last = 0;
    std::vector<uint8_t> up_memwr_data_last;
    // -- Route B B2: upstream non-posted **MemRd** (endpoint -> host) => model host memory => CplD --
    //   The RC plays the host: it answers the read with a completion carrying the
    //   host-memory dword(s).  Default read data is `host_rd_data`; per-address
    //   overrides go in `host_mem` (tests preload known values).
    uint64_t up_memrd_rx = 0, up_memrd_cpl_tx = 0, up_memrd_ur_tx = 0;
    uint64_t up_memrd_addr_last = 0;
    uint16_t up_memrd_reqid_last = 0;
    uint8_t  up_memrd_tag_last = 0;
    uint64_t up_memrd_dwlen_last = 0;
    std::vector<uint8_t> up_memrd_data_last;       // bytes of the last CplD (empty for UR)
    uint32_t host_rd_data = 0xCAFEBABEu;
    std::map<uint64_t, uint32_t> host_mem;         // per-dword defaults
    // Host memory backing store: posted MemWr payload bytes are kept here and
    // read back by upstream MemRd.  Window [host_lo, host_hi) gates the
    // response: outside it the RC returns UR (host_hi <= host_lo => all backed).
    std::map<uint64_t, uint8_t> host_bytes;
    uint64_t host_lo = 0, host_hi = 0;
    uint64_t up_memwr_stored = 0;                  // posted writes stored to host memory
    uint64_t host_wr_sig = 1469598103934665603ULL; // byte/addr signatures for tests
    uint64_t host_rd_sig = 1469598103934665603ULL;
    bool host_in_window(uint64_t a) const {
        return (host_hi <= host_lo) || (a >= host_lo && a < host_hi);
    }
    uint32_t host_read(uint64_t a) const;
    uint8_t  host_byte(uint64_t a) const;
    // -- B8: upstream AtomicOp (endpoint -> host) => execute on the host store ---
    //   FetchAdd/Swap: Length 1|2 DW => 4|8-byte value (payload = operand).
    //   CAS: Length 2|4 DW => value = half the payload; payload = {compare | swap}.
    //   Completion carries the ORIGINAL value (spec); outside the window => UR.
    uint64_t up_atom_rx = 0, up_atom_fadd = 0, up_atom_swap = 0, up_atom_cas = 0;
    uint64_t up_atom_cas_hit = 0, up_atom_ur = 0, up_atom_bad = 0, up_atom_cpl_tx = 0;
    uint64_t up_atom_poison = 0;           // poisoned AtomicOp => UR (never executed)
    uint64_t up_atom_addr_last = 0;
    uint16_t up_atom_reqid_last = 0, up_atom_tag_last = 0;
    uint8_t  up_atom_type_last = 0, up_atom_cpl_status_last = 0xFF;
    std::vector<uint8_t> up_atom_old_last, up_atom_new_last;
    // Red-mode fault injection for the B8 gate (testbench only; 0 = normal):
    //   1 = ignore atomics (no execute, no completion)
    //   2 = completion returns the NEW value instead of the original
    //   3 = CAS writes the swap value even when the compare fails
    int atom_fault = 0;
    // -- * T49: Message face (root side) ------------------------------------------
    //   Send: root-port broadcast class (PME_Turn_Off r=011 etc.); receive: to-root class (r=000/101) => count + record by code.
    //   WARNING: semantics (PME_TO_Ack missing responses / ERR_* classification) in T49-2/T49-3 build on this counting face.
    void     submit_msg(uint8_t mtype, uint16_t code);
    uint64_t msg_rx = 0;                                 // Number of Messages received
    std::vector<uint64_t> msg_code_rx = std::vector<uint64_t>(256, 0);  // By message code (* T55: 0..255 - VDM codes 0x7E/0x7F are tabled)
    // -- C11-b follow-up: **ATS invalidate request/completion (RC side)** -----
    //   `send_ats_invalidate` routes by ID to the target EP (msg_dest); a
    //   completion returns to the root and is counted.  `ats_inv_sweep`
    //   accounts missing completions explicitly (never silently ignored).
    void     send_ats_invalidate(uint16_t dest_reqid = 0);
    uint64_t ats_inv_tx = 0, ats_inv_cpl_rx = 0, ats_inv_pending = 0;
    uint64_t ats_inv_timeout = 0;
    uint64_t ats_inv_sweep(uint64_t expected) {
        const uint64_t miss = (expected > ats_inv_cpl_rx)
                                  ? (expected - ats_inv_cpl_rx) : 0;
        ats_inv_timeout += miss;
        return miss;
    }
    // -- * T49-2: root-port **AER receive** (Root Error Command/Status; spec semantics) --------------
    //   - Status ([0] ERR_COR_RCVD [1] ERR_NONFATAL_RCVD [2] ERR_FATAL_RCVD) is **always set** on receive;
    //   - **Error Source ID** = **first-error** capture (recorded only on the first occurrence per class; RO reg => written by the first error);
    //   - Root Error **Command** ([0]/[1]/[2]) = enable => an interrupt is **generated** when set (counted).
    uint16_t root_err_cmd = 0;                           // Root-port reporting enable (interrupt face)
    uint32_t root_err_status = 0;                        // Received bits ([0]/[1]/[2]) + [5] first uncorrectable fatal
    uint16_t err_src_corr = 0, err_src_uncorr = 0;       // First error source ID (first occurrence per class)
    uint64_t err_cor_rx = 0, err_nonfatal_rx = 0, err_fatal_rx = 0;
    uint64_t root_intr_cor = 0, root_intr_nonfatal = 0, root_intr_fatal = 0;
    uint64_t err_other_rx = 0;                           // To-root class messages that are not ERR_* (never silent)
    // -- * T55 (C6 VDM): vendor-defined message receive (codes = read from RTL 0x7E/0x7F; T0 = r000 to root / T1 = r010 by ID) --
    uint64_t vdm_t0_rx = 0, vdm_t1_rx = 0, vdm_badcode = 0;
    // * T56 (C10): LTR / OBFF to root (hop-by-hop relay terminus; NEVER silent => count + record value)
    uint64_t ltr_rx = 0, obff_rx = 0;
    uint32_t ltr_last = 0; uint16_t ltr_src_last = 0; uint8_t obff_state = 0;
    // -- * T49-5: root-port **INTx receive** (r=100, **folded by the upstream port and then re-sent**) ------------------------
    //   Model semantics: the root port tracks levels for **4 virtual lines** (OR of all source ports; single upstream port => equivalent to the spec's "folding");
    //   duplicate assert / deassert without assert = **no-op** (spec: not an error) => counted separately.
    uint8_t  intx_line_state = 0;                        // Level of 4 lines (bit = line number)
    uint64_t intx_assert_rx = 0, intx_deassert_rx = 0;
    uint64_t intx_noop = 0;                              // Duplicate/dangling => no-op (NEVER silent)
    uint16_t intx_src_last = 0;                          // Last source (model = upstream-port BDF)
    // -- * T50-2: **ECRC** (root side; gates = root-port AER CapCtl[0] generate / [1] check) ------------------
    uint16_t aer_cctl_rc = 0;
    bool     ecrc_reflected = false;                     // * T51-2: root-side reflected switch (per-function semantics)
    uint64_t ecrc_gen_cnt = 0, ecrc_check_ok = 0, ecrc_check_bad = 0;
    bool     ecrc_corrupt = false;                       // Testbench use: stamp a bad value (teeth)
    uint32_t ecrc_last = 0;                              // Last generated value (testbench check)
    // -- * T49-3: power management (root side) ----------------------------------------------
    uint64_t pme_rx = 0;                                 // Received PM_PME (0x18)
    uint16_t pme_src = 0;                                // First PME source (testbench check)
    uint64_t to_ack_rx = 0;                              // Received PME_TO_Ack (0x1B)
    uint16_t to_ack_src_last = 0;                        // Last acknowledging source
    uint64_t to_ack_missing = 0;                         // Missing-response ledger (accumulated by sweep)
    // Testbench/software timeout semantics: **TLM does not do timeouts** => caller provides expected and actual counts => shortfall goes to the ledger
    uint64_t to_ack_sweep(uint64_t expected) {
        const uint64_t miss = (expected > to_ack_rx) ? (expected - to_ack_rx) : 0;
        to_ack_missing += miss;
        return miss;
    }
    void register_irq(const IrqEntry& e) { irq.push_back(e); }
    // -- * T53 (C2): **CRS retry for config requests** (request side = "software" role; bounded + backoff + NEVER silent) ------
    //   Spec (PCIe Base Sec. 2.3.2): software receiving a **CRS** completion => **must retry** that config request (until non-CRS).
    //   Model semantics (documented): **bounded** retry `crs_max_retry` (default 8) + **exponential backoff** (nominal 10ns initial,
    //   capped at 8x; total backoff booked in `crs_backoff_ns`) - NEVER degenerates into an infinite loop; retries exhausted => **UR + ledger**.
    //   * Whether CRS is honored = `crs_honor` (default **true**; red mode false = return CRS as final state => missing tree entries).
    //   WARNING: applies only to **config transactions** (cfg_x); data-plane `xact` receiving CRS just returns it as status (spec: CRS is config-only).
    bool     crs_honor = true;                     // Testbench red mode: false = do not honor CRS (treat as final)
    int      crs_max_retry = 8;                    // Bounded retry limit
    uint64_t crs_rx = 0;                           // Number of CRS completions received
    uint64_t crs_retries = 0;                      // Number of actual retries
    uint64_t crs_retry_ok = 0;                     // Transactions that succeeded after retry
    uint64_t crs_exhausted = 0;                    // Retries exhausted => UR (ledger)
    uint64_t crs_ignored = 0;                      // Red mode: CRS not honored, returned as final state
    uint64_t crs_backoff_ns = 0;                   // Cumulative backoff (ns; nominal value booked)
    // -- * T53 (C13): malformed receive-side checks (root side; **root-port MPS defaults to 4096**) --------------
    uint64_t mlf_drops = 0;
    uint64_t mlf_by_code[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint64_t mlf_last_code = 0;
    bool     ignore_mlf = false;                   // Testbench red mode: checks disabled (silently pass through)
    int      mlf_pad_dws = 8;                      // Wire-level beat padding tolerance (DW; default covers a 256-bit data beat)
    uint32_t rc_mps_bytes = 4096;                  // Root-port MPS (RC has NO DevCtl => nominal value)
    // Statistics (testbench checks)
    uint64_t sent = 0, rcvd = 0, cpl_ok = 0, cpl_bad = 0, cfg_ur_syn = 0;
    uint8_t  next_tag = 0;
    // -- * M6b (2026-10-18): configurable transaction timeout (real DUT link round-trip >> 200ns) --------------
    //   Default = 200ns => existing testbenches (TB-12/TB-15/TB-16/TB-17 etc.) **zero regression**;
    //   real-DUT testbenches set it larger as needed (e.g. 50us). cfg_x and mem_read each use their own.
    sc_core::sc_time cfg_timeout = sc_core::sc_time(200, sc_core::SC_NS);
    sc_core::sc_time io_timeout  = sc_core::sc_time(200, sc_core::SC_NS);

    SC_HAS_PROCESS(RcModule);
    RcModule(sc_core::sc_module_name nm);

    // Task-level API (T1)
    void submit(const PrivTlp& t);
    void submit_memwr(uint64_t addr, const std::vector<uint8_t>& data, uint8_t be = 0);

    // -- cfg transaction API (**blocking**; for enumerator/SC thread; single outstanding = M2 semantics) --
    bool cfg_read(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t reg, uint32_t& dout, uint8_t& status);
    bool cfg_write(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t reg, uint32_t val, uint8_t& status);
    // -- Data plane (M3): MemRd (blocking; receives CplD data) / MemWr (non-blocking; posted) ----------
    bool mem_read(uint64_t addr, size_t nbytes, std::vector<uint8_t>& data, uint8_t& status);
    void submit_memrd(uint64_t addr, size_t nbytes);
    // Generic transaction (**testbench use**: hand-built TLP => blocking Cpl receive; timeout => UR; adjustable timeout = negative timeout mode)
    bool xact(PrivTlp& t, uint32_t& dout, std::vector<uint8_t>& data, uint8_t& status,
              const sc_core::sc_time& timeout = sc_core::sc_time(200, sc_core::SC_NS));

private:
    struct RcPend { uint64_t tag = 0; bool done = false; uint8_t status = 0xFF; uint32_t dout = 0;
                    std::vector<uint8_t> data; sc_core::sc_event ev; };
    std::deque<PrivTlp> q_;
    sc_core::sc_event   ev_;
    std::vector<std::unique_ptr<RcPend>> pend_;
    int  io_recv(const PrivTlp& t);            // MSI message receive (1 = handled as interrupt; 0 = not MSI)
    bool cfg_x(bool is_wr, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t reg, uint32_t val,
               uint32_t& dout, uint8_t& status);
    void tx_run();
    void rx_run();
    static uint8_t aligned_be(size_t nbytes);
};

// -- USP: upstream port (Type1 bridge header; written by M2 enumeration, preset in M1) ------------------------
struct UspModule : sc_core::sc_module {
    // -- MSI interface face (T-2; databook **Sec. 4.12** => **only the upstream port** has this interface) --------------
    //   Spec (p426): synchronous req/ack handshake; `ven_msi_req` **once asserted must be held until grant**;
    //   `ven_msi_grant` = **one-cycle pulse**; req held => send another MSI; the controller generates the corresponding MWr.
    //   Core-side acceptance condition = per **customer RTL read** (`pig_msg_formation.sv`):
    //     `(msi_req_reg && msi_req) & !busy & !msi_grant & !halt` (:556/:582 branches)
    //     => **req must be held for two consecutive cycles**; `msi_grant <= latchd_msi_req & !halt` (:543) => one-cycle pulse;
    //     WARNING: this path has **no enable gate** - `cfg_msi_en` is the **truth value given to the application** (RTL: `pig_cdm_msi_reg.sv:142`
    //      = Message Control.Enable; databook Figure 4-41: the application requests only when cfg_msi_en is valid)
    //      => the "enable gate" is the **application's** responsibility (same in this model's TB side; checks come in pairs).
    sc_core::sc_in<bool> clk;                  // Private-port clock domain (pin-level handshake = **synchronous**)
    bool     msi_req = false;                  // ven_msi_req (application -> core)
    uint64_t msi_func_num = 0;                 // ven_msi_func_num (selects the MSI register set by function)
    uint8_t  msi_tc = 0;                       // ven_msi_tc[2:0]
    uint8_t  msi_vector = 0;                   // ven_msi_vector[4:0]
    bool     msi_grant = false;                // ven_msi_grant (core -> application; **one-cycle pulse**)
    uint64_t cfg_msi_en = 0;                   // cfg_msi_en[CX_MSI_NFUNC-1:0] (MSI Enable truth value)
    bool     msi_halt = false;                 // Internal halt (CDM msg-gen halt semantics; NOT a top-level port)
    // Statistics (testbench checks; NEVER silent)
    uint64_t msi_if_sent = 0;                  // MSI issued on the interface (=> posted MWr upstream to RC)
    uint64_t msi_if_abort = 0;                 // **not held until grant** => drop count (spec violation, ledger)
    uint64_t msi_if_badfn = 0;                 // func_num out of range (>= CX_MSI_NFUNC) => ledger (no grant)

    uint64_t ntb_base = 0, ntb_size = 0;       // * P2: NTB window (host-reachable => passes window check)
    sc_core::sc_fifo_in<PrivTlp>  north_in;    // <- RC
    sc_core::sc_fifo_out<PrivTlp> south_out;   // -> crossbar
    sc_core::sc_fifo_in<PrivTlp>  south_in;    // <- crossbar (upstream)
    sc_core::sc_fifo_out<PrivTlp> north_out;   // -> RC
    std::vector<std::unique_ptr<sc_core::sc_fifo_in<DbiUp>>> dbi_in;   // One per DN port (DBI return path)
    // Type-1 bridge header (M2 = this unit's config space; enumeration writes => side effects update the fields below)
    CfgSpace usp_cfg;
    uint8_t  pbus = 0, sec = 0, sub = 0;
    // Identity used to answer this bridge's own configuration requests.
    // Root upstream port = bus 0 / device 0 (default); a cascaded switch sets
    // its own bus/device before enumeration.
    uint8_t  own_bus = 0, own_dev = 0;
    void set_identity(uint8_t b, uint8_t d) { own_bus = b; own_dev = d; }
    uint64_t mem_base = 0, mem_size = 0;       // This bridge's memory window (NOT covered => drop + M4 reports UR)
    uint64_t pref_base = 0, pref_size = 0;     // This bridge's **prefetch window** (B-1; 64-bit form => carries the >=4G range)
    uint64_t mc_base = 0, mc_stride = 0, bc_base = 0, bc_size = 0;   // private MC/BC address face
    // Standard PCIe Multicast window (ECAP 0x0012) learned from usp_cfg.
    bool     mc_std_enable = false;
    uint64_t mc_std_base = 0;
    uint8_t  mc_std_index = 0;
    uint16_t mc_std_groups = 1;
    // -- Outside window (M4): drop => dropping generates UR back to RC (NEVER silent); `drop_silent` = testbench red mode (drop without reply) --
    bool     drop_silent = false;
    uint64_t drop_ur = 0;                      // Number of URs generated (within drop_owin)
    // Statistics (testbench checks)
    uint64_t fwd_down = 0, drop_owin = 0, fwd_up = 0, dbi_rcvd = 0, cfg_self = 0, cpl_made = 0;
    std::function<void(uint16_t reg, uint32_t val)> mcbc_on_write;   // private MC/BC registers
    std::function<void(uint16_t reg, uint32_t val)> mc_std_on_write; // standard MC ECAP (0x3E0)
    std::function<void(uint16_t reg, uint32_t val)> vc_on_write;     // * P2: VC registers (0xE0/0xE4) => crossbar
    std::function<void(uint16_t reg, uint32_t val)> vc_std_on_write; // standard VC ECAP (0x200) => crossbar
    std::vector<DbiUp> dbi_log;                // DBI return path (with Cpl context)

    SC_HAS_PROCESS(UspModule);
    UspModule(sc_core::sc_module_name nm, int n_dbi);

    // MSI function count of this port (CX_MSI_NFUNC; multi-function requests are arbitrated internally => single function passes through)
    int msi_nfunc() const { return cfg_msi_nfunc_; }
    void cfg_msi_nfunc_set(int n) { cfg_msi_nfunc_ = n; }

private:
    void south_run();
    void north_run();
    void dbi_run();                            // Single-thread polling of each port + event or-table (avoids sc_spawn)
    void msi_run();                            // * T-2: MSI interface handshake (synchronous; must bind clk)
    void msi_emit();                           // Controller generates the corresponding MWr (Sec. 4.12; field semantics per RTL)
    bool handle_cfg(const PrivTlp& t);         // Self-answered by this bridge (Type1) => Cpl back to RC
    int  cfg_msi_nfunc_ = 1;                   // = CoreCfg.CX_MSI_NFUNC (set from cfg after owner construction)
};

// -- crossbar: five-class routing decision (SC assembly of pcie_route.h; includes full MC/BC/P2P matrix) ------
struct CrossbarModule : sc_core::sc_module {
    SwitchCfg sw;                                  // TB preset (written by the enumerator starting M2)
    sc_core::sc_fifo_in<PrivTlp>  up_in;           // <- USP (downstream)
    sc_core::sc_fifo_out<PrivTlp> up_out;          // -> USP (upstream)
    std::vector<std::unique_ptr<sc_core::sc_fifo_in<PrivTlp>>>  dn_in;
    std::vector<std::unique_ptr<sc_core::sc_fifo_out<PrivTlp>>> dn_out;
    // Statistics (NEVER silent: no receiver => count; UR error = M4)
    uint64_t routed = 0, unrouted_up = 0, unrouted_dn = 0;
    // -- * B8-a (2026-10-16): **WAKE# / CLKREQ sideband aggregation** (port -> fabric -> root; contract NEVER changed) ------
    //   Semantics per joint cosim plan Sec. 8.7.2 **sideband option**: physical pins are outside the contract (top `wake`/`clkreq_in_n`
    //   = the 298-item ledger PM bucket) => carried inside the model by a **mask aggregation channel**: bit i = port i asserted (active low);
    //   aggregated result = what the root sees (`sb_wake_n()` high only when all 1). Bridge-side pin wiring = one of the two M6b options.
    uint32_t sb_wake_mask = 0, sb_clkreq_mask = 0;      // 1 = that port has pulled low (asserted)
    uint64_t sb_wake_events = 0, sb_clkreq_events = 0;
    bool sb_wake_n()   const { return sb_wake_mask == 0; }     // Active low: any assertion => 0
    uint32_t sb_sysint_mask = 0;                        // * C3-a: system interrupt summary (bit i = port i active)
    uint64_t sb_sysint_events = 0;
    bool sb_sys_int_n() const { return sb_sysint_mask == 0; }   // What the root sees (active low)
    bool sb_clkreq_n() const { return sb_clkreq_mask == 0; }
    // -- * P2: per-VC egress (one queue per DN port x per VC + arbitration mode => dequeue order = check source) ----
    //   WARNING: queue full => **drop + per-port count** (NEVER silent; same semantics as the old line: drops are attributed to the **source port**,
    //      this line counts by **egress port** - documented difference); `qmax` = **historical peak** of the sum of all VC occupancies on that port (old-line semantics)
    std::vector<std::vector<std::deque<PrivTlp>>> eq;      // [port][vc]
    std::vector<uint64_t> eq_drop;                         // Dropped frames per port
    std::vector<std::vector<uint64_t>> vc_tx;              // Frames sent per port per VC (testbench checks)
    std::vector<uint64_t> qmax;                            // Peak occupancy per port
    std::vector<uint8_t>  rr_ptr;                          // Round-robin pointer (per port)
    // * T48: WRR (mode 2) credits - per port x per VC; weight change/testbench hook => reset to the weight
    std::vector<std::vector<int>> wrr_cred;
    std::vector<uint64_t> wrr_starved;                     // VCs with weight 0 are skipped (NEVER silent: ledger)
    void wrr_reset() {
        for (size_t i = 0; i < wrr_cred.size(); ++i)
            for (int v = 0; v < 8; ++v) wrr_cred[i][(size_t)v] = (int)sw.vc_weight[v];
    }
    std::vector<std::vector<uint8_t>> drain_log;           // Dequeue order (records VC; testbench checks)
    uint64_t vc_enq = 0;                                   // Total enqueued
    // * Round 60 (4-port broadcast checks): **frames enqueued per egress port** (= frames actually copied to that port)
    uint64_t egress_frames = 0;
    std::vector<uint64_t> egress_port_cnt;                 // [port]; zeroed by ctor per nport
    // * Round 60: per-port count of **MC/BC frames** among them (check = broadcast copy; D6 normal-window traffic not mixed in)
    uint64_t bc_mc_copies = 0;
    std::vector<uint64_t> bc_mc_port_cnt;                  // [port]; zeroed by ctor per nport
    sc_core::sc_event ev_egress;
    // -- * T54 (C11-a): decision source for **ACS P2P redirect** (= ACS control bits of each DN port's core-side Type1) --------
    //   Wired by the host (testbench/Top: `xb.dn_cfg[i] = &stubs[i]->port_cfg`) - same way as `route_port`;
    //   not wired (NULL) => treated as not enabled => **zero regression**. Counter `acs_redirect` = ledger (NEVER silent).
    std::vector<CfgSpace*> dn_cfg;
    uint64_t acs_redirect = 0;
    bool     ignore_acs = false;      // Testbench red mode: NEVER honor ACS P2P redirect (discriminative)
    bool acs_r_get(int i) const {
        return i >= 0 && i < (int)dn_cfg.size() && dn_cfg[(size_t)i] && dn_cfg[(size_t)i]->acs_r();
    }
    bool acs_c_get(int i) const {
        return i >= 0 && i < (int)dn_cfg.size() && dn_cfg[(size_t)i] && dn_cfg[(size_t)i]->acs_c();
    }
    // -- * T53 (C4): **egress ordering layer** (RO/IDO detailed semantics; per spec + RTL `pig_radm_order_q.sv`) --
    //   Default = **arrival order preserved within the same VC** (single FIFO queue => dequeue order == enqueue order; gap = existing behavior, zero regression).
    //   Relaxation (meaningful ONLY when the **head is blocked**): head "blocked" = its **TLP type is halted at this port**
    //     (`dn_cls_halt[port]` bits: 0 = P / 1 = NP / 2 = CPL; per databook Sec. 4.5.2/4.5.3 halt semantics
    //      + PCIe **per-type** credit face => when one class has no credits that class holds while others may proceed) <-> RC-side egress FIFO full.
    //   - Pass rule (aligned with spec/RTL text; `tlp_ro/ido` in `pcie_route.h` = single source of truth for bit meaning):
    //     candidate C may pass a blocked TLP_t in the prefix iff **C.RO=1** (spec: RO = relax all ordering; WARNING: the spec prohibits it for
    //     CFG/IO/MSI/Message => such types with RO are **not passed and are logged**) or **C.IDO=1 and C.reqid != t.reqid**
    //     (IDO = relax only between **different Requester IDs**; same requester still ordered - IDO width follows
    //      `FLT_Q_ATTR_WIDTH` (this config = 2 => **no IDO bit**; verifiable on a synthesized 3-bit config testbench).
    //   - NEVER silent: passes/holds/invalid RO each counted; `egress_seq[port]` = dequeue tag order (check source).
    std::vector<uint8_t>  dn_cls_halt;                     // 3 bits per port (0=P 1=NP 2=CPL; model input)
    std::vector<std::vector<uint64_t>> egress_seq;         // Per-port dequeue **tag** order (testbench check)
    uint64_t ro_pass = 0, ido_pass = 0;                    // Pass counts (RO / IDO)
    uint64_t order_hold = 0;                               // Head blocked with no pass rule => hold (ledger)
    uint64_t ro_invalid = 0;                               // RO on a spec-prohibited type => not passed (ledger)
    bool     ignore_order_relax = false;                   // Testbench red mode: NEVER allow any pass ("no rules")
    int      attr_w = 2;                                   // FLT_Q_ATTR_WIDTH (>=3 for an IDO bit)
    void set_cls_halt(int port, int cls, bool h) {
        if (port < 0 || port >= (int)dn_cls_halt.size() || cls < 0 || cls > 2) return;
        if (h) dn_cls_halt[(size_t)port] |= (uint8_t)(1u << cls);
        else   dn_cls_halt[(size_t)port] &= (uint8_t)~(1u << cls);
    }
    // * P2: NTB functional block hook (NTB window hit => send here; response returns by **source port**: -1 = UP / i = DN i)
    sc_core::sc_fifo_out<NtbMsg> ntb_send;                 // -> NTB block (FIFO owned by the upstream side)
    sc_core::sc_fifo_in<NtbMsg>  ntb_resp;                 // <- NTB block (response)
    uint64_t ntb_routed = 0;                               // Messages sent to the NTB block (NEVER silent)

    SC_HAS_PROCESS(CrossbarModule);
    CrossbarModule(sc_core::sc_module_name nm, int n);

    sc_core::sc_fifo_in<PrivTlp>&  dn_in_port(int i);
    sc_core::sc_fifo_out<PrivTlp>& dn_out_port(int i);

private:
    void up_run();
    void dn_run();                             // Single-thread polling of each DN port + event or-table
    void route_from_dn(int i, const PrivTlp& t);
    void egress_enqueue(int port, const PrivTlp& t);   // * P2: enqueue by TC->VC (replaces direct writes to dn_out)
    void egress_run();                                 // * P2: dequeue per arbitration mode (one round per cycle)
    void ntb_run();                                    // * P2: NTB response collection + return by source port
};

// -- Upstream-side merged block (topology unit; DN-side interfaces = TB binds DSP ports) -----------------------
struct PcieScUpstream : sc_core::sc_module {
    RcModule        rc;
    UspModule       usp;
    CrossbarModule  xb;
    NtbModule       ntb;                      // * P2: NTB functional block (attached to crossbar; host-visible)

    // cfg = parameter source (MSI interface function count CX_MSI_NFUNC; **may be null** => keep defaults)
    PcieScUpstream(sc_core::sc_module_name nm, int n, const CoreCfg* cfg = nullptr);

    sc_core::sc_fifo<PrivTlp>&         dn_down(int i);   // xb -> port adapter (downstream)
    sc_core::sc_fifo<PrivTlp>&         dn_up(int i);     // port adapter -> xb (upstream)
    sc_core::sc_fifo<DbiUp>& dbi(int i);            // port adapter -> USP (DBI return path => generate Cpl)

private:
    sc_core::sc_fifo<PrivTlp> f_rc_usp, f_usp_xb, f_xb_usp, f_usp_rc;
    sc_core::sc_fifo<NtbMsg> f_ntb_send, f_ntb_resp;   // * P2: NTB <-> crossbar, two FIFOs
    std::vector<std::unique_ptr<sc_core::sc_fifo<PrivTlp>>>          dn_down_, dn_up_;
    std::vector<std::unique_ptr<sc_core::sc_fifo<DbiUp>>>  dbi_;
};

} // namespace pcie_tlm
