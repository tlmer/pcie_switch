// -----------------------------------------------------------------------------
// Option 1 - SC skeleton - edge units (**declarations**): DSP port adapter / RTL stub / EP stub
//   Boundary contract = [`pcie_priv_port.h`](pcie_priv_port.h) (full set of core top application-side signals;
//   pack/unpack via [`PciePacker`](pcie_priv_port.h), width = parameter).
//     - DspPortModule = private-port TLM side: internal TLP => CoreTopSignals (client0 group + sideband)
//       => packed beat stream; reverse likewise. halt = high halt / active low (finalized).
//     - RtlStubModule = **RTL stub** (DSP IP + link; self-written peer during standalone runs)
//       - future co-sim (T4 bridge) replaces this with real RTL, **everything else unchanged**;
//     - EpStubModule  = **EP stub** (Type0 minimal set + BAR + storage; MemWr lands).
//   * Clock semantics (Sec. 6.3): the whole client group is **same domain** => both private-port sides advance one beat per core clock edge.
// -----------------------------------------------------------------------------
#pragma once
#include "pcie_cfg_space.h"
#include "pcie_route.h"      // rt::TYPE_* / PortCfg (enumeration side-effect landing point)
#include "pcie_sc_wire.h"

#include <systemc>
#include <cstdint>
#include <functional>
#include <unordered_set>
#include <vector>

namespace pcie_tlm {

struct CrossbarModule;   // * B8-a: sideband aggregation point (fabric; defined in pcie_sc_upstream.h)

struct RtlStubModule;    // Forward declaration (the port adapter holds a core-side truth pointer; definition below)

// -- DSP port adapter (design Sec. 4.4): private-port TLM side ------------------
struct DspPortModule : sc_core::sc_module {
    sc_core::sc_in<bool> clk;                   // Private-port clock domain
    sc_core::sc_fifo_in<PrivTlp>  south_in;     // <- crossbar (downstream TLP)
    sc_core::sc_fifo_out<PrivTlp> north_out;    // -> crossbar (upstream TLP)
    sc_core::sc_fifo_out<DbiUp> dbi_out;        // -> USP (DBI return path => generate Cpl)
    CoreCfg     cfg;                            // Core configuration of this port (width parameters)
    PciePacker  pk;                             // Packer (cfg driven)
    PrivWire*   wire = nullptr;
    RtlStubModule* rtl = nullptr;   // Core-side truth source (sideband takes core output: bus face; null => use app_bus_num preset)
    int     port_idx = 0;
    uint8_t app_bus_num = 1;    // Sideband cfg_pbus_num preset (source-port bus; RTL evidence UP=8'h1)
    uint8_t secondbus_num = 0;  // Sideband cfg_2ndbus_num (DN side = p2p context)
    // * M6b (2026-10-19): this port's `client0_req_id` (= contract client0_app_dev_id) -> real core
    //   `pig_DWC_pcie_core.sv:1502`; **unique non-zero per port** (user preset in mem2client0.v:144).
    //   NEVER give the host reqid (0) => the real core's client0 formation/completion LUT may not recognize it (D6 no-send suspect).
    uint16_t client0_id = 0;    // M6b: default 0 (internal testbench semantics); real-DUT TB presets via set_client0_id()
    // -- Real-DWC private port: `radm_trgt1_addr[1:0]` are the TLP header's
    //    **reserved** bits, not address bits (PCIe addresses are [63:2];
    //    `pcie_posted_monitor.sv` masks them the same way).  Set true only
    //    when this port is wired to the real core; the model-owned fake cores
    //    pass exact addresses and (MSI-X) use byte-granular vector addresses,
    //    so the default must stay false.
    bool radm_addr_reserved_low = false;
    // -- Testbench injection modes (FI_*; FI_NONE = normal) / ignore halt (for falsification)
    int  fault = FI_NONE;
    bool ignore_halt = false;
    // -- Port identity + enumeration side-effect landing (M2): mirror = application-side self-check copy (databook H.4 "application must self-check");
    //    truth is in the RTL-side core registers (stub.port_cfg below; read/write via DBI) => synchronize mirror + routing window on write-back
    struct PortMirror {
        uint8_t bus = 0, dev = 0, sec = 0, sub = 0;
        uint16_t mem_base = 0, mem_limit = 0;
        uint16_t pref_base = 0, pref_limit = 0;       // Prefetch window (B-1; lower half)
        uint32_t pref_up_base = 0, pref_up_limit = 0; // Prefetch window high 32 (64-bit form)
        uint32_t io_reg = 0, io_up = 0;               // * T52: IO window registers (0x1C/0x30) mirror
    };
    PortMirror  mirror;
    PortCfg*    route_port = nullptr;           // => crossbar sw.dn[i]
    void set_port(uint8_t bus, uint8_t dev) { mirror.bus = bus; mirror.dev = dev; }
    void set_client0_id(uint16_t id) { client0_id = id; }   // * M6b: per-port preset for real DUT (user preset)
    // -- Statistics (testbench checks)
    uint64_t tx_beats = 0, tx_frames = 0, rx_beats = 0, rx_frames = 0;
    uint64_t cfg_self = 0, cfg_below = 0;
    // -- * T-1b: per-TLP-type free-slot tracker (databook Sec. 4.5.3 Figure 4-14: **application-side implementation**) ----
    //   Rule (per the text): `radm_grant_tlp_type` asserted => **++**; `radm_trgt1_hv` (in this model:
    //   this TLP fully assembled) => **--**; when the application buffer has no free slots => assert `trgt1_radm_pkt_halt[type]`.
    //   WARNING: model semantics: (1) count grants on **rising edges** (equivalent to a per-beat pulse); (2) decrement **per TLP** (NOT per beat);
    //   (3) free-slot count is configurable (default huge => no automatic halt = existing testbenches unchanged).
    int      pkt_free[3] = {1 << 20, 1 << 20, 1 << 20};   // Free slots per type (testbench may set small/0)
    int      pkt_inflight[3] = {0, 0, 0};                 // ++grant / --assembly complete
    uint64_t grant_cnt[3] = {0, 0, 0};                    // Grant rising edges received (per type, testbench check)
    uint64_t grant_deq[3] = {0, 0, 0};                    // Assembly complete (per type) = matching -- count
    uint8_t  prev_grant = 0;                              // Rising-edge detection (previous-cycle sample)
    // -- * T49: Message receive face (r subcode semantics) ---------------------------------------
    //   r=100 **local terminate** = consumed by this receive port (NEVER forwarded as-is) => count + per-code record;
    //   INTx virtual-line tracking (device-number rotation/folding) = T49-5 on top of this face.
    uint64_t msg_local_rx = 0;                            // Number of locally terminated Messages
    std::vector<uint64_t> msg_local_code = std::vector<uint64_t>(64, 0);  // By message code
    // -- * T49-5: INTx **virtual lines** (spec: r=100 hop-by-hop relay => receive port tracks after mapping with **device-number rotation**;
    //   folded (at this port) then re-sent upstream with **this port's reqid**; device number = requester[7:3] mod 4) ---------
    uint8_t  intx_wire = 0;                               // 4 virtual lines of this port (**after** rotation)
    uint64_t intx_rx = 0, intx_fwd = 0;                   // Received (locally terminated) count / re-sent upstream count
    uint8_t  intx_devnum = 0xFF;                          // 0xFF = derive from message reqid (default)
    // -- * C3-a (2026-10-16): **INTx bridge mapping table + dp_int/sys_int sideband** (multiple devices sharing a port) ----------
    //   - **Mapping table**: after rotation (Table 2-22), apply **4->4 remapping** (`intx_map[virtual line] = real line`);
    //     default identity => zero regression; with multiple devices per port the testbench/platform programs the real bridge table (rotation alone is NOT final).
    //   - **dp_int / sys_int sideband**: after virtual-line folding + mapping => (1) `dp_int_n` (this port's downstream interrupt
    //     output, active low), (2) `sys_int_n` (**system interrupt** summary to the root; aggregated through the fabric - same
    //     channel method as B8-a). Contract NEVER changed (top `sys_*` is in the 298 ledger; bridge-side wiring = M6b).
    uint8_t  intx_map[4] = {0, 1, 2, 3};                  // Identity default
    bool     intx_map_en = false;                         // Enabled only when a table is configured (zero regression)
    uint64_t intx_remap_events = 0;
    bool     dp_int_n = true, sys_int_n = true;           // Active low
    uint64_t dp_int_events = 0;
    void set_intx_map(const uint8_t m[4]) { for (int k = 0; k < 4; ++k) intx_map[k] = (uint8_t)(m[k] & 3u); intx_map_en = true; }
    void sb_sys_int_update(uint8_t wire);                 // Folded line => dp_int/sys_int (body in .cpp)
    bool     ignore_intx_local = false;                   // Testbench red mode: NEVER locally terminate (pass through as-is)
    // * T51-2: port-side ECRC recompute semantics (**must be set the same as the device side**; M6b cross-check item)
    bool     ecrc_reflected = false;
    // -- * T52 (B6/B7): **port-side** DPC containment + hot-plug gating (both on this port's bidirectional gate) ----------
    uint64_t dpc_blocked = 0, dpc_irq = 0, dpc_triggers = 0, dpc_cleared = 0;
    uint64_t slot_pwr_blocked = 0;                        // TLPs blocked while powered down (ledger)
    bool     ignore_port_gate = false;                    // Testbench red mode: NEVER honor the power-down/DPC bidirectional gate
    bool     prev_dpc = false;                            // DPC TriggerStatus transition detection (=> dpc_cleared)
    void dpc_poll();                                      // DPC TriggerStatus transition (=> dpc_cleared)
    void slot_insert(bool present);                       // Insert/remove (=> Presence St./Changed + DLLSC + interrupt)
    void slot_attention();                                // Attention button
    bool port_blocked() const;                            // (defined in .cpp; RtlStub is still incomplete here)
    // -- * T-3: beats received upstream (radm) (for bit-by-bit review; testbench reads directly) --------------------
    std::vector<BitVec> rx_log;
    // -- * T53 (C2): **bridge-side CRS gate** (`cfg_bridge_crs_en` = this port's Type1 DevCtl[15]; "Bridge only") --
    //   Spec/RTL semantics: set => this bridge **may return/forward CRS** <-> cleared => "no CRS completion is returned =>
    //   the lack of a completion results in eventual Completion Timeout at the Root Complex"
    //   => model: CRS completions are **not forwarded upstream** (counted; RC-side timeout => UR = existing M3 mechanism).
    bool     bridge_crs_en() const;
    uint64_t crs_fwd = 0, crs_dropped = 0;              // Forwarded upstream / dropped by the bridge gate (NEVER silent)
    uint64_t vdm_fwd = 0, vdm_badcode = 0;              // * T55 (C6): VDM upstream count / code-subcode mismatch (ledger)
    // -- * T55 (C7)/T58: **atomic egress blocking** (**DevCtl2[7]**; port attribute; bit numbers aligned in T58) => this port NEVER sends atomics (ledger) --
    uint64_t atomic_egress_blocked = 0;    // -- * T54 (C11-a ACS): source validation V / translation blocking B (entry below; databook Sec. 3.24 Table 3-95) --------
    uint64_t acs_viol = 0, acs_viol_v = 0, acs_viol_b = 0, acs_ca_sent = 0;
    bool     ignore_acs_vb = false;    // Testbench red mode: NEVER honor V/B enforcement (discriminative)
    // -- * T53 (C13): malformed receive-side checks (upstream direction; semantics in pcie_sc_wire.h) ---------------
    uint64_t mlf_drops = 0;
    uint64_t mlf_by_code[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool     ignore_mlf = false;                        // Testbench red mode: checks off (pass silently)
    uint64_t mlf_last_code = 0;
    // -- * M6b (2026-10-18): **CFG1->CFG0 conversion switch** -----------------------------
    //   Basis = **customer's own DWC downstream-port application logic** `top1/cfg2dbi.v:214`
    //   (`client1_tlp_type = 5'h4` comment "cfg1 to cfg0"; forwards on `cfg_2ndbus_num` match).
    //   real DWC does **not** do this conversion inside the core => the switch application (played by the TLM) handles it - consistent with the fake DUT.
    //   WARNING: this switch only covers the **type**; for the address form see `cfg_addr_wire` (both must be on for a real DUT to work).
    bool     cfg1_to_cfg0 = true;
    // -- * M6b (2026-10-18): **CFG address-form conversion** (model internal => DWC client port) ----------
    //   Model `cfg_addr_make` (pcie_route.cpp:32) = {ext[27:24] | bus[23:16] | dev[15:11] |
    //   fn[10:8] | reg[7:2]}; real DWC `client0/1_tlp_addr` cfg form (read from RTL
    //   `pig_xadm_hdr_form.sv:426-432`, same for CFG0/CFG1) = **wire DW2 byte stream**:
    //   {bus[31:24] | {dev[4:0],fn[2:0]}[23:16] | extreg[11:8] | reg[7:2]}.
    //   Sending directly => model bus (=2) lands in the dev/fn byte (dev0/fn2) => real EP does not respond (M6b measurement:
    //   tree has only 3 entries / all EP cfg lost). WARNING: default false => existing fake-DUT testbenches (TB-7/enum) zero regression.
    bool     cfg_addr_wire = false;

    SC_HAS_PROCESS(DspPortModule);
    DspPortModule(sc_core::sc_module_name nm, const CoreCfg& c, PrivWire* w, int idx,
                  uint8_t appbus, uint8_t secondbus);

    void inject_dbi_up(const DbiSignals& b);    // Testbench use: emulate core completion => return path upstream
    // -- * T56 (C10): LTR / OBFF / PTM port-level units (codes = read from RTL; all **r=100 local terminate** => hop-by-hop) --
    uint64_t ltr_rx = 0, ltr_fwd = 0, obff_rx = 0, obff_fwd = 0;
    uint64_t ptm_req_rx = 0, ptm_res_tx = 0, ptm_res_blocked = 0;
    uint32_t ltr_last = 0;                      // {NoSnoop|Snoop} (format per RTL `pig_cdm_l1sub_reg.sv:334-341`)
    uint8_t  obff_state = 0;                    // CPU Active / DMA Active (databook p.247 trigger face)
    bool     ptm_responder_en = true;           // **DSP = Responder** (databook Sec. 3.23)
    uint32_t ptm_time = 0;                      // Timestamp of this port's responder (master time)
    void send_to_ep(const PrivTlp& t);          // port => EP (response face; pure FIFO write without wait => atomic)
    // -- * B8-a: WAKE#/CLKREQ **sideband assertion** (model-internal channel; see CrossbarModule.sb_*) --------
    CrossbarModule* sb_xb = nullptr;            // Testbench wiring: points to the fabric (aggregation point)
    bool     wake_side_n = true, clkreq_side_n = true;   // Active low
    uint64_t wake_side_events = 0, clkreq_side_events = 0;
    void sb_wake_assert(bool low);              // WAKE#: device => root (wake request; body in .cpp)
    void sb_clkreq_assert(bool low);            // CLKREQ#: device => root (clock request; body in .cpp)


    // -- * T56 (C14): **link status face** (DL state machine + Link Control/Status truth) ------------------
    //   Read from RTL: `Layer2/pig_rdlh_link_cntrl.sv` (**INACTIVE(00) -> FC_INIT(01) -> [FEATURE] -> ACTIVE(11)**;
    //   in ANY state `!smlh_link_up => return to INACTIVE`; `rdlh_link_up = (state == ACTIVE)`) -
    //   Link Status: `DL Active` + **Negotiated Width = link_up ? autoneg : 6'b1** (`cfg_reg:3512`)
    //   - Link Control: `link_dis`/`link_retrain` (WARNING: RTL comment says **DSP only**, not applicable to EP/USP)
    bool     smlh_link_up = true;               // Physical link (truth source of contract `rdlh_link_up`)
    bool     dll_lnk_en = true;                 // Software enable gate (RTL `cfg_dll_lnk_en`)
    bool     link_disable = false, link_retrain = false;
    uint8_t  dl_state = 0;                      // Reset = INACTIVE (RTL semantics) => must climb through FC_INIT
    int      fc_init_left = 0;                  // FC_INIT remaining cycles (done after 8 cycles = `rtlh_fci_done`)
    uint8_t  autoneg_width = 16, autoneg_speed = 4;   // smlh_autoneg_link_width/sp (this config x16)
    uint64_t link_retrains = 0, link_downs = 0, fc_inits = 0;
    bool     bw_mgt_status = false, auto_bw_status = false;   // RW1C
    uint64_t bw_status_set = 0, bw_status_clr = 0, auto_bw_status_set = 0;
    uint64_t link_notif = 0;                    // Status set => interrupt-stage count (`cfg_link_auto_bw_int/msi`)
    bool     ignore_dll_gate = false;           // Testbench red mode: NEVER honor software gate/link down
    bool     dl_active() const { return dl_state == 3; }
    uint8_t  neg_width() const { return dl_state == 3 ? autoneg_width : 1; }   // RTL: smlh_link_up ? w : 1
    void dl_tick();                             // Advance every cycle (host/TB driven; model semantics)
    void set_link_up(bool up);                  // smlh_link_up edge (down => return to INACTIVE + ledger)
    void set_dll_en(bool en)   { dll_lnk_en = en; link_disable = !en; }
    uint64_t link_ctl_retrain();                // Link Control Retrain (=> return to FC_INIT)
    void bw_mgt_w1c()          { if (bw_mgt_status) { bw_mgt_status = false; ++bw_status_clr; } }
    void bw_mgt_set();                          // Set + notification stage


private:
    BeatAsm asm_;                               // Upstream assembly (RX)
    void tx_run();                              // Downstream: TLP => pack => beat stream (clock domain)
    void rx_run();                              // Upstream: beat stream => unpack => TLP
    bool handle_cfg(const PrivTlp& t);          // cfg: own side => in-place DBI; below => convert and forward
    void mirror_apply(uint16_t reg, uint32_t val);   // Enumeration side effects: mirror + routing window
};

// -- RTL stub (DSP IP + link; standalone phase; co-sim replacement seam) -----------------------------
struct RtlStubModule : sc_core::sc_module {
    sc_core::sc_in<bool> clk;
    sc_core::sc_fifo_out<PrivTlp> ep_out;       // -> EP stub (downstream)
    sc_core::sc_fifo_in<PrivTlp>  ep_in;        // <- EP stub (upstream; Cpl/MSI = M3/M4)
    CoreCfg     cfg;
    PciePacker  pk;
    PrivWire*   wire = nullptr;
    // -- Receive-side (beat stream => frame) state and statistics (testbench reads directly)
    BeatAsm asmb;
    std::vector<BitVec> rx_log;                 // Received beats (for bit-by-bit review)
    uint64_t tx_beats = 0, tx_frames = 0;
    // -- Core-side truth: M2 = this port's Type1 config space (read/write via DBI; databook H.4) ----------
    CfgSpace    port_cfg;
    uint64_t    dbi_ops = 0;                    // DBI execution count (testbench check: path really goes through DBI)
    // * T50: **port-side ARI forwarding** (spec: DevCtl2[5] "ARI Forwarding Enable"; in this model = port attribute)
    //   on => the cfg **Device field carries fn8's upper 5 bits** (NO longer treats "dev!=0" as UR); bridge cross-check = M6b
    bool        ari_forward = false;
    // Cascade: when another switch sits behind this port, its downstream bus
    // carries devices at dev != 0 (the switch's own downstream ports).  When
    // set, config requests with dev != 0 are forwarded instead of answered
    // with UR.  Default false => single-device downstream bus (zero regression).
    bool        allow_multi_dev = false;
    // Core-side sideband outputs (CDM; core truth => sideband): bus face + link state (M4: link_up for TB preset)
    uint8_t     side_pbus = 0, side_sec = 0, side_sub = 0;
    bool        side_link_up = false;
    // -- * T-1b: TRGT1 grant pulse (databook Sec. 4.5.3; core -> application; testbench reads directly) ------------
    //   set 1-hot at frame start (bit = vc*3 + cls), deassert at frame end => application counts on **rising edges**;
    //   `pkt_halt_blocked` = number of times frame start was **blocked** by the application-side per-type halt (NEVER silent).
    uint64_t    grants_sent[3] = {0, 0, 0};     // Per type: actual grants issued (frame starts)
    uint64_t    pkt_halt_blocked[3] = {0, 0, 0};// Per type: blocked by pkt_halt and not sent (ledger; one count per halt)
    bool        ignore_pkt_halt = false;        // Testbench **red mode**: ignore per-type halt and send anyway (discriminative)
    // -- * T-3: DBI request trace (Sec. 4.10.1 rule checks: D1 cs / D2 wr all 0 = read / D3 single outstanding) ----
    std::vector<DbiSignals> dbi_log;
    // -- * T53 (C13): malformed receive-side checks (**downstream wire side** = core RTLH position; per customer RTL) -----------
    //   This port = switch port => **not an endpoint** (RTL text: CRS completion is malformed only for endpoints, "does not apply
    //   to switch") => this layer does not check `cpl_crs_err`; on detection => drop + count (+ set Uncorr[18] on the port-side AER face).
    uint64_t mlf_drops = 0;
    uint64_t mlf_by_code[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint64_t mlf_last_code = 0;
    bool     ignore_mlf = false;                // Testbench red mode: checks off (pass silently => discriminative)

    SC_HAS_PROCESS(RtlStubModule);
    RtlStubModule(sc_core::sc_module_name nm, const CoreCfg& c, PrivWire* w);

    // Testbench use: upstream injection (hand-built TLP => pack radm_trgt1 group => onto the wire)
    void inject_up(const PrivTlp& t);

private:
    void rx_run();                              // Receive TLM beat stream => frame => deliver to EP / nonexistent => UR
    void tx_run();                              // EP return path => beat stream (upstream)
    void dbi_slave_run();                       // DBI slave: core register read/write (addr/wr/din => ack/dout)
    void ur_cpl_up(const PrivTlp& t);           // Nonexistent device => UR Cpl sent directly upstream
};

// -- EP stub (standalone Type0 minimal set + BAR + storage) -----------------------------
struct EpStubModule : sc_core::sc_module {
    sc_core::sc_fifo_in<PrivTlp>  in;
    sc_core::sc_fifo_out<PrivTlp> out;
    // -- * T50-2: **ECRC** (C1 closed; gates = AER CapCtl[0] generate / [1] check) ----------------
    void     tx(PrivTlp t);                            // Before send: generation gate open => stamp ECRC + set TD
    void     rx_ecrc(PrivTlp& t);                      // After receive: check gate open AND TD => verify; bad => set status + count
    bool     ecrc_gen()   const { return (ep_cfg.aer_cctl & 0x1u) != 0; }
    bool     ecrc_check() const { return (ep_cfg.aer_cctl & 0x2u) != 0; }
    bool     ecrc_corrupt = false;                     // Testbench use: stamp a **bad** ECRC (teeth: receiver must catch it)
    uint64_t ecrc_gen_cnt = 0, ecrc_check_ok = 0, ecrc_check_bad = 0;
    // Type0 config space (M2: enumeration read/write; BAR sizing => bar_base/bar_size set as side effects)
    CfgSpace ep_cfg;
    uint64_t cfg_ops = 0, cfg_ur = 0;
    uint16_t vid = 0x10EE, did = 0x9024;
    uint8_t  class_code = 0x01, rev = 0x01;
    // -- BAR + storage (G7: **multiple BARs**; each BAR = independent storage segment) ----------------------
    //   * NO cached copy: address decode **reads `ep_cfg.bars` directly** (single truth; old scalar bar_base/bar_size cache retired)
    uint64_t seg_stride = 0x20000;              // Storage segment stride per slot (BAR size must be <= stride)
    std::vector<uint8_t> mem;                   // Storage (initial value 0xFF => unwritten is detectable; segment = slot index x stride)
    int      bar_cnt() const { return ep_cfg.bar_count(); }
    uint64_t bar_base(int k) const { return ep_cfg.bar_base(k); }
    uint64_t bar_size(int k) const { return ep_cfg.bar_size(k); }
    bool     bar_is64(int k) const {
        return (k >= 0 && k < ep_cfg.bar_count()) ? ep_cfg.bars[(size_t)k].is64 : false;
    }
    void preset_bar(int k, uint64_t base, uint64_t size, bool is64 = false);   // Testbench use: preset (does NOT go through cfg)
    void layout_bars(int nbar, uint64_t stride = 0x20000);                     // Testbench use: lay out slots + independent storage segments
    int  decode(uint64_t addr, size_t n, size_t& off) const;                   // BAR decode (-1 = not in any BAR)
    int  io_decode(uint64_t addr, size_t n, size_t& off) const;                // * T52: **IO BAR** decode
    uint64_t io_rds = 0, io_wrs = 0, io_oob = 0;                               // * T52: IO transaction counts
    bool peek(uint64_t addr, size_t len, std::vector<uint8_t>& out) const;     // Testbench use: read segment per decode
    // -- * P2: SR-IOV **VF side** (the PF's VF table is in `ep_cfg`; after VF Enable, fn = first+k*stride is reachable) --
    //   Each VF has an **independent storage segment** (k x vf_bar0.size) + **per-VF Command (MSE gate)**;
    //   WARNING: VF BAR0 readback = PF's VF BAR0 value (hardware offsets the address by k x size - spec semantics)
    std::vector<uint8_t>  vf_mem;               // NumVFs x vf_bar0.size
    std::vector<uint16_t> vf_cmd;               // Command of each VF
    uint64_t vf_cfg_ops = 0, vf_cfg_ur = 0, vf_memwrs = 0, vf_memrds = 0, vf_mse_ur = 0, vf_oob = 0;
    void     layout_vfs();                      // Lay out VF segments (NumVFs x VF BAR size; call set_sriov first)
    int      vf_index_of_fn(uint8_t fn) const;  // fn => VF index (-1 = not enabled/out of range)
    int      vf_decode(uint64_t addr, size_t n, size_t& off) const;
    uint32_t vf_cfg_read(int k, uint16_t reg) const;
    void     vf_cfg_write(int k, uint16_t reg, uint32_t val, uint8_t be);
    // -- * T48-3: **VF-MSIX** (spec: each VF has its own MSI-X capability; the table lives in the **VF's own BAR**) --
    //   Construction face = device attribute (e.g. `set_vf_msix`); config face = **VF cfg space** (CapPtr@0x34 => 0x40 chain);
    //   table-entry truth is in **VF segment storage** (`vf_mem`) => same as PF: entries are NOT registers, programmed one by one with memory writes.
    struct VfMsix {
        bool     cap = false;
        uint16_t ctrl = 0;                      // Message Control ([15]=Enable [14]=FnMask [10:0]=N-1 RO)
        uint32_t tb = 0;                        // Table Offset/BIR (BIR always 0 => VF's BAR0 segment)
        uint32_t pb = 0;                        // PBA Offset/BIR
        int      nvec = 0;
    };
    std::vector<VfMsix> vf_msix;                // Same order as vf_mem/vf_cmd (call layout_vfs first)
    std::vector<uint16_t> vf_devctl;            // * T49-5: per-VF Device Control (FLR = bit15)
    uint64_t vf_flr_count = 0;                  // VF FLR count (independent of PF)
    void set_vf_msix(int nvec, uint32_t tbl_off, uint32_t pba_off = 0);   // Testbench/constructor: attach MSI-X to all VFs
    bool vf_msix_enabled(int k) const {
        return k >= 0 && k < (int)vf_msix.size() && vf_msix[(size_t)k].cap &&
               (vf_msix[(size_t)k].ctrl & 0x8000u) != 0;
    }
    uint16_t vf_reqid(int k) const;             // * ARI: reqid = {bus, fn8} (fn8 = FirstVFOffset + k x Stride)
    // Send a posted MemWr per **table entry**; five no-send classes each counted (same as PF; NEVER silent) --
    //   not enabled / FnMask / vector masked / entry not programmed / out of table (outside segment => unmapped)
    bool send_vf_msix(int k, size_t vector, uint64_t* addr_out = nullptr, uint32_t* data_out = nullptr);
    uint64_t vf_msix_sent = 0, vf_msix_gated = 0, vf_msix_fn_mask_block = 0, vf_msix_masked = 0,
             vf_msix_unprog = 0, vf_msix_oob = 0, vf_msix_unmapped = 0;

    // -- * T49-5: **FLR** (function-level reset; DevCtl bit15 => non-sticky registers return to reset values) ----------
    //   Semantics: sticky bits are preserved (PME_En/PME_Status, AER Mask/Severity); BAR **base cleared to zero** (size preserved)
    //   => memory face rejects after reset (Command.MSE=0); works again after software reconfiguration (`flr_count` recorded).
    uint64_t flr_count = 0;
    bool     ignore_flr = false;                              // Testbench red mode: NEVER execute FLR
    // -- * T49-5: **INTx** (legacy interrupt; message = r=100 **local terminate** => virtual-line tracking done by the **receive port**) --
    //   Gates (spec): (1) **MSI/MSI-X enabled => INTx must not be used** (reject + count)
    //             (2) Command[10] Interrupt Disable => reject assert (deassert is ALWAYS allowed)
    void intx_assert();  void intx_deassert();
    bool intx_active = false;
    uint64_t intx_assert_sent = 0, intx_deassert_sent = 0;
    uint64_t intx_blocked_by_msi = 0, intx_blocked_by_dis = 0, intx_auto_deassert = 0;
    // -- * T49-3: power management (EP side) ----------------------------------------------
    //   - `send_pme()`: PME_Status is **always set** (sticky); PME_En=1 => send **PM_PME** (0x18; r=000) upstream,
    //     otherwise only set the bit + count (never silent); also applies in D3hot (wake source).
    //   - `PME_Turn_Off` received (0x19; r=011 broadcast) => count + **automatically reply PME_TO_Ack** (0x1B; r=101);
    //     testbench may disable `auto_to_ack` (red mode: no reply => root-port missing-response ledger).
    //   - **D3hot semantics** (model): memory transactions are **not accepted** => UR + `d3_blocked` (NEVER silent).
    void     send_pme();
    // -- * T50-3: **WAKE#** (semantic unit; WARNING: the physical pin (`wake`, top line 373) is **outside the contract** => bridge-side wiring is M6b) --
    //   Semantics (spec): WAKE# = sideband for a **low-power-state (D3hot/D3cold) device requesting wake**; NO wake is needed in D0;
    //   once asserted: (1) link **wakes back to L0** (via the `on_wake` hook into the link state machine) (2) PME_En=1 => **PM_PME follows**.
    void     wake_assert();
    std::function<void()> on_wake;                 // Hook (testbench/link owner binds to LinkPmModule::wake)
    uint64_t wake_asserted = 0, wake_blocked_d0 = 0, wake_no_pme = 0;
    bool     ignore_wake_d0 = false;               // Testbench red mode: ignore the D0 gate
    bool     auto_to_ack = true;
    bool     ignore_pme_gate = false;                         // Testbench red mode: ignore the PME_En gate
    bool     ignore_d3 = false;                               // Testbench red mode: ignore D3hot (memory still accepted)
    uint64_t pme_sent = 0, pme_gated = 0;                     // Sent / blocked by the PME_En gate
    uint64_t pme_turn_off_rx = 0, to_ack_sent = 0;
    uint64_t d3_blocked = 0;                                  // Memory transactions rejected during D3hot
    // -- * T49-2: AER error injection + send (spec: Mask suppresses -> Severity classifies -> reporting-enable gate) -----
    //   Three classes (Table 2-20): ERR_COR 0x30 / ERR_NONFATAL 0x31 / ERR_FATAL 0x33 (r=000 upstream).
    //   NEVER silent: suppressed by Mask / blocked by the reporting-enable gate => **each counted separately** (status bits still set = "logged").
    void inject_corr(unsigned bit, const PrivTlp* ctx = nullptr);      // Correctable (bit: Corr Status bit)
    void inject_uncorr(unsigned bit, const PrivTlp* ctx = nullptr);    // Uncorrectable (classified by Severity)
    void aer_log_header(const PrivTlp& ctx);                          // Header Log snapshot (4DW semantics see impl)
    uint64_t aer_cor_sent = 0, aer_nonfatal_sent = 0, aer_fatal_sent = 0;
    uint64_t aer_masked = 0, aer_gated = 0;                           // Suppressed/gated (each in the ledger)
    bool     aer_ignore_mask = false;                                 // Testbench red mode: ignore Mask
    bool     aer_ignore_gate = false;                                 // Testbench red mode: ignore the reporting-enable gate
    // -- MSI initiation (M4): reqid = this EP's BDF (registered by enumeration); config (enable/addr/data) in ep_cfg --
    uint16_t reqid = 0;                         // This device's BDF (used in MSI message header)
    uint64_t msi_sent = 0;                      // Number of MSIs sent
    // -- * T54 (C11-b ATS): **invalidate request/completion** (minimal set; code values and r=010 read from customer RTL) ---------
    //   **Invalidate Request** received (r=010 routed by ID to this EP) => if ATS enabled => **invalidate** this device's
    //   translation cache (model = counting stub) => reply **Invalidate Completion** (r=010) <-> not enabled =>
    //   NO reply + ledger (`ats_inv_noenable`; NEVER silent). WARNING: real hardware = ATC per-entry/global invalidation (STU granularity).
    uint64_t ats_inv_rx = 0, ats_inv_cpl_sent = 0, ats_inv_noenable = 0;
    // -- ATS address-translation cache (model, 4 KB pages) -------------------
    //   AT=2 requests require Enable[15] and a present ATC entry; otherwise a
    //   read is answered UR and a posted write is dropped (each counted).
    //   prefill/clear are testbench-visible; invalidation clears the whole ATC.
    std::unordered_set<std::uint64_t> atc_;
    uint64_t ats_trans_hit = 0, ats_trans_miss = 0, ats_trans_blocked = 0;
    uint64_t ats_atc_fills = 0, ats_atc_cleared = 0;
    void ats_atc_prefill(std::uint64_t iova, std::uint64_t size);
    void ats_atc_clear() { ats_atc_cleared += atc_.size(); atc_.clear(); }
    std::size_t ats_atc_entries() const { return atc_.size(); }
    // -- * T54 (C11-c PASID): **PASID gate** (capability face = ep_cfg; data face = model-internal `PrivTlp.pasid`) --
    //   Gate: PASID enable = 0 while the packet carries PASID => **reject** (NP => UR; posted => drop) + ledger;
    //   enabled => pass + count (`pasid_ok`). WARNING: this config's port has no PRFX face (see note in pcie_cfg_space.h).
    uint64_t pasid_ok = 0, pasid_blocked = 0;
    // -- * T55 (C7): **atomic operation execution** (FetchAdd/Swap/CAS; minimal set) -------------------------
    //   completion = **original value** (CplD; spec); gates = Command.MSE + BAR decode (out of range => UR); NEVER silent.
    //   requester side (testbench `send_atomic`) = **DevCtl2[6]** AtomicOp Requester Enable gate (bit numbers aligned in T58).
    bool     send_atomic(uint8_t atype, uint64_t addr, uint16_t dw_len,
                         const std::vector<uint8_t>& operand, uint64_t tag = 0);
    uint64_t atomic_ops = 0, atomic_fadd = 0, atomic_swap = 0, atomic_cas = 0, atomic_cas_hit = 0;
    uint64_t atomic_req_blocked = 0;            // Request gate closed (DevCtl2[6]=0) => NOT sent (ledger)
    uint64_t atomic_req_sent = 0;
    // -- * T53 (C2): **CRS (not-ready window)** - EP-side "configuration not ready" semantics (programmable delay window) --------------
    //   Semantics (spec + RTL read; do NOT invent): when top inputs `app_req_retry_en` (line 481) /
    //   `app_pf_req_retry_en` (line 482) are enabled, the filter responds to **CFG0 (this port) requests** =>
    //   **Cpl status = CRS (0b010)** (RTL text `Adm/pig_radm_filter_sw.sv:657`:
    //   `(dest==TRGT0 && app_req_retry_en_tmp && (cfg0rd|cfg0wr)) ? CRS : ...`);
    //   databook Sec. 3.1 note 1: "the controller completes incoming CFG requests with a
    //   configuration request retry status (CRS). **Other incoming requests complete normally**".
    //   Model: `crs_window` = the first n **cfg0** requests return CRS (NOT executed - spec: the request was not processed)
    //   => requester side (RC/enumeration) retries; window = 0 (default) => zero behavior change.
    //   Spec hard rule: after returning **any non-CRS completion**, CRS must NEVER be returned again (unless reset) => once `crs_ever_ok` is set,
    //   always **suppress + ledger**; FLR resets this state (`on_flr` hook).
    void     set_crs_window(uint32_t n) { crs_window = crs_left = n; }
    uint32_t crs_window = 0, crs_left = 0;
    uint64_t crs_answered = 0;                  // Number of CRS responses returned
    uint64_t crs_suppressed = 0;                // In window but prohibited by spec (already succeeded) => suppressed (ledger)
    uint64_t crs_cfg_ok = 0;                    // Number of cfg0 completions done normally
    bool     crs_ever_ok = false;               // A non-CRS completion has been returned (spec: CRS forbidden afterwards; cleared by reset)
    bool     ignore_crs = false;                // Testbench red mode: ignore the not-ready window (execute normally)
    // -- * T53 (C13): malformed receive-side checks (EP = **endpoint** => includes the `cpl_crs_err` item) -----------
    uint64_t mlf_drops = 0;                     // Detected and dropped count (NEVER silent)
    uint64_t mlf_by_code[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};   // Count by error code
    uint64_t mlf_ur_replied = 0;                // Non-posted requests dropped => CA/UR returned (databook Sec. 3.4.4.1)
    uint64_t mlf_last_code = 0;                 // Last error code (testbench check)
    bool     ignore_mlf = false;                // Testbench red mode: checks off (pass silently => discriminative)
    int      mlf_pad_dws = 8;                   // Wire-level beat padding tolerance (DW; default covers a 256-bit data beat)
    // Statistics (testbench checks)
    uint64_t memwrs = 0, memrds = 0, other = 0, oob = 0, poisoned_wr = 0, urs = 0;
    // -- * T56 (C12): **Expansion ROM** (window + storage segment; same form as RTL `pig_cdm_bar_match.sv:251`) ----------
    std::vector<uint8_t> rom;                   // ROM storage (separate from BAR segments)
    uint64_t rom_rds = 0, rom_wrs = 0, rom_oob = 0;
    bool rom_rd(uint64_t addr, size_t n, std::vector<uint8_t>& out);
    bool rom_wr(uint64_t addr, size_t n, const std::vector<uint8_t>& d);
    // -- * T56 (C10-d): **DPA** (model feature; RTL macro off `CX_DPA_ENABLE_VALUE = 0`, no register face in RTL/databook) --
    uint8_t  dpa_substate = 0; uint32_t dpa_latency = 0;
    uint64_t dpa_sets = 0, dpa_blocked = 0;
    //   Model feature: RTL macro off + no register face (databook also none) => **spec-shaped** ECAP 0x0016 semantics; substate 0..3
    bool set_dpa(uint8_t ss, uint32_t lat) {
        if (ss > 3u) { ++dpa_blocked; return false; }                  // Illegal substate => reject + ledger
        dpa_substate = ss; dpa_latency = lat; ++dpa_sets; return true;
    }
    // -- * T56 (C10-c): PTM **initiator face** (EP = requester; response comes from the DSP responder) ------------------
    uint32_t ptm_res_time = 0;                  // Last received master time
    uint64_t ptm_req_tx = 0, ptm_res_rx = 0, ptm_timeout = 0;
private:
    sc_core::sc_event ptm_ev_;
public:
    bool send_ptm_req(uint64_t tag, uint32_t timeout_us = 100);   // Timeout = 100us per RTL/databook
    void ptm_rx(const PrivTlp& t);              // Receive PTM Response => record master time
    uint64_t last_wr_addr = 0, last_wr_bytes = 0;

    SC_HAS_PROCESS(EpStubModule);
    EpStubModule(sc_core::sc_module_name nm, size_t memsz = 0x10000);

    std::vector<uint8_t> read_mem(uint64_t off, size_t len) const;
    void fill_mem(uint64_t off, size_t len, uint8_t v);

    // -- MSI initiation (M4): send one MemWr with the given addr/data (posted; typical call = pass this device's MSI Cap
    //    current values, see `msi_enabled()`/`ep_cfg.msi_addr`/`ep_cfg.msi_data`) -
    //    returns false = not enabled (does NOT send; `msi_sent` unchanged, discriminative) --
    bool send_msi(uint64_t addr, uint32_t data);
    bool msi_enabled() const { return ep_cfg.msi_cap && (ep_cfg.msi_ctrl & 0x1u) != 0; }
    // -- * MSI-X initiation (P2): send a posted MemWr per **table entry** (storage in BAR: AddrLo/AddrHi/Data/Ctrl); --
    //   five no-send classes, **each counted** (never silent): not enabled / Function Mask / vector mask / entry not programmed / out of table;
    //   returns true = actually sent (msix_sent +1); optionally returns addr/data (for testbench checks).
    bool send_msix(size_t vector, uint64_t* addr_out = nullptr, uint32_t* data_out = nullptr);
    bool msix_enabled() const { return ep_cfg.msix_enabled(); }
    uint64_t msix_sent = 0, msix_gated = 0, msix_fn_mask_block = 0, msix_masked = 0,
             msix_unprog = 0, msix_oob = 0, msix_unmapped = 0, msi_blocked_by_msix = 0;
    // -- Device-side request initiation (for M4 negative tests): MRd (optionally with poison bit) => upstream port --
    void send_read(uint64_t addr, size_t nbytes, bool poisoned = false);

private:
    void run();
    void handle_atomic(const PrivTlp& t);       // * T55 (C7): atomic execution (defined in .cpp)
    void apply_write(const PrivTlp& t);
    void handle_cfg(const PrivTlp& t);          // cfg => Cpl (SC/UR)
    void handle_read(const PrivTlp& t);         // MemRd => CplD (data; out of range/gate closed => UR)
    static bool lane_on(uint8_t be, uint16_t dw_len, size_t dw, size_t lane);
};

} // namespace pcie_tlm
