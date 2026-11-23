// -----------------------------------------------------------------------------
// Scheme 1 - Configuration space (**declarations**) - Type0 / Type1 function level (layout per spec)
//   - `Command.MSE/IOE` gates; BAR sizing (write all 1s => read back mask => write base address)
//   - Type1 bridge header: 0x18 = pbus/sec/sub (enumeration side effect => routing); 0x20/0x22 = memory window (=> routing window)
//   - Minimal capability chain: PCIe Cap (0x40; type 0/1) + MSI Cap (0x50, EP)
//   * **No ECAM** (decided 2026-10-12: RC is inside the TLM => enumeration issues Cfg TLPs directly, no MMIO window to decode)
//   * Side-effect callback `on_write` = owner lands register writes into the model (per-port RC/USP/DSP mirrors + crossbar windows)
//   * Implementation = pcie_cfg_space.cpp.
// -----------------------------------------------------------------------------
#pragma once
#include <cstdint>
#include <functional>
#include <map>
#include <vector>

namespace pcie_tlm {

// -- Configuration register access context ({bus,dev,fn,reg} of a Cfg TLP) ----------------
struct CfgAddr {
    uint8_t bus = 0, dev = 0, fn = 0;
    uint16_t reg = 0;                     // register byte offset (4-byte aligned)
};

struct CfgBar {
    uint64_t size = 0;                    // size (power of 2; 0 = **not implemented** => hardwired 0: reads and writes return 0)
    uint64_t base = 0;                    // base address (**64-bit**; a 64-bit pair uses both low and high halves)
    bool     sizing = false;              // "write all 1s to probe size" state (**persists across reads**; a base-address write exits it)
    bool     is64 = false;                // 64-bit pair (**physical attribute**; NEVER changed by sizing writes / base-address echoes)
    bool     prefetch = false;            // prefetch bit3 (**physical attribute**)
    bool     is_io = false;               // **IO space BAR** (bit0 = 1; **physical attribute**; NEVER mixed with memory semantics)
    uint8_t  idx = 0;
    // Hardwired type bits (read out from **physical attributes**: IO => bit0; memory => bit2 = 64-bit / bit3 = prefetch)
    uint32_t type_bits() const {
        return is_io ? 0x1u : ((is64 ? 0x4u : 0u) | (prefetch ? 0x8u : 0u));
    }
};

struct CfgSpace {
    uint8_t  cfg_type = 0;                // 0 = Type0 (EP) / 1 = Type1 (bridge port)
    uint16_t vid = 0x10EE, did = 0;
    uint8_t  rev = 0x01, prog_if = 0, sub_class = 0, base_class = 0;
    uint16_t command = 0, status = 0;
    std::vector<CfgBar> bars;             // Type0: 0..5; Type1: 0..1
    // -- Type1 only --
    // -- * P2: SR-IOV (PF) --
    //   WARNING: **placement convention (measured and decided)**: the spec places it in **extended config space 0x100+**, but this model's cfg address encoding
    //     carries only reg[7:2] (cfg_addr_make) => **>=0x100 is unaddressable** (measured readback lands at 0x00) => this model places
    //     the SR-IOV face **on the ext face 0x80..0xAF** (the EP-side ext face is otherwise empty; NEVER on the USP side, where 0x80+ holds MC/BC/VC/NTB,
    //     demultiplexed by `sriov_cap` => no interference). Real RTL co-simulation uses the core-internal extended cfg (NEVER this stub's job).
    //   Layout (base 0x80): **0x80** extended header ([15:0] CapID=0x0010 [19:16] Ver [31:20] Next)
    //   - 0x84 Control/Status ([0] VFEnable [1] VFMSE [4] ARI; Status [16]/[17] real values)
    //   - 0x88 InitialVFs|TotalVFs; 0x8C NumVFs; 0x90 FirstVFOffset|VFStride
    //   - 0x94 VF Device ID; 0x98 Supported Page; 0x9C System Page
    //   - **0xA0/0xA4 = VF BAR0 (low/high halves; 64-bit pair)** (NOTE: all VFs **share one** BAR table;
    //     hardware applies a +k x size address offset per VF order -- this model follows the same convention, see EpStubModule::vf_decode)
    bool     sriov_cap = false;
    uint16_t sriov_ctl = 0;                 // [0]=VF Enable [1]=VF MSE [4]=ARI Capable(RW)
    uint16_t sriov_numvf = 0;               // NumVFs (RW; NEVER > TotalVFs)
    uint16_t sriov_initvf = 0, sriov_totalvf = 0;
    uint16_t sriov_vf_off = 0, sriov_vf_stride = 1;
    uint16_t sriov_vf_did = 0;
    CfgBar   vf_bar0;                       // VF BAR0 (sizing semantics same as normal BAR; single source of truth)
    bool sriov_vf_enabled() const { return sriov_cap && (sriov_ctl & 0x1u) != 0; }
    bool sriov_vf_mse()     const { return sriov_cap && (sriov_ctl & 0x2u) != 0; }
    // For testbench/enumeration: configure SR-IOV in one call (NOTE: VF BAR physical attributes are **pre-set and locked** here)
    void set_sriov(int total, int num, uint16_t vf_did, uint64_t vf_bar_size,
                   uint16_t vf_off = 1, uint16_t stride = 1, bool bar64 = false) {
        sriov_cap = true;
        sriov_initvf = sriov_totalvf = (uint16_t)total;
        sriov_numvf = (uint16_t)num;
        sriov_vf_did = vf_did;
        sriov_vf_off = vf_off; sriov_vf_stride = stride;
        vf_bar0 = CfgBar{}; vf_bar0.idx = 0;
        vf_bar0.size = vf_bar_size; vf_bar0.is64 = bar64;
    }
    // BAR0 base of VF k = PF's VF BAR0 base + k x size (hardware offset convention)
    uint64_t vf_bar_base(int k) const { return vf_bar0.base + (uint64_t)k * vf_bar0.size; }
    // -- * T48: ARI (Alternative RID; extended face **0xB0**, same convention as SR-IOV: spec offset 0x100+ is unaddressable) --
    //   0xB0 extended header (CapID 0x000E); 0xB4 Control/Status ([0] MFVC En [1] ACS FG En [6:4] Function Group;
    //   [15:8]/[23:16] = MFVC/ACS Function Groups(RO=0))
    //   * Gate: **ARI Capable Hierarchy** (SR-IOV ctl bit4) set before NumVFs is written => VF function number is **8 bits**
    bool     ari_cap = false;
    uint16_t ari_ctl = 0;
    bool     ari_hier() const { return sriov_cap && (sriov_ctl & 0x10u) != 0; }   // ARI Capable Hierarchy
    void set_ari() { ari_cap = true; }

    uint8_t  pbus = 0, sec = 0, sub = 0;
    uint16_t mem_base = 0, mem_limit = 0;
    uint32_t io_base_up = 0;                   // * T52: 0x30 I/O Base Upper 16 (32-bit IO; bit0 = indicator bit)
    // -- * T52 (B7): **hot-plug Slot face** (model convention: 0x50 Slot Cap; 0x54 = {Slot Ctl (low) | Slot Sts (high)};
    //   WARNING: real device = PCIe cap +0x14/+0x18/+0x1A (this model's PCIe cap is abbreviated, hence a separate face; bridge maps it at M6b) ----------
    bool     slot_cap = false;                 // set on the port side => this port supports hot-plug
    uint32_t slot_cap_reg = 0;                 // [0] Attention Btn; [1] Power Ctl; [2] MRL; [5] Surprise
    uint16_t slot_ctl = 0;                     // [0..5] per-feature enables; [7:6]/[9:8] indicators; [10] **Power Controller** (1 = power off); [12] DLLSC En
    uint16_t slot_sts = 0;                     // [0] Attention; [1] PowerFault; [2] MRL Chg; [3] **Presence Chg**;
                                               // [4] **Command Completed**; [5] MRL; [6] **Presence State**; [8] DLLSC
    uint64_t slot_irq_cnt = 0;                 // HP interrupt count (NEVER silent)
    uint32_t slot_cmd_cnt = 0;                 // number of Command Completed events
    void set_slot() { slot_cap = true; slot_cap_reg = 0x00000023u; }   // Attention+Power ctl+Surprise
    // -- * B7-a (2026-10-16): **SHPC + MRL interlock + hp_pme** ------------------------------
    //   - **MRL** (Manually Operated Retention Latch; spec Section 6.7.3.2/6.7.3.4): **MRL Sensor State**
    //     is in Slot Status [4] (1 = open); * **interlock**: MRL open => (1) **refuse power-on** (command still completes but no power)
    //     (2) opened while powered => **automatic power-off** (T52's power-off gate => both directions blocked automatically).
    //   - **SHPC** (Standard Hot-Plug Controller; legacy; CapID **0x06**) => model convention = port-side registers
    //     **0x7C = {Ctl (low) | Sts (high)}** (NOTE: the real on-chain layout = M6b mapping table): Ctl[0] Power Controller;
    //     [1] MRL Sensor En; [2] Attention Btn En; Sts[0] Power Fault; [1] **MRL State**;
    //     [2] Presence.
    //   - **hp_pme sideband**: slot events (insert/remove/attention/MRL) => **PME stage** (gate `hp_pme_en`; NEVER silent).
    bool     shpc_cap = false;
    uint16_t shpc_ctl = 0, shpc_sts = 0;
    bool     mrl_open = false;
    bool     ignore_mrl = false;               // testbench red mode: ignore the interlock (discriminative)
    bool     hp_pme_en = true;
    uint64_t mrl_blocked = 0, mrl_power_off = 0, shpc_cmds = 0, hp_pme = 0;
    bool slot_powered() const { return (slot_ctl & 0x0400u) == 0; }     // [10] = 1 => powered off
    void hp_pme_event() { if (hp_pme_en) ++hp_pme; }
    void set_mrl(bool open);                   // interlock: closed=>open while powered => automatic power-off + explicit accounting
    void set_shpc() { shpc_cap = true; }
    // -- * T52 (B6): **DPC face** (extended CapID **0x001D**; ext **0x1B0..0x1B8**; spec = 16-bit register packing) ------
    //   NOTE (2026-11-22): moved off 0x100, which is reserved for the first
    //   extended capability (SR-IOV / DSN anchor).  The old 0x100..0x108
    //   placement collided with SR-IOV and made the ECAP chain ambiguous.
    //   0x1B4 = {DPC Control[31:16] | DPC Capability[15:0]}; 0x1B8 = {DPC Error Source ID[31:16] | DPC Status[15:0]}
    static const uint16_t CAP_DPC = 0x1B0;   // DPC (0x1B0..0x1B8)
    bool     dpc_cap = false;
    uint16_t dpc_ctl = 0, dpc_sts = 0, dpc_err_src = 0;
    void set_dpc() { dpc_cap = true; }        // DPC Cap: software trigger (bit7) + poisoned blocking (bit6) support
    bool dpc_enabled() const { return (dpc_ctl & 0x3u) != 0; }        // [1:0] != 00 = enabled
    bool dpc_triggered() const { return (dpc_sts & 0x1u) != 0; }      // [0] Trigger Status(RW1CS)
    // DPC Capability value ([4:0] interrupt message number; [6] poisoned blocking support; [7] **software trigger support**; [19:16] version)
    uint32_t dpc_cap_reg() const { return 0x000000C0u; }   // [7] SW trigger support + [6] poisoned blocking support (WARNING: version bits are in the **extended header**)
    bool slot_power_off() const { return (slot_ctl & 0x0400u) != 0; }   // [10]
    uint16_t pref_base = 0, pref_limit = 0;   // 0x24/0x26 (prefetch window; low 4 bits = 0x1 => **64-bit form**)
    uint32_t pref_base_up = 0, pref_limit_up = 0;   // 0x28/0x2C (prefetch window high 32; valid only in 64-bit form)
    uint16_t io_base = 0, io_limit = 0;       // * T52: 0x1C (4KB granularity; NOTE: when the low half is void, window = 0)
    uint16_t bridge_ctrl = 0;
    // -- MSI (EP) --
    bool     msi_cap = false;
    uint16_t msi_ctrl = 0;
    uint32_t msi_addr = 0;
    uint16_t msi_data = 0;
    // -- MSI-X (P2; Cap ID 0x11; **table lives in a BAR** => entries are programmed via **memory writes**; NEVER registers) ----
    //   Layout (spec): 0x60 = Message Control ([15]=Enable [14]=Function Mask [10:0]=TableSize=N-1, RO)
    //   0x64 = Table Offset/BIR ([2:0]=BIR; [31:3]=offset); 0x68 = PBA Offset/BIR;
    //   entry 16B each = {AddrLo, AddrHi, Data, Vector Control ([0]=mask)}
    bool     msix_cap = false;
    uint16_t msix_ctrl = 0;
    uint32_t msix_tbir = 0;
    uint32_t msix_pbir = 0;
    int      msix_nvec = 0;
    // -- * T50 (T-5 closed): all capability faces now live at **real extended cfg offsets (>=0x100; reg = 10 bits => 4KB)** ------
    //   NOTE: the old "ext face hack (0x80..0xFC)" is retired; layout (spec form: header + registers following) --
    static const uint16_t CAP_SRIOV = 0x100;   // SR-IOV(0x100..0x127;10 regs)
    static const uint16_t CAP_ARI   = 0x128;   // ARI(0x128..0x12F;2 regs)
    static const uint16_t CAP_AER   = 0x130;   // AER(0x130..0x15B;11 regs)
    static const uint16_t CAP_REBAR = 0x160;   // ReBAR(0x160..0x16F;4 regs)
    static const uint16_t CAP_LNKPM = 0x170;   // link power ASPM/L1SS (0x170..0x17B; 3 regs)
    // -- * T58: **DevCtl2 / DevCap2 bit numbers aligned once against the real device** (NOTE: old T55 bit numbers off by 2 = wrong; three-source cross-check, DO NOT invent) --
    //   (1) DWC reference manual Section 3.2.4.11 `DEVICE_CONTROL2_DEVICE_STATUS2_REG` (B+0x28) bit table:
    //      [5] ARI Fwd; **[6] AtomicOp Requester En; [7] AtomicOp Egress Blocking;
    //      [10] LTR Mechanism En; [12] 10-Bit Tag Requester En; [14:13] OBFF En**;
    //      (same manual `DEVICE_CAPABILITIES2_REG` (B+0x24) bit table: **[11] LTR Supported;
    //      [16] 10-Bit Tag Completer Supported; [17] 10-Bit Tag Requester Supported;
    //      [19:18] OBFF Supported**; NOTE: the spec's DevCtl2 has **no** "Completer Enable" bit -- all 16 bits are taken by the table above)
    //   (2) Linux header `/usr/include/linux/pci_regs.h:655-666` (ATOMIC_REQ 0x40; ATOMIC_EGRESS 0x80;
    //      LTR_EN 0x400; OBFF [14:13]; ARI 0x20); (3) customer RTL `pig_cdm_cfg_reg.sv:4176` --
    //      `cfg_reg_144 = {2'h0, cfg_ari_fwd_en[5], cfg_cpl_timeout_disable[4], cfg_cpl_timeout_range[3:0]}`
    //      + `:4145` `cfg_reg_140/141/142` (DevCap2 bit-for-bit identical to (1)).
    //   * **Receive side now keys off the capability face**: `MLF_TAG10` gate = **DevCap2[16] Completer Supported** (R/O capability) --
    //     the old T55 `tag10_cpl_en()` (DevCtl2[14], a bit that does not exist in the spec) is retired.
    //   model convention: **0x78 = DevCtl2** (low 16; high 16 = **DevSts2 = 0**); **0x6C = DevCap2** (32 bits);
    //   real device = PCIe cap + 0x28 / + 0x24 => bridge mapping table at M6b.
    //   WARNING: this configuration has `CX_TAG_SIZE = 8` => the 10-bit tag **field does not exist** (the testbench verifies with a **synthetic 10-bit config**:
    //     a copy of `CoreCfg` with `CX_TAG_SIZE = 10`; NOTE: the contract follows to 10 bits -- same method as T-1b/T-5).
    uint16_t dev_ctl2 = 0;
    uint32_t dev_cap2 = 0;                  // * T58: 32-bit (real bit numbers [19:18]/[17]/[16]/[11] need > 15 bits)
    void set_10bit_tag(bool cpl_sup, bool req_sup) {      // WARNING: argument order = (Completer, Requester)
        dev_cap2 = (uint32_t)((req_sup ? 0x20000u : 0u) | (cpl_sup ? 0x10000u : 0u));   // [17]/[16]
    }                                                     // (spec: req may be set only if cpl is set => this order)
    bool atomic_req_en()      const { return (dev_ctl2 & 0x0040u) != 0; }   // [6]
    bool atomic_egr_blocked() const { return (dev_ctl2 & 0x0080u) != 0; }   // [7]
    bool tag10_req_en()       const { return (dev_ctl2 & 0x1000u) != 0; }   // [12]
    bool tag10_req_sup()      const { return (dev_cap2 & 0x20000u) != 0; }  // DevCap2[17]
    bool tag10_cpl_sup()      const { return (dev_cap2 & 0x10000u) != 0; }  // DevCap2[16]
    // -- * T56 (C10)/T58: LTR / OBFF (bit numbers aligned once in T58; capability face = real DevCap2 bit numbers) --------------
    //   - **LTR**: DevCtl2[10]; capability = RTL actual read (`pig_cdm_cfg_reg.sv:4119` `cfg_ltr_m_suprtd = 1'b0`;
    //     macro `pig_CX_LTR_M_ENABLE_VALUE = 0` = hardwired 0 in this configuration)
    //   - **OBFF**: DevCtl2[14:13]; capability = RTL actual read (`cfg_obff_support = 2'b00` `:4114`; macro `CX_OBFF_SUPPORT = 0`)
    void set_ltr(bool cap)  {
        ltr_cap = cap;
        if (cap) dev_cap2 |= 0x0800u; else dev_cap2 &= ~0x0800u;              // DevCap2[11]
    }
    bool ltr_sup()    const { return (dev_cap2 & 0x0800u) != 0; }
    bool ltr_en()     const { return (dev_ctl2 & 0x0400u) != 0; }              // [10]
    void set_obff(uint8_t sup) {
        dev_cap2 = (dev_cap2 & ~0x000C0000u) |
                   (uint32_t)((sup & 0x3u) << 18);                             // DevCap2[19:18]
    }
    uint16_t obff_sup() const { return (uint16_t)((dev_cap2 >> 18) & 0x3u); }
    uint8_t  obff_en()  const { return (uint8_t)((dev_ctl2 >> 13) & 0x3u); }   // [14:13]

    // -- * T56 (C12): **Expansion ROM BAR** (Type0 = 0x30; model convention) ----------------------------
    //   Bit level = RTL actual read (`pig_cdm_cfg_reg.sv:2583-2600`): **bit0 Enable**; **[3:1] Validation
    //   Status (3b)**; **[7:4] Validation Details (4b)** (valid when Enable = 0; DBI writes `lbc_cdm_data[3:1]`
    //   /`[7:4]` = same layout); base [31:11]; window = `pig_cdm_bar_match.sv:251` (**32-bit window**, high 32 = 0);
    //   databook Section 3.22 (p.364): fields live in EXP_ROM_BASE_ADDR_REG / TYPE1_EXP_ROM_BASE_REG.
    uint32_t rom_bar = 0;
    uint32_t rom_size = 0;                   // 0 = not implemented
    bool     rom_en = false;
    uint8_t  rom_val_status = 0;             // [3:1]
    uint8_t  rom_val_details = 0;            // [7:4]
    // ROM contents (model profile).  Enabling the window with set_rom()
    // synthesizes a standard PCI expansion-ROM image (55AA / E9 / PCIR);
    // set_rom_content() installs an owner-provided image instead.  The image
    // is kept separate from the BAR segments (EpStubModule decode).
    std::vector<uint8_t> rom_data;
    void set_rom(uint32_t base, uint32_t size, bool en);
    void set_rom_content(const std::vector<uint8_t>& data) { rom_data = data; }
    void ensure_rom_content();               // build the default image when empty
    bool rom_match(uint64_t addr) const {    // inside the window? (same expression as RTL)
        if (!rom_en || rom_size == 0) return false;
        if (addr >> 32) return false;        // 32-bit window only
        return (addr & ~((uint64_t)rom_size - 1)) == rom_bar;
    }
    void rom_validate(uint8_t st, uint8_t det) { rom_val_status = (uint8_t)(st & 7u); rom_val_details = (uint8_t)(det & 0xFu); }
    // -- * T54: C11 (ACS / ATS / PASID) extended capability face (**real extended offsets**; same method as T-5) ---------------
    //   WARNING: actual reads for this configuration: `CX_ACS_ENABLE` not enabled (constant table line 2314 comment) / `CX_ATS_ENABLE_VALUE = 0` /
    //     `CX_PASID_ENABLE_VALUE = 0` / `FLT_Q_PRFX_WIDTH = 0` (=> **the port has no TLP prefix face**;
    //     on the real device PASID goes through `client*_tlp_prfx`; this configuration's width is 0 => no contract carrier on the data plane)
    //   => this line implements all three as **model features** using the **same method as T48/T53** (macros only control parameters); after the customer enables the macros, map at M6b.
    static const uint16_t CAP_ACS   = 0x180;   // ACS (0x180..0x18B; header + Cap + Ctl)
    static const uint16_t CAP_ATS   = 0x190;   // ATS (0x190..0x19B; header + Cap + Ctl)
    static const uint16_t CAP_PASID = 0x1A0;   // PASID (0x1A0..0x1AB; header + Cap + Ctl)
    // -- * Standard PCIe Multicast extended capability (ID 0x0012) ----------
    //   PCIe r4.0 sec 7.9.11; switch/RC ports only (an endpoint must not
    //   implement MC_Overlay_BAR).  Layout (relative to CAP_MC):
    //   +00 header; +04 Cap(16)/Ctl(16); +08 MC_Base(64);
    //   +10 MC_Receive(64); +18 MC_Block_All(64); +20 MC_Block_Untranslated(64);
    //   +28 MC_Overlay_BAR(64).  MaxGroups=64; ECRC regeneration unsupported.
    static const uint16_t CAP_MC = 0x3E0;
    bool     mc_cap = false;
    bool     mc_enable = false;
    uint16_t mc_groups = 1;                 // 1..64
    uint8_t  mc_index = 0;                  // MC_Index_Position
    uint64_t mc_base = 0;                   // MC_Base_Address (4KB aligned)
    uint64_t mc_receive = 0;
    uint64_t mc_block_all = 0;
    uint64_t mc_block_untrans = 0;
    uint32_t mc_overlay_size = 0;
    uint64_t mc_overlay_base = 0;
    uint64_t mc_reject = 0;                 // invalid MC control writes (never silent)
    bool mc_enabled() const { return mc_cap && mc_enable; }
    uint32_t mc_num_groups() const { return mc_groups ? mc_groups : 1u; }
    void set_mc(bool enable, uint64_t base, uint32_t index, uint32_t groups);
    void set_mc_receive(uint64_t bitmap) { mc_receive = bitmap; }
    void set_mc_block(uint64_t all, uint64_t untrans) {
        mc_block_all = all; mc_block_untrans = untrans;
    }
    void set_mc_overlay(uint32_t size, uint64_t base) {
        mc_overlay_size = size; mc_overlay_base = base & ~0x3FULL;
    }
    // -- **ACS** (extended CapID **0x000D**; databook Section 3.24 + Table 3-95 -- switch port feature) ----------
    //   0x184 Capability: [0] Source Validation V [1] Translation Blocking B [2] P2P **Request** Redirect R [3] P2P **Completion** Redirect C
    //     [4] Upstream Forwarding U [5] Egress Control E [6] Direct Translated P2P T; [15:8] Egress Control Vector Size (= 0 => no vector register)
    //   0x188 Control: enable bits in the same order [0..6] (this model: E/T size 0 => **E/T not implemented**, recorded)
    //   * Mandatory behavior (databook Table 3-95): V/B violation => **NP completion with CA** + **AER Uncorr[21] ACS Violation**
    //     + Signaled Target Abort (NEVER silent); R/C => "the controller takes no action; the **application** is responsible for redirect/completion"
    //     => this model (application side) = **does not send to the peer, forwards to RC instead** (+ counters).
    bool     acs_cap = false;
    uint16_t acs_abil = 0;                     // capability bits (V/B/R/C/U/E/T)
    uint16_t acs_ctl = 0;                      // control bits (same-order enables)
    void set_acs(uint16_t abil) { acs_cap = true; acs_abil = abil & 0x007Fu; }
    bool acs_v() const { return acs_cap && (acs_ctl & 0x01u) != 0; }   // source validation enable
    bool acs_b() const { return acs_cap && (acs_ctl & 0x02u) != 0; }   // translation blocking enable
    bool acs_r() const { return acs_cap && (acs_ctl & 0x04u) != 0; }   // P2P request redirect
    bool acs_c() const { return acs_cap && (acs_ctl & 0x08u) != 0; }   // P2P completion redirect
    bool acs_u() const { return acs_cap && (acs_ctl & 0x10u) != 0; }   // upstream forwarding
    // -- **ATS** (extended CapID **0x000F**; minimal EP-side set) ---------------------------------
    //   0x194 Capability: [4:0] Invalidate Queue Depth; [5] Page Aligned Request; [6] Global Invalidate Supported;
    //     [7] Relaxed Ordering Supported; 0x198 Control: **[15] Enable** + [4:0] **STU**
    //     (smallest translation unit = 2^STU bytes; Linux `PCI_ATS_CTRL_ENABLE` = 0x8000).
    bool     ats_cap = false;
    uint16_t ats_cap_reg = 0, ats_ctl = 0;
    void set_ats(uint16_t capreg, uint8_t stu) { ats_cap = true; ats_cap_reg = capreg; ats_ctl = (uint16_t)(stu & 0x1Fu); }
    uint32_t ats_stu_bytes() const { return 1u << (ats_ctl & 0x1Fu); }
    bool ats_en() const { return ats_cap && (ats_ctl & 0x8000u) != 0; }
    // -- **PASID** (extended CapID **0x001B**; RTL actual read `pig_PCIE_PASID_ECAP_ID`; minimal EP-side set; * T56 fixed 0x1F->0x1B) ------------------------------
    //   0x1A4 Capability: [2:0] Max PASID Width (0..20); [7:3] reserved; 0x1A8 Control:
    //   [0] PASID Enable; [1] Execute Permission Enable; [2] Privileged Mode Enable
    bool     pasid_cap = false;
    uint16_t pasid_cap_reg = 0, pasid_ctl = 0;
    void set_pasid(uint16_t capreg) { pasid_cap = true; pasid_cap_reg = capreg; }
    bool pasid_en()   const { return pasid_cap && (pasid_ctl & 0x1u) != 0; }
    // -- * 2026-11-22: Extended-capability chain ---------------------------------
    //   PCIe extended capabilities start at byte offset 0x100 and are linked by
    //   the header Next field (bits [31:20], in DW units).  The model keeps
    //   stable per-capability offsets (below) and emits the real Next pointer.
    //   * 0x100 is the anchor: SR-IOV when enabled, otherwise the standard
    //     Device Serial Number ECAP (ID 0x0003).  This guarantees that a
    //     Linux-style walk starting at 0x100 always finds a valid header.
    //   * DPC moved from 0x100 to 0x1B0 (old placement collided with SR-IOV).
    //   * 0x170 remains the legacy model-only link-power face (no ECAP header);
    //     the standard L1SS ECAP lives at CAP_L1SS.
    static const uint16_t CAP_DSN   = 0x100;   // Device Serial Number (anchor when no SR-IOV)
    static const uint16_t CAP_VC    = 0x200;   // VC (0x200..0x277; VC0..VC7 + WRR table)
    static const uint16_t CAP_LTR   = 0x280;   // LTR (0x280..0x287)
    static const uint16_t CAP_PTM   = 0x2A0;   // PTM (0x2A0..0x2A7)
    static const uint16_t CAP_DOE   = 0x2B0;   // DOE (0x2B0..0x2C7)
    static const uint16_t CAP_L1SS  = 0x2D0;   // L1 PM Substates (0x2D0..0x2E3)
    static const uint16_t CAP_SPE   = 0x300;   // Secondary PCIe (SPE, 0x0019; 0x300..0x317)
    static const uint16_t CAP_PL64  = 0x330;   // Physical Layer 64 GT/s (0x0031; 0x330..0x343)
    static const uint16_t CAP_DEV3  = 0x350;   // Device 3 (0x002F; 0x350..0x35F)
    static const uint16_t CAP_IDE   = 0x360;   // IDE (0x0030; 0x360..0x36F)
    static const uint16_t CAP_FLIT  = 0x380;   // Flit Error Logging (0x0032; 0x380..0x397)
    static const uint16_t CAP_LMARG = 0x3A0;   // Lane Margining (0x0027; 0x3A0..0x3AF)
    //   DSN anchor (standard ECAP 0x0003; 64-bit serial number).
    bool     dsn_cap = true;
    uint64_t dsn_serial = 0x3031303030303031ULL;   // "10000101" model default
    void set_dsn(uint64_t s) { dsn_cap = true; dsn_serial = s; }
    // -- * P2 (2026-11-22): standard VC Extended Capability (ID 0x0002) ------
    //   The data plane (nvc / TC->VC / arbitration) is still SwitchCfg; this
    //   is the standard register view.  VC0 is always enabled; VC1..N follow
    //   vc_nvc.  The VC Arbitration Table = the model's 4-bit WRR weights.
    bool     vc_cap = false;
    uint8_t  vc_nvc = 1;                       // 1..8 (Port VC Cap1 EVCC = nvc-1)
    uint8_t  vc_tc2vc[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    uint8_t  vc_arb = 0;                       // 0 strict / 1 RR / 2 WRR (model control)
    uint8_t  vc_weight[8] = {1, 1, 1, 1, 1, 1, 1, 1};
    uint16_t vc_port_ctl = 0, vc_port_sts = 0;
    uint32_t vc_res_cap[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t vc_res_ctl[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint16_t vc_res_sts[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint16_t vc_arb_table[16] = {0};
    void set_vc(uint8_t nvc, uint8_t arb = 0);
    void vc_refresh_maps();                    // build TC/VC map + enable from vc_nvc/tc2vc
    uint32_t vc_read(uint16_t reg) const;
    void     vc_write(uint16_t reg, uint32_t val, uint8_t be);
    // -- * P1 (2026-11-22): standard LTR / OBFF / PTM ECAP register blocks ----
    //   The message/semantic path already exists (C10/T56); these are the
    //   standard configuration-space faces.  LTR enable remains DevCtl2[10].
    bool     ltr_cap = false;
    uint16_t ltr_max_snoop = 0, ltr_max_nosnoop = 0;
    void set_ltr_ecap(uint16_t snoop, uint16_t nosnoop) {
        ltr_cap = true; ltr_max_snoop = snoop; ltr_max_nosnoop = nosnoop;
        dev_cap2 |= 0x0800u;                          // DevCap2[11] LTR Supported
    }
    //   NOTE: OBFF has **no** PCIe extended capability.  It is fully modeled
    //   by DevCap2[19:18] / DevCtl2[14:13] + the OBFF message path (already
    //   implemented in T56); no ECAP block is added here.
    bool     ptm_cap = false;
    uint32_t ptm_cap_reg = 0, ptm_ctl = 0;
    void set_ptm(bool req, bool resp, uint8_t gran = 0) {
        ptm_cap = true;
        ptm_cap_reg = (req ? 0x1u : 0u) | (resp ? 0x2u : 0u) |
                      ((uint32_t)(gran & 0xFFu) << 8);
    }
    // -- * P3 (2026-11-22): DOE (Data Object Exchange, 0x002E) ---------------
    //   Minimal mailbox: write mailbox loops back to read mailbox and raises
    //   Data Object Ready.  Full DOE/SPDM object protocols are out of scope.
    bool     doe_cap = false;
    uint32_t doe_cap_reg = 0x00000001u;               // [0] interrupt support
    uint32_t doe_ctl = 0, doe_sts = 0;
    uint32_t doe_write_mb = 0, doe_read_mb = 0;
    uint32_t doe_objects = 0;
    void set_doe(uint32_t cap = 0x00000001u) { doe_cap = true; doe_cap_reg = cap; }
    void doe_abort() { doe_sts = 0; }
    // -- * P1 (2026-11-22): L1 PM Substates ECAP (0x001E) --------------------
    bool     l1ss_cap = false;
    uint32_t l1ss_cap_reg = 0x0000001Fu;              // PCIPM/ASPM L1.1+L1.2 + L1 PM SS
    uint32_t l1ss_ctl1 = 0, l1ss_ctl2 = 0;
    void set_l1ss(uint32_t cap = 0x0000001Fu) { l1ss_cap = true; l1ss_cap_reg = cap; }
    uint32_t l1ss_status() const;
    // -- * P4 (2026-11-22): SPE / Gen6 ECAP register faces -------------------
    //   SPE = Secondary PCI Express Extended Capability (ID 0x0019; Intel's
    //   SPEECH abbreviation).  Link training/equalization is downstream RTL;
    //   these are configuration-space model faces, not a second LTSSM.
    bool     spe_cap = false;
    uint32_t spe_link_ctl3 = 0, spe_lane_err = 0, spe_eq_ctl = 0;
    void set_spe() { spe_cap = true; }
    bool     pl64_cap = false;
    uint32_t pl64_cap_reg = 0, pl64_ctl = 0, pl64_sts = 0, pl64_eq_ctl = 0;
    void set_pl64() { pl64_cap = true; }
    bool     dev3_cap = false;
    uint32_t dev3_cap_reg = 0, dev3_ctl = 0, dev3_sts = 0;
    void set_dev3() { dev3_cap = true; }
    bool     ide_cap = false;
    uint32_t ide_cap_reg = 0, ide_ctl = 0, ide_sts = 0;
    void set_ide() { ide_cap = true; }
    bool     flit_cap = false;
    uint32_t flit_log1 = 0, flit_log2 = 0;
    uint16_t flit_cnt_ctl = 0, flit_cnt_sts = 0;
    uint32_t flit_err_count = 0;
    void set_flit() { flit_cap = true; }
    void flit_inject(uint32_t l1, uint32_t l2) {
        flit_log1 = l1; flit_log2 = l2; ++flit_err_count; flit_cnt_sts |= 0x1u;
    }
    bool     lmargin_cap = false;
    uint16_t lm_port_cap = 0, lm_port_sts = 0, lm_lane_ctl = 0, lm_lane_sts = 0;
    void set_lmargin() { lmargin_cap = true; }
    // -- Extended-capability chain helpers -----------------------------------
    struct EcapRef { uint16_t base; uint16_t id; };
    std::vector<EcapRef> ecap_entries() const;
    uint16_t ecap_next(uint16_t base) const;
    uint32_t ecap_hdr(uint16_t base, uint16_t id, uint8_t ver = 1) const;
    // -- * T49-2: **AER Advanced Error Reporting** (extended CapID 0x0001; base = CAP_AER) ------
    //   Layout (+base = 0xC0): 0xC0 extended header / 0xC4 Uncorr Status (RW1C) / 0xC8 Uncorr Mask /
    //   0xCC Uncorr Severity / 0xD0 Corr Status (RW1C) / 0xD4 Corr Mask / 0xD8 CapCtl (ECRC bits) /
    //   0xDC..0xE8 Header Log (4DW; RO -- **captured** on error injection).
    //   Reset values (spec): Severity only **bit4 (Data Link Protocol Error) = 1 (fatal)**, all others 0;
    //   CapCtl: Generation/Check **Capable** (bit5/6) = 1, En = 0.
    // -- * T49-3: **PM capability** (PCI power management; Cap ID 0x01; on-chain **0x70**) ------------------
    //   0x70 = header {CapID 0x01 | Next | PMC} (PMC: Version 3; **PME Support = full D0..D3cold support**;
    //   DSInit; D1/D2 not supported) / 0x74 = PMCSR {PowerState[1:0]; No_Soft_Reset[3];
    //   PME_En[8]; Data_Select[12:9]; Data_Scale[14:13]; **PME_Status[15] (RW1C, sticky)**}
    //   + Data byte ([23:16]; per Data_Select semantics = simplified: direct store/read).
    //   D-state semantics (this model): **D3hot => memory transactions not accepted** (UR + explicit accounting; config face as usual).
    bool     pm_cap = false, pm_is_bridge = false;
    uint16_t pm_ctrl = 0;                    // PMCSR low 16 (PowerState/PME_En/PME_Status...)
    uint8_t  pm_data = 0;                    // Data register ([23:16] bit field)
    void set_pm(bool bridge = false) { pm_cap = true; pm_is_bridge = bridge; }
    int  pm_power_state() const { return (int)(pm_ctrl & 0x3u); }        // 0=D0 3=D3hot
    bool pm_pme_en()      const { return (pm_ctrl & 0x0100u) != 0; }
    bool pm_pme_status()  const { return (pm_ctrl & 0x8000u) != 0; }
    void pm_set_pme_status() { pm_ctrl |= 0x8000u; }                    // device sets it (sticky)
    // -- * T49-4: **link power face (ASPM/L1SS)**; this model places it at ext **0xF0..0xF8** (NOTE: real device has it at
    //   PCIe cap 0x50/0x54/0x58 -- this model already uses 0x50 for MSI, hence a **separate face**; used for bridge-side mapping) ------
    //   0xF0 LinkCtl {[0] L0s En; [1] L1 En; [8] CLKREQ En};
    //   0xF4 LinkSts {[11:10] ASPM Support (RO = 0b11); [1:0] current state (**written by the link state machine**);
    //   [3] L1.1 active; [4] L1.2 active}; 0xF8 L1SS Ctl1 {[2] ASPM L1.1 En; [3] ASPM L1.2 En}.
    bool     link_pm_cap = false;
    uint16_t link_ctl = 0, link_sts = 0, l1ss_ctl = 0;
    void set_link_pm() { link_pm_cap = true; link_sts = 0x0C00u; l1ss_cap = true; }   // ASPM Support = L0s+L1(RO)
    bool link_l0s_en() const { return (link_ctl & 0x1u) != 0; }
    bool link_l1_en()  const { return (link_ctl & 0x2u) != 0; }
    bool link_clkreq_en() const { return (link_ctl & 0x100u) != 0; }
    // Legacy model bits [2]/[3] are ORed with the standard L1SS CTL1 bits
    // (PCI-PM L1.1/L1.2 and ASPM L1.1/L1.2) so both faces drive the LTSSM.
    bool l1ss_l11_en() const { return (l1ss_ctl & 0x4u) || (l1ss_ctl1 & 0x2u) || (l1ss_ctl1 & 0x8u); }
    bool l1ss_l12_en() const { return (l1ss_ctl & 0x8u) || (l1ss_ctl1 & 0x1u) || (l1ss_ctl1 & 0x4u); }
    // -- * T49-5/R5: **ReBAR** (extended CapID 0x0015; Linux `pci_regs.h` standard layout) ----
    //   0x160 extended header / 0x164 Capability: supported-size bitmap[27:4]
    //   / 0x168 Control: BAR Index[2:0] · Number of Resizable BARs[7:5] (RO = 1) · BAR Size N[12:8]
    //   / 0x16C reserved (RO 0).  (Old private layout retired.)
    //   WARNING: semantics: controlled **only when that BAR's MSE (Command[1]) = 0** (otherwise ignored + explicit accounting);
    //   size change => **base address returns to reset value 0** (software must re-probe/re-program).
    bool     rebar_cap = false;
    int      rebar_bar = 0;                          // which BAR is resizable (this model 0 = BAR0)
    uint32_t rebar_sizes = 0;                        // support bitmap (bit N => 1MB<<N)
    uint8_t  rebar_sel = 0;                          // current BAR Size selection (N)
    void set_rebar(int bar, uint32_t sizes, uint8_t init_n) {
        rebar_cap = true; rebar_bar = bar; rebar_sizes = sizes; rebar_sel = init_n;
    }
    bool     rebar_gated_block = false;              // * resize requests ignored while gated (MSE on) (explicit accounting)
    bool     rebar_ignore_gate = false;              // testbench red mode: ignore the MSE gate (fault injection)
    uint32_t rebar_read(uint16_t reg) const;
    void     rebar_write(uint16_t reg, uint32_t val, uint8_t be);
    uint32_t mc_read(uint16_t reg) const;
    void     mc_write(uint16_t reg, uint32_t val, uint8_t be);
    bool     aer_cap = false;
    uint32_t aer_uc_status = 0, aer_uc_mask = 0, aer_uc_sev = 0x00000010u;
    uint32_t aer_cor_status = 0, aer_cor_mask = 0, aer_cctl = 0x00000060u;
    uint32_t aer_hdr_log[4] = {0, 0, 0, 0};
    // * T51-2: **ECRC reflection switch** (per function; default **non-reflected** (PCIe convention); one-line switch for M6b comparison with the real device)
    bool     ecrc_reflected = false;
    void set_aer() { aer_cap = true; }
    uint32_t aer_read(uint16_t reg) const;
    void     aer_write(uint16_t reg, uint32_t val, uint8_t be);
    // -- * T49: Device Control (0x48 low 16; AER report enables [0..3]; **FLR = bit15**) --------
    uint16_t dev_ctl = 0, dev_sts = 0;
    bool     flr_pending = false;                    // * T49-5: FLR request (bit15 write 1 => set; owner executes)
    // -- * T53: C2 CRS / C13 Max_Payload_Size -- DevCtl bit meaning (**demultiplexed by function type**, spec) --------
    //   - **Type0** (EP): bit15 = **FLR** (self-clearing; NEVER stored in dev_ctl); [7:5] = Max_Payload_Size;
    //     [4] = Relaxed Ordering Enable (requester side; this model only stores the real value).
    //   - **Type1** (bridge): bit15 = **Bridge Configuration Retry Enable** ("Bridge only";
    //     RTL `pig_cdm_cfg_reg.sv:3215` original text) => **NEVER executes FLR** => stored in dev_ctl => drives top port
    //     `cfg_bridge_crs_en` (T53 entered into the contract).
    bool bridge_crs_en() const { return cfg_type == 1 && (dev_ctl & 0x8000u) != 0; }
    // Max_Payload_Size (DevCtl[7:5]: 000=128B ... 101=4096B; default 000 = 128B => reset value)
    uint32_t mps_bytes() const { return 128u << ((dev_ctl >> 5) & 0x7u); }
    // * T49-5: FLR reset (non-sticky registers return to reset values; **sticky bits preserved**: PME_En/PME_Status; AER Mask/Severity)
    void apply_flr() {
        command = 0; dev_ctl = 0; dev_sts = 0; flr_pending = false;
        msi_ctrl = 0; msix_ctrl = (uint16_t)((msix_nvec > 0) ? (msix_nvec - 1) : 0);
        pm_ctrl &= 0x8100u;                          // PowerState <- D0; NOTE: PME_En/PME_Status **preserved as sticky**
        msi_addr = 0; msi_data = 0;
        aer_uc_status = 0; aer_cor_status = 0;       // clear status (masks/severity preserved)
        for (size_t k = 0; k < bars.size(); ++k) {
            bars[k].base = 0; bars[k].sizing = false;    // base returns to reset value (size attributes preserved)
        }
        if (vf_cfg_flr) vf_cfg_flr();                // owner hook (VF table etc.)
        if (on_flr) on_flr();                        // * T53: owner hook (reset the CRS "not ready" state)
    }
    std::function<void()> vf_cfg_flr;                // (nullable) VF-side hook
    std::function<void()> on_flr;                    // (nullable) * T53: device reset (clears CRS window / answered state)
    // -- * T49-5: **INTx** (legacy interrupts; Interrupt Line/Pin 0x3C/0x3D; Command[10] Interrupt Disable) --
    uint8_t  int_line = 0, int_pin = 0;              // int_pin: 0 = none; 1..4 = INTA..D
    bool int_disabled() const { return (command & 0x0400u) != 0; }    // Command[10]
    bool devctl_corr_rep_en()     const { return (dev_ctl & 0x1u) != 0; }
    bool devctl_nonfatal_rep_en() const { return (dev_ctl & 0x2u) != 0; }
    bool devctl_fatal_rep_en()    const { return (dev_ctl & 0x4u) != 0; }
    bool devctl_ur_rep_en()       const { return (dev_ctl & 0x8u) != 0; }
    // For testbench/enumeration: configure in one call (entry count + table/PBA offsets + BIR; TableSize is RO => real value set here)
    void set_msix(int nvec, uint32_t tbl_off, uint32_t pba_off, int bir = 0) {
        msix_cap = true; msix_nvec = nvec;
        msix_ctrl = (uint16_t)((nvec > 0) ? (nvec - 1) : 0);
        msix_tbir = (tbl_off & ~0x7u) | ((uint32_t)bir & 0x7u);
        msix_pbir = (pba_off & ~0x7u) | ((uint32_t)bir & 0x7u);
    }
    // Table base = **real base** of the BAR selected by BIR + offset (NOTE: the base is written by enumeration => take this object's bars[])
    uint64_t msix_tbl_base() const {
        const size_t b = msix_tbir & 0x7u;
        return (b < bars.size()) ? (bars[b].base + (uint64_t)(msix_tbir & ~0x7u)) : 0;
    }
    bool msix_enabled()  const { return msix_cap && (msix_ctrl & 0x8000u) != 0; }
    bool msix_fn_mask()  const { return msix_cap && (msix_ctrl & 0x4000u) != 0; }

    // Side effect: register write (reg/val/be)
    std::function<void(uint16_t reg, uint32_t val, uint8_t be)> on_write;
    // Extended register face (>=0x80; private-convention registers such as MC/BC; owner lands semantics via ext_on_write)
    std::map<uint16_t, uint32_t> ext;
    std::function<void(uint16_t reg, uint32_t val)> ext_on_write;

    // Lay out BARs/capability chain by type; **MSI Cap**: Type0 attaches by default; Type1 **must explicitly** enable (T-2: the MSI interface face
    //   `ven_msi_*` is **upstream-port only** (databook Section 4.12/4.13) => the upstream Type1 bridge port attaches MSI; downstream (DSP) NEVER attaches)
    void init(uint8_t typ, int nbar, bool msi_cap_en = false);
    // Re-lay out the number of BAR slots (testbench use: multi-BAR device; NOTE: out-of-range writes to bars[] corrupt the heap -- the user must re-lay out first)
    void set_nbar(int nbar) { bars.assign((size_t)nbar, CfgBar{}); for (int i = 0; i < nbar; ++i) bars[(size_t)i].idx = (uint8_t)i; }
    // Grow slots (preserves existing; used by G7 testbench)
    void ensure_nbar(int nbar) {
        if ((int)bars.size() >= nbar) return;
        size_t old = bars.size();
        bars.resize((size_t)nbar);
        for (size_t i = old; i < bars.size(); ++i) bars[i].idx = (uint8_t)i;
    }
    // * Pre-set a BAR's **physical attributes** (size / 64-bit / prefetch) and clear base/sizing
    //   => type bits are **locked at pre-set time** (pit (2): a sizing write of 0xFFFFFFFF must not change is64; base-address echo bits likewise unchanged)
    void set_bar(int i, uint64_t size, bool is64 = false, bool prefetch = false) {
        ensure_nbar(i + 1);
        CfgBar& b = bars[(size_t)i];
        b.size = size; b.is64 = is64; b.prefetch = prefetch; b.is_io = false;
        b.sizing = false; b.base = 0;
    }
    // * Pre-set an **IO space BAR** (bit0 = 1; IO BARs are always 32-bit and non-prefetchable)
    void set_io_bar(int i, uint64_t size) {
        ensure_nbar(i + 1);
        CfgBar& b = bars[(size_t)i];
        b.size = size; b.is64 = false; b.prefetch = false; b.is_io = true;
        b.sizing = false; b.base = 0;
    }
    // * Size from sizing readback masks (**single source of truth**: enumeration program / testbench / checks all use this expression)
    //   lo_mask/hi_mask = sizing readback values; type bits are in lo bit2/bit3 (this expression takes only address bits)
    //   WARNING: 32-bit case = **32-bit mathematics** (NEVER use 64-bit ~m+1 => it would fill the upper bits with 0xFFFFFFFF_...);
    //     64-bit case = low half | high half << 32; mask 0 => not implemented (size 0)
    static uint64_t probe_size(uint32_t lo_mask, uint32_t hi_mask, bool is64) {
        const uint32_t lm = lo_mask & 0xFFFFFFF0u;
        if (!is64) return lm ? ((uint64_t)(~lm) + 1u) : 0;              // 32-bit
        const uint64_t m = (uint64_t)lm | ((uint64_t)hi_mask << 32);
        return m ? (~m + 1) : 0;
    }
    // * Sizing for an IO BAR (**32-bit IO space**; implemented bits = bits[31:2] => 4-byte granularity; bit0 always 1)
    static uint64_t probe_size_io(uint32_t lo_mask) {
        const uint32_t m = lo_mask & 0xFFFFFFFCu;
        return m ? ((uint64_t)(~m) + 1u) : 0;
    }
    // reg => BAR slot (hi = true means reg is the **high half** of a 64-bit pair); -1 = not a BAR register
    //   * a 64-bit pair occupies two slots => **the high half is not an independent slot** (pit (3): enumeration loops must skip it)
    int bar_of_reg(uint16_t reg, bool& hi) const;
    uint32_t read(uint16_t reg, uint8_t be) const;
    void     write(uint16_t reg, uint32_t val, uint8_t be);
    // * P2: SR-IOV extended face (0x100..0x13F; NOTE: demultiplexed from the ext private face -- see read/write)
    uint32_t sriov_read(uint16_t reg) const;
    void     sriov_write(uint16_t reg, uint32_t val, uint8_t be);

    // Derived
    bool mse() const { return (command & 0x0002u) != 0; }
    bool ioe() const { return (command & 0x0001u) != 0; }
    uint64_t mem_win_base()  const { return ((uint64_t)(mem_base  & 0xFFF0u)) << 16; }
    uint64_t mem_win_limit() const { return (((uint64_t)(mem_limit & 0xFFF0u)) << 16) | 0xFFFFFULL; }
    // -- Prefetch window (B-1): low 4 bits = 0x1 (set on both sides) => **64-bit form** (otherwise 32-bit) --------
    bool pref_is64() const { return (pref_base & 0x1u) != 0 && (pref_limit & 0x1u) != 0; }
    uint64_t pref_win_base() const {
        return (pref_is64() ? ((uint64_t)pref_base_up << 32) : 0) |
               ((uint64_t)(pref_base & 0xFFF0u) << 16);
    }
    uint64_t pref_win_limit() const {
        // WARNING: low 32 bits = register low half + **low 20 granularity bits**; the 64-bit form only **ORs in the high 32 bits**
        //   (NEVER OR 0xFFFFFFFF -- that would set [31:20] as well => the window would stretch to 4G)
        uint64_t lim = (((uint64_t)(pref_limit & 0xFFF0u)) << 16) | 0xFFFFFULL;
        if (pref_is64()) lim |= ((uint64_t)pref_limit_up << 32);
        return lim;
    }
    uint64_t bar_size(int i) const { return (i >= 0 && i < (int)bars.size()) ? bars[(size_t)i].size : 0; }
    uint64_t bar_base(int i) const { return (i >= 0 && i < (int)bars.size()) ? bars[(size_t)i].base : 0; }
    int      bar_count() const { return (int)bars.size(); }

    // Register byte enables (be[3:0], bit0 = least significant byte)
    static uint32_t merge_be(uint32_t oldv, uint32_t val, uint8_t be) {
        uint32_t out = oldv;
        for (int b = 0; b < 4; ++b)
            if ((be >> b) & 1u) out = (out & ~(0xFFu << (8 * b))) | ((val & (0xFFu << (8 * b))));
        return out;
    }
};

} // namespace pcie_tlm
