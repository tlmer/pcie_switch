// -----------------------------------------------------------------------------
// Solution One - private port contract layer (**declarations**) -- full set of core-top
//   application-side signals + packer
//   * Convention (2026-10-12 user directive): **packed signals are taken from the top of
//     `pig_DWC_pcie_ctl.sv`** (the earlier 771/300 "simple packing" was a user
//     simplification, RETIRED and not the contract);
//     **many signal widths are parameters** (TRGT_DATA_WD / CX_TAG_SIZE / FLT_Q_ADDR_WIDTH /
//     CX_NFUNC_WD / CX_NW / CX_NHQ / ...) => always derived from [`pcie_core_cfg.h`](pcie_core_cfg.h).
//   Signal groups (`pig_DWC_pcie_ctl.sv` top port names, copied group by group):
//     - XALI0/XALI1   : client0_*/client1_* (APP->core; including the full tlp_* group)
//                       + xadm_client*_halt (output)
//     - RTRGT1        : radm_trgt1_* (core->APP) + trgt1_radm_halt / trgt1_radm_pkt_halt (inputs)
//     - RCPL (bypass) : radm_bypass_* (core->APP; dimension = CX_NHQ)
//     - DBI           : dbi_addr/din/cs/cs2/wr + lbc_dbi_ack/dout
//     - Sideband/CDM  : cfg_pbus_num/cfg_pbus_dev_num/cfg_2ndbus_num/cfg_subbus_num/
//                       cfg_mem_base/cfg_mem_limit/device_type/upstream_port/rdlh_link_up
//     - MSI interface : ven_msi_req/func_num/tc/vector + ven_msi_grant + cfg_msi_en (T-2; upstream port only)
//     - Error face    : xal_{xmt,rcvd}_cpl_{ca,ur} + radm_{correctable,nonfatal,fatal}_err
//   Literal widths (per top ports, not parameters): fmt2/type5/tc3/reqid16/dw_len10/first_be4/last_be4/
//     byte_cnt12/cmpltr_id16/rsvd15/byte_en8/remote_req_id16/cpl_byte_cnt12/byte_len13/
//     app_dev_id16/hdr_rsvd15/vc3/addr 64? => see the individual fields (addr uses FLT_Q_ADDR_WIDTH).
//   * Packing = bit-by-bit load following the "field order table" (the order table is generated
//     from cfg => the bus width and field positions **vary with configuration**).
//   * Implementation = pcie_priv_port.cpp.
// -----------------------------------------------------------------------------
#pragma once
#include "pcie_core_cfg.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace pcie_tlm {

// -- Dynamic-width bit vector (bus width = parameter; NEVER a compile-time fixed width) --
class BitVec {
public:
    int w = 0;
    std::vector<uint64_t> v64;            // one word per 64 bits (low word first)
    BitVec() = default;
    explicit BitVec(int width_bits) { resize(width_bits); }
    void resize(int width_bits);
    void reset();                         // all zeros (width unchanged)
    int  width() const { return w; }
    void     set(int hi, int lo, uint64_t val);
    uint64_t get(int hi, int lo) const;
    void     set_bit(int i, bool b);
    bool     bit(int i) const;
    // wide field (>64 bits, e.g. data[512]): moved as 64-bit words (low word first)
    void set_wide(int hi, int lo, const std::vector<uint64_t>& words);
    std::vector<uint64_t> get_wide(int hi, int lo) const;
};

// -- Wide-field carrier (width = parameter; stored as 64-bit words) --
struct WideField {
    int w = 0;                            // bit width (derived from cfg)
    std::vector<uint64_t> v;              // (w+63)/64 words
    void init(int width_bits);
    bool operator==(const WideField& o) const { return w == o.w && v == o.v; }
};

// -- Header field group (shared by RTRGT1 and RCPL bypass; literal widths follow top ports) --
struct HdrFields {
    bool     dv = false, hv = false, eot = false;
    bool     tlp_abort = false, dllp_abort = false, ecrc_err = false;
    bool     td = false, poisoned = false, bcm = false, cpl_last = false;
    uint8_t  fmt = 0, type = 0, tc = 0, cpl_status = 0;    // [2]/[5]/[3]/[3]
    uint8_t  first_be = 0, last_be = 0;                    // [4]
    uint16_t attr = 0;                                     // [FLT_Q_ATTR_WIDTH]
    uint16_t reqid = 0;                                    // [16]
    uint16_t dw_len = 0;                                   // [10]
    uint16_t byte_cnt = 0;                                 // [12]
    uint16_t rsvd = 0;                                     // [15]
    uint16_t cmpltr_id = 0;                                // [16]
    uint64_t addr = 0;                                     // [FLT_Q_ADDR_WIDTH]
    uint64_t tag = 0;                                      // [CX_TAG_SIZE]
    uint64_t func_num = 0;                                 // [CX_NFUNC_WD]
};

// -- XALI single expansion slot (client0_* / client1_*; APP->core) --
struct XaliSlot {
    bool     addr_align_en = false, dv = false, eot = false, bad_eot = false;
    bool     hv = false, td = false, ep = false, cpl_bcm = false;
    uint8_t  byte_en = 0;                                  // [8]
    uint16_t remote_req_id = 0;                            // [16]
    uint16_t cpl_byte_cnt = 0;                             // [12]
    uint8_t  cpl_status = 0;                               // [3]
    uint8_t  fmt = 0, type = 0, tc = 0;                    // [2]/[5]/[3]
    uint16_t attr = 0;                                     // [FLT_Q_ATTR_WIDTH]
    uint16_t byte_len = 0;                                 // [13]
    uint64_t addr = 0;                                     // [64]
    uint64_t tid = 0;                                      // [CX_TAG_SIZE]
    WideField data;                                        // [TRGT_DATA_WD]
    uint64_t func_num = 0;                                 // [CX_NFUNC_WD]
    uint16_t app_dev_id = 0;                               // [16]
    uint16_t hdr_rsvd = 0;                                 // [15]
};

struct XaliSignals {
    std::vector<XaliSlot> slot;                            // size = NVC_XALI_EXPANSION
    std::vector<uint8_t>  halt;                            // [slots] (output)
};

// -- RTRGT1 (core->APP) --
struct Trgt1Signals {
    HdrFields h;
    WideField data;                                        // [TRGT_DATA_WD]
    uint64_t  dwen = 0;                                    // [CX_NW]
    uint64_t  hdr_uppr_bytes = 0;                          // [FLT_Q_ADDR_WIDTH]
    uint8_t   vc = 0;                                      // [3]
    bool      radm_halt = false;                           // trgt1_radm_halt (input; high = halt)
    std::vector<uint8_t> pkt_halt;                         // [CX_NVC*3] (input; per-TLP-type stall)
    // * T-1b (real gap found by the 2026-10-13 reconciliation): `radm_grant_tlp_type[CX_NVC*3-1:0]` (**output**) --
    //   databook Section 4.5.3 (p407 figures 4-14/4-15): **one per TLP type** (0 = P / 1 = NP / 2 = CPL),
    //   **clock-driven pulse**: "a transaction has been granted to dequeue from the receive queue";
    //   the application side uses it to implement a per-type vacancy tracker (++grant / --radm_trgt1_hv)
    //   => assert `pkt_halt` when the count is reached.
    //   WARNING timing: at least one cycle from pkt_halt deassertion to grant assertion; halt must be
    //   asserted **combinationally** (NEVER wait for the next cycle).
    std::vector<uint8_t> grant_tlp_type;                   // [CX_NVC*3] (output; 1-hot pulse)
};

// -- MSI interface face (T-2; `ven_msi_*` + `cfg_msi_en`) --
//   * Spec (databook **Section 4.12**, p426): **upstream ports only**; synchronous req/ack handshake:
//     `ven_msi_req` **once asserted must be held until grant**; `ven_msi_grant` = **one-cycle pulse**;
//     req held => the controller **sends another MSI** (level semantics); the controller
//     **generates the corresponding memory write** (posted MWr).
//   * Field semantics (read from customer RTL = single source of truth; NEVER invent):
//     - `ven_msi_vector[4:0]` = **replaces the enabled bits among the low 5 bits of msi_data**
//       (`pig_msg_arbitration.sv:197` original comment + `pig_msg_formation.sv:535`:
//        `{data[31:5], ((data[4:0] & ~vector_en) | (msi_vector & vector_en))}`;
//        `vector_en` is set by multi_msi_en: `000` => 0 bits ... `101` => 5 bits, see :294-300 in the same file);
//     - **the address does not change with vector** (same file :281-287: `int_msi_addr = cfg_msi_addr_v[func]`);
//     - `ven_msi_func_num` = selects the MSI register group by function (`cfg_msi_data_v[func]`);
//     - message reqid = this port's BDF (`get_req_id(cfg_pbus_num, cfg_pbus_dev_num, fn, ...)`).
//   * `cfg_msi_en` = **MSI Capability Message Control.Enable** (`pig_cdm_msi_reg.sv:142`
//     writes `lbc_cdm_data[16]`) => **the real value given to the application** (the application
//     gates requests with it; databook figure 4-41 uses this convention).
struct MsiSignals {
    bool     req = false;                                  // ven_msi_req (application->core; hold until grant)
    uint64_t func_num = 0;                                 // ven_msi_func_num[CX_NFUNC_WD-1:0]
    uint8_t  tc = 0;                                       // ven_msi_tc[2:0]
    uint8_t  vector = 0;                                   // ven_msi_vector[4:0]
    bool     grant = false;                                // ven_msi_grant (core->application; one-cycle pulse)
    uint64_t cfg_msi_en = 0;                               // cfg_msi_en[CX_MSI_NFUNC-1:0] (per-function enable)
};

// -- DBI interface (core<->application; cfg execution face) --
struct DbiSignals {
    uint32_t addr = 0, din = 0, dout = 0;
    uint8_t  wr = 0;                                       // [4]
    bool     cs = false, cs2 = false, ack = false;
};

// -- Sideband (CDM outputs; enumeration/bridge-header face) --
struct SidebandSignals {
    uint8_t  pbus_num = 0;                                 // [CX_BUSNUM_WD]
    uint8_t  pbus_dev_num = 0;                             // [CX_DEVNUM_WD]
    uint8_t  secondbus_num = 0;                            // [8]
    uint8_t  subbus_num = 0;                               // [8]
    std::vector<uint16_t> mem_base, mem_limit;             // [16] (per function)
    uint8_t  device_type = 0;                              // [4]
    bool     upstream_port = false;
    bool     rdlh_link_up = false;
    // -- * T53 (C2 CRS): **`cfg_bridge_crs_en`** (top line 406; output, **1 bit per function**, CX_NFUNC) --
    //   Read from RTL (single source of truth): `pig_cdm_cfg_reg.sv:3215` --
    //     `cfg_bridge_crs_en <= bridge_device ? (write_pulse[1] & cfg_cap_reg_id[8] ?
    //                           lbc_cdm_data[15] : ...) : 0;`
    //   => = **Device Control[15] of a Type1 bridge function** (`pig_CFG_PCIE_CAP+8'h8` = DevCtl reg_id[8]).
    //   Naming (RTL original) / spec: DevCtl[15] for Type0 = FLR, for a **bridge** (Type1) =
    //   **Bridge Configuration Retry Enable** (NEVER: a bridge does not execute FLR).
    //   Semantics = this bridge **is permitted to return CRS to configuration requests** (spec:
    //   cleared to 0 => "no CRS completion is returned => the lack of a completion results in eventual
    //   Completion Timeout at the Root Complex"); this model gates the CRS uplink below on it.
    uint64_t cfg_bridge_crs_en = 0;                        // [CX_NFUNC] (1 bit per function; 0 = not enabled)
};

// -- Error face --
struct ErrorSignals {
    std::vector<uint8_t> xmt_cpl_ca, rcvd_cpl_ca, rcvd_cpl_ur;   // [CX_NFUNC]
    bool correctable = false, nonfatal = false, fatal = false;
};

// -- Full set of core-top application-side signals (packing object; sizes set by cfg) --
struct CoreTopSignals {
    std::vector<XaliSignals> xali;                         // [2]: client0 / client1
    Trgt1Signals  trgt1;
    DbiSignals    dbi;
    MsiSignals    msi;                                     // T-2: MSI interface face (**upstream port only**)
    SidebandSignals sb;
    ErrorSignals  err;
    void init(const CoreCfg& cfg);
};

// -- Packing layout: one field --
struct FieldDesc {
    std::string nm;
    int lo = 0, hi = 0;
    int w() const { return hi - lo + 1; }
};

// -- Packer (cfg-driven; width/bit order fully parameterized) --
class PciePacker {
public:
    explicit PciePacker(const CoreCfg& cfg);
    const CoreCfg& cfg() const { return cfg_; }
    int width() const { return width_; }
    const std::vector<FieldDesc>& fields() const { return fields_; }
    int lo(const std::string& nm) const;                   // -1 = no such field
    int hi(const std::string& nm) const;
    void pack(const CoreTopSignals& s, BitVec& out) const;
    void unpack(const BitVec& in, CoreTopSignals& s) const;

private:
    CoreCfg cfg_;
    std::vector<FieldDesc> fields_;
    std::map<std::string, int> idx_;
    int width_ = 0;
    void add(const std::string& nm, int w);
};

} // namespace pcie_tlm
