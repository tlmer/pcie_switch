// -----------------------------------------------------------------------------
// Scheme 1 - SC skeleton -- private-port "wire" + beat-level encoding/decoding (**declarations**)
//   Boundary contract = [`pcie_priv_port.h`](pcie_priv_port.h) (full set of core-top application-side signals;
//   NEVER the 771/300 simplified packing -- retired by user order on 2026-10-12).
//   Beat semantics (following top signals):
//     - one beat = one bus drive: `CoreTopSignals` packed by `PciePacker` into a `BitVec`
//       (width = Sum of fields = parameter; sw3dn 1219 / ep 2048 / sw5x16 3047 bits)
//     - frame control = carried by the signals: `client*_tlp_{dv,hv,eot,bad_eot}` (TX) /
//       `radm_trgt1_{dv,hv,eot,tlp_abort,dllp_abort,ecrc_err}`(RX)
//     - data per beat = parameter: TX = lanes x TRGT_DATA_WD; RX = TRGT_DATA_WD (<=> beat bytes = WD/8)
//   - halt = **high means halt / low means valid** (decided in G1): rtl_halt => TLM frozen; app_halt => RTL frozen
//   - DBI return path = full signal group (DbiSignals; cross-port = async FIFO, Section 6.3)
//   * This file = the **replacement seam** for future co-simulation (T4 bridge): everything outside the TLM hangs on the other side of the wire.
//   * Implementation = pcie_sc_wire.cpp.
// -----------------------------------------------------------------------------
#pragma once
#include "pcie_core_cfg.h"
#include "pcie_priv_port.h"
#include "pcie_route.h"      // rt::TYPE_* (used for Cpl construction)

#include <systemc>
#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

namespace pcie_tlm {

// -- Internal transaction-level TLP (**not a boundary contract**; abstract carrier between TLM-internal components) ------------------
struct PrivTlp {
    uint8_t  type = 0, fmt = 0, tc = 0, attr = 0, cpl_status = 0;
    bool     td = false, poisoned = false, bcm = false, cpl_last = false;
    bool     dv = false, hv = false, eot = false, err = false;
    uint64_t addr = 0, tag = 0, func_num = 0;          // tag <= CX_TAG_SIZE
    uint16_t reqid = 0, cmpltr_id = 0, byte_cnt = 0, dw_len = 0;
    uint8_t  first_be = 0, last_be = 0, vc = 0;        // [4]/[4]/[3]
    uint8_t  pbus_num = 0, secondbus_num = 0;          // sideband (loaded by the port)
    uint16_t msg_code = 0;                             // * T49: Message code (Table 2-20; on wire = `rsvd[15]`)
    uint32_t ecrc = 0;                                 // * T50: ECRC value (`td` = valid bit; real device = last DW of TLP Digest)
    // -- * T54 (C11) model-internal fields (**no contract carrier**; NEVER packed) ----------------------------
    //   - `at[1:0]` = TLP header **AT** (address type: criterion for C11-b translation blocking B; spec AT[1:0]:
    //     00 = untranslated / 01 = translation request / 10 = translated)
    //   - `pasid` + `has_pasid` = **PASID** (C11-c); WARNING: this configuration has `FLT_Q_PRFX_WIDTH = 0`
    //     => the port has **no TLP prefix face** (real device: PASID goes through `client*_tlp_prfx`) => data plane = model-internal
    //     + capability face per spec; **after the customer enables the PRFX macro, add contract fields per M6b mapping** (same method as T-5/T-1b).
    uint8_t  at = 0;
    uint32_t pasid = 0;
    bool     has_pasid = false;
    // Model-internal by-ID destination for Message r=010 (ATS invalidate /
    // completion).  -1 = use reqid.  No contract carrier (same class as `at`).
    int32_t  msg_dest = -1;
    std::vector<uint8_t> payload;                      // DW-aligned byte stream
    uint8_t  be8() const { return (uint8_t)((last_be << 4) | first_be); }
    size_t   nbytes() const { return payload.size(); }
};

// -- Normalized frame view (isomorphic in both directions: downstream reads the client0 group / upstream reads the radm_trgt1 group) ------------
struct FrameView {
    bool dv = false, hv = false, eot = false, err = false;
    uint8_t    vc = 0;                     // VC (present in the upstream signal group; downstream = 0)
    HdrFields  h;                          // header (participates in per-beat consistency apart from frame-control/error bits)
    WideField  data;                       // data of this beat (downstream = lanes x WD; upstream = WD)
    uint64_t   sideband = 0;               // sideband key (cfg_pbus_num/cfg_2ndbus_num)
};

// -- Frame assembler (errors counted "orphan beats per beat / in-frame errors per frame") --------------------------
struct FrameCheck {
    bool ok = true;
    bool fail_dv = false, fail_hv_miss = false, fail_hv_mid = false, fail_hdr = false;
    std::string what;
};

struct BeatAsm {
    FrameView            hdr{};            // first-beat header (data does not participate in consistency comparison)
    std::vector<uint8_t> payload;          // payload of the completed frame (accumulated beat by beat using beat_bytes)
    int  nbeats = 0;
    bool active = false;
    bool data_frame = false;
    FrameCheck chk;
    bool     frame_had_err = false;
    uint64_t err_frames = 0;

    void set_err(bool dv, bool hv_miss, bool hv_mid, bool hdr, const char* what);
    void count_frame_err();
    bool feed(const FrameView& f, int beat_bytes);      // returns true = this beat completes a frame
};

// -- Transmit-side codec (parameterized: beat bytes/field widths vary with cfg) --------------------------
enum FaultInj : int {
    FI_NONE = 0,
    FI_SWAP_MID         // mid-stream beat misalignment (frame control intact, payload shifted -- data-plane red mode)
    , FI_REVERSE_BEATS  // beats reversed (hv/eot shifted -- frame-control red mode)
    , FI_DROP_EOT       // drop eot on the last beat (frame never completes)
    , FI_HV_TWICE       // assert hv again on beat 2 (frame-control violation)
    , FI_SIDEBAND_DRIFT // sideband drift on beat 2 (sideband differs from the first beat mid-frame)
    // * T-3 (Section 4.1.1 rule red modes): payload-less frame still asserts dv (violates X2); **early eot** (violates X4)
    , FI_DV_ON_NOPAYLOAD
    , FI_EOT_EARLY
};

struct BeatCodec {
    static int nbeats(const CoreCfg& c, size_t nbytes);     // downstream convention (lanes x WD)
    // downstream: APP drives the client0 group (XALI) -> beat stream
    static std::vector<BitVec> pack_xali(const CoreCfg& c, const PciePacker& pk,
                                         const CoreTopSignals& base,
                                         const std::vector<uint8_t>& payload, int fault = FI_NONE);
    // upstream: core drives the radm_trgt1 group -> beat stream
    static std::vector<BitVec> pack_trgt1(const CoreCfg& c, const PciePacker& pk,
                                          const CoreTopSignals& base,
                                          const std::vector<uint8_t>& payload, int fault = FI_NONE);
    // core-side receive (reads client0 group) / APP-side receive (reads radm_trgt1 group)
    static FrameView unpack_xali(const PciePacker& pk, const CoreCfg& c, const BitVec& beat);
    static FrameView unpack_trgt1(const PciePacker& pk, const BitVec& beat);
};

// -- Bookkeeping context for in-place cfg execution (internal; mirrors reqid/tag bookkeeping of the RTL DN wrapper) --
//   NEVER on the wire: wire level = DbiSignals (per top); the context stays inside this model (DSP port -> USP creates the Cpl)
struct CfgCtx {
    uint16_t reqid = 0;                         // original requester (the Cpl goes back to it)
    uint64_t tag = 0;
    uint8_t  fn = 0;
    bool     is_wr = false;
};

// -- DBI return path (DSP port -> USP; databook H.4 "application executes on the DSP's DBI + creates the Cpl at the USP transmit port") --
struct DbiUp { DbiSignals sig; CfgCtx ctx; };

// -- * T53 (C13): **Malformed TLP receive-side checks** (spec mandatory set + item-by-item mapping to customer RTL) --------
//   Source (RTL single source of truth; NEVER invent): `Layer3/pig_rtlh_tlp_check_slv.sv` -- receive-side checker
//     (`rtlh_radm_malform_tlp_err`:"current packet should be dropped because of checkers
//      failed in this module"); databook Section 3.4.4.1 handling: on detection => (1) **drop the TLP**
//     (2) **return a completion (CA/UR) for non-posted requests** (3) set status/AER (4) send an error message on the upstream port (5) return credits for what was consumed.
//   Item by item:
//     - **unrecognized fmt/type combination** (spec: undefined type/format => Malformed).
//     - **4DW header out of bounds** (spec Section 2.2.7 / 2.3 explicit: IO requests / **completions with data** must use a **3DW header**;
//       violation => "must be discarded and a Malformed TLP signaled"; CFG is in the same family) = **"4DW out of bounds"**.
//     - **length vs actual** (same family as RTL `len_mismatch`): this model's convention = declared length (DW; **including digest**,
//       T50 decision <-> RTL internal expression `len+1` => **M6b mapping item**) vs actual DW count (wire-level **beat padding** tolerance
//       = one beat, provided by the caller as `pad_dws`).
//     - **length > Max_Payload_Size** (RTL `max_pyld_err`; gate = **receive-side** MPS = DevCtl[7:5]).
//     - **length vs BE** (spec Section 2.2.5): **has data and len>1DW => FBE != 0 and LBE != 0**.
//       WARNING: spec "len==1DW => LBE must be 0" is **optional** and **the customer RTL does not check it** (its BE check covers only atomic
//       `atomic_be_err`; `hdr_len_mismatch == 0`) -- this model's generation side uses LBE = FBE for 1DW frames
//       => **this model does not check that optional item** (recorded; M6b mapping).
//     - * **endpoint receives a CRS completion** (RTL `cpl_crs_err` original text: "a completion with configuration request
//       retry will result a malformed tlp. For endpoint device ... any cpl with CRS is a
//       malformed. **This does not apply to switch**") => **only `is_endpoint`** (* tied to C2).
//   Returns 0 = pass; >0 = error code (caller counts by code + sets AER Uncorr[18] + does not commit; NEVER silent).
enum MlfCode : int {
    MLF_OK = 0, MLF_FMT_TYPE = 1, MLF_HDR4DW = 2, MLF_LEN = 3, MLF_MAXPL = 4, MLF_BE = 5,
    MLF_CPL_CRS = 6,
    // -- * T55 additions: 10-bit Tag / atomic operations (following the MFPTR code family of RTL `pig_rtlh_tlp_check_slv.sv`) ----
    //   - **TAG10**: tag[9:8] != 0 while the receiver's **10-Bit Tag Completer capability = 0** (**DevCap2[16]**;
    //     spec Section 2.2.6.2: such a TLP is Malformed; * T58: the spec has no "Completer Enable" control bit => keyed on the capability face);
    //     - **ATOM_OPR**: operand size not an architectural size (RTL `atomic_operand_err`;
    //     FetchAdd/Swap: 1|2 DW <-> CAS: 2|4 DW); - **ATOM_ALIGN**: not naturally aligned
    //     (RTL `atomic_addr_align_err`; 32b => addr[1:0]=0 / 64b => addr[2:0]=0).
    MLF_TAG10 = 7, MLF_ATOM_OPR = 8, MLF_ATOM_ALIGN = 9
};
const char* mlf_name(int code);
// `tag10_cpl_sup` = receiver's **10-Bit Tag Completer Supported** (**DevCap2[16]**; R/O capability bit; NOTE: defaults to
//   true => existing testbenches unchanged; WARNING: this configuration has `CX_TAG_SIZE = 8` => tag field is 8 bits => this check is meaningful only for the **synthetic 10-bit config**)
int  mlf_check(const PrivTlp& t, uint32_t max_payload_bytes, bool is_endpoint, size_t pad_dws = 0,
               bool tag10_cpl_sup = true);

// -- Cpl construction (per S.4.1: reqid = original requester; status: 0=SC / 1=UR / 4=CA) ------------
PrivTlp make_cpl(uint8_t status, uint16_t reqid, uint64_t tag, uint8_t fn,
                 const std::vector<uint8_t>& data, uint16_t cmpltr_id = 0);

// -- * T49: Message construction (Table 2-20; fmt = 0b01 (4DW no data; message code in DW2) -------
//   `mtype` = full r-type code (rt::TYPE_MSG_*); `code` = message code (rt::MSG_*);
//   reqid = requester BDF (spec: what the root port sees = **last sender** => rewritten when the switch forwards).
PrivTlp make_msg(uint8_t mtype, uint16_t code, uint16_t reqid);

// -- Private-port "wire" (one per DN port): bidirectional beat streams + bidirectional DBI + halts on both sides --------------
struct PrivWire : sc_core::sc_module {
    sc_core::sc_fifo<BitVec>     to_rtl;      // TLM (app) -> RTL (core): TX beats
    sc_core::sc_fifo<BitVec>     to_tlm;      // RTL (core) -> TLM (app): RX beats
    sc_core::sc_fifo<DbiSignals> dbi_req;     // TLM (app) -> RTL (core): core register access (addr/din/wr/cs)
    sc_core::sc_fifo<DbiSignals> dbi_rsp;     // RTL (core) -> TLM (app): lbc_dbi_ack / lbc_dbi_dout
    bool rtl_halt = false;                    // high halt => TLM side frozen (decided)
    bool app_halt = false;                    // high halt => RTL side frozen (trgt1_radm_halt)
    // -- * M6b (2026-10-18): **real-core XALI handshake convention** -- `xadm_client0_halt` = **active low** --
    //   User convention (2026-10-18) + RTL cross-check (`pig_xadm_mux.sv:437` last term `!(active_client==CLIENT0)`):
    //   - **default high = not valid** (idle / not receiving); **low = valid** (core is receiving client0);
    //   - it **toggles high/low** during a transfer => the application must: present hv/dv first -> halt high **holds the bus**
    //     -> only after halt low (this beat has been taken) switch to the next beat (same as customer `mem2client0.v`: `~halt` clears hv).
    //   => when true:
    //     - the model's `tx_run` NEVER **waits for halt first** (the old `while(rtl_halt)` treats "idle high" as frozen => deadlock);
    //     - `priv_host` drives pin-level hold handshake as above.
    //   WARNING: default false => existing testbenches (tb_smoke R1/R2/R9, TB-7) keep their **testbench-defined** rtl_halt=true freeze
    //     convention => **zero regression** (testbench <-> fake core self-consistent; the real core uses this switch).
    bool halt_pin_handshake = false;
    // -- * T-1b: TRGT1 Packet Grant/Halt pair (databook Section 4.5.3; Figures 4-14/4-15) ---------
    //   - `grant_tlp_type`: core -> application, **per TLP type** (bit = vc*3 + cls, cls: 0=P/1=NP/2=CPL);
    //     set at frame start, deasserted at frame end => the application counts **rising edges** (equivalent to the "clock-driven pulse" of Section 4.5.3); WARNING: bridge-side
    //     pin level value changes are guaranteed by the generator side (eMMC Section 2.3 hard constraint).
    //   - `app_pkt_halt`: application -> core, **level** (high halt) = per-type stall => **that type** is blocked at the
    //     RX queue (does not occupy the output pipeline; databook: "block that particular packet type").
    uint8_t grant_tlp_type = 0;               // [CX_NVC*3] (1-hot level; core -> application)
    uint8_t app_pkt_halt   = 0;               // [CX_NVC*3] (level; application -> core; bit = vc*3 + cls)

    PrivWire(sc_core::sc_module_name nm, int depth = 8);
};

// -- TLP => Packet class (databook Section 4.5.3, 3 classes: 0 = P / 1 = NP / 2 = CPL) ------
//   * T53: IO class maps to **NP** (non-posted; spec) <-> the old version fell into "everything else => P" (a T52-time approximation, fixed).
int tlp_pkt_class(uint8_t tlp_type, bool has_data);

// -- Diagnostics (for logs/evidence) ------------------------------------------------------
std::string hexdump(const std::vector<uint8_t>& v);
std::ostream& operator<<(std::ostream& os, const DbiSignals& b);
std::ostream& operator<<(std::ostream& os, const PrivTlp& t);
std::ostream& operator<<(std::ostream& os, const BitVec& b);
std::ostream& operator<<(std::ostream& os, const DbiUp& u);

} // namespace pcie_tlm
