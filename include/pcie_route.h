// -----------------------------------------------------------------------------
// Solution One - routing layer (**declarations**) -- crossbar arbitration
//   Requirement (7): N=1..32 + unicast (P2P) / multicast / broadcast, full matrix.
//   Source of the conventions: databook S.4.1 (H.2/H.4) + the four classes of the old
//             model's deliver() (Addr/ID/MC/BC) + evidence from reference file dn_crossbar.v
//             (type==0 => address window / ==0xa => reqid bus / cfg => bus-dev).
//   Topology model = "independent decision per destination" (isomorphic to the reference's
//             distributed form): for each TLP, every destination port decides on its own;
//             if nobody accepts => handled by the **source port** (UP source => UR; DN source =>
//             send upstream outside the window).
//   * Pure C++ (same layer as pcie_priv_port.h); implementation = pcie_route.cpp; SC assembly in pcie_sc_upstream.
// -----------------------------------------------------------------------------
#pragma once
#include <cstdint>
#include <vector>

namespace pcie_tlm {

// -- Minimal routing header view (decoupled from the contract layer: any TLP view feeding {type, addr, reqid} works) --
namespace rt {
constexpr uint8_t TYPE_MEM  = 0x0;    // MRd/MWr
constexpr uint8_t TYPE_IO   = 0x2;    // * T52: IO read/write (spec type[4:0] = 0b00010)
constexpr uint8_t TYPE_CFG0 = 0x4;    // CfgRd0/CfgWr0 (cfg2dbi.v:205 "cfg1 to cfg0")
constexpr uint8_t TYPE_CFG1 = 0x5;    // CfgRd1/CfgWr1 (cfg2dbi.v:99 gate)
constexpr uint8_t TYPE_CPL  = 0xa;    // Cpl/CplD (mem2client0.v:38 gate)
// -- * T49: Message class (TLP type[4:0] = 10_rrr; **r = routing subcode**, PCIe spec Table 2-20) --
//   Convention (checked against the spec, NEVER from memory): 000 to root / 001 by address /
//   010 by ID / **011 root-port broadcast** / **100 local termination** (INTx; the switch
//   **relays hop by hop**: the receiving port consumes it => after folding, the upstream port
//   **re-sends** it, and the root sees reqid = **the last sender**) / **101 gather to root**
//   (**the only** r=101 message = PME_TO_Ack).
constexpr uint8_t TYPE_MSG_RC     = 0x10;   // r=000 (ERR_* / PM_PME / ...)
constexpr uint8_t TYPE_MSG_ADDR   = 0x11;   // r=001
constexpr uint8_t TYPE_MSG_ID     = 0x12;   // r=010
constexpr uint8_t TYPE_MSG_BCAST  = 0x13;   // r=011 (PME_Turn_Off)
constexpr uint8_t TYPE_MSG_LOCAL  = 0x14;   // r=100 (Assert/Deassert_INTx)
constexpr uint8_t TYPE_MSG_GATHER = 0x15;   // r=101 (PME_TO_Ack)
constexpr uint8_t MSG_ROUTE_MASK  = 0x07;
constexpr bool is_msg_type(uint8_t t) { return t >= TYPE_MSG_RC && t <= TYPE_MSG_GATHER; }
constexpr uint8_t msg_route(uint8_t t) { return (uint8_t)(t & MSG_ROUTE_MASK); }
// -- Message codes (Table 2-20; those used by this line) --
constexpr uint8_t MSG_PME           = 0x18;  // PM_PME (r=000)
constexpr uint8_t MSG_PME_TURN_OFF  = 0x19;  // r=011 (root port broadcasts downstream)
constexpr uint8_t MSG_PME_TO_ACK    = 0x1B;  // r=101 (gather to root)
constexpr uint8_t MSG_ASSERT_INTX   = 0x20;  // +0..3 => INTA..D (r=100)
constexpr uint8_t MSG_DEASSERT_INTX = 0x24;  // +0..3
constexpr uint8_t MSG_ERR_COR       = 0x30;  // r=000
constexpr uint8_t MSG_ERR_NONFATAL  = 0x31;
constexpr uint8_t MSG_ERR_FATAL     = 0x33;
// -- * T54 (C11-b ATS): **invalidate request/completion** -- code values = **read from customer RTL**
//   (`include/pig_pcie_defs.svh:231-232`: `pig_INVALIDATE_REQUEST = 8'b0000_0001` -
//    `pig_INVALIDATE_COMPLETION = 8'b0000_0010`); **r = 010** (routed by ID;
//    same file `pig_radm_filter_sw.sv:1306` checks `hdr_type[2:0] == 3'b010` && MsgD).
//   WARNING differs from the PCIe spec code table (e.g. spec uses 0x04/0x05) = **M6b cross-check item**
//   (real hardware prevails).
constexpr uint8_t MSG_INV_REQ       = 0x01;  // ATS Invalidate Request (r=010)
constexpr uint8_t MSG_INV_CPL       = 0x02;  // ATS Invalidate Completion (r=010)
// -- * T55 (C6 VDM): **vendor-defined message** codes (read from customer RTL `include/pig_pcie_defs.svh:233-234`) --
//   `pig_VENDOR_TYPE0 = 0b0111_1110` (**r = 000**, to root) - `pig_VENDOR_TYPE1 = 0b0111_1111`
//   (**r = 010**, by ID); WARNING databook Table H-1: "Vendor MSG Type0 and filter mask
//   `CX_FLT_MASK_VENMSG0_DROP` **not set** => **UR**" (core filter-face convention; this model =
//   routing + counting, recorded).
constexpr uint8_t MSG_VDM_T0      = 0x7E;  // Vendor Defined Type 0 (r=000)
constexpr uint8_t MSG_VDM_T1      = 0x7F;  // Vendor Defined Type 1 (r=010)
// * T56 (C10): LTR / OBFF / PTM (read from RTL `pig_pcie_defs.svh:214-246`)
//   - **LTR = 0x10** (`pig_LTR_MSG_CODE`/`pig_LATENCY_TOLERANCE_REPORTING`; r=**100** --
//     `pig_radm_filter_sw.sv:1305` `int_ltr`) - **OBFF = 0x12** (`pig_OBFF`; r=100)
//   - **PTM Req = 0x52** / **PTM Res = 0x53** (`pig_PTM_REQ_TYPE`/`pig_PTM_RES_TYPE`; r=100)
//   => all three are **local-termination (hop-by-hop relay)** class (they use the T49 r=100 infrastructure).
constexpr uint8_t MSG_LTR = 0x10, MSG_OBFF = 0x12;
constexpr uint8_t MSG_PTM_REQ = 0x52, MSG_PTM_RES = 0x53;
// -- * T55 (C7 AtomicOps): **atomic operation type codes** (read from customer RTL `pig_pcie_defs.svh:62-67`:
//   `FETCHADD32 = 7'b1001100` = {fmt 0b10 (3DW+data), type 0b01100} - `FETCHADD64 = 7'b1101100`
//   = {fmt 0b11 (4DW+data), same type}; SWAP/CAS follow the same order (type 0x0D/0x0E) -----------------
//   WARNING spec: **atomics are non-posted** (completion returned, data = **original value**); operand
//   size (FetchAdd/Swap: 32b = 1DW / 64b = 2DW; **CAS: 32b = 2DW / 64b = 4DW**) and **natural
//   alignment** are **malformed criteria** (RTL `atomic_operand_err`/`atomic_addr_align_err`/`atomic_be_err`).
constexpr uint8_t TYPE_ATOMIC_FADD = 0x0C;
constexpr uint8_t TYPE_ATOMIC_SWAP = 0x0D;
constexpr uint8_t TYPE_ATOMIC_CAS  = 0x0E;
constexpr bool is_atomic_type(uint8_t t) { return t >= TYPE_ATOMIC_FADD && t <= TYPE_ATOMIC_CAS; }
} // namespace rt

struct RouteHdr {
    uint8_t  type = 0;                // type code (rt::*)
    uint64_t addr = 0;                // MEM / cfg address
    uint16_t reqid = 0;               // Cpl context (requester ID)
    int32_t  msg_dest = -1;           // Message r=010 destination (-1 = use reqid)
    uint8_t  at = 0;                  // TLP AT: 00 untranslated / 10 translated
    // MC/BC replication applies only to **posted memory writes**; a Memory
    // Read in the same window is an ordinary non-posted unicast request.
    bool     posted = false;
};

// -- * T53 (C4): **attr field bit semantics** (read from the contract = single source of truth; NEVER from memory) --
//   Spec (PCIe Base): TLP header **Attr[2:0] = {IDO(2) - RO(1) - NoSnoop(0)}** --
//     Attr[1] = Relaxed Ordering - Attr[2] = ID-based Ordering - Attr[0] = No Snoop.
//   Read from customer RTL: `src/include/pig_adm_defs.svh` --
//     `pig_SF_HDR_TLP_ATTR = 2 + CX_IDO_ENABLE_VALUE  // IDO enabled => 3 bits`
//     `pig_F_HDR_TLP_ATTR_SN_RO = ATTR[1:0]` (=> **RO = attr[1]**) -
//     `pig_F_HDR_TLP_ATTR_IDO   = ATTR[2]`;
//   Usage read: `Adm/pig_radm_inq_mgr.sv:525` -- `hdrq_wr_relax_ordr = flt_q_header[ATTR_FO + 1]`
//     (= **attr[1]** = RO) (same convention as the spec).
//   WARNING: **this configuration (sw3dn) has `pig_CX_IDO_ENABLE_VALUE = 1'h0` => `FLT_Q_ATTR_WIDTH = 2`**
//     => the field is actually **2 bits = {RO, SN}**, and **the IDO bit does not exist** (with the
//     customer's `CX_IDO_ENABLE` it becomes 3 bits; the model follows cfg.FLT_Q_ATTR_WIDTH
//     automatically => NEVER hard-coded).
//   * Ordering semantics (see the egress ordering layer in pcie_sc_upstream.cpp): SN **does not
//     participate in ordering** (coherency hint only).
static inline bool tlp_ro(uint8_t attr)  { return ((attr >> 1) & 0x1u) != 0; }        // Attr[1]
static inline bool tlp_ido(uint8_t attr, int attr_w) {                                 // Attr[2]
    return (attr_w >= 3) && (((attr >> 2) & 0x1u) != 0);
}
static inline uint8_t tlp_sn(uint8_t attr) { return (uint8_t)(attr & 0x1u); }          // Attr[0]
// Spec: **RO must not be used on** configuration requests / IO requests / MSIs (= memory writes) / Messages
//   (original text: Attr[1] "is not applicable and must be Clear for Configuration Requests,
//    I/O Requests, Memory Requests that are MSIs, and Message Requests") => this model:
//   such TLPs carrying RO are **not passed** + explicitly accounted (NEVER silent).
static inline bool tlp_ro_applicable(uint8_t type, bool has_data, bool is_msi) {
    if (type == rt::TYPE_CFG0 || type == rt::TYPE_CFG1 || type == rt::TYPE_IO) return false;
    if (rt::is_msg_type(type)) return false;
    if (type == rt::TYPE_MEM && has_data && is_msi) return false;
    return true;
}

constexpr int MAX_DN    = 32;    // requirement (7): downstream 1..32
constexpr int MC_GROUPS = 16;    // number of groups (old model convention MCAST_G_MAX = 16)

// Address/identity face of one port (written by instantiation + enumeration program; register face = M2)
struct PortCfg {
    uint8_t  bus = 0;                       // this port's bus
    uint8_t  dev = 0;                       // cfg device num (assigned at instantiation)
    uint8_t  sec = 0;                       // secondary bus (below this port; 0 = not programmed)
    uint8_t  sub = 0;                       // subordinate bus
    uint64_t mem_base = 0, mem_size = 0;    // Type-1 memory window (hit => forward to this port)
    uint64_t pref_base = 0, pref_size = 0;  // Type-1 **prefetch window** (B-1; 64-bit reachable => carries segments >= 4G)
    uint64_t p2p_base = 0, p2p_size = 0;    // P2P window (DN<->DN direct path)
    // -- * T52 (B9): **IO space window** (bridge 0x1C I/O Base/Limit + 0x30 Upper; 1KB granularity encoding) --
    uint64_t io_base = 0, io_size = 0;
    bool     bc_member = false;             // broadcast-domain member
    uint16_t mc_mask = 0;                   // 16 group member bits (private convention)
    // -- * Standard PCIe Multicast (ECAP 0x0012) per-port state ---------------
    uint64_t mc_recv = 0;                   // MC_Receive (one bit per group)
    uint64_t mc_block_all = 0;              // MC_Block_All
    uint64_t mc_block_untrans = 0;          // MC_Block_Untranslated
    uint8_t  mc_overlay_size = 0;           // < 6 => overlay disabled
    uint64_t mc_overlay_base = 0;           // MC_Overlay_BAR [63:6]
    // DM (dual-mode) port metadata.  The DM port uses the same private-port
    // contract and the same fabric routing/BC/MC membership as a swdn port;
    // the only model-visible difference is its configured link role.
    bool     is_dm = false;                 // true = DM/NTB port
    uint8_t  dm_role = 0;                   // 0 = EP-facing, 1 = root-facing (customer config)
};

struct SwitchCfg {
    int      nport = 4;                             // 1..MAX_DN
    uint64_t mc_base = 0, mc_stride = 0x100000;     // private MC window
    uint64_t bc_base = 0, bc_size = 0x100000;       // BC window
    // -- * Standard PCIe Multicast (ECAP 0x0012) switch state ----------------
    //   MC_Base_Address / MC_Index_Position / NumGroups from the MC capability;
    //   an MC hit suspends normal address routing (PCIe r4.0 sec 7.9.11).
    bool     mc_std_enable = false;
    uint64_t mc_std_base = 0;
    uint8_t  mc_std_index = 0;               // bytes per group = 1 << index
    uint16_t mc_std_groups = 1;              // 1..64
    PortCfg  up;
    std::vector<PortCfg> dn;                        // size == nport
    // -- * P2: VC face (per-VC egress + arbitration modes; TC->VC mapping = **newly added here**) --
    //   Old-line convention (ported after reading): frame header cls[2:0]|vc[5:3]; **one
    //   independent queue per port per VC**; two arbitration modes: 0 = **strict priority**
    //   (higher VC first, scan NVC-1 -> 0) - 1 = **round robin** (cyclic scan from rr_ptr);
    //   WARNING four pitfalls of the old line (NEVER copied): (1) no TC->VC mapping (newly added
    //   here, USP 0xE0) (2) no WRR (3) `o_vc_tx` was 8 global entries (here **per port per VC**)
    //   (4) `o_qmax` = port-level peak (this convention is retained).
    int      nvc = 1;                               // number of VCs (1..8; default 1 => existing testbenches **unchanged**)
    uint8_t  tc2vc[8] = {0, 1, 2, 3, 4, 5, 6, 7};   // TC->VC mapping (identity by default; changed via USP 0xE0)
    uint8_t  vc_arb = 0;                            // arbitration mode: 0 = strict priority / 1 = round robin / **2 = WRR**
    // -- * T48: QoS = **WV (VC weighted round robin)** -- convention = simplified PCIe VC capability
    //   "VC Arbitration Table (WRR)": each VC has a **weight** (4 bits; 0 = that VC starves,
    //   NEVER silent: explicitly accounted); default = all 1 (equivalent to round robin)
    uint8_t  vc_weight[8] = {1, 1, 1, 1, 1, 1, 1, 1};
    // TC => this port's egress VC (after mapping, **modulo nvc** => NEVER out of range)
    uint8_t  vc_of(uint8_t tc) const { return (uint8_t)(tc2vc[tc & 7u] % (uint8_t)(nvc > 0 ? nvc : 1)); }
    // -- * P2: NTB window (hit => routed to the **NTB function block**, NEVER sent to any port;
    //   address assigned by enumeration/testbench) --
    uint64_t ntb_base = 0, ntb_size = 0;

    void init(int n);
};

enum class Dlv : uint8_t { NONE = 0, UNICAST, MC, BC, NTB, MSG };  // * T49: MSG = Message class (decided by the r subcode)

// -- Small helpers --
bool    in_win(uint64_t a, uint64_t base, uint64_t size);
uint8_t reqid_bus(uint16_t reqid);
// * T48: **ARI Requester ID** = {bus[7:0], fn[7:0]} (under ARI, dev is not used => fn occupies
//   the low 8 bits; NEVER mix with the classic non-ARI form = {bus, dev[4:0], fn[2:0]} --
//   the two forms **must not be mixed**; pick one form per device)
uint16_t ari_reqid(uint8_t bus, uint8_t fn8);
// Config address bit order = **PCIe spec configuration request header format** (M2 correction;
//   NEVER the old self-consistent convention):
//   byte8 = bus (addr[23:16]) - byte9 = dev[4:0] ([15:11]) + fn[2:0] ([10:8]) -
//   **byte10[3:0] = Extended Register Number[3:0] = addr[27:24]** -
//   byte11 = Register Number[5:0] = addr[7:2] (low 2 bits always 0).
//   * T50 (**T-5 closed; 2026-10-14**): reg => **10-bit DW offset** (ExtReg<<8 | Reg6) =>
//   **4KB extended cfg is addressable**;
//   WARNING **T48 old convention retired**: `fn[7:3]@addr[28:24]` **collides with ExtReg**
//   (byte10[7:4] = reserved) => follow the spec --
//   **ARI fn8 = {Device[4:0], Function[2:0]} (= addr[15:11]+[10:8])**, interpretation decided by
//   the **port-side forwarding switch**; the ARI convention of `reqid = {bus, fn8}` is unchanged
//   (that is the Requester ID, a different field).
uint8_t  cfg_bus(uint64_t addr);
uint8_t  cfg_dev(uint64_t addr);
uint8_t  cfg_fn(uint64_t addr);          // classic fn[2:0] (NOT ARI)
uint8_t  cfg_fn8(uint64_t addr);         // * ARI: fn8 = {Device[4:0], Function[2:0]}
uint16_t cfg_reg(uint64_t addr);
uint64_t cfg_addr_make(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t reg);
// * M6b: **model cfg address convention => real DWC client port convention** (on-wire DW2 byte
//   stream; RTL read `pig_xadm_hdr_form.sv:426` CFG0/1 alike:
//   {bus[31:24]|devfn[23:16]|extreg[11:8]|reg[7:2]})
uint64_t cfg_addr_to_wire(uint64_t addr);

// Class decision (global, independent of destination)
Dlv classify(const SwitchCfg& sw, const RouteHdr& t);
int mc_group(const SwitchCfg& sw, uint64_t addr);

// Standard PCIe Multicast (ECAP 0x0012) helpers.
bool mc_std_hit(const SwitchCfg& sw, const RouteHdr& t);
int  mc_std_group(const SwitchCfg& sw, uint64_t addr);
// Egress MC_Overlay_BAR translation (size < 6 => unchanged).
uint64_t mc_translate(const PortCfg& p, uint64_t addr);

// -- Destination decision: DN ports (0..nport-1) --
// src: -1 = UP source; 0..nport-1 = DN source. Returns true = this destination port should emit this TLP.
bool pick_dn(const SwitchCfg& sw, const RouteHdr& t, int src, int dst);

// -- Destination decision: UP port (when src is a DN; a UP source never goes back to UP) --
bool pick_up(const SwitchCfg& sw, const RouteHdr& t, int src);

} // namespace pcie_tlm
