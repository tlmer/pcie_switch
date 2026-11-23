// -----------------------------------------------------------------------------
// Solution One - core configuration parameters (**declarations**) -- width = parameter,
//   **read from the RTL macro files**
//   Convention (2026-10-12 user directive): (1) packed signals are taken from the
//     `pig_DWC_pcie_ctl.sv` top;
//     (2) **parameters must reference the RTL macro files; every configuration differs**
//         => NEVER hard-code any numeric value in the model:
//         values always come from the generated file [`pcie_core_cfg_gen.h`](pcie_core_cfg_gen.h)
//         (generator = [`../tools/gen_core_cfg.py`](../tools/gen_core_cfg.py),
//          source = the `define` directives in `<RTL tree root>/config_*/src/*cc_constants*.svh`,
//          each recorded with its line number).
//   * Usage: `cfg_by_name("config_sw3dn")` (switch configurations by name only; adding a
//     new configuration only needs a directory, no code change).
//   * Measured (generated table; 2026-10-12): data 64/256/512 - tag 8/10 - addr 64/32 - NHQ 1/2
//     => the total packed width varies with configuration (TB-1 proves 1219 / 2048 / 3047 bits).
//   * Implementation = pcie_core_cfg.cpp.
// -----------------------------------------------------------------------------
#pragma once
#include "pcie_core_cfg_gen.h"          // auto-generated table (NEVER edit by hand)

#include <string>

namespace pcie_tlm {

// -- Core parameter set (`pig_*` / unprefixed macros map one-to-one; names follow RTL) --
struct CoreCfg {
    const char* name = "";              // configuration name (config_* directory name)
    const char* rel_file = "";          // macro file (relative to the RTL tree root)
    const CoreCfgEntry* prov = nullptr; // provenance (per-parameter line numbers => testbench can check the source)
    bool complete = false;              // all required parameters present (missing => testbench criterion fails)

    int TRGT_DATA_WD        = 0;        // data bits per beat (= CX_NW*32)
    int CX_TAG_SIZE         = 0;        // tag / tid
    int FLT_Q_ADDR_WIDTH    = 0;        // addr / hdr_uppr_bytes
    int FLT_Q_ATTR_WIDTH    = 0;        // attr
    int CX_NVC              = 0;        // number of VCs
    int CX_NVC_XALI_EXPANSION = 0;      // XALI expansion slots (client*_tlp_data width = slots x DW)
    int CX_NFUNC            = 0;        // number of functions
    int CX_NFUNC_WD         = 0;        // func_num bit width
    int CX_NHQ              = 0;        // number of header queues (RCPL bypass array dimension)
    int CX_NW               = 0;        // dwen bit width (32-bit words)
    int CX_BUSNUM_WD        = 0;        // cfg_pbus_num
    int CX_DEVNUM_WD        = 0;        // cfg_pbus_dev_num
    int CX_LBC_NW           = 0;        // ELBI word width
    int CX_MSI_NFUNC        = 0;
    int CX_RADM_NFC_TRGT_INTF = 0;      // number of vendor message interfaces
    int CX_NL               = 0;        // number of lanes

    // -- Derived (all computed from parameters; NEVER hard-coded) --
    int beat_bytes() const { return TRGT_DATA_WD / 8; }                  // bytes per beat
    int xali_data_w() const { return CX_NVC_XALI_EXPANSION * TRGT_DATA_WD; }  // client*_tlp_data
    int dwen_w() const { return CX_NW; }
    bool sane() const { return TRGT_DATA_WD == CX_NW * 32 && TRGT_DATA_WD > 0; }  // width self-consistency
};

// -- Lookup API (NEVER hard-coded: all values come from the generated file) --
const CoreCfg& cfg_by_name(const char* name);   // not found => empty group with !complete (NEVER silent)
int  cfg_count();                               // number of configurations in the generated table
const char* cfg_name_at(int i);                 // enumeration (for testbench criteria)
const char* cfg_root();                         // RTL tree root (env PCIE6SW_CFG_ROOT; release default: current directory)
std::string cfg_abs_file(const CoreCfg& c);     // absolute path of the macro file (for source checks)

// -- Named configurations used by this project (all = table lookup, no embedded values) --
const CoreCfg& cfg_sw3dn();     // 4+1 - downstream port (config_sw3dn; core = pig_DWC_pcie_ctl.sv)
const CoreCfg& cfg_sw3();       // 4+1 - upstream port (config_sw3; core = DWC_pcie_ctl.sv)
const CoreCfg& cfg_ep();        // reference: data=256 / addr=32 / NHQ=2
const CoreCfg& cfg_sw5x16();    // reference: data=512 / tag=10 / NW=16
const CoreCfg& cfg_dmx1_ntb();  // customer DM/NTB configuration (config_dmx1_ntb; separate table)

} // namespace pcie_tlm
