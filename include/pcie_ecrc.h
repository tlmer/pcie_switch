// -----------------------------------------------------------------------------
// Solution One - **ECRC** (end-to-end CRC; * T50-2: C1 closed) (**declarations**)
//   Convention (PCIe spec Section 6.2.3 family): ECRC = 32-bit CRC, polynomial **0x04C11DB7**,
//     **initial value 0xFFFFFFFF + final inversion**; WARNING unlike Ethernet's **reflected**
//     form, PCIe uses the **non-reflected (MSB-first)** form (i.e. CRC-32/BZIP2; standard check
//     value below).
//     - Carried in the TLP **TD bit** (this model: `PrivTlp.td`; real hardware: final DW of TLP Digest);
//     - **generation gate** = AER CapCtl[0] - **check gate** = AER CapCtl[1] (this model places
//       them on the AER face of the transmit/receive blocks);
//     - check failure => receiver sets **AER Corr Status bit19 (ECRC Error)** + sideband `ecrc_err`
//       asserted + count (NEVER silent).
//   * Header bytes = **this model's convention** (synthesized from the same source as the AER
//     Header Log: DW0 = fmt/type/td - DW1 = reqid/tag - DW2/DW3 = addr low/high) => cross-checking
//     against the real RTL ECRC engine = **M6b cross-check item**;
//     the `reflected` switch (default false) is reserved for switching if the cross-check shows
//     that the real hardware uses the reflected form.
//   * Self-check anchor (standard catalogue, NEVER self-proven): `ecrc32("123456789") == 0xFC891918`
//     (CRC-32/BZIP2).
// -----------------------------------------------------------------------------
#pragma once
#include "pcie_sc_wire.h"

#include <cstdint>
#include <vector>

namespace pcie_tlm {

// Raw byte-stream CRC (non-reflected; default init 0xFFFFFFFF + final inversion)
uint32_t ecrc32(const uint8_t* p, size_t n, bool reflected = false);

// CRC over TLP => 16B header (model convention) + payload
uint32_t ecrc_of_tlp(const PrivTlp& t, bool reflected = false);

// -- * T51-1: **spec-shaped TLP header 16B** (DW0..DW3; low byte first) --
//   DW0 = {Fmt[31:29] Type[28:24] T9[23] TC[22:20] T8[19] T7[18] Attr[17:16]
//          TD[15] EP[14] AT[13:12] Length[11:0]}
//   DW1 = {Requester ID[31:16] Tag[15:8] Last DW BE[7:4] First DW BE[3:0]} (**request form**;
//         Cpl-form DW1 semantics (Completer ID/Status/BCM/Byte Count) = M6b cross-check)
//   DW2/DW3: 3DW (fmt[0]=0) => DW2 = {Addr[31:2],00} - DW3 = 0;
//            4DW (fmt[0]=1) => DW2 = Addr[63:32] - DW3 = {Addr[31:2],00}
//   => **AER Header Log and ECRC share this synthesis** (single source of truth; no more separate
//      ad-hoc versions).
std::vector<uint8_t> tlp_header_bytes(const PrivTlp& t);
std::vector<uint32_t> tlp_header_dws(const PrivTlp& t);      // same as above, 4x DW view

} // namespace pcie_tlm
