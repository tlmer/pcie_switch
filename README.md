# PCIe Gen6 Switch TLM Model

Host-side TLM model of a PCIe switch: root complex, upstream port and
crossbar are modeled in SystemC; downstream ports connect to the customer's
DWC PCIe RTL through the private-port contract. No operating system and no
HDL simulator are required for model-level use.

## Contents

```
README.md                      this file
USER_GUIDE.md                  integration and API guide
include/                       public headers (English)
lib/libpcie_switch_model.a     binary model library
console/                       standalone interactive console
rtl_bridge/                    private-port adapter reference (.veo included)
```

## Quick start

The package already contains the binary model library and the public headers.

```sh
cd console && make && ./pcie_switch_console
```

Inside the console:

```
run all
cover
quit
```

## Features

* Configurable downstream ports (1..32) plus one dual-mode (DM) port; address-window routing
* Unicast, multicast, broadcast, peer-to-peer, NTB
* **Standard PCIe Multicast extended capability (ECAP ID 0x0012)** on switch
  ports: MC_Base/Index/NumGroups, MC_Receive, MC_Block_All/Untranslated and
  MC_Overlay_BAR; MC window wins over normal address routing; only posted
  memory writes; the legacy private MC (16 groups)/BC windows remain as fallback.
* **Traversable extended-capability chain**: every enabled ECAP is linked by
  the header `Next` field from **0x100** (SR-IOV or the standard DSN
  `0x0003` anchor).  Standard register faces include AER, VC, DSN, ACS,
  ARI, ATS, SR-IOV, MC, ReBAR, PASID, DPC, LTR, L1SS, PTM, DOE,
  SPE/Secondary PCIe, Physical Layer 64 GT/s, Device 3, IDE, Flit Error
  Logging and Lane Margining.  DPC is at 0x1B0 (no longer collides with
  SR-IOV).
* Switch cascading: recursive enumeration with nested downstream windows
* Type0/Type1 configuration spaces, MSI / MSI-X / INTx
* No-OS enumeration with Linux-style `[ENUM]` logging
* Per-port, per-VC egress queues with strict-priority, round-robin and
  weighted round-robin arbitration; the standard VC ECAP exposes the
  TC/VC map and arbitration table.
* AER, PM/ASPM, FLR, ReBAR, ACS, ATS, PASID, 10-bit tag, VDM and atomic
  operations at the model level
* Private-port co-simulation bridge and `+NO_FSDB` diagnostics

## Boundaries

PHY, link training, equalization and flow control belong to the downstream RTL.
SPE, Physical Layer 64 GT/s, IDE, Flit Error Logging and Lane Margining are
configuration-space faces only; their link-training, encryption and flit-replay
semantics are not re-implemented in the model.  This model does not expose an
AXI4 interface in the switch package (the separate VIP co-simulation host has
its own AXI4 model/pin path), does not provide a DTS template or an example
program. See `USER_GUIDE.md` section 11 for details.

## Verification

`sh tlm/run.sh` runs **30 gates = 30/30 PASS · 0 FAIL** (2026-11-22),
including `TB-MC-ECAP 18/18` and `TB-ECAP-CHAIN 118/118`.  Evidence:
`evidence/P137_ecap_chain_p0p4_20261122/` (`run_30.log`, `status.txt`).

## Version

Model 1.1 -- the shipped configuration is recorded in `USER_GUIDE.md`
(section 8) and in the private-port adapter map.

2026-11-22: `lib/libpcie_switch_model.a` (12 objects), `include/` (14 public
headers) and `console/pcie_switch_console` were rebuilt from the source tree
with the standard ECAP batch; console `run all` = **5/5 PASS**.
