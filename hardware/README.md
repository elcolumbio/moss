# Hardware

Designed in Fusion 360. The STL (one file per part) and STEP files land here at the design freeze, with the fastener list and the sources they derive from (SO-101 arm, NormaCore parallel-jaw gripper, widened to 90 mm) with exact revisions and local changes. The freeze happens after one test on the real robot: a full battery emptied over 3 km on the beach, tracks still on, no bolt lost.

Until then: the [parts list](../3DPrinting.md), the [BOM](../BOM.md) and the photos in `media/` are the reference.

`bench/` holds the raw traces of the bench tests, one folder per date. The 20 September INA219 trace is not a motor-current measurement.

## Electronics prototypes

[MOSS Core V1D](electronics/moss-core-v1d/) is the current routed design review: editable KiCad files, schematics, BOM, connector guide and static review evidence. **Draft only: not approved for fabrication or assembly.** Rover fit and hardware validation remain pending.

[V1C](electronics/moss-core-v1c/) is retained for traceability and **must not be fabricated** because of a physical-pin mapping error; see the [critical erratum](electronics/moss-core-v1d/V1C_CRITICAL_ERRATA.txt).
