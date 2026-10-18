# MOSS V0.6 / R92 mechanical package

**[Download the complete CAD & STL package](https://github.com/metrox-eth/moss/releases/download/v0.6-r92/MOSS_R92_Mechanical_20261016.zip)** · [Build guide](https://www.showrobotics.ai/moss/build/) · [BOM](../../../BOM.md) · [Release and individual files](https://github.com/metrox-eth/moss/releases/tag/v0.6-r92)

The package includes 113 part STL files plus two optional hull-joint fit coupons, editable Fusion source, STEP solid geometry, part previews and seven silent assembly clips. Choose the one-piece or two-piece hull and cover, one computer carrier, and one camera option. The updated catalogue uses 106–111 pieces, including the separately supplied Jetson tabs.

- [One-piece printable Fusion source](https://github.com/metrox-eth/moss/releases/download/v0.6-r92/MOSS_Printable_Source_R92.f3d)
- [STEP solids](https://github.com/metrox-eth/moss/releases/download/v0.6-r92/MOSS_Printable_BRep_Parts_R92.step)
- [Two-piece hull Fusion assembly](https://github.com/metrox-eth/moss/releases/download/v0.6-r92/MOSS_Hull_R92_Split_Assembly.f3d)
- [Two-piece upper-cover Fusion assembly](https://github.com/metrox-eth/moss/releases/download/v0.6-r92/MOSS_Upper_Cover_R10_Split_Assembly.f3d)
- [Parts CSV](../../../docs/build/printed_parts.csv)

The guide's catalogue links directly to each STL. The ZIP also contains attribution, licence notices, geometry checks and per-file checksums. Manufacturer electronics/camera reference models and the complete internal integration assembly are excluded.

**Jetson supplement:** print one [JT01](https://github.com/metrox-eth/moss/releases/download/v0.6-r92/Jetson_Retaining_Tab_1_R92.stl) and one [JT02](https://github.com/metrox-eth/moss/releases/download/v0.6-r92/Jetson_Retaining_Tab_2_R92.stl) in PETG. These files are separate from the unchanged original archive. See [fastener measurements](../../../docs/build/FASTENER_MEASUREMENTS.md#jetson-retaining-tabs).

[Printing instructions](../../../3DPrinting.md) · [Written assembly cards](../../../docs/build/ASSEMBLY.md)

## Next archive

The published 16 October 2026 archive is frozen. Do not replace it for the catalogue correction.

For the next archive:

1. Use the current [print catalogue](../../../docs/build/printed_parts.json); include one file per shared reference and keep its quantity.
2. Include both Jetson retaining tabs (JT01 and JT02), their STEP sources, and the checksummed supplements in [next_archive.json](next_archive.json).
3. Include the two optional hull fit coupons and [legacy filename aliases](../../../docs/build/print_aliases.json).
4. Use a new archive filename. Verify that every catalogue file and every required supplement is included.

The current [CSV](../../../docs/build/printed_parts.csv) is the builder-facing print list. Quantities are physical copies, not a count of distinct files.
