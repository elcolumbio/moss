# 3D printing

Choose the hull, upper cover, computer carrier and camera option. With a one-piece hull and cover, the catalogue contains **106 pieces for RealSense or 107 for Gemini**. Each split option adds one piece, for a maximum of **109 pieces**. The download includes 113 part STL files plus two optional hull-joint fit coupons.

| Material | Parts |
|---|---|
| PETG | Hull, motor supports, transmission, electronics carriers, camera inserts and LiDAR mounts |
| PLA | Cover, bin, arm plate, arm and gripper body |
| TPU95A | Tracks |

Use the standard/default profile and **15% gyroid infill**. For the tracks, use **15% infill with the standard profile pattern, not gyroid**.

The STL files are oriented for printing. The [visual catalogue](https://www.showrobotics.ai/moss/build/#print) shows the material and quantity for every part; the [CSV](hardware/mechanical/r92/data/printed_parts.csv) gives the exact filenames.

## Choose the camera option

- RealSense: one RealSense support insert.
- Gemini: one flush insert and one window bezel. The thin straight bridges have been removed; the curved ends remain.

## Current set

The hull is R92, the motor supports R90, the Gemini window bezel R89, and both computer carriers R87. Each filename and catalogue entry carries its own part revision. The arm plate geometry remains R22.

The split hull has been rebuilt from the current R92 native geometry. Choose the single hull or the two halves in the catalogue. [Split-hull assembly and files](docs/build/SPLIT_HULL.md).

## Upper cover and outer track frames

Choose the [two-piece cover](docs/build/SPLIT_COVER.md) for a smaller build volume. Its two integral dovetails assemble from above and are glued before fitting the cover.

Print the outer track frames separately, flat, on the **front-right to rear-left diagonal**. Their STLs are already rotated. See the [bed placement diagram](docs/build/OUTER_FRAME_PRINTING.md).
