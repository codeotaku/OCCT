# Chamfer regression models

`bug1177_tangent_boss.brep` is the valid recomputed Pad077 precursor from the
public FreeCAD [#16782](https://github.com/FreeCAD/FreeCAD/issues/16782) model.
The tangent-boss tests also reconstruct an analytic family with varied radii,
lengths, transforms, distances, and coincident-boundary configurations.

`bug1177_corner.brep` is the valid `Pocket` precursor exported from the public
FreeCAD issue [#30886](https://github.com/FreeCAD/FreeCAD/issues/30886) attachment.
The regression checks exact input validity before constructing any chamfer.
The later invalid `Fillet` feature is not included. No shape repair is applied.

The fixture retains the original edge numbering: Edge31 is the arm edge whose
corner meets a cylindrical living edge. Tests use the explicit second support
face to isolate corner construction from support-order retry policy.

The 27 cases cover three distances (0.5, 1, 2), three scales (0.1, 1, 10), and
identity, rigid, and mirrored placements. Besides exact result validity and
shell closure, they verify that the cylinder endpoint follows the adjacent
chamfer endpoint, rather than the unrelated historical recoil distance.

For upstream integration this small public model can be moved to OCCT's test
data repository by maintainers; its issue-prefixed filename avoids collisions.

## Existing blend behavior

Three small, unmodified public fixtures are included from OCCT's
[7.9.0 test dataset](https://github.com/Open-Cascade-SAS/OCCT/releases/download/V7_9_0_beta1/opencascade-dataset-7.9.0.tar.xz)
to keep the GTests independent of an external DRAW data installation:

| Local fixture | Dataset name | Existing DRAW test | Contract |
| --- | --- | --- | --- |
| `blend_self_intersection.brep` | `CCV_1_h1_gsk.rle` | `blend simple X4` | Repair a self-intersecting trimming wire, even when sampled curve distances pass. |
| `blend_trimmed_restriction.brep` | `CCV_1_n12gsq.rle` | `blend simple Y1` | Preserve existing restriction-edge trimming history. |
| `blend_boundary_continuation.brep` | `7_B3.draw` | `blend tolblend_buildvol A7` | Accept a continuation direction on an angular boundary within `Precision::Angular()`. |

The GTests validate each input and exercise placements and scales. The slot-in-
cylinder family is constructed procedurally and needs no fixture. These are
preservation tests for the fillet machinery shared by chamfer construction, not
new fillet features.
