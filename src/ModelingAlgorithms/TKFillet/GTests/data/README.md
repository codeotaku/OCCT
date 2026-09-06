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
