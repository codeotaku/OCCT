# Chamfer tangent-boss fixture

`ChamferTangentBoss.brep` is the valid Pad077 precursor exported from
`ChamferProblem.FCStd`, attached to FreeCAD issue #16782:
https://github.com/FreeCAD/FreeCAD/issues/16782

The document was fully recomputed before export. No geometric repair,
simplification, or dimensional change was applied to the precursor.
It passes BRepCheck_Analyzer and BRepAlgoAPI_Check and contains one solid,
7 faces, 15 edges, with volume 297.5478637499122 cubic millimeters.

The original operation selects the bottom tip arc (Edge12) at 0.5 mm.
Its tangent contour has two interior terminations at the larger cylinder.
At 1 mm the planar contact on the radius-1 tip collapses to a point.
Tests select the arc geometrically and cover scale, placement, mirroring,
edge orientation, near-limit and exact-limit distances.

The source FCStd SHA-256 is:
`0fbfea5d0a015ad09bae85bac4d7e04432a1149d16d93c2cd409983f7e020e08`.
