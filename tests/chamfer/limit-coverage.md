# Generalized chamfer regression coverage

The issue #1177 stack adds **2,139 registered GTest cases** and two self-contained
DRAW scripts. Parameter combinations are not independent geometric families;
this is broad contract coverage, not exhaustive surface-pair or branch coverage.
The unrelated sharp-corner FreeCAD #12240 patch is excluded.

Configure with `BUILD_GTEST=ON` and build `OpenCascadeGTest`. Sources are registered
in the existing `TKFillet`, `TKBool`, and `TKGeomAlgo` GTest `FILES.cmake` lists.
Reconfigure CMake after adding a source to a list.

## Stage allocation

| Stage | New GTest cases | Responsibility |
| --- | ---: | --- |
| Corner construction | 59 | 27 valid-precursor failures plus 32 surface/mode and adversarial preservation controls. |
| Limit topology | 1,754 | Collapsed contacts, consumed restrictions, shared boundaries/pcurves, partial rims, periodic seams and invalid over-limit rejection. |
| B-spline limits | 320 | Guide accuracy, boundary walking and actual-curve verification of polygonal near-tangency. |
| Equivalent-order recovery | 6 | Two-distance reference ordering, history and public editing/rebuild behavior. |

## Test contracts

| Source / series | Cases | Objective and variations |
| --- | ---: | --- |
| `BRepFilletAPI_ChamferMatrix_Test` — base matrix | 32 | Eight surface families and three chamfer modes; transforms, distance sweeps, tapered arm/cylinder contacts and complex corners. Exact BRep/BOP validation, tolerances, material change and containment. These are preservation controls, not all fail-before reproducers. |
| `CurvedLivingEdges/BRepFilletAPI_ChamferCorner` | 27 | Valid Pocket precursor from FreeCAD #30886; 3 distances × 3 scales × identity/rigid/reflected placement. Explicit support isolates corner construction. Check the cylinder endpoint as well as topology and material removal. |
| `BRepFilletAPI_ChamferLimit_Test` | 955 | Analytic single/opposing and sequential chamfers across below/at/above-limit states, selection/reference orders and placements. Validate shared consumed-boundary topology and reject invalid overshoots. |
| `BRepFilletAPI_ChamferConsumedRim_Test` | 119 | Consumed and partial rims, closed restrictions and adjoining surface reconstruction; validate retained material and connected valid solids. |
| `BRepFilletAPI_ChamferTangentBoss_Test` | 94 | Valid public tangent-boss precursor and generalized variations; distinguish supported contact from invalid over-limit topology. |
| `BRepFilletAPI_ChamferAsymmetricLimit_Test` | 108 | Unequal sections, two-distance/non-45-degree distance-angle APIs, both references, below/at/above consumption, rigid/reflected placements. Compare volume and both Boolean differences with an independently extruded polygon. |
| `BRepFilletAPI_ChamferMultiWire_Test` | 27 | One to three pockets on a shared top face; rim chamfers around half-wall thickness. Require one closed solid/shell and retained floor/lower walls. |
| `BRepFilletAPI_ChamferCurvedSupports_Test` | 12 | Cylinder/cone, cone/cone and cylinder/sphere intersections without a planar support. Equal/unequal distances, rotated seams and equivalent reference orders. |
| `ChFi3d_Contact_Test` | 34 | Crossing, T-junction, endpoint, overlap, tangency, separated near-tangency and nearby crossings in both orders/directions. Cached decision tables separately exercise 144 transition combinations and eight boundedness records. |
| `TopOpeBRepBuild_Coincidence_Test` | 271 | Full/partial bounded coincidence for line, circle, ellipse, Bezier and periodic B-spline; direction versus topology orientation, endpoint contact/separation, tolerances, placement and representation. Inputs must not be mutated. Identity-only `BRepTools::Compare` is explicitly shown insufficient for separate coincident edges. |
| `TopOpeBRepBuild_CompleteCoincidence_Test` | 49 | Cached intersection range policy independent of numerical intersection: complete/partial/reversed/swapped ranges, tolerances, empty and unbounded records, multiple segments. |
| `TopOpeBRepBuild_ClosedRestriction_Test` | 48 | Closed circle/ellipse/rational B-spline restrictions with independent seam vertices. Check shared split edges, preserved vertices/length/tolerances, repeated records, reversal and placement. |
| `TopOpeBRepDS_SharedPCurve_Test` | 36 | Plane, cylinder and Bezier-extrusion supports; nonlinear/rescaled/reversed pcurves, locations and serialization. Check reached geometric error, not just SameParameter flags. |
| `TopOpeBRepDS_BuildTool_Test` — added case | 1 | Shared-edge pcurve parameter consistency for ordinary, reversed and partial reuse. Existing tests in this file are not counted as additions. |
| `BRepFilletAPI_ChamferUniformBSpline_Test` | 312 | Two constant-normal-width rings. Outer 2 mm, inner 2 mm and paired 1 mm limits; 72 nominal/placement/state cases, 48 explicit-reference cases, 24 serialization cases, 24 equivalent representations, 54 independent seams, 12 seam neighbors and 78 guide-accuracy cases. |
| `Geom2dInt_ClosedTangency_Test` | 8 | Separated periodic normal offsets at three gaps and both directions, each with three tolerances; preserve genuine crossing, tangent and coincident controls. |
| `BRepFilletAPI_ChamferMatrix_Test` — retry cases | 6 | Every plane/extrusion boundary/reference, unequal distance ratios, reversed edges, transforms, multiple and mixed-method contours, generated history, parameter queries, edits and rebuilding. |

## Fail-before evidence and limitations

The 27 direct corner cases fail on unchanged IR. Both DRAW scripts below also
fail on unchanged IR. Eleven of 30 selected analytic-limit cases fail before
the topology stage. All six representative B-spline limit geometries, 78 guide
accuracy cases and two separated-offset cases fail before the B-spline stage.
The six retry tests still fail after the first three stages. Preservation
controls are distinguished from these direct reproducers.

A larger pre-topology sweep stopped progressing in an over-limit case and was
terminated; it is not a completed baseline run. The existing DRAW chamfer grid
was attempted before/after but all 279 cases lacked external model data. Those
skips are not successful geometry tests. Cross-platform/private-data CI remains
required before submission. No exhaustive surface Cartesian product, arbitrary
topology, or MC/DC coverage is claimed.

## DRAW issue tests

With `TOPTEST` loaded, run:

```
test bugs modalg_8 bug1177_box_contact
test bugs modalg_8 bug1177_opposing_contacts
```

These use generated cubes, requiring no external data. They test a single
10 mm chamfer and opposing 5 mm chamfers, with validity, topology, volume and
maximum-tolerance checks.

## Existing facilities and precision

Reuse `ShapeAnalysis_Curve::Project` and existing corner construction;
`IntTools_EdgeEdge`, `IntTools_Context`, bounded projection, existing edge
splitting/curve merging and `BRepTools_ReShape` for topology; and
`BRepLib_CheckCurveOnSurface`/SameParameter facilities for pcurves.
`HasCompleteCoincidence` consolidates interpretation of cached intersection
records; it is not another intersection solver. Guide approximation uses
`GeomConvert_ApproxCurve` and existing knot removal, without changing fillet
smoothing. Equivalent-order recovery reuses the existing chamfer builder.

The smallest matrix case explicitly sets approximation tolerances with the
existing `ChFi3d_ChBuilder::SetParams`. This does not change public default
precision policy. The concave torus fixture adds material; its oracle must not
be treated as a cutting-only chamfer. All public geometry fixtures are validated
before the operation; invalid upstream geometry is excluded.

## Diagnostic geometry

Set `CHAMFER_COVERAGE_OUTPUT` to an existing directory for asymmetric, multi-wire
and curved-support input/result BReps (and independent asymmetric references).
Set `BSPLINE_OUTPUT` for B-spline input/result BReps and guide-error CSV files.
Representation-specific filenames preserve seam/knot variants.
