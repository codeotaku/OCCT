# Generalized chamfer regression coverage

The issue #1177 stack adds **3,539 registered GTest cases** and three self-contained
DRAW scripts. Parameter combinations are not independent geometric families;
this is broad contract coverage, not exhaustive surface-pair or branch coverage.
FreeCAD #12240 is integrated: endpoint handling belongs to the corner stage,
and its guide/NURBS coverage reuses the B-spline stage's existing smoothing fix.

The six reversed-curve distance-angle blockers now pass with the original
vertex-tolerance budget. The corrections address the inverse angle Jacobian,
consistent endpoint/restriction coordinates, and cumulative tolerance padding.
No tests are disabled and no acceptance tolerances are increased.

Configure with `BUILD_GTEST=ON` and build `OpenCascadeGTest`. Sources are registered
in the existing `TKFillet`, `TKBool`, and `TKGeomAlgo` GTest `FILES.cmake` lists.
Reconfigure CMake after adding a source to a list.

## Stage allocation

| Stage | New GTest cases | Responsibility |
| --- | ---: | --- |
| Corner construction | 145 | 27 valid-precursor failures, 36 local-recoil predicate cases, 32 surface/mode preservation controls and 50 analytic closed-contour cases. |
| Limit topology | 2,431 | Collapsed contacts, consumed restrictions, shared boundaries/pcurves, partial rims, periodic seams, stationary parameterizations, symmetric tolerance and parameter-unit contracts, invalid over-limit rejection and 360 partial/full lower-contour cases. |
| B-spline limits | 831 | Guide accuracy, boundary walking, inverse chamfer derivatives, measured tolerance propagation and actual-curve verification of polygonal near-tangency; 50 NURBS closed-contour cases and two direct guide-preservation tests. |
| Oriented chamfer construction | 132 | Walking transitions, independent/connected contours, acute/obtuse sections, reference ordering, per-contour modes, settings, history, editing, reset and recompute. No equivalent-order retry remains. |

## Test contracts

| Source / series | Cases | Objective and variations |
| --- | ---: | --- |
| `BRepFilletAPI_ChamferMatrix_Test` — base matrix | 32 | Eight surface families and three chamfer modes; transforms, distance sweeps, tapered arm/cylinder contacts and complex corners. Exact BRep/BOP validation, tolerances, material change and containment. These are preservation controls, not all fail-before reproducers. |
| `CurvedLivingEdges/BRepFilletAPI_ChamferCorner` | 27 | Valid Pocket precursor from FreeCAD #30886; 3 distances × 3 scales × identity/rigid/reflected placement. Explicit support isolates corner construction. Check the cylinder endpoint as well as topology and material removal. |
| `ChFi3d_CornerRecoil_Test` | 36 | Near-closed circles across the seam, rational B-spline representations and folded B-splines; both endpoints, reversed parameters, rigid placement and three scales. Require local interval membership, accept improved local projections, and retain the recoil for collapsed intervals/corner projections. |
| `BRepFilletAPI_ChamferLimit_Test` | 1,315 | Analytic single/opposing and sequential chamfers across below/at/above-limit states, selection/reference orders and placements. Includes 144 open lower-contour cases, 144 complete-loop cases and 72 bored-support cases. Check whether the exact bottom disk/annulus area must remain or disappear, as well as topology, tolerances and invalid overshoot rejection. |
| `ChFi3d_ChamferCornerExtension_Test` | 100 | 50 analytic cases in the corner stage and 50 NURBS variants in the B-spline stage. Sharp closed-contour endpoint extension and public chamfer construction across four angles, three scales, reversal and both reference supports; analytical material-removal and equivalent-volume checks. |
| `ChFi3d_Builder_0_Test` — added guide cases | 2 | Rational tangent joins, periodic seam and exact knot samples, geometric tangency, plus an independent shallow non-rational B-spline guide-deviation oracle. Existing fillet/periodic controls are not counted as additions. |
| `BRepFilletAPI_ChamferConsumedRim_Test` | 119 | Consumed and partial rims, closed restrictions and adjoining surface reconstruction; validate retained material and connected valid solids. |
| `BRepFilletAPI_ChamferTangentBoss_Test` | 94 | Valid public tangent-boss precursor and generalized variations; distinguish supported contact from invalid over-limit topology. |
| `BRepFilletAPI_ChamferAsymmetricLimit_Test` | 108 | Unequal sections, two-distance/non-45-degree distance-angle APIs, both references, below/at/above consumption, rigid/reflected placements. Compare volume and both Boolean differences with an independently extruded polygon. |
| `BRepFilletAPI_ChamferMultiWire_Test` | 27 | One to three pockets on a shared top face; rim chamfers around half-wall thickness. Require one closed solid/shell and retained floor/lower walls. |
| `BRepFilletAPI_ChamferCurvedSupports_Test` | 12 | Cylinder/cone, cone/cone and cylinder/sphere intersections without a planar support. Equal/unequal distances, rotated seams and equivalent reference orders. |
| `ChFi3d_Contact_Test` | 34 | Crossing, T-junction, endpoint, overlap, tangency, separated near-tangency and nearby crossings in both orders/directions. Cached decision tables separately exercise 144 transition combinations and eight boundedness records. |
| `TopOpeBRepBuild_Coincidence_Test` | 343 | Full/partial bounded coincidence for line, circle, ellipse, Bezier and periodic B-spline; direction versus topology orientation, endpoint contact/separation, tolerances, placement and representation. Includes 72 valid stationary-parameter cases with either/both nonlinear representations and orientation/placement permutations. Inputs must not be mutated. The edge overload of `BRepTools::Compare` is identity-only and insufficient for separately built coincident edges. |
| `TopOpeBRepBuild_CompleteCoincidence_Test` | 195 | Cached intersection range policy: complete/partial/reversed/swapped ranges, tolerances, empty/unbounded records and multiple segments. Includes 144 affine/nonlinear parameter-scale cases and one actual-intersector control with 12 range/tolerance combinations. Each curve converts geometric tolerance through its own resolution. One additional control has 12 direction checks down to a 1e-170 parameter range, without collapsing the physical curve. |
| `ChFi3d_CollapsedTrace_Test` | 99 | 48 unequal-tolerance point/point cases; 48 point/curve cases across plane/cylinder/sphere supports, line/arc/Bezier/rational B-spline traces, directions and operand orders (2,160 checks); three controls for explicit enlargement, ambiguous nearest parameters and missing records. These are contact-record tests, not degenerate input solids. |
| `TopOpeBRepBuild_ClosedRestriction_Test` | 48 | Closed circle/ellipse/rational B-spline restrictions with independent seam vertices. Check shared split edges, preserved vertices/length/tolerances, repeated records, reversal and placement. |
| `TopOpeBRepDS_SharedPCurve_Test` | 36 | Plane, cylinder and Bezier-extrusion supports; nonlinear/rescaled/reversed pcurves, locations and serialization. Check reached geometric error, not just SameParameter flags. |
| `TopOpeBRepDS_BuildTool_Test` — added case | 1 | Shared-edge pcurve parameter consistency for ordinary, reversed and partial reuse. Existing tests in this file are not counted as additions. |
| `BRepFilletAPI_ChamferUniformBSpline_Test` | 456 | Two constant-normal-width rings. Outer 2 mm, inner 2 mm and paired 1 mm limits; 72 nominal/placement/state cases, 48 explicit-reference cases, 24 serialization cases, 24 equivalent representations, 54 independent seams, 12 seam neighbors and 78 guide-accuracy cases; 108 distance-angle mode/reference/state cases and 36 guide-step reduction controls across support order, angle, rigid placement, scale and curve reversal. Twelve cases also simulate sections before building. |
| `BlendFunc_ChAsymInv_Test` | 288 | Check all 16 Jacobian entries against central differences of the residual, including separate/combined derivative APIs. Both restrictions and support orders, four orientation choices, three scales, rigid rotation and straight/circular/elliptical guides. Straight-guide controls distinguish the missing guide-normal derivative. |
| `BRepFilletAPI_ChamferTolerance_Test` | 27 | Exact-limit chamfers consume original slab vertices: one/two/four top edges, three scales and three input tolerances around the former fixed threshold. Require BRep/BOP validity and bounded vertex tolerances; visits to shared vertices must not accumulate fixed padding. |
| `Geom2dInt_ClosedTangency_Test` | 8 | Separated periodic normal offsets at three gaps and both directions, each with three tolerances; preserve genuine crossing, tangent and coincident controls. |
| `BRepFilletAPI_ChamferMatrix_Test` — ordering cases | 6 | Every plane/extrusion boundary/reference, unequal distance ratios, reversed edges, transforms, multiple and mixed-method contours, generated history, parameter queries, edits and rebuilding. |
| `ChFi3d_ChamferConfiguration` | 12 | Both support orders, all three future default modes, base/derived configuration, repeated build, distance edit, explicit reset and later base-API parameter edits. Preserve settings, classic contour mode, valid material removal and reference volume. No production-only test accessors remain. |
| `ChFi3d_ChamferMixedModes` | 6 | Classic, constant-throat and penetration contours, both acquisition orders, repeated build and contour removal. Compare valid results to independent component builds using volume and both Boolean differences; preserve each contour mode and the future default. |
| `ChFi3d_ChamferIndependentContours` | 48 | Four independent support-order combinations, two acquisition orders, three placements and disconnected/connected regions. Raw construction must agree with independently constructed components in volume and both Boolean differences. |
| `ChFi3d_ChamferOrientedSection` | 60 | Six signed spline bends, five asymmetric distance ratios and both ends of a valid extrusion. Test both equivalent support orders directly through raw construction, requiring valid closed solids, history, material removal and bidirectional Boolean agreement. |

## Fail-before evidence and limitations

The 27 direct corner cases fail on unchanged IR. Both DRAW scripts below also
fail on unchanged IR. Eleven of 30 selected analytic-limit cases fail before
the topology stage. All six representative B-spline limit geometries, 78 guide
accuracy cases and two separated-offset cases fail before the B-spline stage.
The six original ordering tests still fail after the first three stages. Preservation
controls are distinguished from these direct reproducers.

Adversarial review adds 126 cases. All 36 recoil cases fail with the original
full-edge projector; 64 of the 72 stationary-parameter cases fail with the
single-midpoint direction check. Pre-fix retry probes reproduce loss of angular
tolerance, a wrong-mode crash and failed mixed-mode recovery. Six primary-order
controls also expose stale reconstruction state on repeated computation.

The second review adds 352 registered cases. Before correction, 12 of the first
97 collapsed-contact cases fail and 41 of 194 coincidence cases fail; after
correction all pass. The two later projection controls also pass. Eighteen of
the initial 24 independent-contour raw constructions fail before the oriented
transition fix; all pass afterward, together with the connected-region and
acute/obtuse section extensions. These predicates have direct failing records;
no separate end-to-end fixture is claimed to isolate the unequal-tolerance or
compressed-parameter defect alone. Existing limit/representation series remain
end-to-end preservation controls.

A larger pre-topology sweep stopped progressing in an over-limit case and was
terminated; it is not a completed baseline run. The existing DRAW chamfer grid
was attempted before/after but all 279 cases lacked external model data. Those
skips are not successful geometry tests. Cross-platform/private-data CI remains
required before submission. No exhaustive surface Cartesian product, arbitrary
topology, or MC/DC coverage is claimed.

The lower-contour extension adds 360 cases: 72 fail before the partial-face
correction and all pass afterward. The complete loop already passed and is
preservation coverage, not a claimed failing reproducer. Both open and complete
10 mm operations on the valid supplied BothParallelRails precursor are checked.
The same open-contour failure was reproduced with the older unsplit kernel;
it is not established as a regression introduced by the four-stage split.
Existing all-edge flat-box fillet coverage detects the distinction between
real surviving arcs and degenerate point traces. The correction uses the existing
edge-degeneracy classification, not a new length/tolerance heuristic.

The integrated FreeCAD #12240 tests exercise two defects: closed-contour endpoint
selection and displacement during guide smoothing. Only the endpoint production
change is newly imported; the existing B-spline-stage smoothing cap is retained.
Direct guide regressions have failing evidence with smoothing protection reverted.

The final review adds 133 registered cases. The extreme cached-direction control
fails before replacing multiplied parameter spans with the intersection segment's
stored orientation. The 108 distance-angle cases have 24 failures before enabling
their boundary-contact gate, two afterward, and none with the predictor correction.
A reduced guide step previously retained the Newton estimate for the larger step;
the existing predictor now runs once immediately before solving, replacing four
scattered calls. This is shared walking logic, so existing fillet coverage is
retained. No new projection, solver or face-classification cache is introduced.

The additional 24 guide-step controls had ten failures with the widened gate
alone and six after predictor correction: reversed curves, first support,
45/60 degrees, three placements. The latest corrections resolve all six without
relaxing the 2.5e-4-times-scale vertex budget. The inverse angle Jacobian used a
temporary distance-gradient vector instead of the differentiated cross product.
An endpoint correction then changed the guide parameter without updating the
other contact's UV/restriction parameter. Finally, tolerance propagation added
fixed padding on every visit to a shared vertex.

The blocker pass adds 327 cases: 288 direct Jacobian checks, 27 tolerance controls
and 12 acute-angle integration controls. Before correction, 192 curved-guide
Jacobian cases and 18 above-threshold tolerance cases fail; all 315 pass afterward.
The 96 straight-guide Jacobian cases and nine below-threshold tolerance cases
are preservation controls. The 12 acute-angle integration cases also pass.
The Jacobian expression is shared between restriction orders, endpoint projection
reuses `BRepBlend_BlendTool::Project`, and only measured tolerances are propagated.
The three production files have a net-zero line change (+22/-22).

PR #1507 was checked at commit 8d23b491c74b1bad61184d31dd4859c716aba8d8. Its
walking changes handle open guides on conical periodic supports, not these closed
guides on closed, non-periodic B-spline extrusions. No seam-unwrapping helper or
code from that PR is duplicated here.

## DRAW issue tests

With `TOPTEST` loaded, run:

```
test bugs modalg_8 bug1177_box_contact
test bugs modalg_8 bug1177_opposing_contacts
test bugs modalg_8 bug_freecad12240_closed_contour
```

The first two use generated cubes, requiring no external data. They test a single
10 mm chamfer and opposing 5 mm chamfers, with validity, topology, volume and
maximum-tolerance checks.
The FreeCAD #12240 script generates a NURBS bore with a sharp closed contour and
checks validity, free boundaries, one solid/shell and analytical removed volume.
It requires both the corner and guide-preservation stages.

## Existing facilities and precision

Reuse `ShapeAnalysis_Curve::Project` on an existing trimmed adaptor and existing
corner construction; `IntTools_EdgeEdge`, `BOPTools_AlgoTools::IsSplitToReverse`, bounded projection, existing edge
splitting/curve merging and `BRepTools_ReShape` for topology; and
`BOPTools_AlgoTools::ComputeTolerance`/SameParameter facilities for pcurves.
`HasCompleteCoincidence` consolidates interpretation of cached intersection
records; it is not another intersection solver. Guide approximation uses
`GeomConvert_ApproxCurve` and existing knot removal, without changing fillet
smoothing. Surface orientation reuses the walking transition and `TopAbs::Compose`,
not the support-normal dot product when an oriented crossing is available. The
normal-based fallback remains for unclassified transitions. Distance edits reuse
the support mapping recorded by `PerformElement`. No contour-order search,
alternate state, replay or retry-only accessors remain. Recompute recreates only the reconstruction builder with its
existing BuildTool settings, since Clear() retains prior face-splitting state.

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
