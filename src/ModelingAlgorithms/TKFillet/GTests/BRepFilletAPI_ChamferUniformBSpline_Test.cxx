// Copyright (c) 2026 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.
//
// Alternatively, this file may be used under the terms of Open CASCADE
// commercial license or contractual agreement.

#include <BRepAlgoAPI_Check.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_OffsetCurve.hxx>
#include <GeomConvert_ApproxCurve.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_HArray1.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Pln.hxx>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <tuple>
#include <sstream>
#include <BRep_Builder.hxx>
#include <BSplCLib.hxx>
#include <ChFi3d_Builder_0.hxx>
#include <ChFiDS_ChamfSpine.hxx>
#include <ChFiDS_ElSpine.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <fstream>

class BRepFilletAPI_UniformBSpline
    : public testing::TestWithParam<std::tuple<int, int, int, double>>
{
};

static occ::handle<Geom_BSplineCurve> uniformOuterCurve(const int geometry)
{
  const double                             coords[][2] = {{0, 30},
                                                          {-16, 29},
                                                          {-20, 20},
                                                          {-17, 8},
                                                          {-22, -10},
                                                          {-22, -26},
                                                          {-10, -35},
                                                          {8, -35},
                                                          {23, -24},
                                                          {25, -8},
                                                          {20, 8},
                                                          {25, 22},
                                                          {18, 29}};
  occ::handle<NCollection_HArray1<gp_Pnt>> points      = new NCollection_HArray1<gp_Pnt>(1, 13);
  for (int i = 1; i <= 13; ++i)
  {
    points->SetValue(i,
                     geometry == 0 ? gp_Pnt(coords[i - 1][0], coords[i - 1][1], 0)
                                   : gp_Pnt(24. * std::cos(2. * M_PI * (i - 1) / 13.),
                                            30. * std::sin(2. * M_PI * (i - 1) / 13.),
                                            0));
  }
  GeomAPI_Interpolate interpolation(points, true, 1.e-9);
  interpolation.Perform();
  return interpolation.Curve();
}

static void checkUniformBSpline(const int    geometry,
                                const int    selection,
                                const int    variant,
                                const double fraction,
                                const int    explicitFace   = 0,
                                const bool   roundTrip      = false,
                                const int    representation = 0,
                                const int    seamOrigin     = 4,
                                const double angle          = 0.,
                                const bool   simulate       = false)
{
  const auto        outer = uniformOuterCurve(geometry);
  const std::string seamSuffix =
    representation == 2 && seamOrigin != 4 ? "-origin" + std::to_string(seamOrigin) : "";
  const std::string resultSuffix =
    seamSuffix + (variant ? "-variant" + std::to_string(variant) : "");
  occ::handle<Geom_OffsetCurve> offset = new Geom_OffsetCurve(outer, -2., gp_Dir(0, 0, 1));
  GeomConvert_ApproxCurve       approximation(offset, 1.e-8, GeomAbs_C2, 1000, 14);
  ASSERT_TRUE(approximation.IsDone());
  ASSERT_LE(approximation.MaxError(), 1.e-8);
  const auto inner = approximation.Curve();
  for (int i = 0; i <= 200; ++i)
  {
    const double t =
      outer->FirstParameter() + (outer->LastParameter() - outer->FirstParameter()) * i / 200.;
    EXPECT_NEAR(outer->Value(t).Distance(inner->Value(t)), 2., 2.e-8);
  }
  // These operations change only representation, not the physical offset ring.
  for (const auto& curve : {outer, inner})
  {
    if (representation == 1)
    {
      curve->IncreaseDegree(std::max(8, curve->Degree()));
      curve->InsertKnot(curve->FirstParameter()
                          + .317 * (curve->LastParameter() - curve->FirstParameter()),
                        1);
    }
    else if (representation == 3)
    {
      NCollection_Array1<double> knots(curve->Knots());
      BSplCLib::Reparametrize(curve == outer ? -3. : 7., curve == outer ? 11. : 29., knots);
      curve->SetKnots(knots);
    }
    else if (representation == 4)
    {
      curve->Reverse();
    }
  }
  if (representation == 2)
  {
    outer->SetOrigin(seamOrigin); // Independent periodic seam, not a rigid rotation.
  }
  const auto outerFace =
    BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(), gp_Dir(0, 0, 1)),
                            BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(outer)),
                            true)
      .Face();
  const auto innerFace =
    BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(), gp_Dir(0, 0, 1)),
                            BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(inner)),
                            true)
      .Face();
  const auto source =
    BRepPrimAPI_MakePrism(BRepAlgoAPI_Cut(outerFace, innerFace).Shape(), gp_Vec(0, 0, 10)).Shape();
  ASSERT_TRUE(BRepCheck_Analyzer(source).IsValid());
  ASSERT_TRUE(BRepAlgoAPI_Check(source).IsValid());
  TopoDS_Edge selected, inside;
  double      longest = 0., shortest = RealLast();
  for (TopExp_Explorer it(source, TopAbs_EDGE); it.More(); it.Next())
  {
    const auto    edge = TopoDS::Edge(it.Current());
    TopoDS_Vertex v1, v2;
    TopExp::Vertices(edge, v1, v2);
    if (std::abs(BRep_Tool::Pnt(v1).Z() - 10.) > 1.e-7
        || std::abs(BRep_Tool::Pnt(v2).Z() - 10.) > 1.e-7)
    {
      continue;
    }
    GProp_GProps length;
    BRepGProp::LinearProperties(edge, length);
    if (length.Mass() > longest)
    {
      selected = edge;
      longest  = length.Mass();
    }
    if (length.Mass() < shortest)
    {
      inside   = edge;
      shortest = length.Mass();
    }
  }
  ASSERT_FALSE(selected.IsNull());
  ASSERT_FALSE(inside.IsNull());
  const double scale = variant == 2 ? 2. : 1.;
  gp_Trsf      transform;
  if (variant == 1)
  {
    transform.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 2, 3)), 37. * M_PI / 180.);
  }
  else
  {
    transform.SetScale(gp_Pnt(), scale);
  }
  BRepBuilderAPI_Transform                                      placed(source, transform, true);
  auto                                                          input = placed.Shape();
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> originalEdges, restoredEdges;
  if (roundTrip)
  {
    TopExp::MapShapes(input, TopAbs_EDGE, originalEdges);
    std::stringstream stream;
    BRepTools::Write(input, stream);
    BRep_Builder builder;
    BRepTools::Read(input, stream, builder);
    TopExp::MapShapes(input, TopAbs_EDGE, restoredEdges);
    ASSERT_EQ(originalEdges.Extent(), restoredEdges.Extent());
  }
  if (const char* dir = std::getenv("BSPLINE_OUTPUT"))
  {
    BRepTools::Write(
      input,
      (std::string(dir) + "/cpp-source-" + std::to_string(geometry) + "-" + std::to_string(variant)
       + (representation ? "-repr" + std::to_string(representation) : "") + seamSuffix + ".brep")
        .c_str());
  }
  ASSERT_TRUE(BRepCheck_Analyzer(input).IsValid());
  ASSERT_TRUE(BRepAlgoAPI_Check(input).IsValid());
  BRepFilletAPI_MakeChamfer chamfer(input);
  const double              distance = (selection == 2 ? 1. : 2.) * fraction * scale;
  const auto                addEdge  = [&](const TopoDS_Edge& original) {
    auto edge = TopoDS::Edge(placed.ModifiedShape(original));
    if (roundTrip)
    {
      edge = TopoDS::Edge(restoredEdges(originalEdges.FindIndex(edge)));
    }
    if (explicitFace == 0)
    {
      chamfer.Add(distance, edge);
      return;
    }
    NCollection_IndexedDataMap<TopoDS_Shape,
                                               NCollection_List<TopoDS_Shape>,
                                               TopTools_ShapeMapHasher>
      map;
    TopExp::MapShapesAndAncestors(input, TopAbs_EDGE, TopAbs_FACE, map);
    const auto& faces = map.FindFromKey(edge);
    const auto  face = TopoDS::Face(explicitFace == 1 ? faces.First() : faces.Last());
    if (angle == 0.)
      chamfer.Add(distance, distance, edge, face);
    else
    {
      // Keep the same physical section when exchanging the reference support.
      const bool onPlane = BRepAdaptor_Surface(face).GetType() == GeomAbs_Plane;
      chamfer.AddDA(onPlane ? distance : distance * std::tan(angle),
                    onPlane ? angle : M_PI / 2. - angle,
                    edge,
                    face);
    }
  };
  if (selection != 1)
  {
    addEdge(selected);
  }
  if (selection != 0)
  {
    addEdge(inside);
  }
  if (simulate)
    for (int contour = 1; contour <= chamfer.NbContours(); ++contour)
    {
      ASSERT_NO_THROW(chamfer.Simulate(contour));
      ASSERT_GT(chamfer.NbSurf(contour), 0);
      for (int surface = 1; surface <= chamfer.NbSurf(contour); ++surface)
      {
        const auto sections = chamfer.Sect(contour, surface);
        ASSERT_FALSE(sections.IsNull());
        ASSERT_GT(sections->Length(), 0);
        for (int i = sections->Lower(); i <= sections->Upper(); ++i)
        {
          gp_Lin line;
          double first, last;
          sections->Value(i).Get(line, first, last);
          EXPECT_TRUE(std::isfinite(first) && std::isfinite(last));
          EXPECT_GT(std::abs(last - first), Precision::Confusion());
        }
      }
    }
  chamfer.Build();
  if (fraction > 1.)
  {
    EXPECT_FALSE(chamfer.IsDone());
    return;
  }
  ASSERT_TRUE(chamfer.IsDone());
  const auto result = chamfer.Shape();
  if (representation)
  {
    if (const char* dir = std::getenv("BSPLINE_OUTPUT"))
    {
      BRepTools::Write(result,
                       (std::string(dir) + "/representation-" + std::to_string(geometry) + "-"
                        + std::to_string(selection) + "-" + std::to_string(representation)
                        + resultSuffix + ".brep")
                         .c_str());
    }
  }
  ASSERT_TRUE(BRepCheck_Analyzer(result, true, false, true).IsValid());
  BRepAlgoAPI_Check resultCheck(result);
  if (const char* dir = std::getenv("BSPLINE_OUTPUT"))
  {
    TopoDS_Compound faults;
    BRep_Builder    builder;
    builder.MakeCompound(faults);
    for (const auto& fault : resultCheck.Result())
    {
      for (const auto& shape : fault.GetFaultyShapes1())
      {
        builder.Add(faults, shape);
      }
    }
    BRepTools::Write(faults,
                     (std::string(dir) + "/faults-" + std::to_string(geometry) + "-"
                      + std::to_string(selection) + "-" + std::to_string(representation)
                      + resultSuffix + ".brep")
                       .c_str());
  }
  ASSERT_TRUE(resultCheck.IsValid());
  int solids = 0, shells = 0;
  for (TopExp_Explorer it(result, TopAbs_SOLID); it.More(); it.Next())
  {
    ++solids;
  }
  for (TopExp_Explorer it(result, TopAbs_SHELL); it.More(); it.Next())
  {
    ++shells;
    EXPECT_TRUE(BRep_Tool::IsClosed(it.Current()));
  }
  EXPECT_EQ(solids, 1);
  EXPECT_EQ(shells, 1);
  for (TopExp_Explorer it(result, TopAbs_VERTEX); it.More(); it.Next())
  {
    // Bound the accumulated guide/surface approximation error (default
    // approximation tolerance is 1e-4), without accepting millimetric repairs.
    ASSERT_LE(BRep_Tool::Tolerance(TopoDS::Vertex(it.Current())), 2.5e-4 * scale);
  }
  GProp_GProps before, after;
  // Split integration at B-spline spans: the non-adaptive volume shortcut
  // and unpartitioned adaptive integration are not reliable volume oracles here.
  BRepGProp::VolumePropertiesGK(input, before, 1.e-8, true, true);
  BRepGProp::VolumePropertiesGK(result, after, 1.e-8, true, true);
  EXPECT_GT(after.Mass(), 0.);
  EXPECT_LT(after.Mass(), before.Mass());
  // Independent volume oracle for a planar simple closed contour: the offset
  // perimeter changes by 2*pi*x. Integrating the triangular chamfer section
  // gives d*d*L/2 +/- pi*d*d*d/3. Opposing cuts cancel the curvature terms.
  const double d       = distance / scale;
  double       removed = 0.;
  if (selection != 1)
  {
    removed += d * d * longest / 2. - M_PI * d * d * d / 3.;
  }
  if (selection != 0)
  {
    removed += d * d * shortest / 2. + M_PI * d * d * d / 3.;
  }
  // For a non-45-degree section, depth scales by tan(angle) at every offset.
  if (angle != 0.)
    removed *= std::tan(angle);
  EXPECT_NEAR(before.Mass() - after.Mass(),
              removed * scale * scale * scale,
              std::max(.01, removed * 1.e-4) * scale * scale * scale);
  int topFaces = 0;
  for (TopExp_Explorer it(result, TopAbs_FACE); it.More(); it.Next())
  {
    if (BRepAdaptor_Surface(TopoDS::Face(it.Current())).GetType() != GeomAbs_Plane)
    {
      continue;
    }
    GProp_GProps area;
    BRepGProp::SurfaceProperties(it.Current(), area);
    if (std::abs(area.CentreOfMass().Transformed(transform.Inverted()).Z() - 10.) < 1.e-6)
    {
      ++topFaces;
    }
  }
  EXPECT_EQ(topFaces, fraction == 1. ? 0 : 1);
  if (const char* dir = std::getenv("BSPLINE_OUTPUT"))
  {
    BRepTools::Write(result,
                     (std::string(dir) + "/cpp-uniform-" + std::to_string(geometry) + "-"
                      + std::to_string(selection) + "-" + std::to_string(variant) + "-"
                      + std::to_string(fraction) + ".brep")
                       .c_str());
  }
}

TEST_P(BRepFilletAPI_UniformBSpline, ConsumesConstantWidthRim)
{
  const auto [geometry, selection, variant, fraction] = GetParam();
  checkUniformBSpline(geometry, selection, variant, fraction);
}

class BRepFilletAPI_UniformBSplineDistanceAngle
    : public testing::TestWithParam<std::tuple<int, int, int, double, double>>
{
};

TEST_P(BRepFilletAPI_UniformBSplineDistanceAngle, PreservesSectionAcrossSupportReferences)
{
  const auto [geometry, selection, face, degrees, fraction] = GetParam();
  checkUniformBSpline(geometry,
                      selection,
                      0,
                      fraction,
                      face,
                      false,
                      0,
                      4,
                      degrees * M_PI / 180.,
                      degrees == 45. && fraction == 1.);
}

INSTANTIATE_TEST_SUITE_P(SectionAngles,
                         BRepFilletAPI_UniformBSplineDistanceAngle,
                         testing::Combine(testing::Values(0, 1),
                                          testing::Values(0, 1, 2),
                                          testing::Values(1, 2),
                                          testing::Values(30., 45., 60.),
                                          testing::Values(.95, 1., 1.005)));

class BRepFilletAPI_UniformBSplineExplicitFace
    : public testing::TestWithParam<std::tuple<int, int, int, double>>
{
};

TEST_P(BRepFilletAPI_UniformBSplineExplicitFace, BothSupportOrders)
{
  const auto [geometry, selection, face, fraction] = GetParam();
  checkUniformBSpline(geometry, selection, 0, fraction, face);
}

INSTANTIATE_TEST_SUITE_P(ExplicitSupports,
                         BRepFilletAPI_UniformBSplineExplicitFace,
                         testing::Combine(testing::Values(0, 1),
                                          testing::Values(0, 1, 2),
                                          testing::Values(1, 2),
                                          testing::Values(.95, .9995, 1., 1.005)));

class BRepFilletAPI_UniformBSplineRoundTrip
    : public testing::TestWithParam<std::tuple<int, int, double>>
{
};

TEST_P(BRepFilletAPI_UniformBSplineRoundTrip, SerializedGeometryRemainsSupported)
{
  const auto [geometry, selection, fraction] = GetParam();
  checkUniformBSpline(geometry, selection, 0, fraction, 1, true);
}

INSTANTIATE_TEST_SUITE_P(Serialization,
                         BRepFilletAPI_UniformBSplineRoundTrip,
                         testing::Combine(testing::Values(0, 1),
                                          testing::Values(0, 1, 2),
                                          testing::Values(.95, .9995, 1., 1.005)));

INSTANTIATE_TEST_SUITE_P(NormalOffsetRing,
                         BRepFilletAPI_UniformBSpline,
                         testing::Combine(testing::Values(0, 1),
                                          testing::Values(0, 1, 2),
                                          testing::Values(0, 1, 2),
                                          testing::Values(.95, .9995, 1., 1.005)));

class BRepFilletAPI_UniformBSplineRepresentation
    : public testing::TestWithParam<std::tuple<int, int, int>>
{
};

TEST_P(BRepFilletAPI_UniformBSplineRepresentation, EquivalentCurveRepresentationsConsumeSameRim)
{
  const auto [geometry, selection, representation] = GetParam();
  checkUniformBSpline(geometry, selection, 0, 1., 1, false, representation);
}

INSTANTIATE_TEST_SUITE_P(EquivalentRepresentations,
                         BRepFilletAPI_UniformBSplineRepresentation,
                         testing::Combine(testing::Values(0, 1),
                                          testing::Values(0, 1, 2),
                                          testing::Range(1, 5)));

class BRepFilletAPI_UniformBSplineSeam
    : public testing::TestWithParam<std::tuple<int, int, int, int>>
{
};

TEST_P(BRepFilletAPI_UniformBSplineSeam, IndependentSeamsPreserveValidLimitTopology)
{
  const auto [geometry, selection, origin, variant] = GetParam();
  // Placement also varies reference-face order, with a serialized case.
  checkUniformBSpline(geometry, selection, variant, 1., variant, variant == 2, 2, origin);
}

INSTANTIATE_TEST_SUITE_P(IndependentSeams,
                         BRepFilletAPI_UniformBSplineSeam,
                         testing::Combine(testing::Values(0, 1),
                                          testing::Values(0, 1, 2),
                                          testing::Values(2, 7, 13),
                                          testing::Values(0, 1, 2)));

class BRepFilletAPI_UniformBSplineSeamNeighbor
    : public testing::TestWithParam<std::tuple<int, int, double>>
{
};

TEST_P(BRepFilletAPI_UniformBSplineSeamNeighbor, BelowLimitSucceedsAndAboveLimitIsRejected)
{
  const auto [geometry, selection, fraction] = GetParam();
  checkUniformBSpline(geometry, selection, 0, fraction, 0, false, 2, 7);
}

INSTANTIATE_TEST_SUITE_P(IndependentSeamNeighbors,
                         BRepFilletAPI_UniformBSplineSeamNeighbor,
                         testing::Combine(testing::Values(0, 1),
                                          testing::Values(0, 1, 2),
                                          testing::Values(.9995, 1.005)));

class ChFi3d_ChamferGuide : public testing::TestWithParam<std::tuple<int, int, int>>
{
};

TEST_P(ChFi3d_ChamferGuide, SeamAndInflectionsDoNotRelaxApproximationBudget)
{
  const auto [geometry, origin, variant] = GetParam();
  const auto curve                       = uniformOuterCurve(geometry);
  curve->SetOrigin(origin);
  if (variant == 1)
  {
    curve->Reverse();
  }
  else if (variant == 2)
  {
    curve->Scale(gp_Pnt(), 2.);
  }
  const TopoDS_Edge         edge  = BRepBuilderAPI_MakeEdge(curve);
  occ::handle<ChFiDS_Spine> spine = new ChFiDS_ChamfSpine(1.e-7);
  spine->SetEdges(edge);
  spine->SetFirstStatus(ChFiDS_Closed);
  spine->SetLastStatus(ChFiDS_Closed);
  spine->Load();
  occ::handle<ChFiDS_ElSpine> guide = new ChFiDS_ElSpine();
  guide->FirstParameter(spine->FirstParameter());
  guide->LastParameter(spine->LastParameter());
  guide->SetPeriodic(true);
  ChFi3d_PerformElSpine(guide, spine, GeomAbs_C1, 1.e-7);
  ASSERT_FALSE(guide->BSpline().IsNull());
  EXPECT_TRUE(guide->BSpline()->IsPeriodic());
  std::ofstream csv;
  if (const char* dir = std::getenv("BSPLINE_OUTPUT"))
  {
    const std::string stem = std::string(dir) + "/guide-" + std::to_string(geometry) + "-"
                             + std::to_string(origin) + "-" + std::to_string(variant);
    csv.open(stem + ".csv");
    csv << "fraction,x,y,error\n";
    BRepTools::Write(BRepBuilderAPI_MakeEdge(guide->BSpline()).Edge(), (stem + ".brep").c_str());
  }
  double maxError = 0.;
  // Bidirectional geometric comparison, independent of curve parameterization.
  for (int direction = 0; direction < 2; ++direction)
  {
    const auto                  source = direction ? curve : guide->BSpline();
    const auto                  target = direction ? guide->BSpline() : curve;
    GeomAPI_ProjectPointOnCurve projection;
    projection.Init(target, target->FirstParameter(), target->LastParameter());
    for (int i = 0; i <= 256; ++i)
    {
      const double fraction = i / 256.;
      const auto   p        = source->Value(
        source->FirstParameter() + fraction * (source->LastParameter() - source->FirstParameter()));
      projection.Perform(p);
      ASSERT_GT(projection.NbPoints(), 0);
      maxError = std::max(maxError, projection.LowerDistance());
      if (!direction && csv)
      {
        csv << fraction << ',' << p.X() << ',' << p.Y() << ',' << projection.LowerDistance()
            << '\n';
      }
    }
  }
  EXPECT_LE(maxError, 3.e-7); // Approximation plus subsequent knot-removal budget.
}

INSTANTIATE_TEST_SUITE_P(PeriodicKnots,
                         ChFi3d_ChamferGuide,
                         testing::Combine(testing::Values(0, 1),
                                          testing::Range(1, 14),
                                          testing::Values(0, 1, 2)));
