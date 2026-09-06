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

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <BRep_Tool.hxx>
#include <GeomConvert.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopOpeBRepBuild_Builder.hxx>
#include <TopOpeBRepDS_BuildTool.hxx>
#include <TopOpeBRepDS_CurvePointInterference.hxx>
#include <TopOpeBRepDS_HDataStructure.hxx>
#include <TopOpeBRepDS_Point.hxx>
#include <gtest/gtest.h>
#include <tuple>
#include <vector>

class TopOpeBRepBuild_ClosedRestriction
    : public testing::TestWithParam<std::tuple<int, int, bool, bool>>
{
};

TEST_P(TopOpeBRepBuild_ClosedRestriction, KeepsOriginalAndAllGeneratedSeamVertices)
{
  const auto [kind, seams, located, reversed] = GetParam();
  occ::handle<Geom_Curve> curve;
  if (kind == 0)
  {
    curve = new Geom_Circle(gp_Ax2(), 10.);
  }
  else
  {
    curve = new Geom_Ellipse(gp_Ax2(), 15., 10.);
  }
  if (kind == 2)
  {
    const auto bspline =
      GeomConvert::CurveToBSplineCurve(new Geom_TrimmedCurve(curve, 0., 2. * M_PI));
    bspline->SetPeriodic();
    curve = bspline;
  }
  auto edge = BRepBuilderAPI_MakeEdge(curve).Edge();
  if (located)
  {
    gp_Trsf trsf;
    trsf.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 2, 3)), .731);
    trsf.SetTranslationPart(gp_Vec(13, -7, 23));
    edge.Move(TopLoc_Location(trsf));
    curve = occ::down_cast<Geom_Curve>(curve->Transformed(trsf));
  }
  ASSERT_TRUE(BRepCheck_Analyzer(edge).IsValid());
  if (reversed)
  {
    curve->Reverse();
  }
  const double first = curve->FirstParameter(), period = curve->LastParameter() - first;
  occ::handle<TopOpeBRepDS_HDataStructure> hds = new TopOpeBRepDS_HDataStructure;
  auto&                                    ds  = hds->ChangeDS();
  std::vector<std::pair<int, gp_Pnt>>      points;
  // Repeat the final seam as an independent DS point to exercise coalescing.
  for (int i = 0; i <= seams + 1; ++i)
  {
    const int          seam  = std::min(i, seams);
    const double       start = first + period * .19 * seam;
    TopOpeBRepDS_Curve definition(curve, 1.e-7);
    definition.SetRange(start, start + period);
    definition.SetExistingEdge(edge, reversed);
    const int  curveIndex = ds.AddCurve(definition);
    const auto point      = curve->Value(start);
    const int  pointIndex = ds.AddPoint(TopOpeBRepDS_Point(point, 1.e-7));
    points.emplace_back(pointIndex, point);
    for (int end = 0; end < 2; ++end)
    {
      ds.ChangeCurveInterferences(curveIndex)
        .Append(new TopOpeBRepDS_CurvePointInterference(
          TopOpeBRepDS_Transition(end ? TopAbs_REVERSED : TopAbs_FORWARD),
          TopOpeBRepDS_CURVE,
          curveIndex,
          TopOpeBRepDS_POINT,
          pointIndex,
          start + end * period));
    }
  }
  const TopOpeBRepDS_BuildTool tool;
  TopOpeBRepBuild_Builder      builder(tool);
  builder.BuildVertices(hds);
  builder.BuildEdges(hds);
  const auto& splits = builder.NewEdges(1);
  ASSERT_EQ(splits.Size(), seams + 1);
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> vertices;
  double                                                        length = 0.;
  const BRepAdaptor_Curve                                       originalCurve(edge);
  double previousParameter = originalCurve.FirstParameter();
  for (const auto& split : splits)
  {
    ASSERT_TRUE(BRepCheck_Analyzer(split).IsValid());
    TopExp::MapShapes(split, TopAbs_VERTEX, vertices);
    const BRepAdaptor_Curve splitCurve(TopoDS::Edge(split));
    const double            splitLength = GCPnts_AbscissaPoint::Length(splitCurve, 1.e-10);
    length += splitLength;
    EXPECT_GT(splitLength, 1.e-6);
    EXPECT_NEAR(splitCurve.FirstParameter(), previousParameter, 1.e-12);
    previousParameter = splitCurve.LastParameter();
    EXPECT_LE(BRep_Tool::Tolerance(TopoDS::Edge(split)), 1.e-7);
  }
  EXPECT_EQ(vertices.Extent(), seams + 1);
  EXPECT_TRUE(vertices.Contains(TopExp::FirstVertex(edge)));
  for (const auto& point : points)
  {
    const auto vertex = TopoDS::Vertex(builder.NewVertex(point.first));
    EXPECT_TRUE(vertices.Contains(vertex));
    EXPECT_LE(BRep_Tool::Pnt(vertex).Distance(point.second), 1.e-7);
    EXPECT_LE(BRep_Tool::Tolerance(vertex), 1.e-7);
  }
  EXPECT_NEAR(previousParameter, originalCurve.LastParameter(), 1.e-12);
  // Use tolerance-controlled integration: the fixed quadrature in
  // BRepGProp::LinearProperties changes its ellipse error after splitting.
  EXPECT_NEAR(length, GCPnts_AbscissaPoint::Length(originalCurve, 1.e-10), 1.e-7);
  for (int i = 2; i <= hds->NbCurves(); ++i)
  {
    ASSERT_EQ(builder.NewEdges(i).Size(), splits.Size());
    auto expected = splits.cbegin();
    for (const auto& split : builder.NewEdges(i))
    {
      EXPECT_TRUE(split.IsSame(*expected++));
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  ClosedCurveFamilies,
  TopOpeBRepBuild_ClosedRestriction,
  testing::Combine(testing::Range(0, 3), testing::Range(0, 4), testing::Bool(), testing::Bool()));
