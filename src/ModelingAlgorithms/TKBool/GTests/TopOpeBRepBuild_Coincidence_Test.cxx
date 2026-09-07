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
#include <BRepTools.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <GeomConvert.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Line.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <NCollection_HArray1.hxx>
#include <TopOpeBRepBuild_Tools.hxx>
#include <TopoDS.hxx>
#include <IntTools_EdgeEdge.hxx>
#include <gtest/gtest.h>
#include <cmath>
#include <tuple>

namespace
{
occ::handle<Geom_Curve> makeCurve(int kind)
{
  if (kind == 0)
  {
    return new Geom_TrimmedCurve(new Geom_Line(gp_Pnt(), gp_Dir(1, 0, 0)), 0., 10.);
  }
  if (kind == 1)
  {
    return new Geom_Circle(gp_Ax2(gp_Pnt(), gp_Dir(0, 0, 1)), 5.);
  }
  if (kind == 2)
  {
    return new Geom_Ellipse(gp_Ax2(gp_Pnt(), gp_Dir(0, 0, 1)), 7., 4.);
  }
  if (kind == 3)
  {
    NCollection_Array1<gp_Pnt> poles(1, 4);
    poles(1) = gp_Pnt(0, 0, 0);
    poles(2) = gp_Pnt(2, 4, 0);
    poles(3) = gp_Pnt(8, -2, 0);
    poles(4) = gp_Pnt(10, 0, 0);
    return new Geom_BezierCurve(poles);
  }
  occ::handle<NCollection_HArray1<gp_Pnt>> points = new NCollection_HArray1<gp_Pnt>(1, 13);
  for (int i = 1; i <= 13; ++i)
  {
    const double t = 2 * M_PI * (i - 1) / 13.;
    points->SetValue(i, gp_Pnt(24 * cos(t), 30 * sin(t), 0));
  }
  GeomAPI_Interpolate interpolation(points, true, 1.e-9);
  interpolation.Perform();
  return interpolation.Curve();
}

TopoDS_Edge placedEdge(const TopoDS_Edge& edge, int placement)
{
  if (placement == 0)
  {
    return edge;
  }
  gp_Trsf transform;
  if (placement == 1)
  {
    transform.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 2, 3)), .731);
    transform.SetTranslationPart(gp_Vec(137, -51, 23));
  }
  else
  {
    transform.SetMirror(gp_Ax2(gp_Pnt(1, 2, 3), gp_Dir(2, 1, 3)));
  }
  // A location tests a different representation from modifying curve poles.
  TopoDS_Edge result = edge;
  if (placement == 1)
  {
    result.Move(TopLoc_Location(transform));
  }
  else
  {
    result = TopoDS::Edge(BRepBuilderAPI_Transform(edge, transform, true).Shape());
  }
  return result;
}
} // namespace

class TopOpeBRepBuild_Coincidence : public testing::TestWithParam<std::tuple<int, int, int, bool>>
{
};

TEST(TopOpeBRepBuild_CoincidenceRepresentation,
     AreCoincidentEdges_ClosedCurveAndContainedSplineArc_ReturnsFalse)
{
  for (int kind : {1, 2})
  {
    for (double fraction : {.1, .4, .8})
    {
      for (double start : {0., 1., 3.})
      {
        for (int placement : {0, 1, 2})
        {
          SCOPED_TRACE(testing::Message()
                       << kind << " " << fraction << " " << start << " " << placement);
          const auto curve  = makeCurve(kind);
          const auto spline = GeomConvert::CurveToBSplineCurve(
            new Geom_TrimmedCurve(curve, start, start + fraction * 2. * M_PI));
          const auto whole = placedEdge(BRepBuilderAPI_MakeEdge(curve), placement);
          const auto part  = placedEdge(BRepBuilderAPI_MakeEdge(spline), placement);
          ASSERT_TRUE(BRepCheck_Analyzer(whole).IsValid());
          ASSERT_TRUE(BRepCheck_Analyzer(part).IsValid());
          bool reversed = false;
          EXPECT_FALSE(TopOpeBRepBuild_Tools::AreCoincidentEdges(whole, part, reversed));
          EXPECT_FALSE(TopOpeBRepBuild_Tools::AreCoincidentEdges(part, whole, reversed));
        }
      }
    }
  }
}

TEST_P(TopOpeBRepBuild_Coincidence,
       AreCoincidentEdges_VariedRangesAndDirections_RespectsCoincidenceContract)
{
  const auto [kind, relation, placement, swapped] = GetParam();
  const auto       firstCurve                     = makeCurve(kind);
  const auto       secondCurve                    = occ::down_cast<Geom_Curve>(firstCurve->Copy());
  const double     first = firstCurve->FirstParameter(), last = firstCurve->LastParameter();
  constexpr double tolerance = 1.e-6;
  if (relation == 1)
  {
    secondCurve->Reverse();
  }
  // IntTools compares the sum of the two reached edge tolerances plus its
  // default fuzzy budget, not the tolerance of just one of the input edges.
  const double combinedTolerance = 2. * tolerance + IntTools_EdgeEdge().FuzzyValue();
  if (relation == 7 || relation == 8)
  {
    secondCurve->Translate(gp_Vec(0, 0, combinedTolerance * (relation == 7 ? .99 : 1.01)));
  }
  if (relation == 3 || relation == 4)
  {
    secondCurve->Translate(gp_Vec(0, 0, tolerance * (relation == 3 ? .5 : 4.)));
  }
  TopoDS_Edge a = BRepBuilderAPI_MakeEdge(firstCurve);
  TopoDS_Edge b = relation == 2 ? BRepBuilderAPI_MakeEdge(secondCurve,
                                                          first + .2 * (last - first),
                                                          first + .8 * (last - first))
                                    .Edge()
                                : BRepBuilderAPI_MakeEdge(secondCurve).Edge();
  if (relation == 5)
  {
    const gp_Pnt end = firstCurve->Value(last);
    b                = BRepBuilderAPI_MakeEdge(end, end.Translated(gp_Vec(0, 0, 10)));
  }
  if (relation == 6)
  {
    b.Reverse(); // Topological use orientation is not curve parameter direction.
  }
  BRep_Builder().UpdateEdge(a, tolerance);
  BRep_Builder().UpdateEdge(b, tolerance);
  a = placedEdge(a, placement);
  b = placedEdge(b, placement);
  ASSERT_TRUE(BRepCheck_Analyzer(a).IsValid());
  ASSERT_TRUE(BRepCheck_Analyzer(b).IsValid());
  // BRepTools::Compare only tests edge identity. These independently built
  // edges demonstrate why that facility cannot replace geometric coincidence.
  EXPECT_FALSE(BRepTools::Compare(a, b));
  bool       reversed   = false;
  const bool coincident = swapped ? TopOpeBRepBuild_Tools::AreCoincidentEdges(b, a, reversed)
                                  : TopOpeBRepBuild_Tools::AreCoincidentEdges(a, b, reversed);
  const bool expected =
    relation == 0 || relation == 1 || relation == 3 || relation == 6 || relation == 7;
  EXPECT_EQ(coincident, expected);
  if (coincident)
  {
    EXPECT_EQ(reversed, relation == 1);
  }
  // Classification must not repair inputs or enlarge their tolerances.
  EXPECT_DOUBLE_EQ(BRep_Tool::Tolerance(a), tolerance);
  EXPECT_DOUBLE_EQ(BRep_Tool::Tolerance(b), tolerance);
}

INSTANTIATE_TEST_SUITE_P(Curves,
                         TopOpeBRepBuild_Coincidence,
                         testing::Combine(testing::Range(0, 5),
                                          testing::Range(0, 9),
                                          testing::Range(0, 3),
                                          testing::Bool()));

TEST(TopOpeBRepBuild_CoincidenceRepresentation,
     AreCoincidentEdges_EquivalentSplineRepresentations_PreservesCoincidence)
{
  for (int kind = 0; kind < 5; ++kind)
  {
    for (bool reversed : {false, true})
    {
      SCOPED_TRACE(testing::Message() << "kind=" << kind << " reversed=" << reversed);
      const auto curve      = makeCurve(kind);
      const auto equivalent = GeomConvert::CurveToBSplineCurve(curve);
      equivalent->IncreaseDegree(7);
      equivalent->InsertKnot(equivalent->FirstParameter()
                               + .317
                                   * (equivalent->LastParameter() - equivalent->FirstParameter()),
                             1);
      if (equivalent->IsPeriodic())
      {
        equivalent->SetOrigin(4);
      }
      if (reversed)
      {
        equivalent->Reverse();
      }
      const TopoDS_Edge a = BRepBuilderAPI_MakeEdge(curve);
      const TopoDS_Edge b = BRepBuilderAPI_MakeEdge(equivalent);
      ASSERT_TRUE(BRepCheck_Analyzer(a).IsValid());
      ASSERT_TRUE(BRepCheck_Analyzer(b).IsValid());
      bool direction = false;
      EXPECT_TRUE(TopOpeBRepBuild_Tools::AreCoincidentEdges(a, b, direction));
      EXPECT_EQ(direction, reversed);
    }
  }
}

class TopOpeBRepBuild_StationaryCoincidence
    : public testing::TestWithParam<std::tuple<int, bool, bool, bool, int>>
{
};

TEST_P(TopOpeBRepBuild_StationaryCoincidence,
       AreCoincidentEdges_StationaryParameterization_PreservesDirection)
{
  const auto [aRepresentation, isCurveReversed, isFirstReversed, isSecondReversed, aPlacement] =
    GetParam();
  NCollection_Array1<gp_Pnt> aPoles(1, 4);
  aPoles(1) = gp_Pnt(0, 0, 0);
  aPoles(2) = gp_Pnt(10, 0, 0);
  aPoles(3) = gp_Pnt(0, 0, 0);
  aPoles(4) = gp_Pnt(10, 0, 0);
  // This curve is monotone, but its derivative vanishes exactly at the midpoint.
  const occ::handle<Geom_Curve> aStationary = new Geom_BezierCurve(aPoles);
  const occ::handle<Geom_Curve> aLine =
    new Geom_TrimmedCurve(new Geom_Line(gp_Pnt(), gp_Dir(1, 0, 0)), 0, 10);
  occ::handle<Geom_Curve> aFirst = aRepresentation == 1 ? aLine : aStationary;
  occ::handle<Geom_Curve> aSecond =
    occ::down_cast<Geom_Curve>((aRepresentation == 2 ? aLine : aStationary)->Copy());
  if (isCurveReversed)
  {
    aSecond->Reverse();
  }
  TopoDS_Edge aFirstEdge  = placedEdge(BRepBuilderAPI_MakeEdge(aFirst), aPlacement);
  TopoDS_Edge aSecondEdge = placedEdge(BRepBuilderAPI_MakeEdge(aSecond), aPlacement);
  if (isFirstReversed)
  {
    aFirstEdge.Reverse();
  }
  if (isSecondReversed)
  {
    aSecondEdge.Reverse();
  }
  ASSERT_TRUE(BRepCheck_Analyzer(aFirstEdge, true, false, true).IsValid());
  ASSERT_TRUE(BRepCheck_Analyzer(aSecondEdge, true, false, true).IsValid());
  ASSERT_FALSE(BRep_Tool::Degenerated(aFirstEdge));
  ASSERT_FALSE(BRep_Tool::Degenerated(aSecondEdge));
  bool isReversed = false;
  ASSERT_TRUE(TopOpeBRepBuild_Tools::AreCoincidentEdges(aFirstEdge, aSecondEdge, isReversed));
  EXPECT_EQ(isReversed, isCurveReversed);
}

INSTANTIATE_TEST_SUITE_P(StationaryParameters,
                         TopOpeBRepBuild_StationaryCoincidence,
                         testing::Combine(testing::Values(0, 1, 2),
                                          testing::Bool(),
                                          testing::Bool(),
                                          testing::Bool(),
                                          testing::Values(0, 1, 2)));
