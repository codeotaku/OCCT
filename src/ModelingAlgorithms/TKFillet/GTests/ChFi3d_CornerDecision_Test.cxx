// Copyright (c) 2026 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <ChFi3d_CornerDecision.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Line.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <NCollection_Array1.hxx>
#include <Precision.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>

namespace
{
struct CurveCase
{
  std::string             Name;
  occ::handle<Geom_Curve> Curve;
  double                  First;
  double                  Last;
};

std::array<CurveCase, 5> makeCurveCases()
{
  NCollection_Array1<gp_Pnt> aBezierPoles(1, 4);
  aBezierPoles.SetValue(1, gp_Pnt(0.0, 0.0, 0.0));
  aBezierPoles.SetValue(2, gp_Pnt(3.0, 2.0, 0.0));
  aBezierPoles.SetValue(3, gp_Pnt(7.0, -2.0, 0.0));
  aBezierPoles.SetValue(4, gp_Pnt(10.0, 0.0, 0.0));
  const occ::handle<Geom_BezierCurve> aBezier = new Geom_BezierCurve(aBezierPoles);

  NCollection_Array1<gp_Pnt> aBSplinePoints(1, 6);
  aBSplinePoints.SetValue(1, gp_Pnt(0.0, 0.0, 0.0));
  aBSplinePoints.SetValue(2, gp_Pnt(2.0, 1.0, 0.0));
  aBSplinePoints.SetValue(3, gp_Pnt(4.0, -0.5, 0.0));
  aBSplinePoints.SetValue(4, gp_Pnt(6.0, 1.5, 0.0));
  aBSplinePoints.SetValue(5, gp_Pnt(8.0, -1.0, 0.0));
  aBSplinePoints.SetValue(6, gp_Pnt(10.0, 0.0, 0.0));
  const occ::handle<Geom_BSplineCurve> aBSpline = GeomAPI_PointsToBSpline(aBSplinePoints).Curve();

  const occ::handle<Geom_Curve> aLine =
    new Geom_TrimmedCurve(new Geom_Line(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(1.0, 0.0, 0.0)), 0.0, 10.0);
  const occ::handle<Geom_Curve> aCircle =
    new Geom_Circle(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), 10.0);
  const occ::handle<Geom_Curve> anEllipse =
    new Geom_Ellipse(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), 10.0, 5.0);

  return {{{"Line", aLine, 0.0, 10.0},
           {"Circle", aCircle, 0.0, M_PI_2},
           {"Ellipse", anEllipse, 0.0, M_PI_2},
           {"Bezier", aBezier, aBezier->FirstParameter(), aBezier->LastParameter()},
           {"BSpline", aBSpline, aBSpline->FirstParameter(), aBSpline->LastParameter()}}};
}
} // namespace

TEST(ChFi3d_CornerDecisionTest, ProjectedRecoilUsesNearestPointAcrossCurveFamilies)
{
  for (const CurveCase& aCase : makeCurveCases())
  {
    SCOPED_TRACE(aCase.Name);
    const TopoDS_Edge anEdge = BRepBuilderAPI_MakeEdge(aCase.Curve, aCase.First, aCase.Last);
    ASSERT_FALSE(anEdge.IsNull());
    const BRepAdaptor_Curve aCurve(anEdge);

    const double aProjectedParameter = aCase.First + 0.35 * (aCase.Last - aCase.First);
    const double aRecoilParameter    = aCase.First + 0.80 * (aCase.Last - aCase.First);
    const gp_Pnt anAdjacentPoint  = aCurve.Value(aProjectedParameter).Translated(gp_Vec(0, 0, 2));
    const gp_Pnt aCornerPoint     = aCurve.Value(aCase.First);
    double       aChosenParameter = 0.0;

    ASSERT_TRUE(ChFi3d_ChooseProjectedRecoil(aCurve,
                                             anAdjacentPoint,
                                             aCornerPoint,
                                             aRecoilParameter,
                                             aChosenParameter));
    EXPECT_NEAR(aChosenParameter,
                aProjectedParameter,
                1.0e-7 * std::max(1.0, aCase.Last - aCase.First));

    const TopoDS_Edge       aReversedEdge = TopoDS::Edge(anEdge.Reversed());
    const BRepAdaptor_Curve aReversedCurve(aReversedEdge);
    ASSERT_TRUE(ChFi3d_ChooseProjectedRecoil(aReversedCurve,
                                             anAdjacentPoint,
                                             aCornerPoint,
                                             aRecoilParameter,
                                             aChosenParameter));
    EXPECT_NEAR(aChosenParameter,
                aProjectedParameter,
                1.0e-7 * std::max(1.0, aCase.Last - aCase.First));
  }
}

TEST(ChFi3d_CornerDecisionTest, ProjectedRecoilRejectsNonLocalOrNonImprovingChoices)
{
  const TopoDS_Edge       anEdge = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0));
  const BRepAdaptor_Curve aCurve(anEdge);
  double                  aChosenParameter = -1.0;

  EXPECT_FALSE(
    ChFi3d_ChooseProjectedRecoil(aCurve, gp_Pnt(0, 2, 0), gp_Pnt(0, 0, 0), 8.0, aChosenParameter));
  EXPECT_DOUBLE_EQ(aChosenParameter, 8.0) << "a projection at the corner must not collapse";

  EXPECT_FALSE(
    ChFi3d_ChooseProjectedRecoil(aCurve, gp_Pnt(9, 2, 0), gp_Pnt(0, 0, 0), 3.0, aChosenParameter));
  EXPECT_DOUBLE_EQ(aChosenParameter, 3.0) << "a projection beyond the recoil must not expand it";

  EXPECT_FALSE(
    ChFi3d_ChooseProjectedRecoil(aCurve, gp_Pnt(3, 0, 0), gp_Pnt(0, 0, 0), 3.0, aChosenParameter));
  EXPECT_DOUBLE_EQ(aChosenParameter, 3.0) << "an equally accurate projection is not an improvement";
}

TEST(ChFi3d_CornerDecisionTest, ProjectedRecoilChoosesNearestPeriodicExtremum)
{
  const occ::handle<Geom_Circle> aCircle =
    new Geom_Circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 10.0);
  const TopoDS_Edge       anEdge = BRepBuilderAPI_MakeEdge(aCircle, 0.0, 2.0 * M_PI);
  const BRepAdaptor_Curve aCurve(anEdge);
  const double            aTargetParameter = M_PI / 4.0;
  const gp_Pnt anAdjacentPoint  = aCurve.Value(aTargetParameter).Translated(gp_Vec(0, 0, 2));
  double       aChosenParameter = 0.0;

  ASSERT_TRUE(ChFi3d_ChooseProjectedRecoil(aCurve,
                                           anAdjacentPoint,
                                           aCurve.Value(0.0),
                                           M_PI,
                                           aChosenParameter));
  EXPECT_NEAR(aChosenParameter, aTargetParameter, 1.0e-7)
    << "the nearest periodic extremum must win over the opposite-side extremum";
}

TEST(ChFi3d_CornerDecisionTest, ConnectorEndpointMatchingCoversOrderAndToleranceBoundary)
{
  constexpr double aTolerance = 1.0e-4;
  const gp_Pnt     aTargetFirst(1.0, 2.0, 3.0);
  const gp_Pnt     aTargetLast(4.0, 5.0, 6.0);

  EXPECT_TRUE(ChFi3d_AreConnectorEndpointsMatching(aTargetFirst,
                                                   aTargetLast,
                                                   aTargetFirst,
                                                   aTargetLast,
                                                   aTolerance));
  EXPECT_TRUE(ChFi3d_AreConnectorEndpointsMatching(aTargetLast,
                                                   aTargetFirst,
                                                   aTargetFirst,
                                                   aTargetLast,
                                                   aTolerance));
  EXPECT_TRUE(
    ChFi3d_AreConnectorEndpointsMatching(aTargetFirst.Translated(gp_Vec(aTolerance, 0, 0)),
                                         aTargetLast,
                                         aTargetFirst,
                                         aTargetLast,
                                         aTolerance));
  EXPECT_FALSE(
    ChFi3d_AreConnectorEndpointsMatching(aTargetFirst.Translated(gp_Vec(1.01 * aTolerance, 0, 0)),
                                         aTargetLast,
                                         aTargetFirst,
                                         aTargetLast,
                                         aTolerance));
  EXPECT_FALSE(ChFi3d_AreConnectorEndpointsMatching(gp_Pnt(20, 20, 20),
                                                    gp_Pnt(30, 30, 30),
                                                    aTargetFirst,
                                                    aTargetLast,
                                                    aTolerance));
}
