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

#include <Adaptor3d_CurveOnSurface.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <ChFi3d_Builder_0.hxx>
#include <ChFiDS_SurfData.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_BezierCurve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2dConvert.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <gp_Pln.hxx>
#include <gtest/gtest.h>
#include <tuple>

namespace
{
occ::handle<ChFiDS_SurfData> makePointContact(const gp_Pnt& thePoint, const double theTolerance)
{
  occ::handle<ChFiDS_SurfData> aResult = new ChFiDS_SurfData;
  aResult->ChangeInterference(1).SetInterference(0,
                                                 TopAbs_FORWARD,
                                                 {},
                                                 new Geom2d_Line(gp_Pnt2d(), gp_Dir2d(1, 0)));
  for (bool isFirst : {false, true})
  {
    aResult->ChangeVertex(isFirst, 1).SetPoint(thePoint);
    aResult->ChangeVertex(isFirst, 1).SetTolerance(theTolerance);
  }
  return aResult;
}
} // namespace

class ChFi3d_CollapsedPoints : public testing::TestWithParam<std::tuple<int, bool, bool, double>>
{
};

TEST_P(ChFi3d_CollapsedPoints, SymmetricContactWithoutRelaxingIndividualCollapse)
{
  const auto [aKind, isSwapped, isEndReversed, aScale] = GetParam();
  const double aSmallTolerance                         = 1.e-7 * aScale;
  const double aLargeTolerance                         = 1.e-4 * aScale;
  const double aSeparation                             = aKind == 1   ? .5 * aLargeTolerance
                                                         : aKind == 2 ? 2. * aLargeTolerance
                                                                      : 0.;
  auto         aFirst  = makePointContact(gp_Pnt(), aSmallTolerance);
  auto         aSecond = makePointContact(gp_Pnt(aSeparation, 0, 0), aLargeTolerance);
  if (aKind == 3)
  {
    // The other trace's larger tolerance must not disguise a noncollapsed contact.
    aFirst->ChangeVertex(isEndReversed, 1).SetPoint(gp_Pnt(.25 * aLargeTolerance, 0, 0));
  }
  if (isSwapped)
  {
    std::swap(aFirst, aSecond);
  }
  const TopoDS_Face aSupport =
    BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(), gp_Dir(0, 0, 1)), -1, 1, -1, 1);
  double aParameter1 = -1, aParameter2 = -1;
  EXPECT_EQ(ChFi3d_IntTraces(aFirst,
                             .2,
                             aParameter1,
                             1,
                             1,
                             aSecond,
                             .8,
                             aParameter2,
                             1,
                             1,
                             gp_Pnt2d(),
                             false,
                             false,
                             &aSupport),
            aKind < 2);
}

INSTANTIATE_TEST_SUITE_P(UnequalTolerances,
                         ChFi3d_CollapsedPoints,
                         testing::Combine(testing::Range(0, 4),
                                          testing::Bool(),
                                          testing::Bool(),
                                          testing::Values(1., 10., 100.)));

class ChFi3d_CollapsedTrace : public testing::TestWithParam<std::tuple<int, int, bool, bool>>
{
};

TEST_P(ChFi3d_CollapsedTrace, ProjectsWithinContactToleranceOnBoundedSupport)
{
  const auto [aSurfaceKind, aCurveKind, isReversed, isSwapped] = GetParam();
  for (double aScale : {.1, 1., 10.})
  {
    occ::handle<Geom_Surface> aSurface;
    const gp_Ax3              anAxes(gp_Pnt(11, -7, 13), gp_Dir(1, 2, 3));
    if (aSurfaceKind == 0)
      aSurface = new Geom_Plane(anAxes);
    if (aSurfaceKind == 1)
      aSurface = new Geom_CylindricalSurface(anAxes, 5 * aScale);
    if (aSurfaceKind == 2)
      aSurface = new Geom_SphericalSurface(anAxes, 5 * aScale);
    const TopoDS_Face         aSupport = BRepBuilderAPI_MakeFace(aSurface, -4, 4, -1.4, 1.4, 1.e-7);
    occ::handle<Geom2d_Curve> aCurve;
    if (aCurveKind == 0)
    {
      aCurve = new Geom2d_TrimmedCurve(new Geom2d_Line(gp_Pnt2d(.2, -.2), gp_Dir2d(1, .3)), 0, 1);
    }
    else if (aCurveKind == 1 || aCurveKind == 3)
    {
      const occ::handle<Geom2d_TrimmedCurve> anArc =
        new Geom2d_TrimmedCurve(new Geom2d_Circle(gp_Ax2d(gp_Pnt2d(1, 0), gp_Dir2d(1, 0)), .7),
                                .2,
                                5.9);
      aCurve = aCurveKind == 1
                 ? occ::handle<Geom2d_Curve>(anArc)
                 : occ::handle<Geom2d_Curve>(Geom2dConvert::CurveToBSplineCurve(anArc));
    }
    else
    {
      NCollection_Array1<gp_Pnt2d> aPoles(1, 4);
      aPoles(1) = gp_Pnt2d(0, 0);
      aPoles(2) = gp_Pnt2d(1.1, .8);
      aPoles(3) = gp_Pnt2d(-.4, -.9);
      aPoles(4) = gp_Pnt2d(2, .1);
      aCurve    = new Geom2d_BezierCurve(aPoles);
    }
    if (isReversed)
      aCurve->Reverse();
    const double                 aFirst = aCurve->FirstParameter(), aLast = aCurve->LastParameter();
    Adaptor3d_CurveOnSurface     aTrace(new Geom2dAdaptor_Curve(aCurve, aFirst, aLast),
                                    new BRepAdaptor_Surface(aSupport));
    occ::handle<ChFiDS_SurfData> aTraceData = new ChFiDS_SurfData;
    aTraceData->ChangeInterference(1).SetInterference(1, TopAbs_FORWARD, aCurve, aCurve);
    aTraceData->ChangeInterference(1).SetFirstParameter(aFirst);
    aTraceData->ChangeInterference(1).SetLastParameter(aLast);
    for (double aFraction : {0., 1.e-5, .37, .99999, 1.})
      for (double anOffset : {0., .5, 2.})
      {
        SCOPED_TRACE(testing::PrintToString(std::make_tuple(aScale, aFraction, anOffset)));
        const double   aTolerance = 1.e-6 * aScale;
        const gp_Pnt2d aUV        = aCurve->Value(aFirst + aFraction * (aLast - aFirst));
        gp_Pnt         aPoint;
        gp_Vec         aDU, aDV;
        aSurface->D1(aUV.X(), aUV.Y(), aPoint, aDU, aDV);
        aPoint.Translate(aDU.Crossed(aDV).Normalized() * (anOffset * aTolerance));
        const auto aCollapsed  = makePointContact(aPoint, aTolerance);
        double     aParameter1 = 0, aParameter2 = 0;
        const bool isContact = ChFi3d_IntTraces(isSwapped ? aTraceData : aCollapsed,
                                                0,
                                                aParameter1,
                                                1,
                                                1,
                                                isSwapped ? aCollapsed : aTraceData,
                                                0,
                                                aParameter2,
                                                1,
                                                1,
                                                gp_Pnt2d(),
                                                false,
                                                false,
                                                &aSupport);
        ASSERT_EQ(isContact, anOffset < 1.);
        if (isContact)
        {
          EXPECT_LE(aTrace.Value(isSwapped ? aParameter1 : aParameter2).Distance(aPoint),
                    aTolerance);
        }
      }
  }
}

INSTANTIATE_TEST_SUITE_P(
  AnalyticAndSplineTraces,
  ChFi3d_CollapsedTrace,
  testing::Combine(testing::Range(0, 3), testing::Range(0, 4), testing::Bool(), testing::Bool()));

TEST(ChFi3d_CollapsedTraceControl, RestrictionExtensionIsExplicit)
{
  const TopoDS_Face aSupport =
    BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(), gp_Dir(0, 0, 1)), -1, 2, -1, 1);
  auto aPoint                                        = makePointContact(gp_Pnt(1.02, 0, 0), 1.e-7);
  auto aTrace                                        = makePointContact(gp_Pnt(), 1.e-7);
  aTrace->ChangeInterference(1).ChangePCurveOnFace() = new Geom2d_Line(gp_Pnt2d(), gp_Dir2d(1, 0));
  aTrace->ChangeInterference(1).SetFirstParameter(0);
  aTrace->ChangeInterference(1).SetLastParameter(1);
  for (bool isEnlarged : {false, true})
  {
    double aParameter1 = 0, aParameter2 = 0;
    EXPECT_EQ(ChFi3d_IntTraces(aPoint,
                               0,
                               aParameter1,
                               1,
                               1,
                               aTrace,
                               0,
                               aParameter2,
                               1,
                               1,
                               gp_Pnt2d(),
                               false,
                               isEnlarged,
                               &aSupport),
              isEnlarged);
    if (isEnlarged)
      EXPECT_NEAR(aParameter2, 1.02, 1.e-7);
  }
}

TEST(ChFi3d_CollapsedTraceControl, MultipleNearestParametersRemainBoundedContacts)
{
  const TopoDS_Face aSupport =
    BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(), gp_Dir(0, 0, 1)), -2, 2, -2, 2);
  NCollection_Array1<gp_Pnt2d> aPoles(1, 4);
  NCollection_Array1<double>   aKnots(1, 4);
  NCollection_Array1<int>      aMults(1, 4);
  aPoles(1) = gp_Pnt2d(-1, -1);
  aPoles(2) = gp_Pnt2d(1, 1);
  aPoles(3) = gp_Pnt2d(1, -1);
  aPoles(4) = gp_Pnt2d(-1, 1);
  for (int i = 1; i <= 4; ++i)
  {
    aKnots(i) = i - 1.;
    aMults(i) = i == 1 || i == 4 ? 2 : 1;
  }
  const occ::handle<Geom2d_BSplineCurve> aCurve =
    new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 1);
  const auto aPoint                                  = makePointContact(gp_Pnt(0, 0, 5.e-8), 1.e-7);
  const auto aTrace                                  = makePointContact(gp_Pnt(), 1.e-7);
  aTrace->ChangeInterference(1).ChangePCurveOnFace() = aCurve;
  aTrace->ChangeInterference(1).SetFirstParameter(0.);
  aTrace->ChangeInterference(1).SetLastParameter(3.);
  for (bool isSwapped : {false, true})
  {
    double aParameter1 = 0., aParameter2 = 0.;
    ASSERT_TRUE(ChFi3d_IntTraces(isSwapped ? aTrace : aPoint,
                                 0.,
                                 aParameter1,
                                 1,
                                 1,
                                 isSwapped ? aPoint : aTrace,
                                 0.,
                                 aParameter2,
                                 1,
                                 1,
                                 gp_Pnt2d(),
                                 false,
                                 false,
                                 &aSupport));
    const double aParameter = isSwapped ? aParameter1 : aParameter2;
    EXPECT_GE(aParameter, 0.);
    EXPECT_LE(aParameter, 3.);
    EXPECT_LE(aCurve->Value(aParameter).Distance(gp_Pnt2d()), 1.e-7);
  }
}

TEST(ChFi3d_CollapsedTraceControl, MissingSupportAndTraceAreRejected)
{
  const auto aPoint      = makePointContact(gp_Pnt(), 1.e-7);
  double     aParameter1 = 0., aParameter2 = 0.;
  EXPECT_FALSE(ChFi3d_IntTraces(aPoint,
                                0.,
                                aParameter1,
                                1,
                                1,
                                aPoint,
                                0.,
                                aParameter2,
                                1,
                                1,
                                gp_Pnt2d(),
                                false,
                                false));
  const TopoDS_Face aSupport =
    BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(), gp_Dir(0, 0, 1)), -2, 2, -2, 2);
  aPoint->ChangeInterference(1).ChangePCurveOnSurf().Nullify();
  EXPECT_FALSE(ChFi3d_IntTraces(aPoint,
                                0.,
                                aParameter1,
                                1,
                                1,
                                aPoint,
                                0.,
                                aParameter2,
                                1,
                                1,
                                gp_Pnt2d(),
                                false,
                                false,
                                &aSupport));
}
