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

#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2dInt_GInter.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2dConvert.hxx>
#include <GeomAPI.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <GeomConvert_ApproxCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_OffsetCurve.hxx>
#include <NCollection_HArray1.hxx>
#include <gp_Pln.hxx>
#include <gtest/gtest.h>
#include <tuple>
#include <cmath>

TEST(Geom2dInt_ClosedTangencyControl,
     Perform_CoincidentClosedCurves_PreservesSegmentsInBothDirections)
{
  occ::handle<Geom2d_Ellipse> ellipse =
    new Geom2d_Ellipse(gp_Ax2d(gp_Pnt2d(), gp_Dir2d(1, 0)), 30., 24.);
  const auto c1 = Geom2dConvert::CurveToBSplineCurve(ellipse);
  for (bool reversed : {false, true})
  {
    const auto c2 = occ::down_cast<Geom2d_Curve>(c1->Copy());
    if (reversed)
    {
      c2->Reverse();
    }
    Geom2dInt_GInter inter(Geom2dAdaptor_Curve(c1), Geom2dAdaptor_Curve(c2), 1.e-7, 1.e-7);
    ASSERT_TRUE(inter.IsDone());
    EXPECT_GT(inter.NbSegments(), 0);
  }
}

TEST(Geom2dInt_ClosedTangencyControl, Perform_CrossingAndTangentClosedCurves_PreservesContacts)
{
  occ::handle<Geom2d_Ellipse> ellipse =
    new Geom2d_Ellipse(gp_Ax2d(gp_Pnt2d(), gp_Dir2d(1, 0)), 30., 24.);
  const auto c1 = Geom2dConvert::CurveToBSplineCurve(ellipse);
  for (double shift : {10., 60.})
  {
    const auto c2 = occ::down_cast<Geom2d_Curve>(c1->Copy());
    c2->Translate(gp_Vec2d(shift, 0.));
    Geom2dInt_GInter inter(Geom2dAdaptor_Curve(c1), Geom2dAdaptor_Curve(c2), 1.e-7, 1.e-7);
    ASSERT_TRUE(inter.IsDone());
    // The seam tangency is reported at both parameters of the closed curve.
    EXPECT_EQ(inter.NbPoints(), 2);
    if (shift == 60.)
    {
      for (int i = 1; i <= inter.NbPoints(); ++i)
      {
        EXPECT_LE(inter.Point(i).Value().Distance(gp_Pnt2d(30., 0.)), 1.e-7);
      }
    }
  }
}

class Geom2dInt_ClosedTangency : public testing::TestWithParam<std::tuple<double, bool>>
{
};

TEST_P(Geom2dInt_ClosedTangency, Perform_SeparatedNormalOffsets_ProducesNoIntersections)
{
  const auto [gap, reversed]                      = GetParam();
  occ::handle<NCollection_HArray1<gp_Pnt>> points = new NCollection_HArray1<gp_Pnt>(1, 13);
  for (int i = 1; i <= 13; ++i)
  {
    points->SetValue(i,
                     gp_Pnt(24. * std::cos(2. * M_PI * (i - 1) / 13.),
                            30. * std::sin(2. * M_PI * (i - 1) / 13.),
                            0.));
  }
  GeomAPI_Interpolate interpolation(points, true, 1.e-9);
  interpolation.Perform();
  ASSERT_TRUE(interpolation.IsDone());
  const auto                    outer  = interpolation.Curve();
  occ::handle<Geom_OffsetCurve> offset = new Geom_OffsetCurve(outer, -gap, gp_Dir(0, 0, 1));
  GeomConvert_ApproxCurve       approximation(offset, 1.e-9, GeomAbs_C2, 1000, 14);
  ASSERT_TRUE(approximation.IsDone());
  const gp_Pln plane(gp_Pnt(), gp_Dir(0, 0, 1));
  const auto   c1 = GeomAPI::To2d(outer, plane);
  const auto   c2 = GeomAPI::To2d(approximation.Curve(), plane);
  if (reversed)
  {
    c2->Reverse();
  }
  for (double tolerance : {1.e-9, 1.e-7, 1.e-5})
  {
    SCOPED_TRACE(tolerance);
    Geom2dInt_GInter inter(Geom2dAdaptor_Curve(c1), Geom2dAdaptor_Curve(c2), tolerance, tolerance);
    ASSERT_TRUE(inter.IsDone());
    EXPECT_EQ(inter.NbPoints(), 0);
    EXPECT_EQ(inter.NbSegments(), 0);
  }
}

INSTANTIATE_TEST_SUITE_P(NormalOffsets,
                         Geom2dInt_ClosedTangency,
                         testing::Combine(testing::Values(.001, .01, .1), testing::Bool()));
