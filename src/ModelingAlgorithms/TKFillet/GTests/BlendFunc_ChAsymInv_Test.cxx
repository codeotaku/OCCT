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

#include <BlendFunc_ChAsymInv.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Line.hxx>
#include <algorithm>
#include <cmath>
#include <Geom_Plane.hxx>
#include <Geom_Surface.hxx>
#include <Geom2d_Line.hxx>
#include <gp_Ax3.hxx>
#include <gp_Pln.hxx>
#include <gtest/gtest.h>
#include <tuple>

class BlendFunc_ChAsymInvJacobian
    : public testing::TestWithParam<std::tuple<bool, bool, int, double, int, bool>>
{
};

TEST_P(BlendFunc_ChAsymInvJacobian, Jacobian_AsymmetricChamfer_MatchesResidualFiniteDifferences)
{
  const auto [onFirst, curvedFirst, choice, scale, guideKind, rotated] = GetParam();
  const double radius                                                  = 10. * scale;

  occ::handle<Geom_Surface> cylinderSurface = new Geom_CylindricalSurface(gp_Ax3(), radius);
  occ::handle<Geom_Surface> planeSurface    = new Geom_Plane(gp_Pln(gp_Pnt(), gp_Dir(0, 0, 1)));
  occ::handle<Geom_Curve>   guideCurve;
  if (guideKind == 0)
    guideCurve = new Geom_Line(gp_Pnt(radius, 0., 0.), gp_Dir(0., 1., 0.));
  else if (guideKind == 1)
    guideCurve = new Geom_Circle(gp_Ax2(), radius);
  else
    guideCurve = new Geom_Ellipse(gp_Ax2(), 1.2 * radius, radius);
  if (rotated)
  {
    gp_Trsf transform;
    transform.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1., 2., 3.)), .7);
    cylinderSurface->Transform(transform);
    planeSurface->Transform(transform);
    guideCurve->Transform(transform);
  }
  occ::handle<GeomAdaptor_Surface> cylinder          = new GeomAdaptor_Surface(cylinderSurface);
  occ::handle<GeomAdaptor_Surface> plane             = new GeomAdaptor_Surface(planeSurface);
  occ::handle<GeomAdaptor_Curve>   guide             = new GeomAdaptor_Curve(guideCurve);
  const bool                       curvedRestriction = onFirst == curvedFirst;
  occ::handle<Geom2dAdaptor_Curve> restriction       = new Geom2dAdaptor_Curve(
    new Geom2d_Line(curvedRestriction ? gp_Pnt2d(.4, 0.) : gp_Pnt2d(8. * scale, 0.),
                    gp_Dir2d(0., 1.)));
  BlendFunc_ChAsymInv function(curvedFirst ? cylinder : plane,
                               curvedFirst ? plane : cylinder,
                               guide);
  function.Set(onFirst, restriction);
  function.Set(scale, M_PI / 3., choice);
  math_Vector x(1, 4), value(1, 4), plus(1, 4), minus(1, 4);
  x(1) = curvedRestriction ? -scale : 3. * scale;
  x(2) = .43;
  x(3) = curvedRestriction ? 8. * scale : .4;
  x(4) = curvedRestriction ? 3. * scale : -scale;
  math_Matrix jacobian(1, 4, 1, 4), separate(1, 4, 1, 4);
  ASSERT_TRUE(function.Values(x, value, jacobian));
  ASSERT_TRUE(function.Derivatives(x, separate));
  for (int column = 1; column <= 4; ++column)
  {
    const double step = 1.e-5 * std::max(1., std::abs(x(column)));
    math_Vector  sample(x);
    sample(column) += step;
    ASSERT_TRUE(function.Value(sample, plus));
    sample(column) -= 2. * step;
    ASSERT_TRUE(function.Value(sample, minus));
    for (int row = 1; row <= 4; ++row)
    {
      SCOPED_TRACE(testing::Message() << "row=" << row << " column=" << column);
      EXPECT_NEAR(jacobian(row, column), separate(row, column), 1.e-12);
      const double expected = (plus(row) - minus(row)) / (2. * step);
      EXPECT_NEAR(jacobian(row, column), expected, 2.e-7 * std::max(1., std::abs(expected)));
    }
  }
}

INSTANTIATE_TEST_SUITE_P(CurvedGuides,
                         BlendFunc_ChAsymInvJacobian,
                         testing::Combine(testing::Bool(),
                                          testing::Bool(),
                                          testing::Values(1, 2, 3, 4),
                                          testing::Values(.5, 1., 2.),
                                          testing::Values(0, 1, 2),
                                          testing::Bool()));
