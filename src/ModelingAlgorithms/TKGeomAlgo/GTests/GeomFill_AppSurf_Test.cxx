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

#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <GeomFill_AppSurf.hxx>
#include <GeomFill_Line.hxx>
#include <GeomFill_SectionGenerator.hxx>
#include <Precision.hxx>
#include <gtest/gtest.h>

namespace
{
occ::handle<Geom_BSplineCurve> profile(const double z)
{
  NCollection_Array1<gp_Pnt> p(1, 3);
  p(1) = gp_Pnt(0, 0, z);
  p(2) = gp_Pnt(1, 1, z);
  p(3) = gp_Pnt(2, 0, z);
  NCollection_Array1<double> k(1, 2), w(1, 3);
  k(1) = 0;
  k(2) = 1;
  w(1) = 1;
  w(2) = 0.8;
  w(3) = 1.1;
  NCollection_Array1<int> m(1, 2);
  m.Init(3);
  return new Geom_BSplineCurve(p, w, k, m, 2);
}
}

TEST(GeomFill_AppSurf_Test, AppSurfRefinesHomogeneousWeightsWithoutChangingGeometry)
{
  auto section = [](double t) {
    auto         c = profile(0);
    const double w = 1.0 - 3.2 * t * (1 - t);
    for (int i = 1; i <= 3; ++i)
    {
      const double wi = i == 2 ? w : (3 - w) / 2;
      c->SetPole(i, gp_Pnt(i - 1, i == 2 ? 1 / wi : 0, t / wi), wi);
    }
    return c;
  };
  GeomFill_SectionGenerator g;
  for (double t : {0.0, 0.5, 1.0})
    g.AddCurve(section(t));
  g.Perform(Precision::PConfusion());
  GeomFill_AppSurf app(2, 5, 1.e-7, 1.e-7, 0);
  app.SetParType(Approx_IsoParametric);
  app.Perform(new GeomFill_Line(3), g, true);
  ASSERT_TRUE(app.IsDone());
  EXPECT_GT(app.SurfVKnots().Length(), 2);
  Geom_BSplineSurface surface(app.SurfPoles(),
                              app.SurfWeights(),
                              app.SurfUKnots(),
                              app.SurfVKnots(),
                              app.SurfUMults(),
                              app.SurfVMults(),
                              app.UDegree(),
                              app.VDegree());
  for (double t : {0.0, 0.1, 0.25, 0.5, 0.7, 0.9, 1.0})
    for (double u : {0.0, 0.25, 0.5, 0.75, 1.0})
      EXPECT_LT(surface.Value(u, t).Distance(section(t)->Value(u)), 1.e-8);
  for (const double w : app.SurfWeights())
    EXPECT_GT(w, 0.0);
}

TEST(GeomFill_AppSurf_Test, RefinementRejectsWeightFunctionReachingZero)
{
  GeomFill_SectionGenerator g;
  for (double t : {0.0, 0.5, 1.0})
  {
    auto c = profile(0);
    const double w = 2 * (t - 0.25) * (t - 0.25);
    for (int i = 1; i <= 3; ++i)
    {
      const double wi = i == 2 ? w : (3 - w) / 2;
      c->SetPole(i, gp_Pnt(i - 1, i == 2 ? 1 / wi : 0, t / wi), wi);
    }
    g.AddCurve(c);
  }
  g.Perform(Precision::PConfusion());
  GeomFill_AppSurf app(2, 5, 1.e-7, 1.e-7, 0);
  app.SetParType(Approx_IsoParametric);
  app.Perform(new GeomFill_Line(3), g, true);
  EXPECT_FALSE(app.IsDone());
}
