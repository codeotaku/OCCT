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

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepLib_CheckCurveOnSurface.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BSplCLib.hxx>
#include <GeomConvert.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_SurfaceOfLinearExtrusion.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom2d_Line.hxx>
#include <TopOpeBRepDS_BuildTool.hxx>
#include <TopOpeBRepDS_Curve.hxx>
#include <TopoDS.hxx>
#include <gp_Pln.hxx>
#include <gtest/gtest.h>
#include <cmath>
#include <sstream>
#include <tuple>

class TopOpeBRepDS_SharedPCurve : public testing::TestWithParam<std::tuple<int, int, bool, bool>>
{
};

TEST_P(TopOpeBRepDS_SharedPCurve, PCurve_LocatedSupports_PreservesSharedParameterAccuracy)
{
  const auto [kind, representation, located, serialized] = GetParam();
  occ::handle<Geom_Surface> surface;
  double                    first = 0., last = 10.;
  if (kind == 0)
  {
    surface = new Geom_Plane(gp_Pln(gp_Pnt(), gp_Dir(0, 0, 1)));
  }
  else if (kind == 1)
  {
    surface = new Geom_CylindricalSurface(gp_Ax3(), 5.);
    last    = 2 * M_PI;
  }
  else
  {
    NCollection_Array1<gp_Pnt> poles(1, 4);
    poles(1) = gp_Pnt(0, 0, 0);
    poles(2) = gp_Pnt(3, 4, 0);
    poles(3) = gp_Pnt(7, -2, 0);
    poles(4) = gp_Pnt(10, 0, 0);
    surface  = new Geom_SurfaceOfLinearExtrusion(new Geom_BezierCurve(poles), gp_Dir(0, 0, 1));
    last     = 1.;
  }
  TopoDS_Shape face = BRepBuilderAPI_MakeFace(surface, first, last, 0., 10., 1.e-7).Shape();
  const occ::handle<Geom_TrimmedCurve> original =
    new Geom_TrimmedCurve(surface->VIso(5.), first, last);
  const auto shared = GeomConvert::CurveToBSplineCurve(original);
  // On a cylinder the rational circle and analytic angle have a nonlinear
  // parameter correspondence even before any further range transformation.
  if (representation == 1)
  {
    NCollection_Array1<double> knots(shared->Knots());
    BSplCLib::Reparametrize(-3., 11., knots);
    shared->SetKnots(knots);
  }
  if (representation == 2)
  {
    shared->Reverse();
  }
  TopoDS_Shape edge = BRepBuilderAPI_MakeEdge(shared).Shape();
  if (located)
  {
    gp_Trsf transform;
    transform.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 2, 3)), .731);
    transform.SetTranslationPart(gp_Vec(13, -7, 23));
    const TopLoc_Location location(transform);
    face.Move(location);
    edge.Move(location);
  }
  if (serialized)
  {
    std::stringstream faceStream, edgeStream;
    BRepTools::Write(face, faceStream);
    BRepTools::Write(edge, edgeStream);
    BRep_Builder builder;
    BRepTools::Read(face, faceStream, builder);
    BRepTools::Read(edge, edgeStream, builder);
  }
  ASSERT_TRUE(BRepCheck_Analyzer(face).IsValid());
  ASSERT_TRUE(BRepCheck_Analyzer(edge).IsValid());
  TopOpeBRepDS_Curve definition(original, 1.e-6);
  definition.SetRange(first, last);
  definition.SetEquivalentCurve(1, representation == 2);
  const occ::handle<Geom2d_Line> trace = new Geom2d_Line(gp_Pnt2d(0, 5), gp_Dir2d(1, 0));
  TopOpeBRepDS_BuildTool         builder;
  builder.PCurve(face, edge, definition, trace);
  const auto resultEdge = TopoDS::Edge(edge);
  double     f, l;
  const auto pcurve = BRep_Tool::CurveOnSurface(resultEdge, TopoDS::Face(face), f, l);
  ASSERT_FALSE(pcurve.IsNull());
  EXPECT_NEAR(f, shared->FirstParameter(), 1.e-9);
  EXPECT_NEAR(l, shared->LastParameter(), 1.e-9);
  BRepLib_CheckCurveOnSurface check(resultEdge, TopoDS::Face(face));
  check.Perform();
  ASSERT_TRUE(check.IsDone());
  EXPECT_TRUE(std::isfinite(check.MaxDistance()));
  EXPECT_LE(check.MaxDistance(), 5.e-6);
  EXPECT_TRUE(std::isfinite(BRep_Tool::Tolerance(resultEdge)));
  EXPECT_LE(BRep_Tool::Tolerance(resultEdge), 5.e-6);
}

INSTANTIATE_TEST_SUITE_P(
  Supports,
  TopOpeBRepDS_SharedPCurve,
  testing::Combine(testing::Range(0, 3), testing::Range(0, 3), testing::Bool(), testing::Bool()));
