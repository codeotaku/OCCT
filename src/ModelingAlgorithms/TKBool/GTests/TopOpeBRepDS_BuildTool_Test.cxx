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

#include <gtest/gtest.h>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Line.hxx>
#include <Geom2d_Line.hxx>
#include <Geom_Surface.hxx>
#include <gp_Pln.hxx>
#include <Precision.hxx>
#include <TopOpeBRepDS_BuildTool.hxx>
#include <TopOpeBRepDS_Curve.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp_Explorer.hxx>

TEST(TopOpeBRepDS_BuildToolTest, CopyReversedPeriodicEdgePreservesRange)
{
  const occ::handle<Geom_Circle> aCircle = new Geom_Circle(gp_Ax2(), 1.0);
  TopoDS_Edge                    aSource = BRepBuilderAPI_MakeEdge(aCircle);
  aSource.Reverse();

  TopOpeBRepDS_BuildTool aBuildTool;
  TopoDS_Shape           aCopy;
  aBuildTool.CopyEdge(aSource, aCopy);
  for (TopExp_Explorer aVertexIt(aSource, TopAbs_VERTEX); aVertexIt.More(); aVertexIt.Next())
  {
    aBuildTool.AddEdgeVertex(aSource, aCopy, aVertexIt.Current());
  }

  double aFirst, aLast;
  BRep_Tool::Range(TopoDS::Edge(aCopy), aFirst, aLast);
  EXPECT_NEAR(aFirst, aCircle->FirstParameter(), Precision::PConfusion());
  EXPECT_NEAR(aLast, aCircle->LastParameter(), Precision::PConfusion());
}

TEST(TopOpeBRepDS_BuildToolTest, SharedCurveKeepsPCurveInSharedEdgeParameters)
{
  for (int aCase = 0; aCase < 3; ++aCase)
  {
    SCOPED_TRACE(aCase);
    TopoDS_Shape aFace =
      BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(), gp_Dir(0, 0, 1)), -30., 30., -10., 10.).Shape();
    const occ::handle<Geom_Line> aSharedCurve = new Geom_Line(gp_Pnt(10, 0, 0), gp_Dir(1, 0, 0));
    const bool                   isPartial    = aCase == 2;
    TopoDS_Shape                 anEdge =
      BRepBuilderAPI_MakeEdge(aSharedCurve, isPartial ? 2. : 0., isPartial ? 8. : 10.).Shape();
    const bool                   isReversed = aCase == 1;
    const occ::handle<Geom_Line> anOriginal =
      new Geom_Line(gp_Pnt(isReversed ? 20 : 0, 0, 0), gp_Dir(isReversed ? -1 : 1, 0, 0));
    TopOpeBRepDS_Curve aCurve(anOriginal, Precision::Confusion());
    aCurve.SetRange(isReversed ? 0. : 10., isReversed ? 10. : 20.);
    if (!isPartial)
    {
      aCurve.SetEquivalentCurve(1, isReversed);
    }
    const occ::handle<Geom2d_Line> aPCurve =
      new Geom2d_Line(gp_Pnt2d(isReversed ? 20 : 0, 0), gp_Dir2d(isReversed ? -1 : 1, 0));
    TopOpeBRepDS_BuildTool aBuildTool;
    aBuildTool.PCurve(aFace, anEdge, aCurve, aPCurve);
    double     aFirst, aLast;
    const auto aResultPCurve =
      BRep_Tool::CurveOnSurface(TopoDS::Edge(anEdge), TopoDS::Face(aFace), aFirst, aLast);
    ASSERT_FALSE(aResultPCurve.IsNull());
    const auto aSurface = BRep_Tool::Surface(TopoDS::Face(aFace));
    for (int i = 0; i <= 10; ++i)
    {
      const double   aParameter = aFirst + (aLast - aFirst) * i / 10.;
      const gp_Pnt2d aUV        = aResultPCurve->Value(aParameter);
      EXPECT_LE(aSurface->Value(aUV.X(), aUV.Y()).Distance(aSharedCurve->Value(aParameter)),
                Precision::Confusion());
    }
  }
}
