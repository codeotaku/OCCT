// Copyright (c) 2026 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with the special exception defined in
// OCCT_LGPL_EXCEPTION.txt. See LICENSE_LGPL_21.txt for the complete license.

#include <BRepAdaptor_Curve.hxx>
#include <BRep_Builder.hxx>
#include <BRepAlgoAPI_Check.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <BRepTools_ReShape.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_Circle.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>
#include <string>
#include <tuple>

namespace
{
TopoDS_Shape readUserPrecursor()
{
  // Valid Pad077 exported after fully recomputing ChamferProblem.FCStd
  // from FreeCAD #16782. No geometry repair is applied.
  const std::string aFile = __FILE__;
  const std::string aPath =
    aFile.substr(0, aFile.find_last_of("/\\") + 1) + "data/ChamferTangentBoss.brep";
  TopoDS_Shape aShape;
  BRep_Builder aBuilder;
  BRepTools::Read(aShape, aPath.c_str(), aBuilder);
  return aShape;
}

// Analytic reconstruction of the valid precursor in FreeCAD #16782.
// Both rails are tangent to the boss. The dress-up termination intersects
// the INTERIOR of their vertical common edges, not their extensions.
TopoDS_Shape makeTangentBoss(double theTipRadius = 1., double theLength = 11.2)
{
  const double            aRadius = 4.;
  const double            aSin    = (aRadius - theTipRadius) / theLength;
  const double            aCos    = std::sqrt(1. - aSin * aSin);
  const double            anAngle = std::asin(aSin);
  const gp_Pnt            aLeft(-aRadius * aCos, -aRadius * aSin, 0.);
  const gp_Pnt            aRight(aRadius * aCos, -aRadius * aSin, 0.);
  const gp_Pnt            aTipLeft(-theTipRadius * aCos, -theLength - theTipRadius * aSin, 0.);
  const gp_Pnt            aTipRight(theTipRadius * aCos, -theLength - theTipRadius * aSin, 0.);
  BRepBuilderAPI_MakeWire aWire;
  aWire.Add(BRepBuilderAPI_MakeEdge(aLeft, aTipLeft));
  aWire.Add(BRepBuilderAPI_MakeEdge(
    new Geom_Circle(gp_Ax2(gp_Pnt(0, -theLength, 0), gp_Dir(0, 0, 1)), theTipRadius),
    M_PI + anAngle,
    2 * M_PI - anAngle));
  aWire.Add(BRepBuilderAPI_MakeEdge(aTipRight, aRight));
  aWire.Add(BRepBuilderAPI_MakeEdge(new Geom_Circle(gp_Ax2(gp_Pnt(), gp_Dir(0, 0, 1)), aRadius),
                                    -anAngle,
                                    M_PI + anAngle));
  const TopoDS_Shape aPrism =
    BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(aWire.Wire()), gp_Vec(0, 0, 2.8));
  const TopoDS_Shape aBoss =
    BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -1.2), gp_Dir(0, 0, 1)), aRadius, 4.);
  return BRepAlgoAPI_Fuse(aPrism, aBoss).Shape();
}

TopoDS_Edge bottomTip(const TopoDS_Shape& theShape, double theRadius = 1.)
{
  for (TopExp_Explorer it(theShape, TopAbs_EDGE); it.More(); it.Next())
  {
    BRepAdaptor_Curve aCurve(TopoDS::Edge(it.Current()));
    if (aCurve.GetType() == GeomAbs_Circle
        && std::abs(aCurve.Circle().Radius() - theRadius) < Precision::Confusion()
        && std::abs(aCurve.Circle().Location().Z()) < Precision::Confusion())
      return TopoDS::Edge(it.Current());
  }
  return TopoDS_Edge();
}

void checkResult(const TopoDS_Shape& theSource, const TopoDS_Shape& theResult)
{
  ASSERT_FALSE(theResult.IsNull());
  EXPECT_TRUE(BRepCheck_Analyzer(theResult, true, false, true).IsValid());
  EXPECT_TRUE(BRepAlgoAPI_Check(theResult).IsValid());
  int aSolids = 0, aShells = 0;
  for (TopExp_Explorer it(theResult, TopAbs_SOLID); it.More(); it.Next())
    ++aSolids;
  for (TopExp_Explorer it(theResult, TopAbs_SHELL); it.More(); it.Next())
  {
    ++aShells;
    EXPECT_TRUE(BRep_Tool::IsClosed(it.Current()));
  }
  EXPECT_EQ(aSolids, 1);
  EXPECT_EQ(aShells, 1);
  GProp_GProps aBefore, anAfter;
  BRepGProp::VolumeProperties(theSource, aBefore);
  BRepGProp::VolumeProperties(theResult, anAfter);
  EXPECT_GT(anAfter.Mass(), 0.);
  EXPECT_LT(anAfter.Mass(), aBefore.Mass());
}

void dump(const TopoDS_Shape& theShape, const std::string& theSuffix)
{
  if (const char* aDir = std::getenv("CHAMFER_BOSS_OUTPUT"))
  {
    std::string aName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    for (char& c : aName)
      if (c == '/')
        c = '_';
    BRepTools::Write(theShape, (std::string(aDir) + "/" + aName + theSuffix + ".brep").c_str());
  }
}
} // namespace

// Scale, chamfer distance, placement, and selected-edge orientation.
class BRepFilletAPI_TangentBoss
    : public ::testing::TestWithParam<std::tuple<double, double, int, bool>>
{
};

TEST_P(BRepFilletAPI_TangentBoss, Chamfer)
{
  const auto [aScale, aDistance, aPlacement, isReversed] = GetParam();
  TopoDS_Shape aSource                                   = readUserPrecursor();
  ASSERT_FALSE(aSource.IsNull());
  TopoDS_Edge anEdge = bottomTip(aSource);
  ASSERT_FALSE(anEdge.IsNull());
  gp_Trsf aTransform, aPosition;
  aTransform.SetScale(gp_Pnt(), aScale);
  if (aPlacement == 1)
  {
    aPosition.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 2, 3)), .71);
    aPosition.SetTranslationPart(gp_Vec(13, -7, 3));
  }
  else if (aPlacement == 2)
    aPosition.SetMirror(gp_Ax2(gp_Pnt(), gp_Dir(1, 0, 0)));
  aTransform = aPosition * aTransform;
  BRepBuilderAPI_Transform aMove(aSource, aTransform, true);
  aSource = aMove.Shape();
  anEdge  = TopoDS::Edge(aMove.ModifiedShape(anEdge));
  if (isReversed)
    anEdge.Reverse();
  ASSERT_TRUE(BRepCheck_Analyzer(aSource).IsValid());
  ASSERT_TRUE(BRepAlgoAPI_Check(aSource).IsValid());
  dump(aSource, "-input");
  BRepFilletAPI_MakeChamfer aChamfer(aSource);
  aChamfer.Add(aScale * aDistance, anEdge);
  aChamfer.Build();
  ASSERT_TRUE(aChamfer.IsDone());
  dump(aChamfer.Shape(), "-result");
  checkResult(aSource, aChamfer.Shape());
}

INSTANTIATE_TEST_SUITE_P(InteriorRestriction,
                         BRepFilletAPI_TangentBoss,
                         ::testing::Combine(::testing::Values(.1, 1., 10.),
                                            ::testing::Values(.2, .5, .9, .999, 1.),
                                            ::testing::Values(0, 1, 2),
                                            ::testing::Bool()));

TEST(BRepFilletAPI_TangentBossVariation, TaperAndTipRadius)
{
  for (const double aTipRadius : {.6, 1., 2.})
    for (const double aLength : {8., 16.})
    {
      SCOPED_TRACE(::testing::Message() << "tip=" << aTipRadius << " length=" << aLength);
      const TopoDS_Shape aSource = makeTangentBoss(aTipRadius, aLength);
      ASSERT_TRUE(BRepCheck_Analyzer(aSource).IsValid());
      ASSERT_TRUE(BRepAlgoAPI_Check(aSource).IsValid());
      BRepFilletAPI_MakeChamfer aChamfer(aSource);
      aChamfer.Add(aTipRadius, bottomTip(aSource, aTipRadius));
      aChamfer.Build();
      ASSERT_TRUE(aChamfer.IsDone());
      checkResult(aSource, aChamfer.Shape());
    }
}

TEST(BRepFilletAPI_TangentBossVariation, ChamferAtOneMillimeterTipLimit)
{
  const TopoDS_Shape aSource = readUserPrecursor();
  ASSERT_FALSE(aSource.IsNull());
  ASSERT_TRUE(BRepCheck_Analyzer(aSource).IsValid());
  ASSERT_TRUE(BRepAlgoAPI_Check(aSource).IsValid());
  BRepFilletAPI_MakeChamfer aChamfer(aSource);
  aChamfer.Add(1., bottomTip(aSource));
  aChamfer.Build();
  ASSERT_TRUE(aChamfer.IsDone());
  dump(aChamfer.Shape(), "-result");
  checkResult(aSource, aChamfer.Shape());
}

TEST(BRepFilletAPI_TangentBossVariation, OrdinaryBoxTerminationUnchanged)
{
  const TopoDS_Shape        aSource = BRepPrimAPI_MakeBox(4., 6., 8.);
  const TopoDS_Edge         anEdge  = TopoDS::Edge(TopExp_Explorer(aSource, TopAbs_EDGE).Current());
  BRepFilletAPI_MakeChamfer aChamfer(aSource);
  aChamfer.Add(.5, anEdge);
  aChamfer.Build();
  ASSERT_TRUE(aChamfer.IsDone());
  checkResult(aSource, aChamfer.Shape());
}

TEST(BRepFilletAPI_TangentBossVariation, CylinderApexIndependentOfPlaneUVOrigin)
{
  // Identical cylinders with different UV origins on the top support plane.
  // At the limit that face is consumed; its apex must be classified at the
  // actual UV point, not the default (0, 0).
  for (const double anOffset : {0., 7., -23.})
  {
    for (const double aHeightDistance : {.5, 1., 2.})
      for (const bool isPlaneFirst : {true, false})
      {
        SCOPED_TRACE(::testing::Message() << "UV origin=" << anOffset << " axial distance="
                                          << aHeightDistance << " plane first=" << isPlaneFirst);
        TopoDS_Shape aSource = BRepPrimAPI_MakeCylinder(1., 3.);
        TopoDS_Face  aTop, aCylinder;
        for (TopExp_Explorer it(aSource, TopAbs_FACE); it.More(); it.Next())
        {
          BRepAdaptor_Surface aSurface(TopoDS::Face(it.Current()));
          if (aSurface.GetType() == GeomAbs_Cylinder)
            aCylinder = TopoDS::Face(it.Current());
          if (aSurface.GetType() == GeomAbs_Plane
              && std::abs(aSurface.Plane().Location().Z() - 3.) < Precision::Confusion())
            aTop = TopoDS::Face(it.Current());
        }
        ASSERT_FALSE(aTop.IsNull());
        const gp_Pln      aPlane(gp_Pnt(anOffset, -2. * anOffset, 3.), gp_Dir(0, 0, 1));
        const TopoDS_Face aRebased =
          BRepBuilderAPI_MakeFace(aPlane, BRepTools::OuterWire(aTop), true);
        BRepTools_ReShape aReplace;
        aReplace.Replace(aTop, aRebased);
        aSource = aReplace.Apply(aSource);
        ASSERT_TRUE(BRepCheck_Analyzer(aSource).IsValid());
        ASSERT_TRUE(BRepAlgoAPI_Check(aSource).IsValid());
        const TopoDS_Edge anEdge = TopoDS::Edge(TopExp_Explorer(aRebased, TopAbs_EDGE).Current());
        BRepFilletAPI_MakeChamfer aChamfer(aSource);
        if (isPlaneFirst)
          aChamfer.Add(1., aHeightDistance, anEdge, aRebased);
        else
          aChamfer.Add(aHeightDistance, 1., anEdge, aCylinder);
        aChamfer.Build();
        ASSERT_TRUE(aChamfer.IsDone());
        dump(aChamfer.Shape(),
             "-" + std::to_string(anOffset) + "-" + std::to_string(aHeightDistance)
               + (isPlaneFirst ? "-plane" : "-cylinder"));
        checkResult(aSource, aChamfer.Shape());
        GProp_GProps aProperties;
        BRepGProp::VolumeProperties(aChamfer.Shape(), aProperties);
        EXPECT_NEAR(aProperties.Mass(), M_PI * (3. - 2. * aHeightDistance / 3.), 1.e-6);
      }
  }
}
