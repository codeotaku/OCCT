// Copyright (c) 2026 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.

#include <BOPAlgo_ArgumentAnalyzer.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Check.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <BRep_Builder.hxx>
#include <ChFi3d.hxx>
#include <GProp_GProps.hxx>
#include <Geom_BezierCurve.hxx>
#include <GeomConvert.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Circ.hxx>

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <tuple>

namespace
{
void checkChamfer(const TopoDS_Shape& theSource,
                  const TopoDS_Edge&  theEdge,
                  const double        theDistance,
                  const bool          theSwapSupport)
{
  ASSERT_TRUE(BRepCheck_Analyzer(theSource, true, false, true).IsValid());
  ASSERT_TRUE(BRepAlgoAPI_Check(theSource).IsValid());
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
    anAncestors;
  TopExp::MapShapesAndAncestors(theSource, TopAbs_EDGE, TopAbs_FACE, anAncestors);
  const auto& aFaces = anAncestors.FindFromKey(theEdge);
  ASSERT_EQ(aFaces.Extent(), 2);
  BRepFilletAPI_MakeChamfer aChamfer(theSource);
  aChamfer.Add(theDistance,
               theDistance,
               theEdge,
               TopoDS::Face(theSwapSupport ? aFaces.Last() : aFaces.First()));
  aChamfer.Build();
  ASSERT_TRUE(aChamfer.IsDone());
  const auto& aResult = aChamfer.Shape();
  ASSERT_TRUE(BRepCheck_Analyzer(aResult, true, false, true).IsValid());
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aSolids;
  TopExp::MapShapes(aResult, TopAbs_SOLID, aSolids);
  EXPECT_EQ(aSolids.Extent(), 1);
  for (TopExp_Explorer anIt(aResult, TopAbs_SHELL); anIt.More(); anIt.Next())
  {
    EXPECT_TRUE(BRep_Tool::IsClosed(anIt.Current()));
  }
  for (TopExp_Explorer anIt(aResult, TopAbs_VERTEX); anIt.More(); anIt.Next())
  {
    EXPECT_LT(BRep_Tool::Tolerance(TopoDS::Vertex(anIt.Current())), 0.01 * theDistance);
  }
  for (TopExp_Explorer anIt(aResult, TopAbs_EDGE); anIt.More(); anIt.Next())
  {
    EXPECT_LT(BRep_Tool::Tolerance(TopoDS::Edge(anIt.Current())), 0.01 * theDistance);
  }
  // BRep validity alone accepts the regressed C0 chamfer face and large tolerances.
  BOPAlgo_ArgumentAnalyzer aCheck;
  aCheck.SetShape1(BRepBuilderAPI_Copy(aResult).Shape());
  aCheck.ArgumentTypeMode()   = true;
  aCheck.SelfInterMode()      = true;
  aCheck.SmallEdgeMode()      = true;
  aCheck.RebuildFaceMode()    = true;
  aCheck.ContinuityMode()     = true;
  aCheck.CurveOnSurfaceMode() = true;
  aCheck.Perform();
  EXPECT_FALSE(aCheck.HasFaulty());
  GProp_GProps aBefore, anAfter;
  BRepGProp::VolumeProperties(theSource, aBefore);
  BRepGProp::VolumeProperties(aResult, anAfter);
  EXPECT_GT(anAfter.Mass(), 0.);
  EXPECT_LT(anAfter.Mass(), aBefore.Mass());
}

// A circular B-spline wall with a controllable, genuine crease at the closing
// vertex. The other three joins are tangent. Both signs of the crease are used.
TopoDS_Shape makeWall(const double theAngle)
{
  BRepBuilderAPI_MakeWire anInner;
  constexpr double        aRadius = 20.;
  const double            aHandle = aRadius * 4. / 3. * std::tan(M_PI / 8.);
  for (int anIndex = 0; anIndex < 4; ++anIndex)
  {
    const double               aFirst = anIndex * M_PI / 2.;
    const double               aLast  = aFirst + M_PI / 2.;
    NCollection_Array1<gp_Pnt> aPoles(1, 4);
    aPoles(1)             = gp_Pnt(aRadius * std::cos(aFirst), aRadius * std::sin(aFirst), 0.);
    aPoles(4)             = gp_Pnt(aRadius * std::cos(aLast), aRadius * std::sin(aLast), 0.);
    const double aTangent = anIndex == 0 ? theAngle : aFirst;
    aPoles(2) = aPoles(1).Translated(gp_Vec(-std::sin(aTangent), std::cos(aTangent), 0.) * aHandle);
    aPoles(3) = aPoles(4).Translated(gp_Vec(std::sin(aLast), -std::cos(aLast), 0.) * aHandle);
    anInner.Add(
      BRepBuilderAPI_MakeEdge(GeomConvert::CurveToBSplineCurve(new Geom_BezierCurve(aPoles))));
  }
  const auto anOuter =
    BRepBuilderAPI_MakeWire(
      BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(0., 0., 0.), gp_Dir(0., 0., 1.)), 30.)))
      .Wire();
  BRepBuilderAPI_MakeFace aFace(anOuter);
  aFace.Add(TopoDS::Wire(anInner.Wire().Reversed()));
  return BRepPrimAPI_MakePrism(aFace.Face(), gp_Vec(0., 0., 10.)).Shape();
}
} // namespace

class BRepFilletAPI_ChamferNearTangency : public testing::TestWithParam<std::tuple<double, bool>>
{
};

TEST_P(BRepFilletAPI_ChamferNearTangency, Build_BSplineCrease_PreservesContinuityAndTolerance)
{
  const auto [anAngle, aSwapSupport] = GetParam();
  const auto  aShape                 = makeWall(anAngle);
  TopoDS_Edge anEdge;
  for (TopExp_Explorer anIt(aShape, TopAbs_EDGE); anIt.More(); anIt.Next())
  {
    BRepAdaptor_Curve aCurve(TopoDS::Edge(anIt.Current()));
    if (aCurve.GetType() == GeomAbs_BSplineCurve
        && std::abs(aCurve.Value(aCurve.FirstParameter()).Z() - 10.) < Precision::Confusion())
    {
      anEdge = TopoDS::Edge(anIt.Current());
      break;
    }
  }
  ASSERT_FALSE(anEdge.IsNull());
  checkChamfer(aShape, anEdge, 1., aSwapSupport);
}

INSTANTIATE_TEST_SUITE_P(
  SeamAngles,
  BRepFilletAPI_ChamferNearTangency,
  testing::Combine(testing::Values(0., 1.e-5, .005, .02, .04, .08, -.005, -.04), testing::Bool()));

class BRepFilletAPI_ChamferReportedWall
    : public testing::TestWithParam<std::tuple<bool, double, int, bool>>
{
};

TEST_P(BRepFilletAPI_ChamferReportedWall, Build_FreeCAD17604_PreservesContinuityAndTolerance)
{
  const auto [isAlternate, aScale, aPlacement, aSwapSupport] = GetParam();
  // Unmodified, valid Pad inputs from both attachments to FreeCAD issue #17604.
  // https://github.com/FreeCAD/FreeCAD/issues/17604
  const std::string aFile = __FILE__;
  const std::string aPath =
    aFile.substr(0, aFile.find_last_of("/\\") + 1)
    + (isAlternate ? "data/bug17604_alternate.brep" : "data/bug17604_primary.brep");
  TopoDS_Shape aSource;
  BRep_Builder aBuilder;
  ASSERT_TRUE(BRepTools::Read(aSource, aPath.c_str(), aBuilder));
  gp_Trsf aTransform, aScaleTransform;
  aScaleTransform.SetScale(gp_Pnt(0., 0., 0.), aScale);
  if (aPlacement == 1)
  {
    aTransform.SetRotation(gp_Ax1(gp_Pnt(0., 0., 0.), gp_Dir(1., 2., 3.)), 0.7);
    aTransform.SetTranslationPart(gp_Vec(70., -40., 25.));
  }
  else if (aPlacement == 2)
  {
    aTransform.SetMirror(gp_Ax2(gp_Pnt(0., 0., 0.), gp_Dir(1., 0., 0.)));
  }
  aTransform.Multiply(aScaleTransform);
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> anEdges;
  TopExp::MapShapes(aSource, TopAbs_EDGE, anEdges);
  ASSERT_GE(anEdges.Extent(), 52);
  BRepBuilderAPI_Transform aMoved(aSource, aTransform, true);
  checkChamfer(aMoved.Shape(),
               TopoDS::Edge(aMoved.ModifiedShape(anEdges(52))),
               aScale,
               aSwapSupport);
}

INSTANTIATE_TEST_SUITE_P(ReportedWalls,
                         BRepFilletAPI_ChamferReportedWall,
                         testing::Combine(testing::Bool(),
                                          testing::Values(.5, 1., 2.),
                                          testing::Values(0, 1, 2),
                                          testing::Bool()));

class ChFi3d_NearTangency : public testing::TestWithParam<double>
{
};

TEST_P(ChFi3d_NearTangency, IsTangentFaces_UnencodedCrease_DoesNotRoundAngle)
{
  const double          anAngle = GetParam();
  const gp_Pnt          aStart(0., 0., 0.), anEnd(10., 0., 0.);
  BRepBuilderAPI_Sewing aSewing;
  aSewing.Add(BRepBuilderAPI_MakeFace(
    BRepBuilderAPI_MakePolygon(aStart, anEnd, gp_Pnt(10., 10., 0.), gp_Pnt(0., 10., 0.), true)
      .Wire()));
  const gp_Vec aSide(0., -10. * std::cos(anAngle), -10. * std::sin(anAngle));
  aSewing.Add(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakePolygon(anEnd,
                                                                 aStart,
                                                                 aStart.Translated(aSide),
                                                                 anEnd.Translated(aSide),
                                                                 true)
                                        .Wire()));
  aSewing.Perform();
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
    anAncestors;
  TopExp::MapShapesAndAncestors(aSewing.SewedShape(), TopAbs_EDGE, TopAbs_FACE, anAncestors);
  int aSharedCount = 0;
  for (int anIndex = 1; anIndex <= anAncestors.Extent(); ++anIndex)
  {
    const auto& aFaces = anAncestors(anIndex);
    if (aFaces.Extent() != 2)
      continue;
    ++aSharedCount;
    const auto anEdge = TopoDS::Edge(anAncestors.FindKey(anIndex));
    const auto aFace1 = TopoDS::Face(aFaces.First());
    const auto aFace2 = TopoDS::Face(aFaces.Last());
    BRep_Builder().Continuity(anEdge, aFace1, aFace2, GeomAbs_C0);
    EXPECT_EQ(ChFi3d::IsTangentFaces(anEdge, aFace1, aFace2), anAngle == 0.);
    EXPECT_EQ(ChFi3d::IsTangentFaces(anEdge, aFace2, aFace1), anAngle == 0.);
    BRep_Builder().Continuity(anEdge, aFace1, aFace2, GeomAbs_G1);
    EXPECT_TRUE(ChFi3d::IsTangentFaces(anEdge, aFace1, aFace2));
  }
  EXPECT_EQ(aSharedCount, 1);
}

INSTANTIATE_TEST_SUITE_P(CreaseAngles,
                         ChFi3d_NearTangency,
                         testing::Values(0., 1.e-6, 1.e-4, .001, .005, .02, .04, .08, -.04));
