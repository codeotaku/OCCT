// Copyright (c) 2026 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with the special exception defined in
// OCCT_LGPL_EXCEPTION.txt. See LICENSE_LGPL_21.txt for the complete license.

#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Check.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
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
#include <BRep_Tool.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <ChFi3d_Builder_0.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2dInt_GInter.hxx>
#include <Geom2d_Line.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Trsf.hxx>
#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>
#include <string>
#include <tuple>

namespace
{
// Construct valid analytic sources; no imported defective upstream features.
TopoDS_Shape makeLimitSource(int theKind)
{
  if (theKind == 5)
    return BRepPrimAPI_MakeCylinder(10.0, 20.0).Shape();
  if (theKind == 4 || theKind == 7)
    return BRepPrimAPI_MakeBox(40.0, 20.0, theKind == 4 ? 10.0 : 20.0).Shape();
  if (theKind < 2)
    return BRepPrimAPI_MakeCylinder(10.0, theKind == 0 ? 10.0 : 4.0).Shape();
  if (theKind == 3)
    return BRepPrimAPI_MakeBox(40.0, 30.0, 10.0).Shape();
  BRepBuilderAPI_MakeWire aWire;
  aWire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(0, -10, 0), gp_Pnt(40, -10, 0)));
  aWire.Add(BRepBuilderAPI_MakeEdge(
    GC_MakeArcOfCircle(gp_Pnt(40, -10, 0), gp_Pnt(50, 0, 0), gp_Pnt(40, 10, 0)).Value()));
  aWire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(40, 10, 0), gp_Pnt(0, 10, 0)));
  aWire.Add(BRepBuilderAPI_MakeEdge(
    GC_MakeArcOfCircle(gp_Pnt(0, 10, 0), gp_Pnt(-10, 0, 0), gp_Pnt(0, -10, 0)).Value()));
  TopoDS_Shape aPrism =
    BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(aWire.Wire()), gp_Vec(0, 0, 10)).Shape();
  if (theKind == 8 || theKind == 9)
  {
    BRepAlgoAPI_Fuse             aFuse(aPrism, BRepPrimAPI_MakeCylinder(10.0, 20.0).Shape());
    ShapeUpgrade_UnifySameDomain aUnify(aFuse.Shape(), true, true, true);
    aUnify.Build();
    return aUnify.Shape();
  }
  return aPrism;
}

TopoDS_Edge findTopEdge(const TopoDS_Shape& theShape, int theKind)
{
  const double aHeight = theKind == 1 ? 4.0 : (theKind == 5 || theKind == 7 ? 20.0 : 10.0);
  for (TopExp_Explorer anIt(theShape, TopAbs_EDGE); anIt.More(); anIt.Next())
  {
    const TopoDS_Edge anEdge = TopoDS::Edge(anIt.Current());
    BRepAdaptor_Curve aCurve(anEdge);
    const gp_Pnt      aMid = aCurve.Value((aCurve.FirstParameter() + aCurve.LastParameter()) * 0.5);
    if (std::abs(aMid.Z() - aHeight) > 1.e-7)
      continue;
    if ((theKind < 2 || theKind == 5) && aCurve.GetType() == GeomAbs_Circle)
      return anEdge;
    if ((theKind == 2 || theKind == 6 || theKind == 8 || theKind == 9)
        && std::abs(aMid.Y() - 10.0) < 1.e-7 && aCurve.GetType() == GeomAbs_Line)
      return anEdge;
    if ((theKind == 3 || theKind == 4 || theKind == 7) && std::abs(aMid.Y()) < 1.e-7
        && aCurve.GetType() == GeomAbs_Line)
      return anEdge;
  }
  return TopoDS_Edge();
}

void expectClosedManifold(const TopoDS_Shape& theShape)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aSolids;
  TopExp::MapShapes(theShape, TopAbs_SOLID, aSolids);
  EXPECT_EQ(aSolids.Extent(), 1);
  EXPECT_TRUE(BRepCheck_Analyzer(theShape, true, false, true).IsValid());
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
    aMap;
  TopExp::MapShapesAndAncestors(theShape, TopAbs_EDGE, TopAbs_FACE, aMap);
  for (int i = 1; i <= aMap.Extent(); ++i)
  {
    const TopoDS_Edge anEdge = TopoDS::Edge(aMap.FindKey(i));
    if (!BRep_Tool::Degenerated(anEdge))
      EXPECT_EQ(aMap.FindFromIndex(i).Size(), 2) << "edge " << i;
  }
}

class ChamferGeometricLimit : public testing::TestWithParam<std::tuple<int, int, int, int, int>>
{
};
} // namespace

TEST_P(ChamferGeometricLimit, ConsumedBoundaryHasSharedTopology)
{
  const auto [aKind, aPlacement, aMode, aScaleIndex, aState] = GetParam();
  const double aScale  = aScaleIndex == 0 ? 1.0 : (aScaleIndex == 1 ? 0.1 : 10.0);
  TopoDS_Shape aSource = makeLimitSource(aKind);
  TopoDS_Edge  anEdge  = findTopEdge(aSource, aKind);
  ASSERT_FALSE(anEdge.IsNull());
  TopoDS_Edge aSecondEdge;
  if (aKind == 4 || aKind == 6 || aKind == 7 || aKind == 9)
  {
    for (TopExp_Explorer anIt(aSource, TopAbs_EDGE); anIt.More(); anIt.Next())
    {
      BRepAdaptor_Curve aCurve(TopoDS::Edge(anIt.Current()));
      const gp_Pnt aMid = aCurve.Value((aCurve.FirstParameter() + aCurve.LastParameter()) * 0.5);
      if (aCurve.GetType() == GeomAbs_Line && std::abs(aMid.X() - 20.0) < 1.e-7
          && std::abs(aMid.Z() - (aKind == 7 ? 20.0 : 10.0)) < 1.e-7
          && std::abs(aMid.Y() - (aKind == 6 || aKind == 9 ? -10.0 : 20.0)) < 1.e-7)
      {
        aSecondEdge = TopoDS::Edge(anIt.Current());
        break;
      }
    }
    ASSERT_FALSE(aSecondEdge.IsNull());
  }
  if (aScale != 1.0)
  {
    gp_Trsf aScaling;
    aScaling.SetScale(gp_Pnt(), aScale);
    BRepBuilderAPI_Transform aTransform(aSource, aScaling, true);
    anEdge = TopoDS::Edge(aTransform.ModifiedShape(anEdge));
    if (!aSecondEdge.IsNull())
      aSecondEdge = TopoDS::Edge(aTransform.ModifiedShape(aSecondEdge));
    aSource = aTransform.Shape();
  }
  if (aPlacement != 0)
  {
    gp_Trsf aTransform;
    if (aPlacement == 2)
      aTransform.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 2, 3)), 0.71);
    aTransform.SetTranslationPart(gp_Vec(17, -23, 31));
    const TopLoc_Location aLocation(aTransform);
    aSource.Move(aLocation);
    anEdge.Move(aLocation);
    if (!aSecondEdge.IsNull())
      aSecondEdge.Move(aLocation);
  }
  ASSERT_TRUE(BRepCheck_Analyzer(aSource, true, false, true).IsValid());
  const double              aDistance = ((aKind == 1 ? 4.0 : 10.0) + (aState - 1) * 0.01) * aScale;
  BRepFilletAPI_MakeChamfer aChamfer(aSource);
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
    aMap;
  TopExp::MapShapesAndAncestors(aSource, TopAbs_EDGE, TopAbs_FACE, aMap);
  const TopoDS_Face aFace = TopoDS::Face(aMap.FindFromKey(anEdge).First());
  if (aMode == 0)
    aChamfer.Add(aDistance, anEdge);
  else if (aMode == 1)
    aChamfer.Add(aDistance, aDistance, anEdge, aFace);
  else
    aChamfer.AddDA(aDistance, std::acos(-1.0) / 4.0, anEdge, aFace);
  if (!aSecondEdge.IsNull())
  {
    const TopoDS_Face aSecondFace = TopoDS::Face(aMap.FindFromKey(aSecondEdge).First());
    if (aMode == 0)
      aChamfer.Add(aDistance, aSecondEdge);
    else if (aMode == 1)
      aChamfer.Add(aDistance, aDistance, aSecondEdge, aSecondFace);
    else
      aChamfer.AddDA(aDistance, std::acos(-1.0) / 4.0, aSecondEdge, aSecondFace);
  }
  ASSERT_EQ(aChamfer.NbContours(), (aKind == 4 || aKind == 7) ? 2 : 1);
  std::string aStem;
  if (const char* aDirectory = std::getenv("CHAMFER_LIMIT_OUTPUT"))
  {
    aStem = std::string(aDirectory) + "/limit-" + std::to_string(aKind) + "-"
            + std::to_string(aPlacement) + "-" + std::to_string(aMode)
            + (aScaleIndex == 0 ? "" : "-scale" + std::to_string(aScaleIndex))
            + (aState == 1 ? "" : "-state" + std::to_string(aState));
    BRepTools::Write(aSource, (aStem + "-source.brep").c_str());
  }
  ASSERT_NO_THROW(aChamfer.Build());
  EXPECT_TRUE(BRepCheck_Analyzer(aSource, true, false, true).IsValid())
    << "Building a chamfer must not invalidate its source";
  if (aState == 2)
  {
    EXPECT_FALSE(aChamfer.IsDone()) << "Over-limit operation must not return a false success";
    return;
  }
  if (!aStem.empty() && aChamfer.IsDone())
    BRepTools::Write(aChamfer.Shape(), (aStem + "-result.brep").c_str());
  ASSERT_TRUE(aChamfer.IsDone());
  EXPECT_FALSE(aChamfer.Generated(anEdge).IsEmpty());
  expectClosedManifold(aChamfer.Shape());
  GProp_GProps aBefore, aAfter, aSourceArea;
  BRepGProp::VolumeProperties(aSource, aBefore);
  BRepGProp::VolumeProperties(aChamfer.Shape(), aAfter);
  BRepGProp::SurfaceProperties(aSource, aSourceArea);
  EXPECT_GT(aAfter.Mass(), 0.0);
  EXPECT_LT(aAfter.Mass(), aBefore.Mass());
  const double aPi                 = std::acos(-1.0);
  const double anExpectedVolumes[] = {1000 * aPi / 3,
                                      784 * aPi / 3,
                                      4000 + 1000 * aPi / 3,
                                      10000,
                                      4000,
                                      4000 * aPi / 3,
                                      4000 + 1000 * aPi / 3,
                                      12000,
                                      (14000 + 5000 * aPi) / 3,
                                      (14000 + 5000 * aPi) / 3};
  if (aState == 1)
    EXPECT_NEAR(
      aAfter.Mass(),
      anExpectedVolumes[aKind] * aScale * aScale * aScale,
      std::max(1.e-6 * aScale * aScale * aScale, Precision::Confusion() * aSourceArea.Mass()));
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aResultMap;
  TopExp::MapShapes(aChamfer.Shape(), aResultMap);
  for (const TopoDS_Shape& aGenerated : aChamfer.Generated(anEdge))
    EXPECT_TRUE(aResultMap.Contains(aGenerated));
  for (TopExp_Explorer anIt(aChamfer.Shape(), TopAbs_FACE); anIt.More(); anIt.Next())
  {
    GProp_GProps anArea;
    BRepGProp::SurfaceProperties(anIt.Current(), anArea);
    EXPECT_GT(anArea.Mass(), 1.e-9 * aScale * aScale) << "Consumed face was retained";
  }
}

INSTANTIATE_TEST_SUITE_P(AnalyticLimits,
                         ChamferGeometricLimit,
                         testing::Combine(testing::Range(0, 10),
                                          testing::Range(0, 3),
                                          testing::Range(0, 3),
                                          testing::Range(0, 3),
                                          testing::Range(0, 3)));

// Distinguish an allowed angular endpoint contact from a true crossing.
TEST(ChamferLimitIntersection, EndpointContactIsNotInteriorCrossing)
{
  Geom2dAdaptor_Curve aFirst(new Geom2d_Line(gp_Pnt2d(), gp_Dir2d(1, 0)), 0.0, 10.0);
  for (double anX : {0.0, 5.0, 10.0})
  {
    for (double aStart : {-1.0, 0.0})
    {
      Geom2dAdaptor_Curve aSecond(new Geom2d_Line(gp_Pnt2d(anX, 0), gp_Dir2d(0, 1)), aStart, 1.0);
      Geom2dInt_GInter    anIntersector(aFirst, aSecond, 1.e-9, 1.e-9);
      ASSERT_EQ(anIntersector.NbPoints(), 1);
      EXPECT_EQ(ChFi3d_HasTransversalIntersection(anIntersector), anX == 5.0 || aStart < 0.0);
    }
  }
}

class ChamferLimitNeighborhood : public testing::TestWithParam<std::tuple<int, int, bool>>
{
};

TEST_P(ChamferLimitNeighborhood, NearExactAndOverLimit)
{
  const auto [aKind, aState, isReversed] = GetParam();
  TopoDS_Shape aSource                   = makeLimitSource(aKind);
  ASSERT_TRUE(BRepCheck_Analyzer(aSource, true, false, true).IsValid());
  TopoDS_Edge anEdge = findTopEdge(aSource, aKind);
  ASSERT_FALSE(anEdge.IsNull());
  if (isReversed)
    anEdge.Reverse();
  const double              aDistance = 10.0 + (aState - 1) * 0.01;
  BRepFilletAPI_MakeChamfer aChamfer(aSource);
  aChamfer.Add(aDistance, anEdge);
  if (aKind == 4 || aKind == 7)
  {
    for (TopExp_Explorer anIt(aSource, TopAbs_EDGE); anIt.More(); anIt.Next())
    {
      BRepAdaptor_Curve aCurve(TopoDS::Edge(anIt.Current()));
      const gp_Pnt aMid = aCurve.Value((aCurve.FirstParameter() + aCurve.LastParameter()) * 0.5);
      if (aCurve.GetType() == GeomAbs_Line && std::abs(aMid.X() - 20) < 1.e-7
          && std::abs(aMid.Y() - 20) < 1.e-7 && std::abs(aMid.Z() - (aKind == 4 ? 10 : 20)) < 1.e-7)
      {
        aChamfer.Add(aDistance, TopoDS::Edge(anIt.Current()));
        break;
      }
    }
  }
  ASSERT_NO_THROW(aChamfer.Build());
  if (aState == 2)
  {
    EXPECT_FALSE(aChamfer.IsDone());
    return;
  }
  ASSERT_TRUE(aChamfer.IsDone());
  expectClosedManifold(aChamfer.Shape());
  GProp_GProps aBefore, aAfter;
  BRepGProp::VolumeProperties(aSource, aBefore);
  BRepGProp::VolumeProperties(aChamfer.Shape(), aAfter);
  EXPECT_GT(aAfter.Mass(), 0.0);
  EXPECT_LT(aAfter.Mass(), aBefore.Mass());
  if (const char* aDirectory = std::getenv("CHAMFER_LIMIT_OUTPUT"))
  {
    const std::string aStem = std::string(aDirectory) + "/neighborhood-" + std::to_string(aKind)
                              + "-" + std::to_string(aState) + "-" + std::to_string(isReversed);
    BRepTools::Write(aChamfer.Shape(), (aStem + "-result.brep").c_str());
  }
}

INSTANTIATE_TEST_SUITE_P(Boundary,
                         ChamferLimitNeighborhood,
                         testing::Combine(testing::Values(0, 2, 4, 5, 7, 8),
                                          testing::Range(0, 3),
                                          testing::Bool()));

class ChamferSequentialLimit : public testing::TestWithParam<std::tuple<int, int, int, bool, bool>>
{
};

TEST_P(ChamferSequentialLimit, ConsumeLowerContourAfterUpperChamfer)
{
  const auto [aKind, aState, aScaleIndex, isLocated, isReversed] = GetParam();
  TopoDS_Shape              aSource = makeLimitSource(aKind == 0 ? 2 : 8);
  BRepFilletAPI_MakeChamfer anUpper(aSource);
  anUpper.Add(10.0, findTopEdge(aSource, aKind == 0 ? 2 : 8));
  anUpper.Build();
  ASSERT_TRUE(anUpper.IsDone());
  ShapeUpgrade_UnifySameDomain anUpperUnify(anUpper.Shape(), true, true, true);
  anUpperUnify.Build();
  aSource = anUpperUnify.Shape();
  if (aKind == 2)
  {
    TopoDS_Edge aTop;
    for (TopExp_Explorer anIt(aSource, TopAbs_EDGE); anIt.More(); anIt.Next())
    {
      BRepAdaptor_Curve aCurve(TopoDS::Edge(anIt.Current()));
      if (aCurve.GetType() == GeomAbs_Circle
          && std::abs(aCurve.Value(aCurve.FirstParameter()).Z() - 20.0) < 1.e-7)
      {
        aTop = TopoDS::Edge(anIt.Current());
        break;
      }
    }
    ASSERT_FALSE(aTop.IsNull());
    BRepFilletAPI_MakeChamfer aCap(aSource);
    aCap.Add(10.0, aTop);
    aCap.Build();
    ASSERT_TRUE(aCap.IsDone());
    ShapeUpgrade_UnifySameDomain aCapUnify(aCap.Shape(), true, true, true);
    aCapUnify.Build();
    aSource = aCapUnify.Shape();
  }
  ASSERT_TRUE(BRepCheck_Analyzer(aSource, true, false, true).IsValid());
  TopoDS_Edge aRail, anArc;
  for (TopExp_Explorer anIt(aSource, TopAbs_EDGE); anIt.More(); anIt.Next())
  {
    BRepAdaptor_Curve aCurve(TopoDS::Edge(anIt.Current()));
    const gp_Pnt      aMid = aCurve.Value((aCurve.FirstParameter() + aCurve.LastParameter()) * 0.5);
    if (std::abs(aMid.Z()) > 1.e-7)
      continue;
    if (aCurve.GetType() == GeomAbs_Line && std::abs(aMid.Y() - 10.0) < 1.e-7)
      aRail = TopoDS::Edge(anIt.Current());
    if (aCurve.GetType() == GeomAbs_Circle && aMid.X() < -1.0)
      anArc = TopoDS::Edge(anIt.Current());
  }
  ASSERT_FALSE(aRail.IsNull());
  ASSERT_FALSE(anArc.IsNull());
  const double aScale = aScaleIndex == 0 ? 1.0 : (aScaleIndex == 1 ? 0.1 : 10.0);
  gp_Trsf      aScaleTransform;
  aScaleTransform.SetScale(gp_Pnt(), aScale);
  BRepBuilderAPI_Transform aScaled(aSource, aScaleTransform, true);
  aRail   = TopoDS::Edge(aScaled.ModifiedShape(aRail));
  anArc   = TopoDS::Edge(aScaled.ModifiedShape(anArc));
  aSource = aScaled.Shape();
  if (isLocated)
  {
    gp_Trsf aTransform;
    aTransform.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 2, 3)), 0.71);
    aTransform.SetTranslationPart(gp_Vec(17, -23, 31));
    TopLoc_Location aLocation(aTransform);
    aSource.Move(aLocation);
    aRail.Move(aLocation);
    anArc.Move(aLocation);
  }
  const double aDistance = (10.0 + (aState - 1) * 0.01) * aScale;
  ASSERT_TRUE(BRepCheck_Analyzer(aSource, true, false, true).IsValid());
  ASSERT_TRUE(BRepAlgoAPI_Check(aSource).IsValid());
  BRepFilletAPI_MakeChamfer aLower(aSource);
  aLower.Add(aDistance, isReversed ? anArc : aRail);
  aLower.Add(aDistance, isReversed ? aRail : anArc);
  std::string aStem;
  if (const char* aDirectory = std::getenv("CHAMFER_LIMIT_OUTPUT"))
  {
    aStem = std::string(aDirectory) + "/sequential-" + std::to_string(aKind) + "-"
            + std::to_string(aState) + "-" + std::to_string(aScaleIndex) + "-"
            + std::to_string(isLocated) + "-" + std::to_string(isReversed);
    BRepTools::Write(aSource, (aStem + "-source.brep").c_str());
  }
  ASSERT_NO_THROW(aLower.Build());
  if (!aStem.empty() && aLower.IsDone())
    BRepTools::Write(aLower.Shape(), (aStem + "-result.brep").c_str());
  EXPECT_TRUE(BRepCheck_Analyzer(aSource, true, false, true).IsValid());
  if (aState == 2 && aKind != 0)
  {
    EXPECT_FALSE(aLower.IsDone());
    return;
  }
  // The plain arm has no remaining cylindrical side: consuming its bottom
  // face need not be the final admissible distance. Any accepted continuation
  // must nevertheless remain a valid material-removing solid.
  if (aState == 2 && !aLower.IsDone())
    return;
  ASSERT_TRUE(aLower.IsDone());
  expectClosedManifold(aLower.Shape());
  EXPECT_TRUE(BRepAlgoAPI_Check(aLower.Shape()).IsValid());
  for (TopExp_Explorer anIt(aLower.Shape(), TopAbs_VERTEX); anIt.More(); anIt.Next())
    EXPECT_LT(BRep_Tool::Tolerance(TopoDS::Vertex(anIt.Current())), 1.e-3 * aScale);
  for (TopExp_Explorer anIt(aLower.Shape(), TopAbs_EDGE); anIt.More(); anIt.Next())
    EXPECT_LT(BRep_Tool::Tolerance(TopoDS::Edge(anIt.Current())), 1.e-3 * aScale);
  GProp_GProps aBefore, anAfter;
  BRepGProp::VolumeProperties(aSource, aBefore);
  BRepGProp::VolumeProperties(aLower.Shape(), anAfter);
  EXPECT_GT(anAfter.Mass(), 0.0);
  EXPECT_LT(anAfter.Mass(), aBefore.Mass());
}

INSTANTIATE_TEST_SUITE_P(Sequential,
                         ChamferSequentialLimit,
                         testing::Combine(testing::Range(0, 3),
                                          testing::Range(0, 3),
                                          testing::Range(0, 3),
                                          testing::Bool(),
                                          testing::Bool()));
