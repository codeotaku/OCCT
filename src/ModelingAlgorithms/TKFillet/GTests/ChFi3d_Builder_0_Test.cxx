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

#include <algorithm>
#include <cmath>

#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <ChFi3d_Builder_0.hxx>
#include <ChFi3d_FilletShape.hxx>
#include <ChFiDS_ChamfSpine.hxx>
#include <ChFiDS_ElSpine.hxx>
#include <ChFiDS_Spine.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GeomAbs_Shape.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_LinearVector.hxx>
#include <Precision.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Elips.hxx>
#include <gp_Pnt.hxx>

#include <gtest/gtest.h>

namespace
{

TopoDS_Wire makeRectangle(const double theWidth, const double theDepth, const double theZ)
{
  BRepBuilderAPI_MakePolygon aPolygon;
  aPolygon.Add(gp_Pnt(-theWidth / 2.0, -theDepth / 2.0, theZ));
  aPolygon.Add(gp_Pnt(theWidth / 2.0, -theDepth / 2.0, theZ));
  aPolygon.Add(gp_Pnt(theWidth / 2.0, theDepth / 2.0, theZ));
  aPolygon.Add(gp_Pnt(-theWidth / 2.0, theDepth / 2.0, theZ));
  aPolygon.Close();
  return aPolygon.Wire();
}

void edgeZRange(const TopoDS_Edge& theEdge, double& theMinZ, double& theMaxZ)
{
  TopoDS_Vertex aFirstVertex;
  TopoDS_Vertex aLastVertex;
  TopExp::Vertices(theEdge, aFirstVertex, aLastVertex);
  const double aFirstZ = BRep_Tool::Pnt(aFirstVertex).Z();
  const double aLastZ  = BRep_Tool::Pnt(aLastVertex).Z();
  theMinZ              = std::min(aFirstZ, aLastZ);
  theMaxZ              = std::max(aFirstZ, aLastZ);
}

bool spansHeight(const TopoDS_Edge& theEdge, const double theHeight)
{
  double aMinZ = 0.0;
  double aMaxZ = 0.0;
  edgeZRange(theEdge, aMinZ, aMaxZ);
  return aMinZ < Precision::Confusion() && aMaxZ > theHeight - Precision::Confusion();
}

bool liesOnBottom(const TopoDS_Edge& theEdge)
{
  double aMinZ = 0.0;
  double aMaxZ = 0.0;
  edgeZRange(theEdge, aMinZ, aMaxZ);
  return std::abs(aMinZ) < Precision::Confusion() && std::abs(aMaxZ) < Precision::Confusion();
}

int countC0BSplineFaces(const TopoDS_Shape& theShape)
{
  int                                                           aC0FaceCount = 0;
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aFaces;
  TopExp::MapShapes(theShape, TopAbs_FACE, aFaces);
  for (int aFaceIndex = 1; aFaceIndex <= aFaces.Extent(); ++aFaceIndex)
  {
    const occ::handle<Geom_BSplineSurface> aSurface =
      occ::down_cast<Geom_BSplineSurface>(BRep_Tool::Surface(TopoDS::Face(aFaces(aFaceIndex))));
    if (aSurface.IsNull())
    {
      continue;
    }

    bool hasC0Knot = false;
    for (int aKnotIndex = 2; aKnotIndex < aSurface->NbUKnots(); ++aKnotIndex)
    {
      if (aSurface->UMultiplicity(aKnotIndex) >= aSurface->UDegree())
      {
        hasC0Knot = true;
        break;
      }
    }
    for (int aKnotIndex = 2; !hasC0Knot && aKnotIndex < aSurface->NbVKnots(); ++aKnotIndex)
    {
      hasC0Knot = aSurface->VMultiplicity(aKnotIndex) >= aSurface->VDegree();
    }
    if (hasC0Knot)
    {
      ++aC0FaceCount;
    }
  }
  return aC0FaceCount;
}

occ::handle<Geom_BSplineCurve> performPeriodicSpine(
  const NCollection_LinearVector<TopoDS_Edge>& theEdges,
  const double                                 theTolerance)
{
  occ::handle<ChFiDS_Spine> aSpine = new ChFiDS_Spine(theTolerance);
  for (const TopoDS_Edge& anEdge : theEdges)
  {
    aSpine->SetEdges(anEdge);
  }
  aSpine->SetFirstStatus(ChFiDS_Closed);
  aSpine->SetLastStatus(ChFiDS_Closed);
  aSpine->Load();

  occ::handle<ChFiDS_ElSpine> anElSpine = new ChFiDS_ElSpine();
  anElSpine->FirstParameter(aSpine->FirstParameter());
  anElSpine->LastParameter(aSpine->LastParameter());
  anElSpine->SetPeriodic(true);
  ChFi3d_PerformElSpine(anElSpine, aSpine, GeomAbs_C1, theTolerance);
  return anElSpine->BSpline();
}

NCollection_LinearVector<TopoDS_Edge> makeStadiumEdges(const double theScale)
{
  const double aRadius = 10.0 * theScale;
  const double aLength = 50.0 * theScale;
  const gp_Pnt aP1(0.0, aRadius, 0.0);
  const gp_Pnt aP2(aLength, aRadius, 0.0);
  const gp_Pnt aP3(aLength, -aRadius, 0.0);
  const gp_Pnt aP4(0.0, -aRadius, 0.0);

  NCollection_LinearVector<TopoDS_Edge> anEdges(4);
  anEdges.Append(BRepBuilderAPI_MakeEdge(aP1, aP2));
  anEdges.Append(BRepBuilderAPI_MakeEdge(
    GC_MakeArcOfCircle(aP2, gp_Pnt(aLength + aRadius, 0.0, 0.0), aP3).Value()));
  anEdges.Append(BRepBuilderAPI_MakeEdge(aP3, aP4));
  anEdges.Append(
    BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(aP4, gp_Pnt(-aRadius, 0.0, 0.0), aP1).Value()));
  return anEdges;
}

NCollection_LinearVector<TopoDS_Edge> makeSplitCircleEdges()
{
  const gp_Circ aCircle(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), 20.0);
  NCollection_LinearVector<TopoDS_Edge> anEdges(4);
  anEdges.Append(BRepBuilderAPI_MakeEdge(aCircle, 0.0, 0.5 * M_PI));
  anEdges.Append(BRepBuilderAPI_MakeEdge(aCircle, 0.5 * M_PI, M_PI));
  anEdges.Append(BRepBuilderAPI_MakeEdge(aCircle, M_PI, 1.5 * M_PI));
  anEdges.Append(BRepBuilderAPI_MakeEdge(aCircle, 1.5 * M_PI, 2.0 * M_PI));
  return anEdges;
}

NCollection_LinearVector<TopoDS_Edge> makeSplitEllipseEdges(const double theScale)
{
  const gp_Elips anEllipse(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                           30.0 * theScale,
                           20.0 * theScale);
  NCollection_LinearVector<TopoDS_Edge> anEdges(2);
  anEdges.Append(BRepBuilderAPI_MakeEdge(anEllipse, 0.0, M_PI));
  anEdges.Append(BRepBuilderAPI_MakeEdge(anEllipse, M_PI, 2.0 * M_PI));
  return anEdges;
}

} // namespace

TEST(ChFi3d_PerformElSpineTest, ExactMultiEdgeCircle_NoApproximation_KeepsClosureMultiplicity)
{
  constexpr double                     aTolerance = 1.0e-4;
  const occ::handle<Geom_BSplineCurve> aCurve =
    performPeriodicSpine(makeSplitCircleEdges(), aTolerance);

  ASSERT_FALSE(aCurve.IsNull());
  ASSERT_TRUE(aCurve->IsPeriodic());
  EXPECT_EQ(aCurve->Multiplicity(1), aCurve->Degree())
    << "An exact periodic guide must retain the OCC22163 guarded behavior";
}

TEST(ChFi3d_PerformElSpineTest, ExactMultiEdgeEllipse_NoApproximation_KeepsClosureMultiplicity)
{
  constexpr double aTolerance = 1.0e-4;
  for (const double aScale : {1.0, 1.0e4})
  {
    const occ::handle<Geom_BSplineCurve> aCurve =
      performPeriodicSpine(makeSplitEllipseEdges(aScale), aTolerance);

    ASSERT_FALSE(aCurve.IsNull()) << "Scale: " << aScale;
    ASSERT_TRUE(aCurve->IsPeriodic()) << "Scale: " << aScale;
    EXPECT_EQ(aCurve->Multiplicity(1), aCurve->Degree()) << "Scale: " << aScale;
  }
}

TEST(BRepFilletAPI_MakeFilletTest, PeriodicAnalyticSpineKeepsParticularSurface)
{
  NCollection_LinearVector<TopoDS_Shape> aShapes(2);
  aShapes.Append(BRepPrimAPI_MakeCylinder(20.0, 50.0).Shape());
  aShapes.Append(BRepPrimAPI_MakeCone(25.0, 12.0, 50.0).Shape());
  for (size_t aShapeIndex = 0; aShapeIndex < aShapes.Size(); ++aShapeIndex)
  {
    const TopoDS_Shape&                                           aShape = aShapes[aShapeIndex];
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aEdges;
    TopExp::MapShapes(aShape, TopAbs_EDGE, aEdges);
    int aCircularEdgeCount = 0;
    for (int anEdgeIndex = 1; anEdgeIndex <= aEdges.Extent(); ++anEdgeIndex)
    {
      const TopoDS_Edge anEdge = TopoDS::Edge(aEdges(anEdgeIndex));
      if (BRepAdaptor_Curve(anEdge).GetType() != GeomAbs_Circle)
      {
        continue;
      }

      ++aCircularEdgeCount;
      BRepFilletAPI_MakeFillet aFillet(aShape, ChFi3d_Rational);
      aFillet.Add(4.0, anEdge);
      aFillet.Build();
      ASSERT_TRUE(aFillet.IsDone())
        << "Shape: " << aShapeIndex + 1 << ", circular edge: " << aCircularEdgeCount;

      const TopoDS_Shape& aResult = aFillet.Shape();
      ASSERT_FALSE(aResult.IsNull())
        << "Shape: " << aShapeIndex + 1 << ", circular edge: " << aCircularEdgeCount;
      EXPECT_TRUE(BRepCheck_Analyzer(aResult).IsValid())
        << "Shape: " << aShapeIndex + 1 << ", circular edge: " << aCircularEdgeCount;

      NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aFaces;
      TopExp::MapShapes(aResult, TopAbs_FACE, aFaces);
      int aTorusCount   = 0;
      int aBSplineCount = 0;
      for (int aFaceIndex = 1; aFaceIndex <= aFaces.Extent(); ++aFaceIndex)
      {
        const GeomAbs_SurfaceType aType =
          BRepAdaptor_Surface(TopoDS::Face(aFaces(aFaceIndex))).GetType();
        aTorusCount += aType == GeomAbs_Torus ? 1 : 0;
        aBSplineCount += aType == GeomAbs_BSplineSurface ? 1 : 0;
      }
      EXPECT_EQ(aTorusCount, 1) << "Shape: " << aShapeIndex + 1
                                << ", circular edge: " << aCircularEdgeCount;
      EXPECT_EQ(aBSplineCount, 0) << "Shape: " << aShapeIndex + 1
                                  << ", circular edge: " << aCircularEdgeCount;
    }
    EXPECT_EQ(aCircularEdgeCount, 2) << "Shape: " << aShapeIndex + 1;
  }
}

TEST(ChFi3d_PerformElSpineTest, TangentCurvatureJump_ScaledGuide_ReducesClosureMultiplicity)
{
  constexpr double aTolerance = 1.0e-4;
  for (const double aScale : {1.0, 1.0e4})
  {
    const occ::handle<Geom_BSplineCurve> aCurve =
      performPeriodicSpine(makeStadiumEdges(aScale), aTolerance);

    ASSERT_FALSE(aCurve.IsNull()) << "Scale: " << aScale;
    ASSERT_TRUE(aCurve->IsPeriodic()) << "Scale: " << aScale;
    EXPECT_LE(aCurve->Multiplicity(1), aCurve->Degree() - 2) << "Scale: " << aScale;
  }
}

TEST(BRepFilletAPI_MakeFilletTest, PeriodicSpine_TangentCurvatureJump_DoesNotCreateC0Surface)
{
  constexpr double aHeight       = 30.0;
  constexpr double aTaperDegrees = 10.0;
  const double     aGrowth       = 2.0 * aHeight * std::tan(aTaperDegrees * M_PI / 180.0);

  BRepOffsetAPI_ThruSections aLoft(true, true);
  aLoft.AddWire(makeRectangle(50.0, 70.0, 0.0));
  aLoft.AddWire(makeRectangle(50.0 + aGrowth, 70.0 + aGrowth, aHeight));
  aLoft.Build();
  ASSERT_TRUE(aLoft.IsDone()) << "Tapered loft should be built";

  BRepFilletAPI_MakeFillet                                      aVerticalFillet(aLoft.Shape());
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aLoftEdges;
  TopExp::MapShapes(aLoft.Shape(), TopAbs_EDGE, aLoftEdges);
  int aVerticalEdgeCount = 0;
  for (int anEdgeIndex = 1; anEdgeIndex <= aLoftEdges.Extent(); ++anEdgeIndex)
  {
    const TopoDS_Edge anEdge = TopoDS::Edge(aLoftEdges(anEdgeIndex));
    if (spansHeight(anEdge, aHeight))
    {
      aVerticalFillet.Add(10.0, anEdge);
      ++aVerticalEdgeCount;
    }
  }
  ASSERT_EQ(aVerticalEdgeCount, 4);
  aVerticalFillet.Build();
  ASSERT_TRUE(aVerticalFillet.IsDone()) << "Vertical fillet should be built";

  NCollection_LinearVector<TopoDS_Edge>                         aBottomEdges(8);
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aRoundedEdges;
  TopExp::MapShapes(aVerticalFillet.Shape(), TopAbs_EDGE, aRoundedEdges);
  for (int anEdgeIndex = 1; anEdgeIndex <= aRoundedEdges.Extent(); ++anEdgeIndex)
  {
    const TopoDS_Edge anEdge = TopoDS::Edge(aRoundedEdges(anEdgeIndex));
    if (liesOnBottom(anEdge))
    {
      aBottomEdges.Append(anEdge);
    }
  }
  ASSERT_EQ(aBottomEdges.Size(), 8U);

  for (size_t aSeedIndex = 0; aSeedIndex < aBottomEdges.Size(); ++aSeedIndex)
  {
    BRepFilletAPI_MakeFillet aBottomFillet(aVerticalFillet.Shape(), ChFi3d_Rational);
    aBottomFillet.SetParams(1.0e-2, 1.0e-4, 1.0e-5, 1.0e-4, 1.0e-5, 1.0e-3);
    aBottomFillet.SetContinuity(GeomAbs_C1, 1.0e-2);
    aBottomFillet.Add(5.0, aBottomEdges[aSeedIndex]);
    aBottomFillet.Build();
    ASSERT_TRUE(aBottomFillet.IsDone()) << "Seed edge: " << aSeedIndex + 1;

    const TopoDS_Shape& aResult = aBottomFillet.Shape();
    ASSERT_FALSE(aResult.IsNull()) << "Seed edge: " << aSeedIndex + 1;
    EXPECT_TRUE(BRepCheck_Analyzer(aResult).IsValid()) << "Seed edge: " << aSeedIndex + 1;
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aResultFaces;
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aResultEdges;
    TopExp::MapShapes(aResult, TopAbs_FACE, aResultFaces);
    TopExp::MapShapes(aResult, TopAbs_EDGE, aResultEdges);
    EXPECT_EQ(aResultFaces.Extent(), 18) << "Seed edge: " << aSeedIndex + 1;
    EXPECT_EQ(aResultEdges.Extent(), 40) << "Seed edge: " << aSeedIndex + 1;
    EXPECT_EQ(countC0BSplineFaces(aResult), 0) << "Seed edge: " << aSeedIndex + 1;
  }
}

TEST(ChFi3d_PerformElSpineTest, ChamferGuide_TangentCurvatureJumps_PreservesGeometry)
{
  constexpr double aTolerance = 1.e-4;
  for (const double aScale : {0.1, 1., 1.e4})
  {
    for (const bool isPeriodic : {false, true})
    {
      SCOPED_TRACE(testing::Message() << "scale=" << aScale << " periodic=" << isPeriodic);
      occ::handle<ChFiDS_Spine> aSpine  = new ChFiDS_ChamfSpine(aTolerance);
      const auto                anEdges = makeStadiumEdges(aScale);
      // The open case ends after the second straight edge; the closed case
      // also includes the return semicircle and tests the periodic seam.
      const size_t anEdgeCount = isPeriodic ? 4 : 3;
      for (size_t anIndex = 0; anIndex < anEdgeCount; ++anIndex)
      {
        aSpine->SetEdges(anEdges[anIndex]);
      }
      if (isPeriodic)
      {
        aSpine->SetFirstStatus(ChFiDS_Closed);
        aSpine->SetLastStatus(ChFiDS_Closed);
      }
      aSpine->Load();
      occ::handle<ChFiDS_ElSpine> aGuide = new ChFiDS_ElSpine();
      aGuide->FirstParameter(aSpine->FirstParameter());
      aGuide->LastParameter(aSpine->LastParameter());
      aGuide->SetPeriodic(isPeriodic);
      gp_Pnt aPoint;
      gp_Vec aTangent;
      aSpine->D1(aSpine->FirstParameter(), aPoint, aTangent);
      aGuide->SetFirstPointAndTgt(aPoint, aTangent);
      aSpine->D1(aSpine->LastParameter(), aPoint, aTangent);
      aGuide->SetLastPointAndTgt(aPoint, aTangent);
      ChFi3d_PerformElSpine(aGuide, aSpine, GeomAbs_C1, aTolerance);
      ASSERT_FALSE(aGuide->BSpline().IsNull());
      // Knot multiplicity can report C0 for an exact, geometrically tangent
      // rational join. Check the one-sided tangent directions instead.
      EXPECT_TRUE(aGuide->BSpline()->IsG1(aGuide->FirstParameter(),
                                          aGuide->LastParameter(),
                                          Precision::Angular()));
      if (isPeriodic)
      {
        EXPECT_TRUE(aGuide->BSpline()->IsPeriodic());
        gp_Pnt    aFirstPoint, aLastPoint;
        gp_Vec    aFirstTangent, aLastTangent;
        const int aFirstKnot = aGuide->BSpline()->FirstUKnotIndex();
        const int aLastKnot  = aGuide->BSpline()->LastUKnotIndex();
        aGuide->BSpline()->LocalD1(aGuide->FirstParameter(),
                                   aFirstKnot,
                                   aFirstKnot + 1,
                                   aFirstPoint,
                                   aFirstTangent);
        aGuide->BSpline()->LocalD1(aGuide->LastParameter(),
                                   aLastKnot - 1,
                                   aLastKnot,
                                   aLastPoint,
                                   aLastTangent);
        EXPECT_NEAR(aFirstPoint.Distance(aLastPoint), 0., aTolerance);
        EXPECT_NEAR(aFirstTangent.Angle(aLastTangent), 0., Precision::Angular());
      }
      const auto checkPoint = [&](const double theParam) {
        const gp_Pnt aP = aGuide->Value(theParam);
        // Signed distance to the exact stadium: the two horizontal segments
        // and the semicircles centered at x=0 and x=50*scale.
        const double aX        = std::clamp(aP.X(), 0., 50. * aScale);
        const double aDistance = std::hypot(aP.X() - aX, aP.Y()) - 10. * aScale;
        EXPECT_NEAR(aDistance, 0., aTolerance) << "parameter=" << theParam;
        EXPECT_NEAR(aP.Z(), 0., Precision::Confusion());
      };
      for (int anIndex = 0; anIndex <= 200; ++anIndex)
      {
        const double aParam =
          aGuide->FirstParameter()
          + (aGuide->LastParameter() - aGuide->FirstParameter()) * anIndex / 200.;
        checkPoint(aParam);
      }
      // Uniform samples alone can miss the joins and the periodic seam.
      for (int aKnot = 1; aKnot <= aGuide->BSpline()->NbKnots(); ++aKnot)
      {
        const double aParam = aGuide->BSpline()->Knot(aKnot);
        if (aParam >= aGuide->FirstParameter() && aParam <= aGuide->LastParameter())
        {
          checkPoint(aParam);
        }
      }
    }
  }
}

TEST(ChFi3d_PerformElSpineTest, ChamferGuide_ShallowBSplineFeature_PreservesGeometry)
{
  constexpr double aTolerance = 1.e-4;
  // A C2 edge with a small, intentional feature on a long span. Increasing
  // its continuity must not flatten the feature beyond the geometric tolerance.
  NCollection_Array1<gp_Pnt> aPoles(1, 7);
  const double               anX[] = {0., 100., 300., 500., 700., 900., 1000.};
  for (int anIndex = 1; anIndex <= 7; ++anIndex)
  {
    aPoles(anIndex) = gp_Pnt(anX[anIndex - 1], anIndex == 4 ? .01 : 0., 0.);
  }
  NCollection_Array1<double> aKnots(1, 5);
  NCollection_Array1<int>    aMults(1, 5);
  for (int anIndex = 1; anIndex <= 5; ++anIndex)
  {
    aKnots(anIndex) = .25 * (anIndex - 1);
    aMults(anIndex) = (anIndex == 1 || anIndex == 5) ? 4 : 1;
  }
  occ::handle<Geom_BSplineCurve> aCurve     = new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
  occ::handle<ChFiDS_Spine>      aSpine     = new ChFiDS_ChamfSpine(aTolerance);
  const occ::handle<Geom_Curve>  anOriginal = occ::down_cast<Geom_Curve>(aCurve->Copy());
  aSpine->SetEdges(BRepBuilderAPI_MakeEdge(aCurve));
  aSpine->Load();
  occ::handle<ChFiDS_ElSpine> aGuide = new ChFiDS_ElSpine();
  aGuide->FirstParameter(aSpine->FirstParameter());
  aGuide->LastParameter(aSpine->LastParameter());
  gp_Pnt aPoint;
  gp_Vec aTangent;
  aSpine->D1(aSpine->FirstParameter(), aPoint, aTangent);
  aGuide->SetFirstPointAndTgt(aPoint, aTangent);
  aSpine->D1(aSpine->LastParameter(), aPoint, aTangent);
  aGuide->SetLastPointAndTgt(aPoint, aTangent);
  ChFi3d_PerformElSpine(aGuide, aSpine, GeomAbs_C1, aTolerance);
  ASSERT_FALSE(aGuide->BSpline().IsNull());
  double aMaxDistance = 0.;
  for (int anIndex = 0; anIndex <= 200; ++anIndex)
  {
    const double aParam = aGuide->FirstParameter()
                          + (aGuide->LastParameter() - aGuide->FirstParameter()) * anIndex / 200.;
    GeomAPI_ProjectPointOnCurve aProjection(aGuide->Value(aParam), anOriginal);
    ASSERT_GT(aProjection.NbPoints(), 0);
    aMaxDistance = std::max(aMaxDistance, aProjection.LowerDistance());
  }
  EXPECT_LE(aMaxDistance, aTolerance);
}
