// Copyright (c) 2026 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom_BSplineCurve.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Shell.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace
{
enum class SurfaceFamily
{
  PlanePlane,
  PlaneCylinder,
  PlaneCone,
  PlaneSphere,
  PlaneTorus,
  PlaneExtrusion,
  PlaneRevolution,
  PlaneBSpline
};

enum class ChamferMode
{
  EqualDistance,
  TwoDistances,
  DistanceAngle
};

struct MatrixCase
{
  SurfaceFamily Family;
  ChamferMode   Mode;
};

struct EdgeContext
{
  TopoDS_Edge Edge;
  TopoDS_Face FirstFace;
  TopoDS_Face SecondFace;
};

const char* familyName(const SurfaceFamily theFamily)
{
  switch (theFamily)
  {
    case SurfaceFamily::PlanePlane:
      return "PlanePlane";
    case SurfaceFamily::PlaneCylinder:
      return "PlaneCylinder";
    case SurfaceFamily::PlaneCone:
      return "PlaneCone";
    case SurfaceFamily::PlaneSphere:
      return "PlaneSphere";
    case SurfaceFamily::PlaneTorus:
      return "PlaneTorus";
    case SurfaceFamily::PlaneExtrusion:
      return "PlaneExtrusion";
    case SurfaceFamily::PlaneRevolution:
      return "PlaneRevolution";
    case SurfaceFamily::PlaneBSpline:
      return "PlaneBSpline";
  }
  return "Unknown";
}

const char* modeName(const ChamferMode theMode)
{
  switch (theMode)
  {
    case ChamferMode::EqualDistance:
      return "EqualDistance";
    case ChamferMode::TwoDistances:
      return "TwoDistances";
    case ChamferMode::DistanceAngle:
      return "DistanceAngle";
  }
  return "Unknown";
}

TopoDS_Shape makeFamilyShape(const SurfaceFamily theFamily)
{
  switch (theFamily)
  {
    case SurfaceFamily::PlanePlane:
      return BRepPrimAPI_MakeBox(20.0, 18.0, 16.0).Shape();
    case SurfaceFamily::PlaneCylinder:
      return BRepPrimAPI_MakeCylinder(8.0, 20.0).Shape();
    case SurfaceFamily::PlaneCone:
      return BRepPrimAPI_MakeCone(9.0, 5.0, 20.0).Shape();
    case SurfaceFamily::PlaneSphere:
      return BRepPrimAPI_MakeSphere(10.0, -55.0 * M_PI / 180.0, 50.0 * M_PI / 180.0).Shape();
    case SurfaceFamily::PlaneTorus:
      return BRepPrimAPI_MakeTorus(12.0, 3.0, -2.0 * M_PI / 3.0, 2.0 * M_PI / 3.0, 3.0 * M_PI / 2.0)
        .Shape();
    case SurfaceFamily::PlaneExtrusion: {
      NCollection_Array1<gp_Pnt> aPoints(1, 4);
      aPoints.SetValue(1, gp_Pnt(0, 0, 0));
      aPoints.SetValue(2, gp_Pnt(3, 1, 0));
      aPoints.SetValue(3, gp_Pnt(7, -1, 0));
      aPoints.SetValue(4, gp_Pnt(10, 0, 0));
      const occ::handle<Geom_BSplineCurve> aCurve = GeomAPI_PointsToBSpline(aPoints).Curve();
      BRepBuilderAPI_MakeWire              aWire;
      aWire.Add(BRepBuilderAPI_MakeEdge(aCurve));
      aWire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 5, 0)));
      aWire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 5, 0), gp_Pnt(0, 5, 0)));
      aWire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 5, 0), gp_Pnt(0, 0, 0)));
      return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(aWire.Wire()), gp_Vec(0, 0, 12)).Shape();
    }
    case SurfaceFamily::PlaneRevolution: {
      NCollection_Array1<gp_Pnt> aPoints(1, 4);
      aPoints.SetValue(1, gp_Pnt(4, 0, 0));
      aPoints.SetValue(2, gp_Pnt(5, 0, 3));
      aPoints.SetValue(3, gp_Pnt(4.5, 0, 7));
      aPoints.SetValue(4, gp_Pnt(4, 0, 10));
      const occ::handle<Geom_BSplineCurve> aCurve = GeomAPI_PointsToBSpline(aPoints).Curve();
      BRepBuilderAPI_MakeWire              aWire;
      aWire.Add(BRepBuilderAPI_MakeEdge(aCurve));
      aWire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(4, 0, 10), gp_Pnt(0, 0, 10)));
      aWire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 10), gp_Pnt(0, 0, 0)));
      aWire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(4, 0, 0)));
      return BRepPrimAPI_MakeRevol(BRepBuilderAPI_MakeFace(aWire.Wire()),
                                   gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)),
                                   2.0 * M_PI,
                                   true)
        .Shape();
    }
    case SurfaceFamily::PlaneBSpline: {
      BRepOffsetAPI_ThruSections aLoft(true, false);
      for (const std::array<double, 2>& aSection : {std::array<double, 2>{0.0, 0.0},
                                                    std::array<double, 2>{6.0, 1.0},
                                                    std::array<double, 2>{12.0, 0.0}})
      {
        const double               aZ      = aSection[0];
        const double               anInset = aSection[1];
        BRepBuilderAPI_MakePolygon aPolygon;
        aPolygon.Add(gp_Pnt(-5 + anInset, -4, aZ));
        aPolygon.Add(gp_Pnt(5 + anInset, -4, aZ));
        aPolygon.Add(gp_Pnt(5 - anInset, 4, aZ));
        aPolygon.Add(gp_Pnt(-5 - anInset, 4, aZ));
        aPolygon.Close();
        aLoft.AddWire(aPolygon.Wire());
      }
      aLoft.Build();
      return aLoft.Shape();
    }
  }
  return TopoDS_Shape();
}

TopoDS_Shape makeTaperedArmCylinder(const double theCylinderRadius, const double theArmHalfWidth)
{
  // In the YZ section a rectangular arm overlaps the lower part of a circular boss.  The arm
  // flank meets the circle at an angle controlled by halfWidth / radius: the limit tends to a
  // tangent join as the ratio tends to one.  Extruding both along X reproduces the issue's long
  // chamfer edge and curved termination without importing any degenerate upstream features.
  const TopoDS_Shape anArm =
    BRepPrimAPI_MakeBox(gp_Pnt(0.0, -theArmHalfWidth, -8.0 * theCylinderRadius),
                        0.5 * theCylinderRadius,
                        2.0 * theArmHalfWidth,
                        8.0 * theCylinderRadius)
      .Shape();
  const TopoDS_Shape aBoss =
    BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(1.0, 0.0, 0.0)),
                             theCylinderRadius,
                             0.5 * theCylinderRadius)
      .Shape();
  BRepAlgoAPI_Fuse aFuse(anArm, aBoss);
  aFuse.SimplifyResult();
  return aFuse.Shape();
}

EdgeContext findTaperedArmEdge(const TopoDS_Shape& theShape,
                               const double        theCylinderRadius,
                               const double        theArmHalfWidth,
                               const bool          thePositiveSide)
{
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
    anEdgeFaceMap;
  TopExp::MapShapesAndAncestors(theShape, TopAbs_EDGE, TopAbs_FACE, anEdgeFaceMap);
  const double aTargetY = thePositiveSide ? theArmHalfWidth : -theArmHalfWidth;
  for (int anEdgeIndex = 1; anEdgeIndex <= anEdgeFaceMap.Extent(); ++anEdgeIndex)
  {
    const TopoDS_Edge anEdge = TopoDS::Edge(anEdgeFaceMap.FindKey(anEdgeIndex));
    if (BRepAdaptor_Curve(anEdge).GetType() != GeomAbs_Line)
    {
      continue;
    }
    TopoDS_Vertex aFirstVertex;
    TopoDS_Vertex aLastVertex;
    TopExp::Vertices(anEdge, aFirstVertex, aLastVertex, true);
    if (aFirstVertex.IsNull() || aLastVertex.IsNull())
    {
      continue;
    }
    const gp_Pnt aFirst = BRep_Tool::Pnt(aFirstVertex);
    const gp_Pnt aLast  = BRep_Tool::Pnt(aLastVertex);
    if (std::abs(aFirst.X()) > Precision::Confusion()
        || std::abs(aLast.X()) > Precision::Confusion()
        || std::abs(aFirst.Y() - aTargetY) > Precision::Confusion()
        || std::abs(aLast.Y() - aTargetY) > Precision::Confusion()
        || std::min(aFirst.Z(), aLast.Z()) > -7.9 * theCylinderRadius)
    {
      continue;
    }
    const NCollection_List<TopoDS_Shape>& aFaces = anEdgeFaceMap.FindFromIndex(anEdgeIndex);
    if (aFaces.Size() == 2)
    {
      return {anEdge, TopoDS::Face(aFaces.First()), TopoDS::Face(aFaces.Last())};
    }
  }
  return {};
}

std::pair<GeomAbs_SurfaceType, GeomAbs_SurfaceType> expectedSurfaceTypes(
  const SurfaceFamily theFamily)
{
  switch (theFamily)
  {
    case SurfaceFamily::PlanePlane:
      return {GeomAbs_Plane, GeomAbs_Plane};
    case SurfaceFamily::PlaneCylinder:
      return {GeomAbs_Plane, GeomAbs_Cylinder};
    case SurfaceFamily::PlaneCone:
      return {GeomAbs_Plane, GeomAbs_Cone};
    case SurfaceFamily::PlaneSphere:
      return {GeomAbs_Plane, GeomAbs_Sphere};
    case SurfaceFamily::PlaneTorus:
      return {GeomAbs_Plane, GeomAbs_Torus};
    case SurfaceFamily::PlaneExtrusion:
      return {GeomAbs_Plane, GeomAbs_SurfaceOfExtrusion};
    case SurfaceFamily::PlaneRevolution:
      return {GeomAbs_Plane, GeomAbs_SurfaceOfRevolution};
    case SurfaceFamily::PlaneBSpline:
      return {GeomAbs_Plane, GeomAbs_BSplineSurface};
  }
  return {GeomAbs_OtherSurface, GeomAbs_OtherSurface};
}

EdgeContext findFamilyEdge(const TopoDS_Shape& theShape,
                           const SurfaceFamily theFamily,
                           const bool          thePreferShortest = false)
{
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
    anEdgeFaceMap;
  TopExp::MapShapesAndAncestors(theShape, TopAbs_EDGE, TopAbs_FACE, anEdgeFaceMap);
  const auto  anExpected = expectedSurfaceTypes(theFamily);
  EdgeContext aBestContext;
  double      aBestLength = RealLast();

  for (TopExp_Explorer anExp(theShape, TopAbs_EDGE); anExp.More(); anExp.Next())
  {
    const TopoDS_Edge anEdge = TopoDS::Edge(anExp.Current());
    if (!anEdgeFaceMap.Contains(anEdge) || anEdgeFaceMap.FindFromKey(anEdge).Size() < 2)
    {
      continue;
    }
    const NCollection_List<TopoDS_Shape>& aFaces      = anEdgeFaceMap.FindFromKey(anEdge);
    const TopoDS_Face                     aFirst      = TopoDS::Face(aFaces.First());
    const TopoDS_Face                     aSecond     = TopoDS::Face(aFaces.Last());
    const GeomAbs_SurfaceType             aFirstType  = BRepAdaptor_Surface(aFirst).GetType();
    const GeomAbs_SurfaceType             aSecondType = BRepAdaptor_Surface(aSecond).GetType();
    if ((aFirstType == anExpected.first && aSecondType == anExpected.second)
        || (aFirstType == anExpected.second && aSecondType == anExpected.first))
    {
      GProp_GProps aProperties;
      BRepGProp::LinearProperties(anEdge, aProperties);
      if (aProperties.Mass() >= aBestLength)
      {
        continue;
      }
      aBestLength = aProperties.Mass();
      if (aSecondType == GeomAbs_Plane && aFirstType != GeomAbs_Plane)
      {
        aBestContext = {anEdge, aSecond, aFirst};
      }
      else
      {
        aBestContext = {anEdge, aFirst, aSecond};
      }
      if (!thePreferShortest)
      {
        return aBestContext;
      }
    }
  }
  return aBestContext;
}

std::vector<EdgeContext> findFamilyEdges(const TopoDS_Shape& theShape,
                                         const SurfaceFamily theFamily)
{
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
    anEdgeFaceMap;
  TopExp::MapShapesAndAncestors(theShape, TopAbs_EDGE, TopAbs_FACE, anEdgeFaceMap);
  const auto               anExpected = expectedSurfaceTypes(theFamily);
  std::vector<EdgeContext> aContexts;
  for (int anEdgeIndex = 1; anEdgeIndex <= anEdgeFaceMap.Extent(); ++anEdgeIndex)
  {
    const TopoDS_Edge                     anEdge = TopoDS::Edge(anEdgeFaceMap.FindKey(anEdgeIndex));
    const NCollection_List<TopoDS_Shape>& aFaces = anEdgeFaceMap.FindFromIndex(anEdgeIndex);
    if (aFaces.Size() < 2)
    {
      continue;
    }

    const TopoDS_Face         aFirst      = TopoDS::Face(aFaces.First());
    const TopoDS_Face         aSecond     = TopoDS::Face(aFaces.Last());
    const GeomAbs_SurfaceType aFirstType  = BRepAdaptor_Surface(aFirst).GetType();
    const GeomAbs_SurfaceType aSecondType = BRepAdaptor_Surface(aSecond).GetType();
    if (aFirstType == anExpected.first && aSecondType == anExpected.second)
    {
      aContexts.push_back({anEdge, aFirst, aSecond});
    }
    else if (aFirstType == anExpected.second && aSecondType == anExpected.first)
    {
      aContexts.push_back({anEdge, aSecond, aFirst});
    }
  }
  return aContexts;
}

double shapeVolume(const TopoDS_Shape& theShape)
{
  GProp_GProps aProperties;
  BRepGProp::VolumeProperties(theShape, aProperties);
  return aProperties.Mass();
}

void expectMaterialRemoved(const TopoDS_Shape& theResult, const TopoDS_Shape& theInput)
{
  const double anInputVolume = shapeVolume(theInput);
  EXPECT_LT(shapeVolume(theResult), anInputVolume - std::max(1.0e-12, anInputVolume * 1.0e-10))
    << "a successful chamfer must not return an unchanged input solid";
}

bool containsSubShape(const TopoDS_Shape& theShape, const TopoDS_Shape& theSubShape)
{
  for (TopExp_Explorer anExp(theShape, theSubShape.ShapeType()); anExp.More(); anExp.Next())
  {
    if (anExp.Current().IsSame(theSubShape))
    {
      return true;
    }
  }
  return false;
}

double maximumTolerance(const TopoDS_Shape& theShape)
{
  double aMaximum = 0.0;
  for (TopExp_Explorer anExp(theShape, TopAbs_VERTEX); anExp.More(); anExp.Next())
  {
    aMaximum = std::max(aMaximum, BRep_Tool::Tolerance(TopoDS::Vertex(anExp.Current())));
  }
  for (TopExp_Explorer anExp(theShape, TopAbs_EDGE); anExp.More(); anExp.Next())
  {
    aMaximum = std::max(aMaximum, BRep_Tool::Tolerance(TopoDS::Edge(anExp.Current())));
  }
  for (TopExp_Explorer anExp(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
  {
    aMaximum = std::max(aMaximum, BRep_Tool::Tolerance(TopoDS::Face(anExp.Current())));
  }
  return aMaximum;
}

void expectClosedValidSolid(const TopoDS_Shape& theResult,
                            const TopoDS_Shape& theInput,
                            const double        theScale = 1.0)
{
  ASSERT_FALSE(theResult.IsNull());
  BRepCheck_Analyzer anAnalyzer(theResult);
  EXPECT_TRUE(anAnalyzer.IsValid()) << "the builder reported success with an invalid result";

  int aSolidCount = 0;
  for (TopExp_Explorer anExp(theResult, TopAbs_SOLID); anExp.More(); anExp.Next())
  {
    ++aSolidCount;
  }
  EXPECT_EQ(aSolidCount, 1) << "a single-solid input must remain a single solid";

  int aShellCount = 0;
  for (TopExp_Explorer anExp(theResult, TopAbs_SHELL); anExp.More(); anExp.Next())
  {
    ++aShellCount;
    EXPECT_TRUE(BRep_Tool::IsClosed(TopoDS::Shell(anExp.Current())))
      << "successful fillet/chamfer result contains an unconnected shell";
  }
  EXPECT_EQ(aShellCount, 1);

  const double anInputVolume  = shapeVolume(theInput);
  const double aResultVolume  = shapeVolume(theResult);
  const double aVolumeEpsilon = std::max(1.0e-12, anInputVolume * 1.0e-8);
  EXPECT_GT(aResultVolume, 0.0);
  EXPECT_LE(aResultVolume, 1.25 * anInputVolume + aVolumeEpsilon)
    << "a local edge treatment changed the global volume implausibly";

  EXPECT_LE(maximumTolerance(theResult), 2.01e-4 * std::max(1.0, theScale))
    << "topology must not be joined by inflating tolerances";

  Bnd_Box anInputBox;
  Bnd_Box aResultBox;
  BRepBndLib::AddOptimal(theInput, anInputBox, false, true);
  BRepBndLib::AddOptimal(theResult, aResultBox, false, true);
  double anIXMin, anIYMin, anIZMin, anIXMax, anIYMax, anIZMax;
  double aRXMin, aRYMin, aRZMin, aRXMax, aRYMax, aRZMax;
  anInputBox.Get(anIXMin, anIYMin, anIZMin, anIXMax, anIYMax, anIZMax);
  aResultBox.Get(aRXMin, aRYMin, aRZMin, aRXMax, aRYMax, aRZMax);
  const double aBoundTolerance = std::max(1.1 * theScale, 1.0e-8);
  EXPECT_GE(aRXMin, anIXMin - aBoundTolerance);
  EXPECT_GE(aRYMin, anIYMin - aBoundTolerance);
  EXPECT_GE(aRZMin, anIZMin - aBoundTolerance);
  EXPECT_LE(aRXMax, anIXMax + aBoundTolerance);
  EXPECT_LE(aRYMax, anIYMax + aBoundTolerance);
  EXPECT_LE(aRZMax, anIZMax + aBoundTolerance);
}

TopoDS_Shape buildChamfer(const TopoDS_Shape& theShape,
                          const EdgeContext&  theContext,
                          const ChamferMode   theMode,
                          const double        theScale         = 1.0,
                          const bool          theUseSecondFace = false)
{
  BRepFilletAPI_MakeChamfer aChamfer(theShape);
  const TopoDS_Face&        aReferenceFace =
    theUseSecondFace ? theContext.SecondFace : theContext.FirstFace;
  switch (theMode)
  {
    case ChamferMode::EqualDistance:
      aChamfer.Add(0.5 * theScale, theContext.Edge);
      break;
    case ChamferMode::TwoDistances:
      if (theUseSecondFace)
      {
        aChamfer.Add(0.65 * theScale, 0.35 * theScale, theContext.Edge, aReferenceFace);
      }
      else
      {
        aChamfer.Add(0.35 * theScale, 0.65 * theScale, theContext.Edge, aReferenceFace);
      }
      break;
    case ChamferMode::DistanceAngle:
      aChamfer.AddDA(0.25 * theScale, M_PI / 4.0, theContext.Edge, aReferenceFace);
      break;
  }
  aChamfer.Build();
  return aChamfer.IsDone() ? aChamfer.Shape() : TopoDS_Shape();
}

TopoDS_Shape buildTwoDistanceChamfer(const TopoDS_Shape& theShape,
                                     const EdgeContext&  theContext,
                                     const double        theFirstDistance,
                                     const double        theSecondDistance,
                                     const bool          theUseSecondFace)
{
  BRepFilletAPI_MakeChamfer aChamfer(theShape);
  if (theUseSecondFace)
  {
    aChamfer.Add(theSecondDistance, theFirstDistance, theContext.Edge, theContext.SecondFace);
  }
  else
  {
    aChamfer.Add(theFirstDistance, theSecondDistance, theContext.Edge, theContext.FirstFace);
  }
  aChamfer.Build();
  return aChamfer.IsDone() ? aChamfer.Shape() : TopoDS_Shape();
}

class ChamferSurfaceModeMatrix : public testing::TestWithParam<MatrixCase>
{
};

std::string matrixCaseName(const testing::TestParamInfo<MatrixCase>& theInfo)
{
  return std::string(familyName(theInfo.param.Family)) + "_" + modeName(theInfo.param.Mode);
}
} // namespace

TEST_P(ChamferSurfaceModeMatrix, ProducesClosedValidContainedSolid)
{
  const MatrixCase   aCase   = GetParam();
  const TopoDS_Shape anInput = makeFamilyShape(aCase.Family);
  ASSERT_FALSE(anInput.IsNull());
  ASSERT_TRUE(BRepCheck_Analyzer(anInput, true, false, true).IsValid());
  const bool preferShortest =
    (aCase.Family == SurfaceFamily::PlaneTorus && aCase.Mode == ChamferMode::DistanceAngle)
    || (aCase.Family == SurfaceFamily::PlaneExtrusion && aCase.Mode == ChamferMode::TwoDistances);
  const EdgeContext aContext = findFamilyEdge(anInput, aCase.Family, preferShortest);
  ASSERT_FALSE(aContext.Edge.IsNull()) << "surface-pair edge is absent from the fixture";

  TopoDS_Shape aResult;
  ASSERT_NO_THROW(aResult = buildChamfer(anInput, aContext, aCase.Mode));
  ASSERT_FALSE(aResult.IsNull()) << "the surface/mode matrix case did not build";
  expectClosedValidSolid(aResult, anInput);
  if (aCase.Family <= SurfaceFamily::PlaneTorus)
  {
    EXPECT_TRUE(BRepCheck_Analyzer(aResult, true, false, true).IsValid())
      << "analytic surface cases must also pass exact geometric validation";
  }
}

INSTANTIATE_TEST_SUITE_P(
  AnalyticSurfaceFamilies,
  ChamferSurfaceModeMatrix,
  testing::Values(MatrixCase{SurfaceFamily::PlanePlane, ChamferMode::EqualDistance},
                  MatrixCase{SurfaceFamily::PlanePlane, ChamferMode::TwoDistances},
                  MatrixCase{SurfaceFamily::PlanePlane, ChamferMode::DistanceAngle},
                  MatrixCase{SurfaceFamily::PlaneCylinder, ChamferMode::EqualDistance},
                  MatrixCase{SurfaceFamily::PlaneCylinder, ChamferMode::TwoDistances},
                  MatrixCase{SurfaceFamily::PlaneCylinder, ChamferMode::DistanceAngle},
                  MatrixCase{SurfaceFamily::PlaneCone, ChamferMode::EqualDistance},
                  MatrixCase{SurfaceFamily::PlaneCone, ChamferMode::TwoDistances},
                  MatrixCase{SurfaceFamily::PlaneCone, ChamferMode::DistanceAngle},
                  MatrixCase{SurfaceFamily::PlaneSphere, ChamferMode::EqualDistance},
                  MatrixCase{SurfaceFamily::PlaneSphere, ChamferMode::TwoDistances},
                  MatrixCase{SurfaceFamily::PlaneSphere, ChamferMode::DistanceAngle},
                  MatrixCase{SurfaceFamily::PlaneTorus, ChamferMode::EqualDistance},
                  MatrixCase{SurfaceFamily::PlaneTorus, ChamferMode::TwoDistances},
                  MatrixCase{SurfaceFamily::PlaneTorus, ChamferMode::DistanceAngle},
                  MatrixCase{SurfaceFamily::PlaneExtrusion, ChamferMode::EqualDistance},
                  MatrixCase{SurfaceFamily::PlaneExtrusion, ChamferMode::TwoDistances},
                  MatrixCase{SurfaceFamily::PlaneExtrusion, ChamferMode::DistanceAngle},
                  MatrixCase{SurfaceFamily::PlaneRevolution, ChamferMode::EqualDistance},
                  MatrixCase{SurfaceFamily::PlaneRevolution, ChamferMode::TwoDistances},
                  MatrixCase{SurfaceFamily::PlaneRevolution, ChamferMode::DistanceAngle},
                  MatrixCase{SurfaceFamily::PlaneBSpline, ChamferMode::EqualDistance},
                  MatrixCase{SurfaceFamily::PlaneBSpline, ChamferMode::TwoDistances},
                  MatrixCase{SurfaceFamily::PlaneBSpline, ChamferMode::DistanceAngle}),
  matrixCaseName);

TEST(BRepFilletAPI_ChamferMatrixTest, AsymmetricReferenceFaceSwapIsEquivalent)
{
  for (const SurfaceFamily aFamily : {SurfaceFamily::PlanePlane,
                                      SurfaceFamily::PlaneCylinder,
                                      SurfaceFamily::PlaneCone,
                                      SurfaceFamily::PlaneSphere,
                                      SurfaceFamily::PlaneTorus,
                                      SurfaceFamily::PlaneExtrusion,
                                      SurfaceFamily::PlaneRevolution,
                                      SurfaceFamily::PlaneBSpline})
  {
    SCOPED_TRACE(familyName(aFamily));
    const TopoDS_Shape anInput = makeFamilyShape(aFamily);
    const EdgeContext  aContext =
      findFamilyEdge(anInput, aFamily, aFamily == SurfaceFamily::PlaneExtrusion);
    const TopoDS_Shape aForward =
      buildChamfer(anInput, aContext, ChamferMode::TwoDistances, 1.0, false);
    const TopoDS_Shape aReverse =
      buildChamfer(anInput, aContext, ChamferMode::TwoDistances, 1.0, true);
    ASSERT_FALSE(aForward.IsNull());
    ASSERT_FALSE(aReverse.IsNull());
    expectClosedValidSolid(aForward, anInput);
    expectClosedValidSolid(aReverse, anInput);
    EXPECT_NEAR(shapeVolume(aForward), shapeVolume(aReverse), shapeVolume(anInput) * 1.0e-8);
  }
}

TEST(BRepFilletAPI_ChamferMatrixTest, TaperedArmCylinderContactAngleSweepRemainsClosed)
{
  const double aRadius   = 10.0;
  const double aDistance = 0.5;
  // Ratios approach, but deliberately do not equal, the tangent limit.  This sweeps ordinary
  // transverse joins, shallow joins, and the numerically sensitive near-tangent corner while all
  // inputs remain exact, closed, valid solids.
  for (const double aWidthRatio : {0.20, 0.50, 0.80, 0.95, 0.995})
  {
    const double       aHalfWidth = aRadius * aWidthRatio;
    const TopoDS_Shape anInput    = makeTaperedArmCylinder(aRadius, aHalfWidth);
    SCOPED_TRACE("width/radius=" + std::to_string(aWidthRatio));
    ASSERT_FALSE(anInput.IsNull());
    ASSERT_TRUE(BRepCheck_Analyzer(anInput, true, false, true).IsValid());

    for (const bool isPositiveSide : {false, true})
    {
      const EdgeContext aBaseContext =
        findTaperedArmEdge(anInput, aRadius, aHalfWidth, isPositiveSide);
      ASSERT_FALSE(aBaseContext.Edge.IsNull());
      for (const bool isReversed : {false, true})
      {
        EdgeContext aContext = aBaseContext;
        if (isReversed)
        {
          aContext.Edge.Reverse();
        }
        SCOPED_TRACE(std::string(isPositiveSide ? "positive" : "negative")
                     + (isReversed ? "/reversed" : "/forward"));

        BRepFilletAPI_MakeChamfer aSymmetric(anInput);
        aSymmetric.Add(aDistance, aContext.Edge);
        aSymmetric.Build();
        ASSERT_TRUE(aSymmetric.IsDone());
        expectClosedValidSolid(aSymmetric.Shape(), anInput);
        expectMaterialRemoved(aSymmetric.Shape(), anInput);

        const TopoDS_Shape aFirstReference =
          buildTwoDistanceChamfer(anInput, aContext, aDistance, aDistance, false);
        const TopoDS_Shape aSecondReference =
          buildTwoDistanceChamfer(anInput, aContext, aDistance, aDistance, true);
        ASSERT_FALSE(aFirstReference.IsNull());
        ASSERT_FALSE(aSecondReference.IsNull());
        expectClosedValidSolid(aFirstReference, anInput);
        expectClosedValidSolid(aSecondReference, anInput);
        expectMaterialRemoved(aFirstReference, anInput);
        expectMaterialRemoved(aSecondReference, anInput);
        const double aVolumeTolerance = shapeVolume(anInput) * 1.0e-8;
        EXPECT_NEAR(shapeVolume(aFirstReference), shapeVolume(aSecondReference), aVolumeTolerance);
        EXPECT_NEAR(shapeVolume(aFirstReference),
                    shapeVolume(aSymmetric.Shape()),
                    aVolumeTolerance);
      }
    }
  }
}

TEST(BRepFilletAPI_ChamferMatrixTest, AsymmetricPlaneExtrusionEveryBoundaryAndReferenceIsValid)
{
  const TopoDS_Shape             anInput = makeFamilyShape(SurfaceFamily::PlaneExtrusion);
  const std::vector<EdgeContext> aContexts =
    findFamilyEdges(anInput, SurfaceFamily::PlaneExtrusion);
  ASSERT_EQ(aContexts.size(), 4u);

  for (std::size_t anEdgeIndex = 0; anEdgeIndex < aContexts.size(); ++anEdgeIndex)
  {
    SCOPED_TRACE("edge=" + std::to_string(anEdgeIndex));
    const TopoDS_Shape aFirstReference =
      buildTwoDistanceChamfer(anInput, aContexts[anEdgeIndex], 0.35, 0.65, false);
    const TopoDS_Shape aSecondReference =
      buildTwoDistanceChamfer(anInput, aContexts[anEdgeIndex], 0.35, 0.65, true);
    ASSERT_FALSE(aFirstReference.IsNull());
    ASSERT_FALSE(aSecondReference.IsNull());
    expectClosedValidSolid(aFirstReference, anInput);
    expectClosedValidSolid(aSecondReference, anInput);
    expectMaterialRemoved(aFirstReference, anInput);
    expectMaterialRemoved(aSecondReference, anInput);
    EXPECT_NEAR(shapeVolume(aFirstReference),
                shapeVolume(aSecondReference),
                shapeVolume(anInput) * 1.0e-8);
  }
}

TEST(BRepFilletAPI_ChamferMatrixTest,
     TwoDistancePlaneExtrusionRatiosAndEdgeOrientationsRemainEquivalent)
{
  const TopoDS_Shape             anInput = makeFamilyShape(SurfaceFamily::PlaneExtrusion);
  const std::vector<EdgeContext> aContexts =
    findFamilyEdges(anInput, SurfaceFamily::PlaneExtrusion);
  ASSERT_EQ(aContexts.size(), 4u);
  // Include equal values deliberately: FreeCAD's equal-distance command calls OCCT's
  // face-referenced two-distance overload with D1 == D2, which is a different builder path from
  // the symmetric Add(distance, edge) overload.
  const std::array<std::array<double, 2>, 7> aDistances = {{{0.05, 0.15},
                                                            {0.20, 0.80},
                                                            {0.35, 0.65},
                                                            {0.49, 0.51},
                                                            {0.50, 0.50},
                                                            {0.65, 0.35},
                                                            {0.80, 0.20}}};

  for (std::size_t anEdgeIndex = 0; anEdgeIndex < aContexts.size(); ++anEdgeIndex)
  {
    for (const std::array<double, 2>& aDistance : aDistances)
    {
      for (const bool isReversed : {false, true})
      {
        EdgeContext aContext = aContexts[anEdgeIndex];
        if (isReversed)
        {
          aContext.Edge.Reverse();
        }
        SCOPED_TRACE("edge=" + std::to_string(anEdgeIndex) + "/d1=" + std::to_string(aDistance[0])
                     + "/d2=" + std::to_string(aDistance[1])
                     + (isReversed ? "/reversed" : "/forward"));
        const TopoDS_Shape aFirstReference =
          buildTwoDistanceChamfer(anInput, aContext, aDistance[0], aDistance[1], false);
        const TopoDS_Shape aSecondReference =
          buildTwoDistanceChamfer(anInput, aContext, aDistance[0], aDistance[1], true);
        ASSERT_FALSE(aFirstReference.IsNull());
        ASSERT_FALSE(aSecondReference.IsNull());
        expectClosedValidSolid(aFirstReference, anInput);
        expectClosedValidSolid(aSecondReference, anInput);
        expectMaterialRemoved(aFirstReference, anInput);
        expectMaterialRemoved(aSecondReference, anInput);
        EXPECT_NEAR(shapeVolume(aFirstReference),
                    shapeVolume(aSecondReference),
                    shapeVolume(anInput) * 1.0e-8);
      }
    }
  }
}

TEST(BRepFilletAPI_ChamferMatrixTest, AsymmetricPlaneExtrusionTransformsRemainEquivalent)
{
  struct TransformCase
  {
    const char* Name;
    gp_Trsf     Transform;
    double      Scale;
  };

  gp_Trsf aRigid;
  aRigid.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 3, 2)), 0.61);
  aRigid.SetTranslationPart(gp_Vec(-13, 17, 9));
  gp_Trsf aMirror;
  aMirror.SetMirror(gp_Ax2(gp_Pnt(1, -2, 3), gp_Dir(2, 1, 1)));
  gp_Trsf aSmallScale;
  aSmallScale.SetScale(gp_Pnt(0, 0, 0), 1.0e-2);
  gp_Trsf aLargeScale;
  aLargeScale.SetScale(gp_Pnt(0, 0, 0), 1.0e2);
  const std::array<TransformCase, 4> aTransforms = {{{"Rigid", aRigid, 1.0},
                                                     {"Mirror", aMirror, 1.0},
                                                     {"Scale1eMinus2", aSmallScale, 1.0e-2},
                                                     {"Scale1e2", aLargeScale, 1.0e2}}};

  const TopoDS_Shape             anInput = makeFamilyShape(SurfaceFamily::PlaneExtrusion);
  const std::vector<EdgeContext> aContexts =
    findFamilyEdges(anInput, SurfaceFamily::PlaneExtrusion);
  ASSERT_EQ(aContexts.size(), 4u);
  for (const TransformCase& aTransformCase : aTransforms)
  {
    BRepBuilderAPI_Transform aTransform(anInput, aTransformCase.Transform, true);
    ASSERT_TRUE(aTransform.IsDone());
    const TopoDS_Shape aTransformedInput = aTransform.Shape();
    for (std::size_t anEdgeIndex = 0; anEdgeIndex < aContexts.size(); ++anEdgeIndex)
    {
      SCOPED_TRACE(std::string(aTransformCase.Name) + "/edge=" + std::to_string(anEdgeIndex));
      EdgeContext aContext;
      aContext.Edge      = TopoDS::Edge(aTransform.ModifiedShape(aContexts[anEdgeIndex].Edge));
      aContext.FirstFace = TopoDS::Face(aTransform.ModifiedShape(aContexts[anEdgeIndex].FirstFace));
      aContext.SecondFace =
        TopoDS::Face(aTransform.ModifiedShape(aContexts[anEdgeIndex].SecondFace));
      const TopoDS_Shape aFirstReference  = buildTwoDistanceChamfer(aTransformedInput,
                                                                   aContext,
                                                                   0.35 * aTransformCase.Scale,
                                                                   0.65 * aTransformCase.Scale,
                                                                   false);
      const TopoDS_Shape aSecondReference = buildTwoDistanceChamfer(aTransformedInput,
                                                                    aContext,
                                                                    0.35 * aTransformCase.Scale,
                                                                    0.65 * aTransformCase.Scale,
                                                                    true);
      ASSERT_FALSE(aFirstReference.IsNull());
      ASSERT_FALSE(aSecondReference.IsNull());
      expectClosedValidSolid(aFirstReference, aTransformedInput, aTransformCase.Scale);
      expectClosedValidSolid(aSecondReference, aTransformedInput, aTransformCase.Scale);
      expectMaterialRemoved(aFirstReference, aTransformedInput);
      expectMaterialRemoved(aSecondReference, aTransformedInput);
      EXPECT_NEAR(shapeVolume(aFirstReference),
                  shapeVolume(aSecondReference),
                  std::max(1.0e-12, shapeVolume(aTransformedInput) * 1.0e-8));
    }
  }
}

TEST(BRepFilletAPI_ChamferMatrixTest, AsymmetricPlaneExtrusionMultipleContoursRemainValid)
{
  const TopoDS_Shape             anInput = makeFamilyShape(SurfaceFamily::PlaneExtrusion);
  const std::vector<EdgeContext> aContexts =
    findFamilyEdges(anInput, SurfaceFamily::PlaneExtrusion);
  std::vector<EdgeContext> aVerticalContexts;
  for (const EdgeContext& aContext : aContexts)
  {
    GProp_GProps aProperties;
    BRepGProp::LinearProperties(aContext.Edge, aProperties);
    if (std::abs(aProperties.Mass() - 12.0) <= Precision::Confusion())
    {
      aVerticalContexts.push_back(aContext);
    }
  }
  ASSERT_EQ(aVerticalContexts.size(), 2u);

  BRepFilletAPI_MakeChamfer aChamfer(anInput);
  for (const EdgeContext& aContext : aVerticalContexts)
  {
    aChamfer.Add(0.35, 0.65, aContext.Edge, aContext.FirstFace);
  }
  aChamfer.Build();
  ASSERT_TRUE(aChamfer.IsDone());
  expectClosedValidSolid(aChamfer.Shape(), anInput);
  expectMaterialRemoved(aChamfer.Shape(), anInput);
  ASSERT_EQ(aChamfer.NbContours(), 2);
  for (int aContour = 1; aContour <= aChamfer.NbContours(); ++aContour)
  {
    double aFirstDistance  = 0.0;
    double aSecondDistance = 0.0;
    aChamfer.Dists(aContour, aFirstDistance, aSecondDistance);
    EXPECT_DOUBLE_EQ(aFirstDistance, 0.35);
    EXPECT_DOUBLE_EQ(aSecondDistance, 0.65);
  }
}

TEST(BRepFilletAPI_ChamferMatrixTest, AsymmetricRetryPreservesMixedContourMethods)
{
  const TopoDS_Shape             anInput = makeFamilyShape(SurfaceFamily::PlaneExtrusion);
  const std::vector<EdgeContext> aContexts =
    findFamilyEdges(anInput, SurfaceFamily::PlaneExtrusion);
  std::vector<EdgeContext> aVerticalContexts;
  for (const EdgeContext& aContext : aContexts)
  {
    GProp_GProps aProperties;
    BRepGProp::LinearProperties(aContext.Edge, aProperties);
    if (std::abs(aProperties.Mass() - 12.0) <= Precision::Confusion())
    {
      aVerticalContexts.push_back(aContext);
    }
  }
  ASSERT_EQ(aVerticalContexts.size(), 2u);

  for (const ChamferMode aSecondMode : {ChamferMode::EqualDistance, ChamferMode::DistanceAngle})
  {
    SCOPED_TRACE(modeName(aSecondMode));
    BRepFilletAPI_MakeChamfer aChamfer(anInput);
    aChamfer.Add(0.35, 0.65, aVerticalContexts.front().Edge, aVerticalContexts.front().FirstFace);
    if (aSecondMode == ChamferMode::EqualDistance)
    {
      aChamfer.Add(0.40, aVerticalContexts.back().Edge);
    }
    else
    {
      aChamfer.AddDA(0.25,
                     M_PI / 4.0,
                     aVerticalContexts.back().Edge,
                     aVerticalContexts.back().FirstFace);
    }
    aChamfer.Build();
    ASSERT_TRUE(aChamfer.IsDone());
    expectClosedValidSolid(aChamfer.Shape(), anInput);
    expectMaterialRemoved(aChamfer.Shape(), anInput);
    ASSERT_EQ(aChamfer.NbContours(), 2);
    EXPECT_TRUE(aChamfer.IsTwoDistances(1));
    EXPECT_EQ(aChamfer.IsSymetric(2), aSecondMode == ChamferMode::EqualDistance);
    EXPECT_EQ(aChamfer.IsDistanceAngle(2), aSecondMode == ChamferMode::DistanceAngle);

    double aFirstDistance  = 0.0;
    double aSecondDistance = 0.0;
    aChamfer.Dists(1, aFirstDistance, aSecondDistance);
    EXPECT_DOUBLE_EQ(aFirstDistance, 0.35);
    EXPECT_DOUBLE_EQ(aSecondDistance, 0.65);
    if (aSecondMode == ChamferMode::EqualDistance)
    {
      double aDistance = 0.0;
      aChamfer.GetDist(2, aDistance);
      EXPECT_DOUBLE_EQ(aDistance, 0.40);
    }
    else
    {
      double aDistance = 0.0;
      double anAngle   = 0.0;
      aChamfer.GetDistAngle(2, aDistance, anAngle);
      EXPECT_DOUBLE_EQ(aDistance, 0.25);
      EXPECT_DOUBLE_EQ(anAngle, M_PI / 4.0);
    }

    for (const EdgeContext& aContext : aVerticalContexts)
    {
      EXPECT_FALSE(aChamfer.Generated(aContext.Edge).IsEmpty());
    }

    aChamfer.Remove(aVerticalContexts.front().Edge);
    ASSERT_EQ(aChamfer.NbContours(), 1);
    aChamfer.Build();
    ASSERT_TRUE(aChamfer.IsDone());
    expectClosedValidSolid(aChamfer.Shape(), anInput);
    expectMaterialRemoved(aChamfer.Shape(), anInput);
    BRepFilletAPI_MakeChamfer aSingleContourBuilder(anInput);
    if (aSecondMode == ChamferMode::EqualDistance)
    {
      aSingleContourBuilder.Add(0.40, aVerticalContexts.back().Edge);
    }
    else
    {
      aSingleContourBuilder.AddDA(0.25,
                                  M_PI / 4.0,
                                  aVerticalContexts.back().Edge,
                                  aVerticalContexts.back().FirstFace);
    }
    aSingleContourBuilder.Build();
    ASSERT_TRUE(aSingleContourBuilder.IsDone());
    const TopoDS_Shape aSingleContour = aSingleContourBuilder.Shape();
    EXPECT_NEAR(shapeVolume(aChamfer.Shape()),
                shapeVolume(aSingleContour),
                shapeVolume(anInput) * 1.0e-8)
      << "removing the retried contour must rebuild rather than return the stale two-contour shape";
  }
}

TEST(BRepFilletAPI_ChamferMatrixTest,
     AsymmetricPlaneExtrusionRetryPreservesHistoryParametersAndRebuilds)
{
  const TopoDS_Shape             anInput = makeFamilyShape(SurfaceFamily::PlaneExtrusion);
  const std::vector<EdgeContext> aContexts =
    findFamilyEdges(anInput, SurfaceFamily::PlaneExtrusion);
  ASSERT_EQ(aContexts.size(), 4u);
  const EdgeContext& aContext = aContexts.front();

  BRepFilletAPI_MakeChamfer aChamfer(anInput);
  aChamfer.Add(0.35, 0.65, aContext.Edge, aContext.FirstFace);
  aChamfer.Build();
  ASSERT_TRUE(aChamfer.IsDone());
  expectClosedValidSolid(aChamfer.Shape(), anInput);
  expectMaterialRemoved(aChamfer.Shape(), anInput);

  const NCollection_List<TopoDS_Shape>& aGenerated = aChamfer.Generated(aContext.Edge);
  ASSERT_FALSE(aGenerated.IsEmpty()) << "the retry must retain chamfer history for the source edge";
  for (const TopoDS_Shape& aGeneratedShape : aGenerated)
  {
    EXPECT_TRUE(containsSubShape(aChamfer.Shape(), aGeneratedShape));
  }

  double aFirstDistance  = 0.0;
  double aSecondDistance = 0.0;
  aChamfer.Dists(1, aFirstDistance, aSecondDistance);
  EXPECT_DOUBLE_EQ(aFirstDistance, 0.35);
  EXPECT_DOUBLE_EQ(aSecondDistance, 0.65);

  const double aFirstBuildVolume = shapeVolume(aChamfer.Shape());
  aChamfer.Build();
  ASSERT_TRUE(aChamfer.IsDone());
  expectClosedValidSolid(aChamfer.Shape(), anInput);
  EXPECT_NEAR(shapeVolume(aChamfer.Shape()), aFirstBuildVolume, shapeVolume(anInput) * 1.0e-10);

  aChamfer.SetDists(0.20, 0.80, 1, aContext.FirstFace);
  aChamfer.Build();
  ASSERT_TRUE(aChamfer.IsDone());
  expectClosedValidSolid(aChamfer.Shape(), anInput);
  expectMaterialRemoved(aChamfer.Shape(), anInput);
  const TopoDS_Shape anEquivalent = buildTwoDistanceChamfer(anInput, aContext, 0.20, 0.80, true);
  ASSERT_FALSE(anEquivalent.IsNull());
  EXPECT_NEAR(shapeVolume(aChamfer.Shape()),
              shapeVolume(anEquivalent),
              shapeVolume(anInput) * 1.0e-8);
  aChamfer.Dists(1, aFirstDistance, aSecondDistance);
  EXPECT_DOUBLE_EQ(aFirstDistance, 0.20);
  EXPECT_DOUBLE_EQ(aSecondDistance, 0.80);
}

TEST(BRepFilletAPI_ChamferMatrixTest, RigidMirrorAndScaleTransformsPreserveValidity)
{
  struct TransformCase
  {
    const char* Name;
    gp_Trsf     Transform;
    double      Scale;
  };

  gp_Trsf aRigid;
  aRigid.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 2, 3)), 0.73);
  aRigid.SetTranslationPart(gp_Vec(17, -11, 23));
  gp_Trsf aMirror;
  aMirror.SetMirror(gp_Ax2(gp_Pnt(2, -3, 1), gp_Dir(1, 1, 2)));
  gp_Trsf aSmallScale;
  aSmallScale.SetScale(gp_Pnt(0, 0, 0), 1.0e-2);
  gp_Trsf aLargeScale;
  aLargeScale.SetScale(gp_Pnt(0, 0, 0), 1.0e3);

  const std::array<TransformCase, 4> aTransforms = {{{"Rigid", aRigid, 1.0},
                                                     {"Mirror", aMirror, 1.0},
                                                     {"Scale1eMinus2", aSmallScale, 1.0e-2},
                                                     {"Scale1e3", aLargeScale, 1.0e3}}};
  for (const SurfaceFamily aFamily : {SurfaceFamily::PlanePlane,
                                      SurfaceFamily::PlaneCylinder,
                                      SurfaceFamily::PlaneCone,
                                      SurfaceFamily::PlaneSphere,
                                      SurfaceFamily::PlaneTorus,
                                      SurfaceFamily::PlaneExtrusion,
                                      SurfaceFamily::PlaneRevolution,
                                      SurfaceFamily::PlaneBSpline})
  {
    const TopoDS_Shape anInput  = makeFamilyShape(aFamily);
    const EdgeContext  aContext = findFamilyEdge(anInput, aFamily);
    ASSERT_FALSE(aContext.Edge.IsNull());
    for (const TransformCase& aTransformCase : aTransforms)
    {
      SCOPED_TRACE(std::string(familyName(aFamily)) + "/" + aTransformCase.Name);
      BRepBuilderAPI_Transform aTransform(anInput, aTransformCase.Transform, true);
      ASSERT_TRUE(aTransform.IsDone());
      EdgeContext aTransformedContext;
      aTransformedContext.Edge       = TopoDS::Edge(aTransform.ModifiedShape(aContext.Edge));
      aTransformedContext.FirstFace  = TopoDS::Face(aTransform.ModifiedShape(aContext.FirstFace));
      aTransformedContext.SecondFace = TopoDS::Face(aTransform.ModifiedShape(aContext.SecondFace));
      ASSERT_FALSE(aTransformedContext.Edge.IsNull());
      const TopoDS_Shape aTransformedInput = aTransform.Shape();
      const TopoDS_Shape aResult           = buildChamfer(aTransformedInput,
                                                aTransformedContext,
                                                ChamferMode::EqualDistance,
                                                aTransformCase.Scale);
      ASSERT_FALSE(aResult.IsNull());
      expectClosedValidSolid(aResult, aTransformedInput, aTransformCase.Scale);
    }
  }
}

TEST(BRepFilletAPI_ChamferMatrixTest, EdgeOrientationAndOrderDoNotChangeResultVolume)
{
  const TopoDS_Shape       anInput = BRepPrimAPI_MakeBox(20.0, 18.0, 16.0).Shape();
  std::vector<TopoDS_Edge> anEdges;
  for (TopExp_Explorer anExp(anInput, TopAbs_EDGE); anExp.More(); anExp.Next())
  {
    anEdges.push_back(TopoDS::Edge(anExp.Current()));
  }
  ASSERT_GE(anEdges.size(), 7u);

  BRepFilletAPI_MakeChamfer aForward(anInput);
  aForward.Add(0.5, anEdges[0]);
  aForward.Add(0.5, anEdges[6]);
  aForward.Build();
  ASSERT_TRUE(aForward.IsDone());
  expectClosedValidSolid(aForward.Shape(), anInput);

  BRepFilletAPI_MakeChamfer aReverse(anInput);
  aReverse.Add(0.5, TopoDS::Edge(anEdges[6].Reversed()));
  aReverse.Add(0.5, TopoDS::Edge(anEdges[0].Reversed()));
  aReverse.Build();
  ASSERT_TRUE(aReverse.IsDone());
  expectClosedValidSolid(aReverse.Shape(), anInput);
  EXPECT_NEAR(shapeVolume(aForward.Shape()),
              shapeVolume(aReverse.Shape()),
              shapeVolume(anInput) * 1.e-8);
}

TEST(BRepFilletAPI_ChamferMatrixTest, Scale1eMinus3AnalyticEdgesRemainClosed)
{
  gp_Trsf aScaleTransform;
  aScaleTransform.SetScale(gp_Pnt(0, 0, 0), 1.0e-3);
  for (const SurfaceFamily aFamily :
       {SurfaceFamily::PlanePlane, SurfaceFamily::PlaneCylinder, SurfaceFamily::PlaneCone})
  {
    SCOPED_TRACE(familyName(aFamily));
    const TopoDS_Shape       anInput  = makeFamilyShape(aFamily);
    const EdgeContext        aContext = findFamilyEdge(anInput, aFamily);
    BRepBuilderAPI_Transform aTransform(anInput, aScaleTransform, true);
    ASSERT_TRUE(aTransform.IsDone());
    const TopoDS_Shape aTransformedInput = aTransform.Shape();
    const TopoDS_Edge  aTransformedEdge  = TopoDS::Edge(aTransform.ModifiedShape(aContext.Edge));
    BRepFilletAPI_MakeChamfer aChamfer(aTransformedInput);
    aChamfer.Add(5.0e-4, aTransformedEdge);
    aChamfer.Build();
    ASSERT_TRUE(aChamfer.IsDone());
    expectClosedValidSolid(aChamfer.Shape(), aTransformedInput, 1.0e-3);
  }
}

TEST(BRepFilletAPI_ChamferMatrixTest, ShortEdgesAndThresholdPerturbationsRemainClosed)
{
  for (const double aThickness : {1.0e-3, 1.0e-2, 1.0e-1, 1.0})
  {
    SCOPED_TRACE(aThickness);
    const TopoDS_Shape anInput  = BRepPrimAPI_MakeBox(20.0, 18.0, aThickness).Shape();
    const EdgeContext  aContext = findFamilyEdge(anInput, SurfaceFamily::PlanePlane);
    ASSERT_FALSE(aContext.Edge.IsNull());
    BRepFilletAPI_MakeChamfer aChamfer(anInput);
    aChamfer.Add(0.2 * aThickness, aContext.Edge);
    aChamfer.Build();
    ASSERT_TRUE(aChamfer.IsDone());
    expectClosedValidSolid(aChamfer.Shape(), anInput, std::max(1.0e-3, aThickness));
  }

  const TopoDS_Shape anInput  = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
  const EdgeContext  aContext = findFamilyEdge(anInput, SurfaceFamily::PlanePlane);
  for (const double aDistance : {1.0e-4, 1.0e-3, 0.1, 4.9, 4.999})
  {
    SCOPED_TRACE(aDistance);
    BRepFilletAPI_MakeChamfer aChamfer(anInput);
    aChamfer.Add(aDistance, aContext.Edge);
    aChamfer.Build();
    ASSERT_TRUE(aChamfer.IsDone());
    expectClosedValidSolid(aChamfer.Shape(), anInput);
  }
}

TEST(BRepFilletAPI_ChamferMatrixTest, SuccessfulComplexCornerBuildsAreNeverInvalid)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
  const TopoDS_Shape aCylinder =
    BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(5.0, 0.0, 5.0), gp_Dir(0.0, 1.0, 0.0)), 3.0, 10.0)
      .Shape();
  BRepAlgoAPI_Fuse aFuse(aBox, aCylinder);
  ASSERT_TRUE(aFuse.IsDone());
  const TopoDS_Shape aFused = aFuse.Shape();
  ASSERT_TRUE(BRepCheck_Analyzer(aFused, true, false, true).IsValid());

  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
    aVertexFaceMap;
  TopExp::MapShapesAndAncestors(aFused, TopAbs_VERTEX, TopAbs_FACE, aVertexFaceMap);

  int aComplexAttemptCount = 0;
  int aSuccessCount        = 0;
  for (TopExp_Explorer anEdgeExp(aFused, TopAbs_EDGE); anEdgeExp.More(); anEdgeExp.Next())
  {
    const TopoDS_Edge anEdge    = TopoDS::Edge(anEdgeExp.Current());
    bool              isComplex = false;
    for (TopExp_Explorer aVertexExp(anEdge, TopAbs_VERTEX); aVertexExp.More(); aVertexExp.Next())
    {
      if (aVertexFaceMap.Contains(aVertexExp.Current())
          && aVertexFaceMap.FindFromKey(aVertexExp.Current()).Size() >= 3)
      {
        isComplex = true;
      }
    }
    if (!isComplex)
    {
      continue;
    }
    ++aComplexAttemptCount;
    BRepFilletAPI_MakeChamfer aChamfer(aFused);
    aChamfer.Add(0.25, anEdge);
    try
    {
      aChamfer.Build();
    }
    catch (const Standard_Failure&)
    {
      continue;
    }
    if (aChamfer.IsDone())
    {
      ++aSuccessCount;
      expectClosedValidSolid(aChamfer.Shape(), aFused);
    }
  }
  EXPECT_GT(aComplexAttemptCount, 0);
  EXPECT_GT(aSuccessCount, 0) << "the complex-corner sweep did not exercise a successful path";
}

TEST(BRepFilletAPI_ChamferMatrixTest, SeededTransformAndDistanceSweepRemainsValid)
{
  // A fixed generator makes every failure exactly reproducible while sampling points between the
  // hand-picked matrix values.  Keep the arithmetic local instead of std::uniform_distribution,
  // whose mapping is not required to be identical across standard-library implementations.
  std::uint64_t aState   = UINT64_C(0x30886c0ffee12345);
  const auto    nextUnit = [&aState]() {
    aState = aState * UINT64_C(6364136223846793005) + UINT64_C(1442695040888963407);
    return static_cast<double>(aState >> 11) * (1.0 / 9007199254740992.0);
  };

  const std::array<SurfaceFamily, 8> aFamilies = {{SurfaceFamily::PlanePlane,
                                                   SurfaceFamily::PlaneCylinder,
                                                   SurfaceFamily::PlaneCone,
                                                   SurfaceFamily::PlaneSphere,
                                                   SurfaceFamily::PlaneTorus,
                                                   SurfaceFamily::PlaneExtrusion,
                                                   SurfaceFamily::PlaneRevolution,
                                                   SurfaceFamily::PlaneBSpline}};
  for (int anIteration = 0; anIteration < 64; ++anIteration)
  {
    const SurfaceFamily aFamily =
      aFamilies[static_cast<std::size_t>(anIteration) % aFamilies.size()];
    const TopoDS_Shape anInput  = makeFamilyShape(aFamily);
    const EdgeContext  aContext = findFamilyEdge(anInput, aFamily);
    ASSERT_FALSE(aContext.Edge.IsNull());

    const double aScale = std::pow(10.0, -2.0 + 4.0 * nextUnit());
    gp_Dir       aDirection(0.2 + nextUnit(), 0.2 + nextUnit(), 0.2 + nextUnit());
    gp_Trsf      aRotation;
    aRotation.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), aDirection), 2.0 * M_PI * nextUnit());
    gp_Trsf aScaleTransform;
    aScaleTransform.SetScale(gp_Pnt(0, 0, 0), aScale);
    const gp_Trsf            aTransformValue = aRotation.Multiplied(aScaleTransform);
    BRepBuilderAPI_Transform aTransform(anInput, aTransformValue, true);
    ASSERT_TRUE(aTransform.IsDone());

    const TopoDS_Shape aTransformedInput = aTransform.Shape();
    const TopoDS_Edge  aTransformedEdge  = TopoDS::Edge(aTransform.ModifiedShape(aContext.Edge));
    const double       aDistance         = aScale * (0.05 + 0.45 * nextUnit());
    SCOPED_TRACE(std::string("seed=0x30886c0ffee12345 iteration=") + std::to_string(anIteration)
                 + " family=" + familyName(aFamily) + " scale=" + std::to_string(aScale)
                 + " distance=" + std::to_string(aDistance));

    BRepFilletAPI_MakeChamfer aChamfer(aTransformedInput);
    aChamfer.Add(aDistance, aTransformedEdge);
    aChamfer.Build();
    ASSERT_TRUE(aChamfer.IsDone());
    expectClosedValidSolid(aChamfer.Shape(), aTransformedInput, aScale);
  }
}
