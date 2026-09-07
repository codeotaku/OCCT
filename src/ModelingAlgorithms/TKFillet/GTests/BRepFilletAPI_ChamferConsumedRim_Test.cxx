// Copyright (c) 2026 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with the special exception defined in
// OCCT_LGPL_EXCEPTION.txt. See LICENSE_LGPL_21.txt for the complete license.

#include <BRepAlgoAPI_Check.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Ax1.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>
#include <string>
#include <tuple>

class BRepFilletAPI_ConsumedRim : public ::testing::TestWithParam<std::tuple<int, double>>
{
};

TEST_P(BRepFilletAPI_ConsumedRim, Build_HalfWallThickness_ProducesClosedValidSolid)
{
  const auto [aSelection, aDistance] = GetParam();
  const TopoDS_Shape aSource         = BRepAlgoAPI_Cut(BRepPrimAPI_MakeBox(12., 10., 4.),
                                               BRepPrimAPI_MakeBox(gp_Pnt(1., 1., 1.), 10., 8., 4.))
                                 .Shape();
  ASSERT_TRUE(BRepCheck_Analyzer(aSource).IsValid());
  ASSERT_TRUE(BRepAlgoAPI_Check(aSource).IsValid());
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> anEdges;
  TopExp::MapShapes(aSource, TopAbs_EDGE, anEdges);
  BRepFilletAPI_MakeChamfer aChamfer(aSource);
  for (int i = 1; i <= anEdges.Extent(); ++i)
  {
    const TopoDS_Edge anEdge = TopoDS::Edge(anEdges(i));
    TopoDS_Vertex     aFirst, aLast;
    TopExp::Vertices(anEdge, aFirst, aLast);
    const double z1 = BRep_Tool::Pnt(aFirst).Z(), z2 = BRep_Tool::Pnt(aLast).Z();
    const bool   isRim      = std::abs(z1 - 4.) < 1.e-7 && std::abs(z2 - 4.) < 1.e-7;
    const bool   isVertical = std::abs(z1 - z2) > 1.e-7;
    const bool   isOneWall =
      isRim && std::abs(BRep_Tool::Pnt(aFirst).X() - BRep_Tool::Pnt(aLast).X()) < 1.e-7
      && BRep_Tool::Pnt(aFirst).X() < 1.01;
    if (aSelection == 0 || (aSelection == 1 && isRim) || (aSelection == 2 && (isRim || isVertical))
        || (aSelection == 3 && !isRim) || (aSelection == 4 && isOneWall))
    {
      aChamfer.Add(aDistance, anEdge);
    }
  }
  aChamfer.Build();
  ASSERT_TRUE(aChamfer.IsDone());
  const TopoDS_Shape aResult = aChamfer.Shape();
  if (const char* dir = std::getenv("CHAMFER_RIM_OUTPUT"))
  {
    BRepTools::Write(aResult,
                     (std::string(dir) + "/rim-" + std::to_string(aSelection) + "-"
                      + std::to_string(aDistance) + ".brep")
                       .c_str());
  }
  EXPECT_TRUE(BRepCheck_Analyzer(aResult, true, false, true).IsValid());
  EXPECT_TRUE(BRepAlgoAPI_Check(aResult).IsValid());
  int nSolids = 0, nShells = 0;
  for (TopExp_Explorer it(aResult, TopAbs_SOLID); it.More(); it.Next())
  {
    ++nSolids;
  }
  for (TopExp_Explorer it(aResult, TopAbs_SHELL); it.More(); it.Next())
  {
    ++nShells;
    EXPECT_TRUE(BRep_Tool::IsClosed(it.Current()));
  }
  EXPECT_EQ(nSolids, 1);
  EXPECT_EQ(nShells, 1);
  GProp_GProps aProps;
  BRepGProp::VolumeProperties(aResult, aProps);
  EXPECT_GT(aProps.Mass(), 0.);
  EXPECT_LT(aProps.Mass(), 240.);
  if (aDistance == .5 && (aSelection == 0 || aSelection == 2))
  {
    // Consuming the straight rim does not consume the four corner regions.
    int    aTopFaces = 0;
    double aTopArea  = 0.;
    for (TopExp_Explorer it(aResult, TopAbs_FACE); it.More(); it.Next())
    {
      GProp_GProps aFaceProps;
      BRepGProp::SurfaceProperties(it.Current(), aFaceProps);
      if (std::abs(aFaceProps.CentreOfMass().Z() - 4.) < Precision::Confusion())
      {
        ++aTopFaces;
        aTopArea += aFaceProps.Mass();
      }
    }
    EXPECT_EQ(aTopFaces, 4);
    EXPECT_NEAR(aTopArea, 2., 1.e-7);
  }
}

TEST(BRepFilletAPI_ConsumedRimControl, Build_BeyondLimitInteriorCrossing_IsRejected)
{
  const TopoDS_Shape aSource = BRepAlgoAPI_Cut(BRepPrimAPI_MakeBox(12., 10., 4.),
                                               BRepPrimAPI_MakeBox(gp_Pnt(1., 1., 1.), 10., 8., 4.))
                                 .Shape();
  ASSERT_TRUE(BRepCheck_Analyzer(aSource).IsValid());
  ASSERT_TRUE(BRepAlgoAPI_Check(aSource).IsValid());
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> anEdges;
  TopExp::MapShapes(aSource, TopAbs_EDGE, anEdges);
  for (double aDistance : {.5005, .501, .51})
  {
    SCOPED_TRACE(aDistance);
    BRepFilletAPI_MakeChamfer aChamfer(aSource);
    for (const TopoDS_Shape& anEdge : anEdges)
    {
      aChamfer.Add(aDistance, TopoDS::Edge(anEdge));
    }
    aChamfer.Build();
    EXPECT_FALSE(aChamfer.IsDone());
  }
}

INSTANTIATE_TEST_SUITE_P(IndependentHollowBox,
                         BRepFilletAPI_ConsumedRim,
                         ::testing::Combine(::testing::Values(0, 1, 2, 3, 4),
                                            ::testing::Values(.49, .5)));

class BRepFilletAPI_RimVariants : public ::testing::TestWithParam<std::tuple<int, int, double, int>>
{
};

TEST_P(BRepFilletAPI_RimVariants, Build_VariedCornerAnglesAndPlacements_PreservesLimitContact)
{
  const auto [aSides, aSelection, aScale, aVariant] = GetParam();
  const auto prism = [nSides = aSides](double radius, double z, double height) {
    BRepBuilderAPI_MakePolygon polygon;
    for (int i = 0; i < nSides; ++i)
    {
      polygon.Add(
        gp_Pnt(radius * cos(2 * M_PI * i / nSides), radius * sin(2 * M_PI * i / nSides), z));
    }
    polygon.Close();
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(polygon.Wire()), gp_Vec(0, 0, height))
      .Shape();
  };
  const TopoDS_Shape source =
    BRepAlgoAPI_Cut(prism(8., 0., 4.), prism(8. - 1. / cos(M_PI / aSides), 1., 4.)).Shape();
  ASSERT_TRUE(BRepCheck_Analyzer(source).IsValid());
  ASSERT_TRUE(BRepAlgoAPI_Check(source).IsValid());
  gp_Trsf scale;
  scale.SetScale(gp_Pnt(), aScale);
  gp_Trsf place;
  if (aVariant)
  {
    place.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 2, 3)), .731);
    place.SetTranslationPart(gp_Vec(13, -7, 5));
  }
  BRepBuilderAPI_Transform transform(source, place * scale, true);
  const auto               transformed = transform.Shape();
  ASSERT_TRUE(BRepCheck_Analyzer(transformed).IsValid());
  ASSERT_TRUE(BRepAlgoAPI_Check(transformed).IsValid());
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> edges;
  TopExp::MapShapes(source, TopAbs_EDGE, edges);
  BRepFilletAPI_MakeChamfer chamfer(transformed);
  for (int n = 1; n <= edges.Extent(); ++n)
  {
    const int     i    = aVariant ? edges.Extent() + 1 - n : n;
    const auto    edge = TopoDS::Edge(edges(i));
    TopoDS_Vertex v1, v2;
    TopExp::Vertices(edge, v1, v2);
    const double z1 = BRep_Tool::Pnt(v1).Z(), z2 = BRep_Tool::Pnt(v2).Z();
    const bool   rim      = abs(z1 - 4.) < 1.e-7 && abs(z2 - 4.) < 1.e-7;
    const bool   vertical = abs(z1 - z2) > 1.e-7;
    if (aSelection == 0 || (aSelection == 1 && rim) || (aSelection == 2 && (rim || vertical)))
    {
      chamfer.Add(.5 * aScale, TopoDS::Edge(transform.ModifiedShape(edge)));
    }
  }
  chamfer.Build();
  ASSERT_TRUE(chamfer.IsDone());
  const auto result = chamfer.Shape();
  EXPECT_TRUE(BRepCheck_Analyzer(result, true, false, true).IsValid());
  EXPECT_TRUE(BRepAlgoAPI_Check(result).IsValid());
  int nSolids = 0, nShells = 0;
  for (TopExp_Explorer it(result, TopAbs_SOLID); it.More(); it.Next())
  {
    ++nSolids;
  }
  for (TopExp_Explorer it(result, TopAbs_SHELL); it.More(); it.Next())
  {
    ++nShells;
    EXPECT_TRUE(BRep_Tool::IsClosed(it.Current()));
  }
  EXPECT_EQ(nSolids, 1);
  EXPECT_EQ(nShells, 1);
  for (TopExp_Explorer it(result, TopAbs_VERTEX); it.More(); it.Next())
  {
    EXPECT_LE(BRep_Tool::Tolerance(TopoDS::Vertex(it.Current())), 1.1e-4 * std::max(1., aScale));
  }
  GProp_GProps before, after;
  BRepGProp::VolumeProperties(transformed, before);
  BRepGProp::VolumeProperties(result, after);
  EXPECT_GT(after.Mass(), 0.);
  EXPECT_LT(after.Mass(), before.Mass());
}

INSTANTIATE_TEST_SUITE_P(PolygonalTrays,
                         BRepFilletAPI_RimVariants,
                         ::testing::Combine(::testing::Values(3, 4, 5, 6, 8, 12),
                                            ::testing::Values(0, 1, 2),
                                            ::testing::Values(.1, 1., 10.),
                                            ::testing::Values(0, 1)));
