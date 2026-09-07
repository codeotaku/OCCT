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

#include <BRepAlgoAPI_Check.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>
#include <string>
#include <tuple>

class BRepFilletAPI_ChamferMultiWire : public testing::TestWithParam<std::tuple<int, double, int>>
{
};

TEST_P(BRepFilletAPI_ChamferMultiWire, Build_MultiWireConsumedRims_PreservesUnaffectedMaterial)
{
  const auto [holes, distance, placement] = GetParam();
  const double length                     = 1. + 11. * holes;
  TopoDS_Shape source                     = BRepPrimAPI_MakeBox(length, 10., 4.);
  for (int i = 0; i < holes; ++i)
  {
    BRepAlgoAPI_Cut cut(source, BRepPrimAPI_MakeBox(gp_Pnt(1. + 11. * i, 1., 1.), 10., 8., 4.));
    ASSERT_TRUE(cut.IsDone());
    source = cut.Shape();
  }
  ASSERT_TRUE(BRepCheck_Analyzer(source, true, false, true).IsValid());
  ASSERT_TRUE(BRepAlgoAPI_Check(source).IsValid());
  gp_Trsf transform;
  if (placement == 1)
  {
    transform.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 2, 3)), .731);
    transform.SetTranslationPart(gp_Vec(13, -17, 23));
  }
  else if (placement == 2)
  {
    transform.SetMirror(gp_Ax2(gp_Pnt(), gp_Dir(1, 2, 3)));
  }
  BRepBuilderAPI_Transform                                      placed(source, transform, true);
  const auto                                                    input = placed.Shape();
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> edges;
  TopExp::MapShapes(source, TopAbs_EDGE, edges);
  BRepFilletAPI_MakeChamfer chamfer(input);
  int                       selected = 0;
  for (const auto& shape : edges)
  {
    TopoDS_Vertex a, b;
    TopExp::Vertices(TopoDS::Edge(shape), a, b);
    if (std::abs(BRep_Tool::Pnt(a).Z() - 4.) < 1.e-7
        && std::abs(BRep_Tool::Pnt(b).Z() - 4.) < 1.e-7)
    {
      chamfer.Add(distance, TopoDS::Edge(placed.ModifiedShape(shape)));
      ++selected;
    }
  }
  ASSERT_EQ(selected, 4 * (holes + 1));
  chamfer.Build();
  if (distance > .5)
  {
    EXPECT_FALSE(chamfer.IsDone());
    return;
  }
  ASSERT_TRUE(chamfer.IsDone());
  const auto result = chamfer.Shape();
  EXPECT_TRUE(BRepCheck_Analyzer(result, true, false, true).IsValid());
  EXPECT_TRUE(BRepAlgoAPI_Check(result).IsValid());
  int solids = 0, shells = 0;
  for (TopExp_Explorer it(result, TopAbs_SOLID); it.More(); it.Next())
  {
    ++solids;
  }
  for (TopExp_Explorer it(result, TopAbs_SHELL); it.More(); it.Next())
  {
    ++shells;
    EXPECT_TRUE(BRep_Tool::IsClosed(it.Current()));
  }
  EXPECT_EQ(solids, 1);
  EXPECT_EQ(shells, 1);
  GProp_GProps before, after;
  BRepGProp::VolumeProperties(input, before);
  BRepGProp::VolumeProperties(result, after);
  EXPECT_GT(after.Mass(), 0.);
  EXPECT_LT(after.Mass(), before.Mass() - 1.e-3);
  for (TopExp_Explorer it(result, TopAbs_EDGE); it.More(); it.Next())
  {
    EXPECT_LE(BRep_Tool::Tolerance(TopoDS::Edge(it.Current())), 2.e-4);
  }
  // Removing complete top wires must not consume the common floor or lower walls.
  const auto lower =
    BRepBuilderAPI_Transform(BRepPrimAPI_MakeBox(length, 10., 3.), transform, true).Shape();
  BRepAlgoAPI_Common beforeLower(input, lower), afterLower(result, lower);
  ASSERT_TRUE(beforeLower.IsDone());
  ASSERT_TRUE(afterLower.IsDone());
  for (bool reverse : {false, true})
  {
    BRepAlgoAPI_Cut difference(reverse ? beforeLower.Shape() : afterLower.Shape(),
                               reverse ? afterLower.Shape() : beforeLower.Shape());
    ASSERT_TRUE(difference.IsDone());
    BRepGProp::VolumeProperties(difference.Shape(), after);
    EXPECT_NEAR(after.Mass(), 0., 1.e-7);
  }
  if (const char* directory = std::getenv("CHAMFER_COVERAGE_OUTPUT"))
  {
    const std::string stem = std::string(directory) + "/multiwire-" + std::to_string(holes) + "-"
                             + std::to_string(distance) + "-" + std::to_string(placement);
    BRepTools::Write(input, (stem + "-input.brep").c_str());
    BRepTools::Write(result, (stem + "-result.brep").c_str());
  }
}

INSTANTIATE_TEST_SUITE_P(SharedFaceWithHoles,
                         BRepFilletAPI_ChamferMultiWire,
                         testing::Combine(testing::Values(1, 2, 3),
                                          testing::Values(.499, .5, .501),
                                          testing::Range(0, 3)));
