// Copyright (c) 2026 OPEN CASCADE SAS
// SPDX-License-Identifier: LGPL-2.1-only WITH OCCT-exception-1.0

#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Check.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
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

class BRepFilletAPI_ChamferCurvedSupports
    : public testing::TestWithParam<std::tuple<int, bool, bool>>
{
};

TEST_P(BRepFilletAPI_ChamferCurvedSupports, NoPlanarSupportAndReferenceSwapIsEquivalent)
{
  const auto [kind, unequal, seamShift] = GetParam();
  const gp_Dir x                        = seamShift ? gp_Dir(0, 1, 0) : gp_Dir(1, 0, 0);
  const gp_Ax2 bottom(gp_Pnt(), gp_Dir(0, 0, 1), x), top(gp_Pnt(0, 0, 10), gp_Dir(0, 0, 1), x);
  TopoDS_Shape lower, upper;
  if (kind == 0)
  {
    lower = BRepPrimAPI_MakeCylinder(bottom, 8., 10.);
    upper = BRepPrimAPI_MakeCone(top, 8., 3., 10.);
  }
  else if (kind == 1)
  {
    lower = BRepPrimAPI_MakeCone(bottom, 10., 8., 10.);
    upper = BRepPrimAPI_MakeCone(top, 8., 3., 10.);
  }
  else
  {
    lower = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -12), gp_Dir(0, 0, 1), x), 8., 16.);
    upper = BRepPrimAPI_MakeSphere(gp_Ax2(gp_Pnt(0, 0, 4), gp_Dir(0, 0, 1), x), 10.);
  }
  BRepAlgoAPI_Fuse fuse(lower, upper);
  ASSERT_TRUE(fuse.IsDone());
  const auto input = fuse.Shape();
  ASSERT_TRUE(BRepAlgoAPI_Check(input).IsValid());
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
    map;
  TopExp::MapShapesAndAncestors(input, TopAbs_EDGE, TopAbs_FACE, map);
  TopoDS_Edge edge;
  TopoDS_Face a, b;
  for (int i = 1; i <= map.Extent(); ++i)
  {
    const auto& faces = map.FindFromIndex(i);
    if (faces.Size() != 2 || faces.First().IsSame(faces.Last()))
    {
      continue;
    }
    const auto first = TopoDS::Face(faces.First()), second = TopoDS::Face(faces.Last());
    if (BRepAdaptor_Surface(first).GetType() == GeomAbs_Plane
        || BRepAdaptor_Surface(second).GetType() == GeomAbs_Plane)
    {
      continue;
    }
    edge = TopoDS::Edge(map.FindKey(i));
    a    = first;
    b    = second;
    break;
  }
  ASSERT_FALSE(edge.IsNull()) << "fixture must exercise two genuinely curved support faces";
  TopoDS_Shape results[2];
  for (int order = 0; order < 2; ++order)
  {
    BRepFilletAPI_MakeChamfer chamfer(input);
    if (unequal)
    {
      chamfer.Add(order ? .3 : .2, order ? .2 : .3, edge, order ? b : a);
    }
    else
    {
      chamfer.Add(.2, order ? TopoDS::Edge(edge.Reversed()) : edge);
    }
    chamfer.Build();
    ASSERT_TRUE(chamfer.IsDone());
    results[order] = chamfer.Shape();
    EXPECT_TRUE(BRepCheck_Analyzer(results[order], true, false, true).IsValid());
    EXPECT_TRUE(BRepAlgoAPI_Check(results[order]).IsValid());
    GProp_GProps before, after;
    BRepGProp::VolumePropertiesGK(input, before, 1.e-8, true, true);
    BRepGProp::VolumePropertiesGK(results[order], after, 1.e-8, true, true);
    EXPECT_GT(std::abs(before.Mass() - after.Mass()), 1.e-4);
    for (TopExp_Explorer it(results[order], TopAbs_EDGE); it.More(); it.Next())
    {
      EXPECT_LE(BRep_Tool::Tolerance(TopoDS::Edge(it.Current())), 2.e-4);
    }
  }
  for (int order = 0; order < 2; ++order)
  {
    BRepAlgoAPI_Cut difference(results[order], results[1 - order]);
    ASSERT_TRUE(difference.IsDone());
    GProp_GProps props;
    BRepGProp::VolumeProperties(difference.Shape(), props);
    EXPECT_NEAR(props.Mass(), 0., 1.e-5);
  }
  if (const char* directory = std::getenv("CHAMFER_COVERAGE_OUTPUT"))
  {
    const std::string stem = std::string(directory) + "/curved-supports-" + std::to_string(kind)
                             + "-" + std::to_string(unequal) + "-" + std::to_string(seamShift);
    BRepTools::Write(input, (stem + "-input.brep").c_str());
    BRepTools::Write(results[0], (stem + "-result.brep").c_str());
  }
}

INSTANTIATE_TEST_SUITE_P(AnalyticPairs,
                         BRepFilletAPI_ChamferCurvedSupports,
                         testing::Combine(testing::Range(0, 3), testing::Bool(), testing::Bool()));
