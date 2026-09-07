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
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
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
#include <vector>

class BRepFilletAPI_ChamferAsymmetricLimit
    : public testing::TestWithParam<std::tuple<double, bool, bool, double, int>>
{
};

TEST_P(BRepFilletAPI_ChamferAsymmetricLimit,
       Build_AsymmetricFaceConsumption_AgreesWithIndependentPrism)
{
  const auto [heightCut, angleAPI, swapped, fraction, placement] = GetParam();
  const TopoDS_Shape source = BRepPrimAPI_MakeBox(10., 30., 6.);
  ASSERT_TRUE(BRepAlgoAPI_Check(source).IsValid());
  TopoDS_Edge edge;
  TopoDS_Face top, side;
  for (TopExp_Explorer it(source, TopAbs_EDGE); it.More(); it.Next())
  {
    TopoDS_Vertex a, b;
    TopExp::Vertices(TopoDS::Edge(it.Current()), a, b);
    const auto p = BRep_Tool::Pnt(a), q = BRep_Tool::Pnt(b);
    if (std::abs(p.X()) < 1.e-7 && std::abs(q.X()) < 1.e-7 && std::abs(p.Z() - 6.) < 1.e-7
        && std::abs(q.Z() - 6.) < 1.e-7)
    {
      edge = TopoDS::Edge(it.Current());
    }
  }
  for (TopExp_Explorer it(source, TopAbs_FACE); it.More(); it.Next())
  {
    GProp_GProps props;
    BRepGProp::SurfaceProperties(it.Current(), props);
    if (std::abs(props.CentreOfMass().Z() - 6.) < 1.e-7)
    {
      top = TopoDS::Face(it.Current());
    }
    if (std::abs(props.CentreOfMass().X()) < 1.e-7)
    {
      side = TopoDS::Face(it.Current());
    }
  }
  ASSERT_FALSE(edge.IsNull());
  ASSERT_FALSE(top.IsNull());
  ASSERT_FALSE(side.IsNull());
  gp_Trsf transform;
  if (placement == 1)
  {
    transform.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 2, 3)), .731);
    transform.SetTranslationPart(gp_Vec(17, -31, 53));
  }
  else if (placement == 2)
  {
    transform.SetMirror(gp_Ax2(gp_Pnt(), gp_Dir(1, 2, 3)));
  }
  BRepBuilderAPI_Transform  placed(source, transform, true);
  const auto                input     = placed.Shape();
  const auto                selected  = TopoDS::Edge(placed.ModifiedShape(edge));
  const auto                reference = TopoDS::Face(placed.ModifiedShape(swapped ? side : top));
  const double              dx = 10. * fraction, dz = heightCut * fraction;
  BRepFilletAPI_MakeChamfer chamfer(input);
  if (angleAPI)
  {
    chamfer.AddDA(swapped ? dz : dx, std::atan(swapped ? dx / dz : dz / dx), selected, reference);
  }
  else
  {
    chamfer.Add(swapped ? dz : dx, swapped ? dx : dz, selected, reference);
  }
  chamfer.Build();
  if (fraction > 1.)
  {
    EXPECT_FALSE(chamfer.IsDone()) << "a cut beyond the opposite support is not this chamfer";
    return;
  }
  ASSERT_TRUE(chamfer.IsDone());
  const auto result = chamfer.Shape();
  EXPECT_TRUE(BRepCheck_Analyzer(result, true, false, true).IsValid());
  EXPECT_TRUE(BRepAlgoAPI_Check(result).IsValid());
  // Independent analytic section, not another invocation of the chamfer builder.
  std::vector<gp_Pnt>        points = {gp_Pnt(0, 0, 0),
                                       gp_Pnt(10, 0, 0),
                                       gp_Pnt(10, 0, 6),
                                       gp_Pnt(dx, 0, 6),
                                       gp_Pnt(0, 0, 6 - dz)};
  BRepBuilderAPI_MakePolygon polygon;
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    if ((i == 0 || points[i].Distance(points[i - 1]) > 1.e-7)
        && (i == 0 || points[i].Distance(points[0]) > 1.e-7))
    {
      polygon.Add(points[i]);
    }
  }
  polygon.Close();
  const TopoDS_Shape expected =
    BRepBuilderAPI_Transform(
      BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(polygon.Wire()), gp_Vec(0, 30, 0)),
      transform,
      true)
      .Shape();
  ASSERT_TRUE(BRepAlgoAPI_Check(expected).IsValid());
  GProp_GProps props;
  BRepGProp::VolumeProperties(result, props);
  EXPECT_NEAR(props.Mass(), 30. * (60. - .5 * dx * dz), 1.e-6);
  for (bool reverse : {false, true})
  {
    BRepAlgoAPI_Cut difference(reverse ? expected : result, reverse ? result : expected);
    ASSERT_TRUE(difference.IsDone());
    BRepGProp::VolumeProperties(difference.Shape(), props);
    EXPECT_NEAR(props.Mass(), 0., 1.e-6);
  }
  for (TopExp_Explorer it(result, TopAbs_EDGE); it.More(); it.Next())
  {
    EXPECT_LE(BRep_Tool::Tolerance(TopoDS::Edge(it.Current())), 2.e-4);
  }
  if (const char* directory = std::getenv("CHAMFER_COVERAGE_OUTPUT"))
  {
    const std::string stem = std::string(directory) + "/asymmetric-" + std::to_string(heightCut)
                             + "-" + std::to_string(angleAPI) + "-" + std::to_string(swapped) + "-"
                             + std::to_string(fraction) + "-" + std::to_string(placement);
    BRepTools::Write(input, (stem + "-input.brep").c_str());
    BRepTools::Write(result, (stem + "-result.brep").c_str());
    BRepTools::Write(expected, (stem + "-expected.brep").c_str());
  }
}

INSTANTIATE_TEST_SUITE_P(AnalyticSections,
                         BRepFilletAPI_ChamferAsymmetricLimit,
                         testing::Combine(testing::Values(2., 4., 6.),
                                          testing::Bool(),
                                          testing::Bool(),
                                          testing::Values(.999, 1., 1.001),
                                          testing::Range(0, 3)));
