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
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <tuple>

class BRepFilletAPI_ChamferTolerance
    : public testing::TestWithParam<std::tuple<int, double, double>>
{
};

TEST_P(BRepFilletAPI_ChamferTolerance, Build_SharedVertices_DoesNotAccumulateTolerancePadding)
{
  const auto [edgeCount, scale, tolerance] = GetParam();
  const auto source = BRepPrimAPI_MakeBox(20. * scale, 20. * scale, 2. * scale).Shape();
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> vertices, edges;
  TopExp::MapShapes(source, TopAbs_VERTEX, vertices);
  TopExp::MapShapes(source, TopAbs_EDGE, edges);
  for (const auto& vertex : vertices)
    BRep_Builder().UpdateVertex(TopoDS::Vertex(vertex), tolerance);
  ASSERT_TRUE(BRepCheck_Analyzer(source).IsValid());
  ASSERT_TRUE(BRepAlgoAPI_Check(source).IsValid());
  BRepFilletAPI_MakeChamfer chamfer(source);
  int                       selected = 0;
  for (const auto& shape : edges)
  {
    const auto        edge = TopoDS::Edge(shape);
    BRepAdaptor_Curve curve(edge);
    if (std::abs(curve.Value(curve.FirstParameter()).Z() - 2. * scale) < 1.e-7
        && std::abs(curve.Value(curve.LastParameter()).Z() - 2. * scale) < 1.e-7
        && selected < edgeCount)
    {
      chamfer.Add(2. * scale, edge);
      ++selected;
    }
  }
  ASSERT_EQ(selected, edgeCount);
  chamfer.Build();
  ASSERT_TRUE(chamfer.IsDone());
  ASSERT_TRUE(BRepCheck_Analyzer(chamfer.Shape(), true, false, true).IsValid());
  ASSERT_TRUE(BRepAlgoAPI_Check(chamfer.Shape()).IsValid());
  vertices.Clear();
  TopExp::MapShapes(chamfer.Shape(), TopAbs_VERTEX, vertices);
  for (const auto& vertex : vertices)
    EXPECT_LE(BRep_Tool::Tolerance(TopoDS::Vertex(vertex)), 1.01 * std::max(1.e-4, tolerance));
}

INSTANTIATE_TEST_SUITE_P(IncidenceAndUnits,
                         BRepFilletAPI_ChamferTolerance,
                         testing::Combine(testing::Values(1, 2, 4),
                                          testing::Values(.5, 1., 2.),
                                          testing::Values(9.e-5, 1.1e-4, 5.e-4)));
