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

#include <ChFi3d_Builder_0.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2dInt_GInter.hxx>
#include <Geom2d_BezierCurve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <NCollection_Array1.hxx>
#include <IntRes2d_IntersectionSegment.hxx>
#include <gtest/gtest.h>
#include <tuple>

class ChFi3d_Contact : public testing::TestWithParam<std::tuple<int, bool, bool>>
{
};

TEST_P(ChFi3d_Contact, DistinguishesCrossingTangencyAndBoundedEndpoints)
{
  const auto [kind, reversed, swapped] = GetParam();
  occ::handle<Geom2d_Curve> first =
    new Geom2d_TrimmedCurve(new Geom2d_Line(gp_Pnt2d(), gp_Dir2d(1, 0)), -5., 5.);
  occ::handle<Geom2d_Curve> second;
  if (kind < 3)
  {
    second =
      new Geom2d_TrimmedCurve(new Geom2d_Line(gp_Pnt2d(kind == 2 ? 5. : 0., 0), gp_Dir2d(0, 1)),
                              kind == 0 ? -5. : 0.,
                              5.);
  }
  else if (kind < 5)
  {
    second = new Geom2d_TrimmedCurve(new Geom2d_Line(gp_Pnt2d(), gp_Dir2d(1, 0)),
                                     kind == 3 ? 5. : 0.,
                                     10.);
  }
  else
  {
    const double                 offset = kind == 5 ? 0. : (kind == 6 ? 1.e-4 : -1.e-4);
    NCollection_Array1<gp_Pnt2d> poles(1, 3);
    poles(1) = gp_Pnt2d(-5, 25 + offset);
    poles(2) = gp_Pnt2d(0, -25 + offset);
    poles(3) = gp_Pnt2d(5, 25 + offset);
    second   = new Geom2d_BezierCurve(poles);
  }
  if (reversed)
  {
    second->Reverse();
  }
  const Geom2dAdaptor_Curve a(swapped ? second : first), b(swapped ? first : second);
  Geom2dInt_GInter          inter(a, b, 1.e-8, 1.e-8);
  ASSERT_TRUE(inter.IsDone());
  EXPECT_EQ(ChFi3d_HasTransversalIntersection(inter), kind == 0 || kind == 7);
  bool       firstStart = false, secondStart = false;
  const bool endpoint = ChFi3d_HasCommonEndpoint(inter, a, b, 1.e-7, firstStart, secondStart);
  EXPECT_EQ(endpoint, kind == 2 || kind == 3);
  if (endpoint)
  {
    const auto pa = a.Value(firstStart ? a.FirstParameter() : a.LastParameter());
    const auto pb = b.Value(secondStart ? b.FirstParameter() : b.LastParameter());
    EXPECT_LE(pa.Distance(pb), 1.e-7);
    EXPECT_LE(pa.Distance(gp_Pnt2d(5, 0)), 1.e-7);
  }
  if (kind == 6)
  {
    EXPECT_EQ(inter.NbPoints(), 0);
    EXPECT_EQ(inter.NbSegments(), 0);
  }
}

INSTANTIATE_TEST_SUITE_P(BoundedCurves,
                         ChFi3d_Contact,
                         testing::Combine(testing::Range(0, 8), testing::Bool(), testing::Bool()));

namespace
{
class ContactRecords : public Geom2dInt_GInter
{
public:
  ContactRecords() { done = true; }

  using IntRes2d_Intersection::Append;
};
} // namespace

TEST(ChFi3d_ContactRecords, TransitionDecisionTable)
{
  // A transition can be undecided when derivatives do not determine a tangent.
  // Test the predicate's contract independently of how an intersector arrived
  // at that record; no malformed upstream solid is used as a fixture.
  for (auto position1 : {IntRes2d_Head, IntRes2d_Middle, IntRes2d_End})
  {
    for (auto position2 : {IntRes2d_Head, IntRes2d_Middle, IntRes2d_End})
    {
      for (int type1 = 0; type1 < 4; ++type1)
      {
        for (int type2 = 0; type2 < 4; ++type2)
        {
          const auto transition = [](int type, IntRes2d_Position position) {
            if (type < 2)
            {
              return IntRes2d_Transition(false, position, type == 0 ? IntRes2d_In : IntRes2d_Out);
            }
            if (type == 2)
            {
              return IntRes2d_Transition(true, position, IntRes2d_Unknown, false);
            }
            return IntRes2d_Transition(position);
          };
          ContactRecords records;
          records.Append(IntRes2d_IntersectionPoint(gp_Pnt2d(),
                                                    0.,
                                                    0.,
                                                    transition(type1, position1),
                                                    transition(type2, position2),
                                                    false));
          EXPECT_EQ(ChFi3d_HasTransversalIntersection(records),
                    position1 == IntRes2d_Middle && position2 == IntRes2d_Middle
                      && (type1 < 2 || type2 < 2));
        }
      }
    }
  }
}

TEST(ChFi3d_ContactRecords, SegmentEndpointDecisionTable)
{
  const occ::handle<Geom2d_Curve> curve = new Geom2d_Line(gp_Pnt2d(), gp_Dir2d(1, 0));
  for (int kind = 0; kind < 8; ++kind)
  {
    ContactRecords                   records;
    const IntRes2d_Transition        transition;
    const IntRes2d_IntersectionPoint start(gp_Pnt2d(), 0., 0., transition, transition, false);
    if (kind == 0)
    {
      records.Append(IntRes2d_IntersectionSegment(false));
    }
    if (kind == 1 || kind == 2)
    {
      records.Append(IntRes2d_IntersectionSegment(start, kind == 1, false, false));
    }
    // Check the reported interval against the caller's parametric tolerance
    // on both curves, not only on the first one.
    const double last2 = kind == 3 ? 4.e-7 : 0.;
    if (kind >= 3)
    {
      records.Append(IntRes2d_IntersectionSegment(
        start,
        IntRes2d_IntersectionPoint(gp_Pnt2d(), 0., last2, transition, transition, false),
        false,
        false));
    }
    const Geom2dAdaptor_Curve first(curve, kind == 4 || kind == 7 ? -1. : 0., kind == 7 ? 0. : 1.);
    const Geom2dAdaptor_Curve second(curve, kind == 5 ? -1. : 0., 1.);
    bool                      firstStart = false, secondStart = false;
    EXPECT_EQ(ChFi3d_HasCommonEndpoint(records, first, second, 1.e-7, firstStart, secondStart),
              kind >= 6);
    if (kind >= 6)
    {
      EXPECT_EQ(firstStart, kind == 6);
      EXPECT_TRUE(secondStart);
    }
  }
}
