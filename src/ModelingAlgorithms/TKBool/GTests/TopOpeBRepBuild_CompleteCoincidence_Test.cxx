// Copyright (c) 2026 OPEN CASCADE SAS
// SPDX-License-Identifier: LGPL-2.1-only WITH OCCT-exception-1.0

#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2dInt_GInter.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_BezierCurve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <TopOpeBRepBuild_Tools.hxx>
#include <IntRes2d_IntersectionSegment.hxx>
#include <gtest/gtest.h>
#include <tuple>

class TopOpeBRepBuild_CompleteCoincidence
    : public testing::TestWithParam<std::tuple<int, int, bool, bool>>
{
};

namespace
{
// Exercise the range predicate independently of intersection approximation.
// The geometric intersector itself is covered by the contact/closed-tangency
// tests; it is not an exact oracle for sub-tolerance endpoint parameters.
class CachedIntersection : public Geom2dInt_GInter
{
public:
  CachedIntersection() { done = true; }

  using IntRes2d_Intersection::Append;

  CachedIntersection(const Geom2dAdaptor_Curve& a,
                     double                     af,
                     double                     al,
                     double                     bf,
                     double                     bl,
                     bool                       opposite)
  {
    done = true;
    const IntRes2d_Transition transition;
    Append(IntRes2d_IntersectionSegment(
      IntRes2d_IntersectionPoint(a.Value(af), af, bf, transition, transition, false),
      IntRes2d_IntersectionPoint(a.Value(al), al, bl, transition, transition, false),
      opposite,
      false));
  }
};
} // namespace

TEST_P(TopOpeBRepBuild_CompleteCoincidence, CoversBothRangesWithinCallerTolerance)
{
  const auto [kind, relation, reversed, swapped] = GetParam();
  occ::handle<Geom2d_Curve> curve;
  if (kind == 0)
  {
    curve = new Geom2d_Line(gp_Pnt2d(), gp_Dir2d(1, 0));
  }
  else if (kind == 1)
  {
    curve = new Geom2d_Circle(gp_Ax2d(gp_Pnt2d(), gp_Dir2d(1, 0)), 5.);
  }
  else
  {
    NCollection_Array1<gp_Pnt2d> poles(1, 4);
    poles(1) = gp_Pnt2d(0, 0);
    poles(2) = gp_Pnt2d(2, 4);
    poles(3) = gp_Pnt2d(8, -2);
    poles(4) = gp_Pnt2d(10, 0);
    curve    = new Geom2d_BezierCurve(poles);
  }
  constexpr double                tolerance  = 1.e-6;
  const double                    shortening = relation == 0   ? 0.
                                               : relation == 1 ? .1
                                               : relation == 2 ? .5 * tolerance
                                                               : 4 * tolerance;
  const occ::handle<Geom2d_Curve> first      = new Geom2d_TrimmedCurve(curve, .2, .8);
  const occ::handle<Geom2d_Curve> second =
    new Geom2d_TrimmedCurve(curve, .2 + shortening, .8 - shortening);
  if (reversed)
  {
    second->Reverse();
  }
  const Geom2dAdaptor_Curve a(swapped ? second : first), b(swapped ? first : second);
  const double              f = .2 + shortening, l = .8 - shortening;
  const double              af = swapped ? a.FirstParameter() : f;
  const double              al = swapped ? a.LastParameter() : l;
  const double              bf =
    swapped ? (reversed ? l : f) : (reversed ? b.LastParameter() : b.FirstParameter());
  const double bl =
    swapped ? (reversed ? f : l) : (reversed ? b.FirstParameter() : b.LastParameter());
  CachedIntersection intersection(a, af, al, bf, bl, reversed);
  ASSERT_TRUE(intersection.IsDone());
  bool       direction = false;
  const bool result =
    TopOpeBRepBuild_Tools::HasCompleteCoincidence(intersection, a, b, tolerance, direction);
  EXPECT_EQ(result, relation == 0 || relation == 2);
  if (result)
  {
    EXPECT_EQ(direction, reversed);
  }
  // The very same intersection is not complete under a stricter caller policy.
  if (relation == 2)
  {
    EXPECT_FALSE(
      TopOpeBRepBuild_Tools::HasCompleteCoincidence(intersection, a, b, 1.e-9, direction));
  }
}

INSTANTIATE_TEST_SUITE_P(
  BoundedRanges,
  TopOpeBRepBuild_CompleteCoincidence,
  testing::Combine(testing::Range(0, 3), testing::Range(0, 4), testing::Bool(), testing::Bool()));

TEST(TopOpeBRepBuild_CompleteCoincidenceControl, IncompleteAndAmbiguousRecordsAreRejected)
{
  const occ::handle<Geom2d_Curve>  curve = new Geom2d_Line(gp_Pnt2d(), gp_Dir2d(1, 0));
  const Geom2dAdaptor_Curve        a(curve, 0., 1.);
  const IntRes2d_Transition        transition;
  const IntRes2d_IntersectionPoint p(gp_Pnt2d(), 0., 0., transition, transition, false);
  for (int kind = 0; kind < 5; ++kind)
  {
    CachedIntersection intersection;
    if (kind == 1)
    {
      intersection.Append(IntRes2d_IntersectionSegment(false));
    }
    if (kind == 2 || kind == 3)
    {
      intersection.Append(IntRes2d_IntersectionSegment(p, kind == 2, false, false));
    }
    if (kind == 4)
    {
      intersection.Append(IntRes2d_IntersectionSegment(false));
      intersection.Append(IntRes2d_IntersectionSegment(false));
    }
    bool reversed = false;
    EXPECT_FALSE(
      TopOpeBRepBuild_Tools::HasCompleteCoincidence(intersection, a, a, 1.e-7, reversed));
  }
}
