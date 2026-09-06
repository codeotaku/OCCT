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

#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2dInt_GInter.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_BezierCurve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2d_BSplineCurve.hxx>
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
  constexpr double tolerance            = 1.e-6;
  const double     parameterTolerance   = Geom2dAdaptor_Curve(curve).Resolution(tolerance);
  const double     shortening           = relation == 0   ? 0.
                                          : relation == 1 ? .1
                                          : relation == 2 ? .5 * parameterTolerance
                                                          : 4 * parameterTolerance;
  const occ::handle<Geom2d_Curve> first = new Geom2d_TrimmedCurve(curve, .2, .8);
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

class TopOpeBRepBuild_CoincidenceParameterization
    : public testing::TestWithParam<std::tuple<double, bool, int, bool, bool>>
{
};

TEST_P(TopOpeBRepBuild_CoincidenceParameterization, CoverageDependsOnGeometryNotParameterScale)
{
  const auto [aRange, isNonlinear, aRelation, isReversed, isSwapped] = GetParam();
  for (const double aTolerance : {Precision::PConfusion(), 1.e-7})
  {
    const double aLength2 = aRelation == 0 ? 10. : aRelation == 1 ? 9. : 10. - .25 * aTolerance;
    const auto   makeSegment =
      [](const double theLength, const bool theNonlinear, const double theRange) {
        const int                    aDegree = theNonlinear ? 3 : 1;
        NCollection_Array1<gp_Pnt2d> aPoles(1, aDegree + 1);
        for (int i = 1; i <= aDegree + 1; ++i)
        {
          aPoles(i) = gp_Pnt2d(i == aDegree + 1 ? theLength : 0., 0.);
        }
        NCollection_Array1<double> aKnots(1, 2);
        aKnots(1) = 0.;
        aKnots(2) = theRange;
        NCollection_Array1<int> aMultiplicities(1, 2);
        aMultiplicities(1) = aMultiplicities(2) = aDegree + 1;
        return new Geom2d_BSplineCurve(aPoles, aKnots, aMultiplicities, aDegree);
      };
    const occ::handle<Geom2d_Curve> aFirst  = makeSegment(10., false, aRange);
    const occ::handle<Geom2d_Curve> aSecond = makeSegment(aLength2, isNonlinear, aRange);
    if (isReversed)
      aSecond->Reverse();
    const Geom2dAdaptor_Curve a(isSwapped ? aSecond : aFirst), b(isSwapped ? aFirst : aSecond);
    // Exact contact record, independent of the intersector's treatment of very short
    // parameter ranges. The straight-curve control below also runs the actual intersector.
    const double             aEndOnFirst    = aLength2 / 10. * aRange;
    const double             aStartOnSecond = isReversed ? aRange : 0.;
    const double             aEndOnSecond   = isReversed ? 0. : aRange;
    const CachedIntersection anIntersection(a,
                                            isSwapped ? aStartOnSecond : 0.,
                                            isSwapped ? aEndOnSecond : aEndOnFirst,
                                            isSwapped ? 0. : aStartOnSecond,
                                            isSwapped ? aEndOnFirst : aEndOnSecond,
                                            isReversed);
    ASSERT_TRUE(anIntersection.IsDone());
    bool       isOpposite = false;
    const bool isComplete =
      TopOpeBRepBuild_Tools::HasCompleteCoincidence(anIntersection, a, b, aTolerance, isOpposite);
    EXPECT_EQ(isComplete, aRelation != 1);
    if (isComplete)
      EXPECT_EQ(isOpposite, isReversed);
  }
}

INSTANTIATE_TEST_SUITE_P(GeometricRanges,
                         TopOpeBRepBuild_CoincidenceParameterization,
                         testing::Combine(testing::Values(1., 1.e-4, 1.e-6, 1.e-7, 1.e-8, 5.e-9),
                                          testing::Bool(),
                                          testing::Range(0, 3),
                                          testing::Bool(),
                                          testing::Bool()));

TEST(TopOpeBRepBuild_CompleteCoincidenceControl, ActualIntersectionRetainsUnmatchedTail)
{
  for (double aRange : {1., 1.e-4, 1.e-6, 1.e-7, 1.e-8, 5.e-9})
    for (double aTolerance : {Precision::PConfusion(), 1.e-7})
    {
      NCollection_Array1<double> aKnots(1, 2);
      aKnots(1) = 0.;
      aKnots(2) = aRange;
      NCollection_Array1<int> aMultiplicities(1, 2);
      aMultiplicities(1) = aMultiplicities(2) = 2;
      NCollection_Array1<gp_Pnt2d> aPoles(1, 2);
      aPoles(1) = gp_Pnt2d();
      aPoles(2) = gp_Pnt2d(10, 0);
      const Geom2dAdaptor_Curve a(new Geom2d_BSplineCurve(aPoles, aKnots, aMultiplicities, 1));
      aPoles(2) = gp_Pnt2d(9, 0);
      const Geom2dAdaptor_Curve b(new Geom2d_BSplineCurve(aPoles, aKnots, aMultiplicities, 1));
      const Geom2dInt_GInter    anIntersection(a, b, aTolerance, aTolerance);
      ASSERT_TRUE(anIntersection.IsDone());
      ASSERT_EQ(anIntersection.NbSegments(), 1);
      bool isReversed = false;
      EXPECT_FALSE(TopOpeBRepBuild_Tools::HasCompleteCoincidence(anIntersection,
                                                                 a,
                                                                 b,
                                                                 aTolerance,
                                                                 isReversed));
    }
}

TEST(TopOpeBRepBuild_CompleteCoincidenceControl, DirectionUsesStoredIntersectionOrientation)
{
  // A nonzero geometric segment can have a parameter span whose square underflows.
  // Test cached contact records, not the intersector's extreme-parameter behavior.
  for (const double aRange : {1., 1.e-100, 1.e-170})
    for (const bool isReversed : {false, true})
      for (const bool isSwapped : {false, true})
      {
        SCOPED_TRACE(testing::Message() << aRange << ", " << isReversed << ", " << isSwapped);
        NCollection_Array1<gp_Pnt2d> aPoles(1, 2);
        aPoles(1) = gp_Pnt2d();
        aPoles(2) = gp_Pnt2d(10., 0.);
        NCollection_Array1<double> aKnots(1, 2);
        aKnots(1) = 0.;
        aKnots(2) = aRange;
        NCollection_Array1<int> aMultiplicities(1, 2);
        aMultiplicities.Init(2);
        const occ::handle<Geom2d_Curve> aFirst =
          new Geom2d_BSplineCurve(aPoles, aKnots, aMultiplicities, 1);
        const auto aSecond = occ::down_cast<Geom2d_Curve>(aFirst->Copy());
        if (isReversed)
          aSecond->Reverse();
        const Geom2dAdaptor_Curve a(isSwapped ? aSecond : aFirst), b(isSwapped ? aFirst : aSecond);
        ASSERT_NEAR(a.Value(0.).Distance(a.Value(aRange)), 10., 1.e-12);
        const CachedIntersection anIntersection(a,
                                                0.,
                                                aRange,
                                                isReversed ? aRange : 0.,
                                                isReversed ? 0. : aRange,
                                                isReversed);
        bool                     isOpposite = !isReversed;
        ASSERT_TRUE(
          TopOpeBRepBuild_Tools::HasCompleteCoincidence(anIntersection, a, b, 1.e-7, isOpposite));
        EXPECT_EQ(isOpposite, isReversed);
      }
}
