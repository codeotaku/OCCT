// Created on: 1993-06-14
// Created by: Jean Yves LEBEY
// Copyright (c) 1993-1999 Matra Datavision
// Copyright (c) 1999-2014 OPEN CASCADE SAS
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

#include <Standard_Integer.hxx>
#include <BOPAlgo_Builder.hxx>
#include <BOPTools_AlgoTools.hxx>
#include <IntTools_Context.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <NCollection_IndexedMap.hxx>
#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2dInt_GInter.hxx>
#include <IntRes2d_IntersectionSegment.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_HArray1.hxx>
#include <NCollection_LinearVector.hxx>
#include <Precision.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Shape.hxx>
#include <TopOpeBRepBuild_define.hxx>
#include <TopOpeBRepBuild_EdgeBuilder.hxx>
#include <TopOpeBRepBuild_Tools.hxx>
#include <TopOpeBRepBuild_PaveSet.hxx>
#include <TopOpeBRepDS_BuildTool.hxx>
#include <TopOpeBRepDS_Curve.hxx>
#include <TopOpeBRepDS_CurveExplorer.hxx>
#include <TopOpeBRepDS_CurveIterator.hxx>
#include <TopOpeBRepDS_HDataStructure.hxx>
#include <TopOpeBRepDS_PointIterator.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Standard_ConstructionError.hxx>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
struct TopOpeBRepBuild_SurfaceCurve
{
  int                       Index;
  occ::handle<Geom2d_Curve> PCurve;
  double                    FirstParameter;
  double                    LastParameter;
};

bool TopOpeBRepBuild_CurveRange(const TopOpeBRepDS_Curve&        theCurve,
                                const occ::handle<Geom2d_Curve>& thePCurve,
                                double&                          theFirst,
                                double&                          theLast)
{
  const auto isValidRange = [](const double theRangeFirst, const double theRangeLast) {
    return theRangeFirst < theRangeLast && !Precision::IsInfinite(theRangeFirst)
           && !Precision::IsInfinite(theRangeLast);
  };

  if (theCurve.Range(theFirst, theLast) && isValidRange(theFirst, theLast))
  {
    return true;
  }
  if (!thePCurve.IsNull())
  {
    theFirst = thePCurve->FirstParameter();
    theLast  = thePCurve->LastParameter();
    if (isValidRange(theFirst, theLast))
    {
      return true;
    }
  }

  const occ::handle<Geom_Curve>& aCurve3d = theCurve.Curve();
  if (!aCurve3d.IsNull())
  {
    theFirst = aCurve3d->FirstParameter();
    theLast  = aCurve3d->LastParameter();
    return isValidRange(theFirst, theLast);
  }
  return false;
}

bool TopOpeBRepBuild_FindCurveEnd(const occ::handle<TopOpeBRepDS_HDataStructure>& theHDS,
                                  const int                                       theCurveIndex,
                                  const bool                                      theCurveStart,
                                  int&                                            theGeometryIndex,
                                  bool&                                           theIsPoint)
{
  double aFirst, aLast;
  if (!TopOpeBRepBuild_CurveRange(theHDS->Curve(theCurveIndex), nullptr, aFirst, aLast))
  {
    return false;
  }

  const double aParameter    = theCurveStart ? aFirst : aLast;
  double       aBestDistance = RealLast();
  for (TopOpeBRepDS_PointIterator aPointIt(theHDS->CurvePoints(theCurveIndex)); aPointIt.More();
       aPointIt.Next())
  {
    const double aDistance = std::abs(aPointIt.Parameter() - aParameter);
    if (aDistance < aBestDistance)
    {
      aBestDistance    = aDistance;
      theGeometryIndex = aPointIt.Current();
      theIsPoint       = aPointIt.IsPoint();
    }
  }
  return aBestDistance <= Precision::PConfusion();
}

bool TopOpeBRepBuild_IsCoincidentRestriction(const TopOpeBRepDS_Curve& theCurve,
                                             const double              theFirst,
                                             const double              theLast,
                                             const TopoDS_Edge&        theRestriction,
                                             bool&                     theIsReversed)
{
  BRepBuilderAPI_MakeEdge aMaker(theCurve.Curve(), theFirst, theLast);
  if (!aMaker.IsDone())
  {
    return false;
  }
  const TopoDS_Edge aTrace = aMaker.Edge();
  BRep_Builder().UpdateEdge(aTrace, theCurve.Tolerance());
  return TopOpeBRepBuild_Tools::AreCoincidentEdges(aTrace, theRestriction, theIsReversed);
}
} // namespace

//=================================================================================================

void TopOpeBRepBuild_Builder::BuildEdges(const int                                       iC,
                                         const occ::handle<TopOpeBRepDS_HDataStructure>& HDS)
{
  const TopOpeBRepDS_Curve&                     C   = HDS->Curve(iC);
  const occ::handle<Geom_Curve>&                C3D = C.Curve();
  const occ::handle<TopOpeBRepDS_Interference>& I1  = C.GetSCI1();
  const occ::handle<TopOpeBRepDS_Interference>& I2  = C.GetSCI2();
  bool                                          nnn = C3D.IsNull() && I1.IsNull() && I2.IsNull();
  if (nnn)
  {
    return;
  }

  if (C.EquivalentCurve() > 0 && C.EquivalentCurve() != iC)
  {
    const int aReferenceCurve = C.EquivalentCurve();
    if (NewEdges(aReferenceCurve).IsEmpty())
    {
      BuildEdges(aReferenceCurve, HDS);
    }
    ChangeNewEdges(iC) = NewEdges(aReferenceCurve);
    return;
  }

  if (!C.ExistingEdge().IsNull())
  {
    if (!myCoincidentEdges.IsBound(C.ExistingEdge()))
    {
      TopoDS_Shape aCopiedEdge;
      myBuildTool.CopyEdge(C.ExistingEdge(), aCopiedEdge);
      for (TopExp_Explorer aVertexIt(C.ExistingEdge(), TopAbs_VERTEX); aVertexIt.More();
           aVertexIt.Next())
      {
        myBuildTool.AddEdgeVertex(C.ExistingEdge(), aCopiedEdge, aVertexIt.Current());
      }
      const auto anOriginal  = TopoDS::Edge(C.ExistingEdge().Oriented(TopAbs_FORWARD));
      const auto anOldVertex = TopExp::FirstVertex(anOriginal);
      if (!anOldVertex.IsNull() && anOldVertex.IsSame(TopExp::LastVertex(anOriginal)))
      {
        // Full coincidence of closed curves does not imply coincidence of
        // their seam vertices. Preserve all seams instead of moving a vertex
        // or projecting a whole edge across a surface's parameter cut.
        double                      aFirst, aLast;
        const auto                  aCurve = BRep_Tool::Curve(anOriginal, aFirst, aLast);
        GeomAPI_ProjectPointOnCurve aProjection;
        aProjection.Init(aCurve, aFirst, aLast);
        std::vector<std::pair<double, TopoDS_Vertex>> aCuts = {{aFirst, anOldVertex},
                                                               {aLast, anOldVertex}};
        for (TopOpeBRepDS_CurveExplorer aCurveIt(HDS->DS(), false); aCurveIt.More();
             aCurveIt.Next())
        {
          if (!aCurveIt.Curve().ExistingEdge().IsSame(anOriginal))
          {
            continue;
          }
          for (int anEnd = 0; anEnd < 2; ++anEnd)
          {
            int  aPointIndex = 0;
            bool isPoint     = false;
            if (!TopOpeBRepBuild_FindCurveEnd(HDS,
                                              aCurveIt.Index(),
                                              anEnd == 0,
                                              aPointIndex,
                                              isPoint)
                || !isPoint)
            {
              continue;
            }
            const auto aVertex = TopoDS::Vertex(NewVertex(aPointIndex));
            const auto aPoint  = BRep_Tool::Pnt(aVertex);
            bool       isKnown = false;
            for (const auto& aCut : aCuts)
            {
              if (aPoint.Distance(BRep_Tool::Pnt(aCut.second))
                  <= std::max(BRep_Tool::Tolerance(aVertex), BRep_Tool::Tolerance(aCut.second)))
              {
                ChangeNewVertex(aPointIndex) = aCut.second;
                isKnown                      = true;
                break;
              }
            }
            if (isKnown)
            {
              continue;
            }
            aProjection.Perform(aPoint);
            const double aTolerance = std::max({BRep_Tool::Tolerance(anOriginal),
                                                BRep_Tool::Tolerance(aVertex),
                                                aCurveIt.Curve().Tolerance()});
            if (!aProjection.NbPoints() || aProjection.LowerDistance() > aTolerance)
            {
              throw Standard_ConstructionError("BuildEdges: coincident seam projection failed");
            }
            aCuts.emplace_back(aProjection.LowerDistanceParameter(), aVertex);
          }
        }
        if (aCuts.size() > 2)
        {
          std::sort(aCuts.begin(), aCuts.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
          });
          TopoDS_Wire  aWire;
          BRep_Builder aBuilder;
          aBuilder.MakeWire(aWire);
          for (size_t i = 1; i < aCuts.size(); ++i)
          {
            TopoDS_Edge aSplit;
            BOPTools_AlgoTools::MakeSplitEdge(anOriginal,
                                              aCuts[i - 1].second,
                                              aCuts[i - 1].first,
                                              aCuts[i].second,
                                              aCuts[i].first,
                                              aSplit);
            aBuilder.Add(aWire, aSplit);
          }
          aCopiedEdge = aWire;
        }
      }
      myCoincidentEdges.Bind(C.ExistingEdge(), aCopiedEdge);
    }
    TopoDS_Shape anEdge = myCoincidentEdges(C.ExistingEdge());
    anEdge.Orientation(TopAbs_FORWARD);
    if (anEdge.ShapeType() == TopAbs_WIRE)
    {
      for (TopExp_Explorer it(anEdge, TopAbs_EDGE); it.More(); it.Next())
      {
        ChangeNewEdges(iC).Append(it.Current());
      }
    }
    else
    {
      ChangeNewEdges(iC).Append(anEdge);
    }
    return;
  }

  TopoDS_Shape              anEdge;
  const TopOpeBRepDS_Curve& curC = HDS->Curve(iC);
  myBuildTool.MakeEdge(anEdge, curC, HDS->DS());
  TopOpeBRepBuild_PaveSet    PVS(anEdge);
  TopOpeBRepDS_PointIterator CPIT(HDS->CurvePoints(iC));
  FillVertexSet(CPIT, TopAbs_IN, PVS);
  TopOpeBRepBuild_PaveClassifier VCL(anEdge);
  bool                           equalpar = PVS.HasEqualParameters();
  if (equalpar)
  {
    VCL.SetFirstParameter(PVS.EqualParameters());
  }
  bool closvert = PVS.ClosedVertices();
  VCL.ClosedVertices(closvert);
  PVS.InitLoop();
  if (!PVS.MoreLoop())
  {
    return;
  }
  TopOpeBRepBuild_EdgeBuilder     EDBU(PVS, VCL);
  NCollection_List<TopoDS_Shape>& EL = ChangeNewEdges(iC);
  MakeEdges(anEdge, EDBU, EL);
  NCollection_List<TopoDS_Shape>::Iterator It(EL);
  int                                      inewC = -1;
  for (; It.More(); It.Next())
  {
    TopoDS_Edge& newEdge = TopoDS::Edge(It.ChangeValue());
    myBuildTool.RecomputeCurves(curC, TopoDS::Edge(anEdge), newEdge, inewC, HDS);
    if (inewC != -1)
    {
      ChangeNewEdges(inewC).Append(newEdge);
    }
  }
  if (inewC != -1)
  {
    HDS->RemoveCurve(iC);
  }
  else
  {
    for (It.Initialize(EL); It.More(); It.Next())
    {
      TopoDS_Edge& newEdge = TopoDS::Edge(It.ChangeValue());
      myBuildTool.UpdateEdge(anEdge, newEdge);
    }
  }
}

//=================================================================================================

void TopOpeBRepBuild_Builder::BuildEdges(const occ::handle<TopOpeBRepDS_HDataStructure>& HDS)
{
  TopOpeBRepDS_DataStructure& BDS = HDS->ChangeDS();

  myNewEdges.Clear();
  myCoincidentEdges.Clear();
  TopOpeBRepDS_CurveExplorer cex;
  bool                       hasTraceContacts = false;

  NCollection_LinearVector<TopOpeBRepBuild_SurfaceCurve> aSurfaceCurves;
  const int                                              aNbSurfaces = HDS->NbSurfaces();
  const int                                              aNbShapes   = HDS->NbShapes();
  for (int aSurfaceIndex = 1; aSurfaceIndex <= aNbSurfaces + aNbShapes; ++aSurfaceIndex)
  {
    aSurfaceCurves.Clear();
    TopoDS_Face aSupport;
    if (aSurfaceIndex > aNbSurfaces)
    {
      const TopoDS_Shape& aShape = HDS->Shape(aSurfaceIndex - aNbSurfaces, false);
      if (aShape.IsNull() || aShape.ShapeType() != TopAbs_FACE)
      {
        continue;
      }
      aSupport = TopoDS::Face(aShape);
    }
    for (TopOpeBRepDS_CurveIterator aCurveIt(aSupport.IsNull() ? HDS->SurfaceCurves(aSurfaceIndex)
                                                               : HDS->FaceCurves(aSupport));
         aCurveIt.More();
         aCurveIt.Next())
    {
      const occ::handle<Geom2d_Curve>& aPCurve = aCurveIt.PCurve();
      const TopOpeBRepDS_Curve&        aCurve  = HDS->Curve(aCurveIt.Current());
      if (aPCurve.IsNull() || aCurve.Curve().IsNull())
      {
        continue;
      }

      double aFirst, aLast;
      if (TopOpeBRepBuild_CurveRange(aCurve, aPCurve, aFirst, aLast))
      {
        aSurfaceCurves.Append({aCurveIt.Current(), aPCurve, aFirst, aLast});
      }
    }

    // Corner trimming can extend a previously partial limiting domain to a
    // complete restriction. Recheck on the original support after all corners
    // have been built, using the same coincidence test as generated curves.
    if (!aSupport.IsNull())
    {
      for (const TopOpeBRepBuild_SurfaceCurve& aCurve : aSurfaceCurves)
      {
        if (!BDS.Curve(aCurve.Index).ExistingEdge().IsNull())
        {
          continue;
        }
        Geom2dAdaptor_Curve aNewCurve(aCurve.PCurve, aCurve.FirstParameter, aCurve.LastParameter);
        for (TopExp_Explorer anIt(aSupport, TopAbs_EDGE); anIt.More(); anIt.Next())
        {
          const TopoDS_Edge anEdge = TopoDS::Edge(anIt.Current());
          if (BRep_Tool::Degenerated(anEdge))
          {
            continue;
          }
          double                          aFirst, aLast;
          const occ::handle<Geom2d_Curve> aPCurve =
            BRep_Tool::CurveOnSurface(anEdge, aSupport, aFirst, aLast);
          if (aPCurve.IsNull())
          {
            continue;
          }
          Geom2dAdaptor_Curve aRestriction(aPCurve, aFirst, aLast);
          Geom2dInt_GInter    anIntersector(aNewCurve,
                                         aRestriction,
                                         Precision::PConfusion(),
                                         Precision::PConfusion());
          bool                isReversed = false;
          if (TopOpeBRepBuild_Tools::HasCompleteCoincidence(anIntersector,
                                                            aNewCurve,
                                                            aRestriction,
                                                            Precision::PConfusion(),
                                                            isReversed)
              || TopOpeBRepBuild_IsCoincidentRestriction(BDS.Curve(aCurve.Index),
                                                         aCurve.FirstParameter,
                                                         aCurve.LastParameter,
                                                         anEdge,
                                                         isReversed))
          {
            // Only unchanged boundary vertices permit reuse of the whole edge.
            // Other interferences trim it and must retain the existing splitting
            // path; copying the edge would restore a consumed piece.
            TopoDS_Vertex aFirstVertex, aLastVertex;
            TopExp::Vertices(anEdge, aFirstVertex, aLastVertex);
            bool isTrimmed = false;
            for (TopOpeBRepDS_PointIterator aPointIt(HDS->EdgePoints(anEdge));
                 aPointIt.More() && !isTrimmed;
                 aPointIt.Next())
            {
              if (aPointIt.IsPoint())
              {
                isTrimmed = true;
                break;
              }
              const auto aVertex =
                HDS->Shape(aPointIt.Current()).Oriented(aPointIt.Orientation(TopAbs_IN));
              isTrimmed = !aVertex.IsEqual(aFirstVertex) && !aVertex.IsEqual(aLastVertex);
            }
            if (isTrimmed)
            {
              continue;
            }
            BDS.ChangeCurve(aCurve.Index).SetExistingEdge(anEdge, isReversed);
            BDS.ChangeCurve(aCurve.Index).SetRange(aCurve.FirstParameter, aCurve.LastParameter);
            for (int anEnd = 0; anEnd < 2; ++anEnd)
            {
              int  aGeometryIndex = 0;
              bool isPoint        = false;
              if (TopOpeBRepBuild_FindCurveEnd(HDS,
                                               aCurve.Index,
                                               anEnd == 0,
                                               aGeometryIndex,
                                               isPoint)
                  && isPoint)
              {
                const bool isFirst = (anEnd == 0) != isReversed;
                const auto aVertex =
                  isFirst ? TopExp::FirstVertex(anEdge) : TopExp::LastVertex(anEdge);
                if (BRep_Tool::Pnt(aVertex).Distance(BDS.Point(aGeometryIndex).Point())
                    <= std::max(BRep_Tool::Tolerance(aVertex),
                                BDS.Point(aGeometryIndex).Tolerance()))
                {
                  ChangeNewVertex(aGeometryIndex) = aVertex;
                }
              }
            }
            break;
          }
        }
      }
    }

    // Opposing chamfers also meet on an original support face.
    for (size_t aFirstIndex = 0; aFirstIndex < aSurfaceCurves.Size(); ++aFirstIndex)
    {
      for (size_t aSecondIndex = aFirstIndex + 1; aSecondIndex < aSurfaceCurves.Size();
           ++aSecondIndex)
      {
        const TopOpeBRepBuild_SurfaceCurve& aFirst  = aSurfaceCurves[aFirstIndex];
        const TopOpeBRepBuild_SurfaceCurve& aSecond = aSurfaceCurves[aSecondIndex];
        Geom2dAdaptor_Curve aFirstCurve(aFirst.PCurve, aFirst.FirstParameter, aFirst.LastParameter);
        Geom2dAdaptor_Curve aSecondCurve(aSecond.PCurve,
                                         aSecond.FirstParameter,
                                         aSecond.LastParameter);
        Geom2dInt_GInter    anIntersector(aFirstCurve,
                                       aSecondCurve,
                                       Precision::PConfusion(),
                                       Precision::PConfusion());
        bool                isReversed = false;
        if (TopOpeBRepBuild_Tools::HasCompleteCoincidence(anIntersector,
                                                          aFirstCurve,
                                                          aSecondCurve,
                                                          Precision::PConfusion(),
                                                          isReversed))
        {
          BDS.MergeEquivalentCurves(aFirst.Index, aSecond.Index, isReversed);
        }
        else
        {
          for (int aSegmentIndex = 1; aSegmentIndex <= anIntersector.NbSegments(); ++aSegmentIndex)
          {
            const auto& aSegment = anIntersector.Segment(aSegmentIndex);
            if (aSegment.HasFirstPoint() && aSegment.HasLastPoint()
                && std::abs(aSegment.LastPoint().ParamOnFirst()
                            - aSegment.FirstPoint().ParamOnFirst())
                     > Precision::PConfusion())
            {
              hasTraceContacts = true;
            }
          }
        }
        // Original support faces need shared vertices for surviving T-junctions.
        // Generated blend faces already select their trimming wires; connecting
        // provisional end-cap traces here can attach discarded branches to them.
        for (int aPointIndex = 1; !aSupport.IsNull() && aPointIndex <= anIntersector.NbPoints();
             ++aPointIndex)
        {
          const auto&  aPoint    = anIntersector.Point(aPointIndex);
          const auto&  aFirst3d  = BDS.Curve(aFirst.Index);
          const auto&  aSecond3d = BDS.Curve(aSecond.Index);
          const double aFirstTolerance =
            GeomAdaptor_Curve(aFirst3d.Curve())
              .Resolution(std::max(Precision::Confusion(), aFirst3d.Tolerance()));
          const double aSecondTolerance =
            GeomAdaptor_Curve(aSecond3d.Curve())
              .Resolution(std::max(Precision::Confusion(), aSecond3d.Tolerance()));
          const bool isFirstInterior =
            aPoint.ParamOnFirst() > aFirst.FirstParameter + aFirstTolerance
            && aPoint.ParamOnFirst() < aFirst.LastParameter - aFirstTolerance;
          const bool isSecondInterior =
            aPoint.ParamOnSecond() > aSecond.FirstParameter + aSecondTolerance
            && aPoint.ParamOnSecond() < aSecond.LastParameter - aSecondTolerance;
          if (isFirstInterior != isSecondInterior)
          {
            hasTraceContacts = true;
          }
        }
      }
    }
  }

  for (cex.Init(BDS, false); cex.More(); cex.Next())
  {
    const int aCurveIndex = cex.Index();
    if (cex.Curve().EquivalentCurve() <= 0)
    {
      continue;
    }
    bool      isReversed = false;
    const int aRoot      = BDS.FindEquivalentCurve(aCurveIndex, isReversed);
    BDS.ChangeCurve(aCurveIndex).SetEquivalentCurve(aRoot, isReversed);
  }

  NCollection_DataMap<int, TopoDS_Shape> anEquivalentPointVertices;
  for (cex.Init(BDS, false); cex.More(); cex.Next())
  {
    const int aCurveIndex = cex.Index();
    for (int anEnd = 0; anEnd < 2; ++anEnd)
    {
      const bool isCurveStart     = (anEnd == 0);
      bool       isReferenceStart = false;
      const int  aReferenceCurve =
        BDS.FindEquivalentCurvePoint(aCurveIndex, isCurveStart, isReferenceStart);
      if (aReferenceCurve == aCurveIndex && isReferenceStart == isCurveStart)
      {
        continue;
      }

      const int aReference = 2 * aReferenceCurve + (isReferenceStart ? 0 : 1);
      if (anEquivalentPointVertices.IsBound(aReference))
      {
        continue;
      }

      int  aGeometryIndex = 0;
      bool isPoint        = false;
      if (TopOpeBRepBuild_FindCurveEnd(HDS,
                                       aReferenceCurve,
                                       isReferenceStart,
                                       aGeometryIndex,
                                       isPoint))
      {
        anEquivalentPointVertices.Bind(aReference,
                                       isPoint ? NewVertex(aGeometryIndex)
                                               : HDS->Shape(aGeometryIndex));
      }
    }
  }

  for (cex.Init(BDS, false); cex.More(); cex.Next())
  {
    const int aCurveIndex = cex.Index();
    for (int anEnd = 0; anEnd < 2; ++anEnd)
    {
      const bool isCurveStart     = (anEnd == 0);
      bool       isReferenceStart = false;
      const int  aReferenceCurve =
        BDS.FindEquivalentCurvePoint(aCurveIndex, isCurveStart, isReferenceStart);
      const int aReference = 2 * aReferenceCurve + (isReferenceStart ? 0 : 1);
      if (!anEquivalentPointVertices.IsBound(aReference))
      {
        continue;
      }

      int  aGeometryIndex = 0;
      bool isPoint        = false;
      if (TopOpeBRepBuild_FindCurveEnd(HDS, aCurveIndex, isCurveStart, aGeometryIndex, isPoint)
          && isPoint)
      {
        ChangeNewVertex(aGeometryIndex) = anEquivalentPointVertices(aReference);
      }
    }
  }

  for (cex.Init(BDS, false); cex.More(); cex.Next())
  {
    const int aCurveIndex     = cex.Index();
    const int aReferenceCurve = cex.Curve().EquivalentCurve();
    if (aReferenceCurve <= 0 || aReferenceCurve == aCurveIndex)
    {
      continue;
    }

    for (TopOpeBRepDS_PointIterator aPointIt(HDS->CurvePoints(aCurveIndex)); aPointIt.More();
         aPointIt.Next())
    {
      if (!aPointIt.IsPoint())
      {
        continue;
      }
      const int                 aPointIndex = aPointIt.Current();
      const TopOpeBRepDS_Point& aPoint      = HDS->Point(aPointIndex);
      for (TopOpeBRepDS_PointIterator aReferencePointIt(HDS->CurvePoints(aReferenceCurve));
           aReferencePointIt.More();
           aReferencePointIt.Next())
      {
        if (!aReferencePointIt.IsPoint())
        {
          continue;
        }
        const int                 aReferencePointIndex = aReferencePointIt.Current();
        const TopOpeBRepDS_Point& aReferencePoint      = HDS->Point(aReferencePointIndex);
        if (aPoint.IsEqual(aReferencePoint))
        {
          ChangeNewVertex(aPointIndex) = NewVertex(aReferencePointIndex);
          break;
        }
      }
    }
  }

  int ick = 0;
  for (cex.Init(BDS, false); cex.More(); cex.Next())
  {
    int  ic = cex.Index();
    bool ck = cex.IsCurveKeep(ic);
    int  im = cex.Curve(ic).Mother();
    if (ck == 1 && im != 0 && ick == 0)
    {
      ick = ic;
      break;
    }
  }
  if (ick)
  {
    for (cex.Init(BDS, true); cex.More(); cex.Next())
    {
      int ic = cex.Index();
      BDS.RemoveCurve(ic);
    }
    BDS.ChangeNbCurves(ick - 1);
  }

  for (cex.Init(BDS, false); cex.More(); cex.Next())
  {
    int ic = cex.Index();
    int im = cex.Curve(ic).Mother();
    if (im != 0)
    {
      continue;
    }
    BuildEdges(ic, HDS);
  }

  // Whole-curve equivalence cannot represent partial coincidence or a T-junction.
  // Reuse General Fuse to split the generated boundaries into shared intervals.
  // The history maps preserve each curve's association with its split edges;
  // their orientations remain relative to that curve, not to the shared 3D curve.
  if (hasTraceContacts)
  {
    BOPAlgo_Builder aSplitter;
    aSplitter.SetNonDestructive(true);
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> anArguments;
    for (cex.Init(BDS, false); cex.More(); cex.Next())
    {
      for (const TopoDS_Shape& anEdge : NewEdges(cex.Index()))
      {
        if (!BRep_Tool::Degenerated(TopoDS::Edge(anEdge)) && !anArguments.Contains(anEdge))
        {
          anArguments.Add(anEdge);
          aSplitter.AddArgument(anEdge);
        }
      }
    }
    aSplitter.Perform();
    if (aSplitter.HasErrors())
    {
      throw Standard_ConstructionError("Coincident trace splitting failed");
    }
    occ::handle<IntTools_Context> aContext = new IntTools_Context;
    for (cex.Init(BDS, false); cex.More(); cex.Next())
    {
      NCollection_List<TopoDS_Shape> aSplits;
      for (const TopoDS_Shape& anEdge : NewEdges(cex.Index()))
      {
        const auto& anImages = aSplitter.Modified(anEdge);
        if (anImages.IsEmpty())
        {
          if (!aSplitter.IsDeleted(anEdge))
          {
            aSplits.Append(anEdge);
          }
          continue;
        }
        for (TopoDS_Shape aSplit : anImages)
        {
          if (aSplit.ShapeType() != TopAbs_EDGE)
          {
            continue;
          }
          aSplit.Orientation(TopAbs_FORWARD);
          int anError = 0;
          aSplit.Orientation(
            BOPTools_AlgoTools::IsSplitToReverse(aSplit, anEdge, aContext, &anError)
              ? TopAbs_REVERSED
              : TopAbs_FORWARD);
          if (anError != 0)
          {
            throw Standard_ConstructionError("Cannot orient split intersection curve");
          }
          aSplits.Append(aSplit);
        }
      }
      ChangeNewEdges(cex.Index()) = aSplits;
    }
    for (int aPoint = 1; aPoint <= HDS->NbPoints(); ++aPoint)
    {
      const auto& anImages = aSplitter.Modified(NewVertex(aPoint));
      if (!anImages.IsEmpty())
      {
        ChangeNewVertex(aPoint) = anImages.First();
      }
    }
  }

  int                      ip, np = HDS->NbPoints();
  NCollection_HArray1<int> tp(0, np, 0);
  for (cex.Init(BDS); cex.More(); cex.Next())
  {
#ifdef OCCT_DEBUG
//    const TopOpeBRepDS_Curve& C = cex.Curve();
#endif
    int                                                                ic = cex.Index();
    NCollection_List<occ::handle<TopOpeBRepDS_Interference>>::Iterator it(
      BDS.CurveInterferences(ic));
    for (; it.More(); it.Next())
    {
      const occ::handle<TopOpeBRepDS_Interference>& I = it.Value();
      {
        int               ig = I->Geometry();
        TopOpeBRepDS_Kind kg = I->GeometryType();
        if (kg == TopOpeBRepDS_POINT && ig <= np)
        {
          tp.ChangeValue(ig) = tp.Value(ig) + 1;
        }
      }
      {
        int               is = I->Support();
        TopOpeBRepDS_Kind ks = I->SupportType();
        if (ks == TopOpeBRepDS_POINT)
        {
          tp.ChangeValue(is) = tp.Value(is) + 1;
        }
      }
    }
  }
  int is, ns = BDS.NbShapes();
  for (is = 1; is <= ns; is++)
  {
    const TopoDS_Shape& S = BDS.Shape(is);
    if (S.IsNull())
    {
      continue;
    }
    bool test = (S.ShapeType() == TopAbs_EDGE);
    if (!test)
    {
      continue;
    }
    NCollection_List<occ::handle<TopOpeBRepDS_Interference>>::Iterator it(
      BDS.ShapeInterferences(is));
    for (; it.More(); it.Next())
    {
      const occ::handle<TopOpeBRepDS_Interference>& I = it.Value();
      {
        int               ig = I->Geometry();
        TopOpeBRepDS_Kind kg = I->GeometryType();
        if (kg == TopOpeBRepDS_POINT)
        {
          tp.ChangeValue(ig) = tp.Value(ig) + 1;
        }
      }
      {
        int               is1 = I->Support();
        TopOpeBRepDS_Kind ks  = I->SupportType();
        if (ks == TopOpeBRepDS_POINT)
        {
          tp.ChangeValue(is1) = tp.Value(is1) + 1;
        }
      }
    }
  }
  for (ip = 1; ip <= np; ip++)
  {
    if (tp.Value(ip) == 0)
    {
      BDS.RemovePoint(ip);
    }
  }
}
