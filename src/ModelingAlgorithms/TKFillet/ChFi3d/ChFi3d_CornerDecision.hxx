// Copyright (c) 2026 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.

#ifndef ChFi3d_CornerDecision_HeaderFile
#define ChFi3d_CornerDecision_HeaderFile

#include <BRepAdaptor_Curve.hxx>
#include <Standard.hxx>
#include <gp_Pnt.hxx>

//! Selects the projection of the adjacent stripe endpoint on a living edge when that projection
//! is a shorter and more accurate local termination than the legacy recoil parameter.
//!
//! @param theCurve curve of the living edge
//! @param theAdjacentPoint endpoint of the one adjacent fillet/chamfer stripe
//! @param theCornerPoint corner vertex from which the termination advances
//! @param theRecoilParameter parameter proposed by the legacy common-recoil calculation
//! @param theParameter receives either the recoil parameter or the accepted projection parameter
//! @return true when the projection was accepted
Standard_EXPORT bool ChFi3d_ChooseProjectedRecoil(const BRepAdaptor_Curve& theCurve,
                                                  const gp_Pnt&            theAdjacentPoint,
                                                  const gp_Pnt&            theCornerPoint,
                                                  const double             theRecoilParameter,
                                                  double&                  theParameter);

//! Checks that an already projected connector represents the intended local corner segment.
//! Endpoint order may be either forward or reversed.
Standard_EXPORT bool ChFi3d_AreConnectorEndpointsMatching(const gp_Pnt& theProjectedFirst,
                                                          const gp_Pnt& theProjectedLast,
                                                          const gp_Pnt& theTargetFirst,
                                                          const gp_Pnt& theTargetLast,
                                                          const double  theTolerance);

#endif
