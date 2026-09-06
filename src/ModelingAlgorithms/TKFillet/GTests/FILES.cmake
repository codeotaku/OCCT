# Test source files for TKFillet
set(OCCT_TKFillet_GTests_FILES_LOCATION "${CMAKE_CURRENT_LIST_DIR}")

set(OCCT_TKFillet_GTests_FILES
  BlendFunc_ChAsymInv_Test.cxx
  BRepFilletAPI_ChamferTolerance_Test.cxx
  BRepFilletAPI_ChamferUniformBSpline_Test.cxx
  BRepFilletAPI_ChamferAsymmetricLimit_Test.cxx
  BRepFilletAPI_ChamferConsumedRim_Test.cxx
  BRepFilletAPI_ChamferCurvedSupports_Test.cxx
  BRepFilletAPI_ChamferLimit_Test.cxx
  BRepFilletAPI_ChamferMultiWire_Test.cxx
  BRepFilletAPI_ChamferTangentBoss_Test.cxx
  ChFi3d_Contact_Test.cxx
  ChFi3d_CollapsedTrace_Test.cxx
  BRepFilletAPI_ChamferMatrix_Test.cxx
  BRepFilletAPI_MakeChamfer_Test.cxx
  BRepFilletAPI_MakeFillet_Test.cxx
  ChFi3d_Builder_0_Test.cxx
  ChFi3d_CornerClassification_Test.cxx
  ChFi3d_CornerRecoil_Test.cxx
  ChFi3d_ChamferCornerExtension_Test.cxx
  ChFi3d_Hatching_Test.cxx
)
