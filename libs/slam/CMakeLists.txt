INCLUDE_DIRECTORIES(${OPENSLAM_INCLUDE_DIR}/3rdparty/g2o)
INCLUDE_DIRECTORIES(${OPENSLAM_INCLUDE_DIR}/3rdparty/DBoW2)
#---------------------------------------------
# Macro declared in "DeclareOPENSLAMLib.cmake":
#---------------------------------------------
define_openslam_lib(
	# Lib name
	slam 		
	# Dependencies
	openslam_base
	)

IF(BUILD_openslam_slam) 
	target_link_libraries(openslam_slam ${OpenCV_LIBRARIES} g2o DBoW2)
ENDIF(BUILD_openslam_slam) 
