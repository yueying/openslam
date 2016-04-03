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
	target_link_libraries(openslam_slam ${OpenCV_LIBRARIES})
ENDIF(BUILD_openslam_slam) 
