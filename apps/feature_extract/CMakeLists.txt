INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # 确保在根目录的cmake中添加

PROJECT(feature_extract)

ADD_EXECUTABLE(feature_extract
               feature_extract.cpp)

INCLUDE_DIRECTORIES(${OPENSLAM_INCLUDE_DIR}/libs/base/include)
INCLUDE_DIRECTORIES(${OPENSLAM_INCLUDE_DIR}/libs/slam/include)
INCLUDE_DIRECTORIES(${OPENSLAM_INCLUDE_DIR}/3rdparty/DBoW2)

SET(OPENSLAM_LINKER_LIBS
	openslam_base
	openslam_slam
	)

TARGET_LINK_LIBRARIES(feature_extract ${OPENSLAM_LINKER_LIBS} ${OpenCV_LIBS})

# 设置将项目添加到包中
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(feature_extract PROPERTIES FOLDER "app")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(feature_extract  PROPERTIES PROJECT_LABEL "(app) feature_extract")
endif(ENABLE_SOLUTION_FOLDERS)



