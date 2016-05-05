INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # 确保在根目录的cmake中添加

PROJECT(test_framework)

ADD_EXECUTABLE(test_framework
               test_framework.cpp)

INCLUDE_DIRECTORIES(${OPENSLAM_INCLUDE_DIR}/libs/base/include)
INCLUDE_DIRECTORIES(${OPENSLAM_INCLUDE_DIR}/libs/slam/include)
INCLUDE_DIRECTORIES(${OPENSLAM_INCLUDE_DIR}/3rdparty/DBoW2)

SET(OPENSLAM_LINKER_LIBS
	openslam_base
	openslam_slam
	)

TARGET_LINK_LIBRARIES(test_framework ${OPENSLAM_LINKER_LIBS} ${OpenCV_LIBS})

# 设置将项目添加到包中
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(test_framework PROPERTIES FOLDER "app")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(test_framework  PROPERTIES PROJECT_LABEL "(app) test_framework")
endif(ENABLE_SOLUTION_FOLDERS)



