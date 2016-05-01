INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # 确保在根目录的cmake中添加

PROJECT(initial_position)

ADD_EXECUTABLE(initial_position
               initial_position.cpp)

INCLUDE_DIRECTORIES(${OPENSLAM_INCLUDE_DIR}/libs/base/include)
INCLUDE_DIRECTORIES(${OPENSLAM_INCLUDE_DIR}/libs/slam/include)
INCLUDE_DIRECTORIES(${OPENSLAM_INCLUDE_DIR}/3rdparty/DBoW2)

SET(OPENSLAM_LINKER_LIBS
	openslam_base
	openslam_slam
	)

TARGET_LINK_LIBRARIES(initial_position ${OPENSLAM_LINKER_LIBS} ${OpenCV_LIBS})

# 设置将项目添加到包中
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(initial_position PROPERTIES FOLDER "app")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(initial_position  PROPERTIES PROJECT_LABEL "(app) initial_position")
endif(ENABLE_SOLUTION_FOLDERS)



