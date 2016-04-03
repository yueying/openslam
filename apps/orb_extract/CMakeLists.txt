INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # 确保在根目录的cmake中添加

PROJECT(orb_extract)

ADD_EXECUTABLE(orb_extract
               orb_extract.cpp)

INCLUDE_DIRECTORIES(${OPENSLAM_INCLUDE_DIR}/libs/base/include)
INCLUDE_DIRECTORIES(${OPENSLAM_INCLUDE_DIR}/libs/slam/include)

SET(OPENSLAM_LINKER_LIBS
	openslam_base
	openslam_slam
	)

TARGET_LINK_LIBRARIES(orb_extract ${OPENSLAM_LINKER_LIBS} ${OpenCV_LIBS})

# 设置将项目添加到包中
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(orb_extract PROPERTIES FOLDER "app")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(orb_extract  PROPERTIES PROJECT_LABEL "(app) orb_extract")
endif(ENABLE_SOLUTION_FOLDERS)



