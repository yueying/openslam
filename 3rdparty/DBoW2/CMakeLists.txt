cmake_minimum_required(VERSION 2.8)
project(DBoW2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

IF(WIN32)
  ADD_DEFINITIONS(-DDBOW2_SHARED_LIBS)
  SET (DBOW2_LIB_TYPE SHARED)
  ADD_DEFINITIONS(-DWINDOWS)
ENDIF(WIN32)

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

set(HDRS_DBOW2
  DBoW2/DBoW2Api.h
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/DUtilsApi.h
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${HDRS_DBOW2} ${HDRS_DUTILS} ${SRCS_DBOW2} ${SRCS_DUTILS})

target_link_libraries(DBoW2 ${OpenCV_LIBS})

# 设置将项目添加到包中
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(DBoW2 PROPERTIES FOLDER "3rd party")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(DBoW2  PROPERTIES PROJECT_LABEL "(3rd party) DBoW2")
endif(ENABLE_SOLUTION_FOLDERS)