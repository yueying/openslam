cmake_minimum_required(VERSION 2.8)
project(DBoW2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(HDRS_DBOW2
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
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

find_package(OpenCV REQUIRED)

IF(WIN32)
  include_directories(${OpenCV_INCLUDE_DIRS})
  add_library(DBoW2 STATIC ${SRCS_DBOW2} ${SRCS_DUTILS})
ELSE(WIN32)
   include_directories(${OpenCV_INCLUDE_DIRS})
  add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
ENDIF(WIN32)

target_link_libraries(DBoW2 ${OpenCV_LIBS})

# 设置将项目添加到包中
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(DBoW2 PROPERTIES FOLDER "3rd party")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(DBoW2  PROPERTIES PROJECT_LABEL "(3rd party) DBoW2")
endif(ENABLE_SOLUTION_FOLDERS)