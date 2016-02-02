# define_openslam_lib(): Declares an OPENSLAM library target:
#-----------------------------------------------------------------------
macro(define_openslam_lib name)
	internal_define_openslam_lib(${name} 0 0 ${ARGN}) # headers_only = 0, is_metalib=0
endmacro(define_openslam_lib)

# define_openslam_lib_header_only(): Declares an OPENSLAM headers-only library:
#-----------------------------------------------------------------------
macro(define_openslam_lib_header_only name)
	internal_define_openslam_lib(${name} 1 0 ${ARGN}) # headers_only = 1, is_metalib=0
endmacro(define_openslam_lib_header_only)

# define_openslam_metalib(): Declares an OPENSLAM meta-lib:
#-----------------------------------------------------------------------
macro(define_openslam_metalib name)
	internal_define_openslam_lib(${name} 1 1 ${ARGN}) # headers_only = 1, is_metalib=1
endmacro(define_openslam_metalib)

# Implementation of both define_openslam_lib() and define_openslam_lib_headers_only():
#-----------------------------------------------------------------------------
macro(internal_define_openslam_lib name headers_only is_metalib)
	INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # 确保用户已经添加到根目录下的CMake文件中

	# 允许对lib进行控制，设置是否编译，默认是编译
	SET(_DEFVAL "${DEFAULT_BUILD_openslam_${name}}")
	IF ("${_DEFVAL}" STREQUAL "")
		SET(_DEFVAL "ON")
	ENDIF ("${_DEFVAL}" STREQUAL "")

	SET(BUILD_openslam_${name} ${_DEFVAL} CACHE BOOL "Build the library openslam_${name}")
	IF(BUILD_openslam_${name}) 
	# --- 进行模块生成 ---
	
	IF(NOT ${is_metalib})
		PROJECT(openslam_${name})
	ENDIF(NOT ${is_metalib})
	
	# There is an optional LISTS of extra sources from the caller: 
	#  "${name}_EXTRA_SRCS" and 
	#  "${name}_EXTRA_SRCS_NAME"   <--- 必须不要包含空格!!
	#
	#  At return from this macro, there'll be defined a variable:
	#	   "${${name}_EXTRA_SRCS_NAME}_FILES"
	#   with the list of all files under that group.
	#
	#  For code simplicity, let's use the same list, just adding the default sources there:
	LIST(APPEND ${name}_EXTRA_SRCS 
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.cpp"
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.c"
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.cxx"
		"${CMAKE_SOURCE_DIR}/libs/${name}/include/openslam/${name}/*.h"
		"${CMAKE_SOURCE_DIR}/libs/${name}/include/openslam/${name}/*.hpp"
		"${CMAKE_SOURCE_DIR}/doc/doxygen-pages/lib_openslam_${name}.h"
		)
	LIST(APPEND ${name}_EXTRA_SRCS_NAME
		"Source"
		"Source"
		"Source"
		"Header"
		"Header"
		"Document"
		)
	# 对动态库的宏声明
	IF (NOT ${headers_only})
		LIST(APPEND ${name}_EXTRA_SRCS 
			"${CMAKE_SOURCE_DIR}/libs/${name}/include/openslam/${name}/link_pragmas.h"
			)
		LIST(APPEND ${name}_EXTRA_SRCS_NAME
			"DLL link macros"
			)
	ENDIF (NOT ${headers_only})

	# Collect files
	# ---------------------------------------------------------
	LIST(LENGTH ${name}_EXTRA_SRCS N_SRCS)
	LIST(LENGTH ${name}_EXTRA_SRCS_NAME N_SRCS_NAMES)
	
	IF (NOT N_SRCS EQUAL N_SRCS_NAMES)
		MESSAGE(FATAL_ERROR "Mismatch length in ${name}_EXTRA_SRCS and ${name}_EXTRA_SRCS_NAME!")
	ENDIF (NOT N_SRCS EQUAL N_SRCS_NAMES)
	
	SET(${name}_srcs "")  # ALL the files
	
	MATH(EXPR N_SRCS "${N_SRCS}-1")  # Indices are 0-based
	
	foreach(i RANGE 0 ${N_SRCS})
		# Get i'th expression & its name:
		LIST(GET ${name}_EXTRA_SRCS      ${i} FILS_EXPR)
		LIST(GET ${name}_EXTRA_SRCS_NAME ${i} FILS_GROUP_NAME)
		
		FILE(GLOB aux_list ${FILS_EXPR})
		
		SOURCE_GROUP("${FILS_GROUP_NAME} Files" FILES ${aux_list})
		
		# 添加项目对应筛选器:
		LIST(APPEND ${name}_srcs ${aux_list})
		# All to group lists, may be used by the user upon return from this macro:
		LIST(APPEND ${FILS_GROUP_NAME}_FILES ${aux_list})
	endforeach(i)

	# 在windows下编译则移除 _LIN 的文件，在Linux下编译则移除 _WIN 文件
	IF(WIN32)		
		REMOVE_MATCHING_FILES_FROM_LIST(".*_LIN.cpp" ${name}_srcs)		# Win32
	ELSE(WIN32)
		REMOVE_MATCHING_FILES_FROM_LIST(".*_WIN.cpp" ${name}_srcs)		# Apple & Unix
	ENDIF(WIN32)

	# 将单元测试的文件进行汇总，添加到unittest项目下:
	set(lstunittests ${${name}_srcs})
	KEEP_MATCHING_FILES_FROM_LIST(".*unittest.cpp" lstunittests)
	if(NOT "${lstunittests}" STREQUAL "")
		# 标识这个模块有单元测试:
		get_property(_lst_lib_test GLOBAL PROPERTY "OPENSLAM_TEST_LIBS")
		set_property(GLOBAL PROPERTY "OPENSLAM_TEST_LIBS" ${_lst_lib_test} openslam_${name})
		set_property(GLOBAL PROPERTY "openslam_${name}_UNIT_TEST_FILES" ${lstunittests})
	endif(NOT "${lstunittests}" STREQUAL "")


	# Don't include here the unit testing code:
	REMOVE_MATCHING_FILES_FROM_LIST(".*unittest.cpp" ${name}_srcs)


	#  Define the target:
	set(all_${name}_srcs  ${${name}_srcs})
	
	# Add main lib header (may not exist in meta-libs only):
	IF (EXISTS "${CMAKE_SOURCE_DIR}/libs/${name}/include/openslam/${name}.h")
		set(all_${name}_srcs ${all_${name}_srcs} "${CMAKE_SOURCE_DIR}/libs/${name}/include/openslam/${name}.h")
	ENDIF (EXISTS "${CMAKE_SOURCE_DIR}/libs/${name}/include/openslam/${name}.h")
		
	IF (NOT ${headers_only})

		# A libray target:
		ADD_LIBRARY(openslam_${name}   
			${all_${name}_srcs}      # sources
			${OPENSLAM_VERSION_RC_FILE}  # Only !="" in Win32: the .rc file with version info
			)

	ELSE(NOT ${headers_only})

		# A custom target (needs no real compiling)
		add_custom_target(openslam_${name} DEPENDS ${all_${name}_srcs} SOURCES ${all_${name}_srcs})

	ENDIF (NOT ${headers_only})

	# Append to list of all openslam_* libraries:
	if("${ALL_OPENSLAM_LIBS}" STREQUAL "")  # first one is different to avoid an empty first list element ";openslam_xxx"
		SET(ALL_OPENSLAM_LIBS "openslam_${name}" CACHE INTERNAL "")  # This emulates global vars
	else("${ALL_OPENSLAM_LIBS}" STREQUAL "")
		SET(ALL_OPENSLAM_LIBS "${ALL_OPENSLAM_LIBS};openslam_${name}" CACHE INTERNAL "")  # This emulates global vars
	endif("${ALL_OPENSLAM_LIBS}" STREQUAL "")
	
	# Include dir for this lib:
	INCLUDE_DIRECTORIES("${OPENSLAM_SOURCE_DIR}/libs/${name}/include")
	
	# Include dirs for openslam_XXX libs:
	set(AUX_DEPS_LIST "")
	set(AUX_EXTRA_LINK_LIBS "")
	set(AUX_ALL_DEPS_BUILD 1)  # Will be set to "0" if any dependency if not built
	FOREACH(DEP ${ARGN})
		# Only for "openslam_XXX" libs:
		IF (${DEP} MATCHES "openslam_")
			STRING(REGEX REPLACE "openslam_(.*)" "\\1" DEP_OPENSLAM_NAME ${DEP})
			IF(NOT "${DEP_OPENSLAM_NAME}" STREQUAL "")
				# Include dir:
				INCLUDE_DIRECTORIES("${OPENSLAM_SOURCE_DIR}/libs/${DEP_OPENSLAM_NAME}/include")
				
				# Link "-lopenslam_name", only for GCC/CLang and if both THIS and the dependence are non-header-only:
				IF(NOT ${headers_only})
					IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
						get_property(_LIB_HDRONLY GLOBAL PROPERTY "${DEP}_LIB_IS_HEADERS_ONLY")
						IF(NOT _LIB_HDRONLY)
							#MESSAGE(STATUS "adding link dep: openslam_${name} -> ${DEP}")
							LIST(APPEND AUX_EXTRA_LINK_LIBS ${DEP}${OPENSLAM_LINKER_LIBS_POSTFIX})
						ENDIF(NOT _LIB_HDRONLY)
					ENDIF()
				ENDIF(NOT ${headers_only})
				
				# Append to list of openslam_* lib dependences:
				LIST(APPEND AUX_DEPS_LIST ${DEP})
				
				# Check if all dependencies are to be build: 
				if ("${BUILD_openslam_${DEP_OPENSLAM_NAME}}" STREQUAL "OFF")
					SET(AUX_ALL_DEPS_BUILD 0)
					MESSAGE(STATUS "*Warning*: Lib openslam_${name} cannot be built because dependency openslam_${DEP_OPENSLAM_NAME} has been disabled!")
				endif ()
				
			ENDIF(NOT "${DEP_OPENSLAM_NAME}" STREQUAL "")
		ENDIF (${DEP} MATCHES "openslam_")
	ENDFOREACH(DEP)
	
	# Impossible to build? 
	if (NOT AUX_ALL_DEPS_BUILD)
		MESSAGE(STATUS "*Warning* ==> Disabling compilation of lib openslam_${name} for missing dependencies listed above.")		
		SET(BUILD_openslam_${name} OFF CACHE BOOL "Build the library openslam_${name}" FORCE)
	endif (NOT AUX_ALL_DEPS_BUILD)
	
	
	# Emulates a global variable:
	set_property(GLOBAL PROPERTY "openslam_${name}_LIB_DEPS" "${AUX_DEPS_LIST}")
	set_property(GLOBAL PROPERTY "openslam_${name}_LIB_IS_HEADERS_ONLY" "${headers_only}")
	set_property(GLOBAL PROPERTY "openslam_${name}_LIB_IS_METALIB" "${is_metalib}")

	# Dependencies between projects:
	IF(NOT "${ARGN}" STREQUAL "")
		ADD_DEPENDENCIES(openslam_${name} ${ARGN})
	ENDIF(NOT "${ARGN}" STREQUAL "")

	IF (NOT ${headers_only})
		TARGET_LINK_LIBRARIES(openslam_${name} 
			${OPENSLAMLIB_LINKER_LIBS}
			${AUX_EXTRA_LINK_LIBS}
			)
	ENDIF (NOT ${headers_only})

	#添加筛选器
	if(ENABLE_SOLUTION_FOLDERS)
		set_target_properties(openslam_${name} PROPERTIES FOLDER "modules")
	else(ENABLE_SOLUTION_FOLDERS)
		SET_TARGET_PROPERTIES(openslam_${name} PROPERTIES PROJECT_LABEL "(LIB) openslam_${name}")
	endif(ENABLE_SOLUTION_FOLDERS)

	# Set custom name of lib + dynamic link numbering convenions in Linux:
	IF (NOT ${headers_only})
		SET_TARGET_PROPERTIES(openslam_${name} PROPERTIES 
			OUTPUT_NAME ${OPENSLAM_LIB_PREFIX}openslam_${name}${OPENSLAM_DLL_VERSION_POSTFIX}
			ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/"
			RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/"
			VERSION "${CMAKE_OPENSLAM_VERSION_NUMBER_MAJOR}.${CMAKE_OPENSLAM_VERSION_NUMBER_MINOR}.${CMAKE_OPENSLAM_VERSION_NUMBER_PATCH}"
			SOVERSION ${CMAKE_OPENSLAM_VERSION_NUMBER_MAJOR}.${CMAKE_OPENSLAM_VERSION_NUMBER_MINOR}
			)
		
		# Set all header files as "ignored" (don't build!):
		# -----------------------------------------------------
		set(AUX_LIST_TO_IGNORE ${all_${name}_srcs})
		KEEP_MATCHING_FILES_FROM_LIST("^.*h$" AUX_LIST_TO_IGNORE)
		set_source_files_properties(${AUX_LIST_TO_IGNORE} PROPERTIES HEADER_FILE_ONLY true)
	
		INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/libs/${name}/src/") # For include "${name}_precomp.h"
		IF(OPENSLAM_ENABLE_PRECOMPILED_HDRS)
			IF (MSVC)
				# Precompiled hdrs for MSVC:
				# --------------------------------------
				STRING(TOUPPER ${name} NAMEUP)

				# The "use precomp.headr" for all the files...
				set_target_properties(openslam_${name}
					PROPERTIES
					COMPILE_FLAGS "/Yu${name}_precomp.h")

				# But for the file used to build the precomp. header:
				set_source_files_properties("${CMAKE_SOURCE_DIR}/libs/${name}/src/${name}_precomp.cpp"
					PROPERTIES
					COMPILE_FLAGS "/Yc${name}_precomp.h")
			ENDIF (MSVC)
		
			SOURCE_GROUP("Precompiled headers" FILES 
				"${CMAKE_SOURCE_DIR}/libs/${name}/src/${name}_precomp.cpp"
				"${CMAKE_SOURCE_DIR}/libs/${name}/include/${name}_precomp.h"
				)	
		ENDIF(OPENSLAM_ENABLE_PRECOMPILED_HDRS)

		# make sure the library gets installed
		#IF (NOT is_metalib)
		#	INSTALL(TARGETS openslam_${name}
		#		RUNTIME DESTINATION ${OPENSLAM_PREFIX_INSTALL}bin  COMPONENT Libraries
		#		LIBRARY DESTINATION ${OPENSLAM_PREFIX_INSTALL}${CMAKE_INSTALL_LIBDIR} COMPONENT Libraries
		#		ARCHIVE DESTINATION ${OPENSLAM_PREFIX_INSTALL}${CMAKE_INSTALL_LIBDIR} COMPONENT Libraries  # WAS: lib${LIB_SUFFIX}
		#		)
		#	
		#	# Collect .pdb debug files for optional installation:
		#	IF (MSVC)
		#		SET(PDB_FILE "${CMAKE_BINARY_DIR}/bin/Debug/openslam_${name}${CMAKE_OPENSLAM_VERSION_NUMBER_MAJOR}${CMAKE_OPENSLAM_VERSION_NUMBER_MINOR}${CMAKE_OPENSLAM_VERSION_NUMBER_PATCH}d.pdb")
		#		IF (EXISTS "${PDB_FILE}")
		#			INSTALL(FILES ${PDB_FILE} DESTINATION bin COMPONENT LibrariesDebugInfoPDB)
		#		ENDIF (EXISTS "${PDB_FILE}")
		#	ENDIF(MSVC)		
		#ENDIF (NOT is_metalib)
	ENDIF (NOT ${headers_only})


	# --- End of conditional build of module ---
	ENDIF(BUILD_openslam_${name}) 

endmacro(internal_define_openslam_lib)

