CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(showPointCloud)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)
set(CMAKE_CXX_STANDARD 14)


IF (WIN32)
	MESSAGE(STATUS "Now is windows")
	set(WINDOWS TRUE CACHE INTERNAL "")
	set(PLATFORM_PREFIX "win64" CACHE INTERNAL "")
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)
	if (CMAKE_HOST_WIN32)
		set(HOME_DIR $ENV{HOMEDRIVE}$ENV{HOMEPATH} CACHE INTERNAL "")
	else()
		set(HOME_DIR $ENV{HOME} CACHE INTERNAL "")
	endif()
	if ("TEST$ENV{MAKE_DEPS_WORKING_DIR}" STREQUAL "TEST")
		set(MAKE_DEPS_WORKING_DIR ${HOME_DIR}/makedeps)
	else()    
		set(MAKE_DEPS_WORKING_DIR $ENV{MAKE_DEPS_WORKING_DIR})	
	endif()

	if (NOT ROOT_DIR_DEFINED)
		add_definitions(-DROOT_DIR="${PROJECT_SOURCE_DIR}")
		set(ROOT_DIR_DEFINED ON)
	endif()

	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

	include(${MAKE_DEPS_WORKING_DIR}/${PLATFORM_PREFIX}/deps.cmake)
ELSEIF (APPLE)
	MESSAGE(STATUS "Now is Apple systens.")
ELSEIF (UNIX)
	MESSAGE(STATUS "Now is UNIX-like OS's.")
ENDIF ()




if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)


if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

if (NOT ROOT_DIR_DEFINED)
    add_definitions(-DROOT_DIR="${PROJECT_SOURCE_DIR}")
    set(ROOT_DIR_DEFINED ON)
endif()


SET(EXECUTABLE_OUTPUT_PATH ".")


find_package(PCL REQUIRED)

find_package(OpenCV REQUIRED)


if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
 endif()



include_directories(${OpenCV_INCLUDE_DIRS})


link_directories(${OpenCV_LIBRARY_DIRS})



					
file(GLOB VIDEO_SRC_FILES src/main.cpp 
					
					)
					
									


ADD_EXECUTABLE(showPointCloud  ${VIDEO_SRC_FILES})


TARGET_LINK_LIBRARIES(showPointCloud ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_definitions(-std=c++14 -O3)