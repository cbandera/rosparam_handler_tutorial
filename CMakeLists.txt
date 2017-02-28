cmake_minimum_required(VERSION 2.8.3)
project(rosparam_handler_tutorial)
find_package(catkin REQUIRED COMPONENTS roscpp rosparam_handler dynamic_reconfigure )

# set compiler flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" Cpp14CompilerFlag)
if (${Cpp14CompilerFlag})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "17")
	#no additional flag is required
else()
	message(FATAL_ERROR "Compiler does not have c++14 support. Use at least g++4.9 or Visual Studio 2013 and newer.")
endif()

# Generate parameter files
generate_ros_parameter_files(cfg/Demo.params)

# Create package
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

# Node
add_executable(demo src/demo/demo.cpp src/demo/demo_node.cpp)
add_dependencies(demo ${catkin_EXPORTED_TARGETS} rosparam_handler_tutorial_gencfg)
target_link_libraries(demo ${catkin_LIBRARIES})
install(TARGETS demo
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)

# Nodelet
add_library(demo_nodelet src/demo/demo.cpp src/demo/demo_nodelet.cpp)
target_link_libraries(demo_nodelet ${catkin_LIBRARIES})
install(TARGETS demo_nodelet LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})