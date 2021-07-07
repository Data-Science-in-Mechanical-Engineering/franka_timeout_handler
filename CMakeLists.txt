project(franka_real_time)

################
# Dependencies #
################

#cmake
cmake_minimum_required(VERSION 3.10.2)

#c++
set(CMAKE_CXX_STANDARD 11)

#eigen3
find_package(Eigen3 REQUIRED)

#pybind11
find_package(pybind11)

#libfranka
find_package(catkin REQUIRED COMPONENTS franka_control)

###############
# C++ library #
###############

add_library(franka_real_time_cpp SHARED
source/cartesian_controller.cpp
source/robot.cpp)

target_include_directories(franka_real_time_cpp PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(franka_real_time_cpp PUBLIC ${franka_control_INCLUDE_DIRS})

target_link_directories(franka_real_time_cpp PUBLIC ${franka_control_LIBRARY_DIRS})
target_link_libraries(franka_real_time_cpp PUBLIC franka)

##################
# Python library #
##################


if (pybind11_FOUND)
	add_library(franka_real_time SHARED source/wrappers.cpp)
	#Not working when TARGET franka_real_time. WHY??? It wasn't like that before!!!
	target_include_directories(franka_real_time_cpp PUBLIC ${PYTHON_INCLUDE_DIRS})
	target_link_libraries(franka_real_time_cpp PUBLIC ${PYTHON_LIBRARIES})
	target_link_libraries(franka_real_time PUBLIC franka_real_time_cpp)
	set_target_properties(franka_real_time PROPERTIES PREFIX "")
	CONFIGURE_FILE(python-setup-files/__init__.py
	${CMAKE_CURRENT_BINARY_DIR}/__init__.py
	  )

	  # Ditto for the setup file.
	  CONFIGURE_FILE(python-setup-files/setup.py
	  ${CMAKE_CURRENT_BINARY_DIR}/setup.py
	  )

endif()

###########
# Example #
###########

add_executable(franka_real_time_hold
example/hold.cpp)

target_link_libraries(franka_real_time_hold PUBLIC franka_real_time_cpp)
