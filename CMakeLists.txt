#################
# Configuretion #
#################

cmake_minimum_required(VERSION 3.14.0)
cmake_policy(SET CMP0048 NEW)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

project(Franka_timeout_handler VERSION 1.1.0)

################
# Dependencies #
################

#c++
set(CMAKE_CXX_STANDARD 11)

#libfranka
find_package(Franka)
if (NOT Franka_FOUND)
	find_package(catkin REQUIRED COMPONENTS franka_control)
endif()

#eigen3
find_package(Eigen3 REQUIRED)

#pybind11
find_package(pybind11)
if(pybind11_FOUND)
	exec_program(python3 ARGS -m pybind11 --includes | cut -d ' ' -f1 | sed 's+-I/+/+g' OUTPUT_VARIABLE pybind11_INCLUDE_DIR1)
	exec_program(python3 ARGS -m pybind11 --includes | cut -d ' ' -f2 | sed 's+-I/+/+g' OUTPUT_VARIABLE pybind11_INCLUDE_DIR2)
	exec_program(python3 ARGS --version | cut -d ' ' -f2 | cut -d '.' -f1 OUTPUT_VARIABLE python_VERSION_MAJOR)
	exec_program(python3 ARGS --version | cut -d ' ' -f2 | cut -d '.' -f2 OUTPUT_VARIABLE python_VERSION_MINOR)
	set(pybind11_INCLUDE_DIRS ${pybind11_INCLUDE_DIR1} ${pybind11_INCLUDE_DIR2})
	unset(pybind11_INCLUDE_DIR1)
	unset(pybind11_INCLUDE_DIR2)
endif()

###########
# Library #
###########

# Franka_timeout_handler
add_library(Franka_timeout_handler SHARED
source/cartesian_controller.cpp
source/controller.cpp
source/gripper.cpp
source/joint_controller.cpp
source/robot.cpp
source/robot_core.cpp)
if (NOT "${Franka_timeout_handler_omit_include_directories}" MATCHES "yes")
	target_include_directories(Franka_timeout_handler PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
endif()
target_compile_definitions(Franka_timeout_handler PUBLIC FRANKA_TIMEOUT_HANDLER_VERSION_MAJOR=${PROJECT_VERSION_MAJOR})
target_compile_definitions(Franka_timeout_handler PUBLIC FRANKA_TIMEOUT_HANDLER_VERSION_MINOR=${PROJECT_VERSION_MINOR})
target_compile_definitions(Franka_timeout_handler PUBLIC FRANKA_TIMEOUT_HANDLER_VERSION_PATCH=${PROJECT_VERSION_PATCH})
target_include_directories(Franka_timeout_handler PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_directories(Franka_timeout_handler PUBLIC $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}> $<INSTALL_INTERFACE:lib>)
target_link_libraries(Franka_timeout_handler PUBLIC Franka::Franka)
set_target_properties(Franka_timeout_handler PROPERTIES OUTPUT_NAME "franka_timeout_handler")
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${PROJECT_SOURCE_DIR}/Franka_timeout_handler-config-version.cmake" VERSION ${PROJECT_VERSION} COMPATIBILITY AnyNewerVersion)
export(TARGETS Franka_timeout_handler FILE "${PROJECT_SOURCE_DIR}/Franka_timeout_handler-config.cmake")
install(DIRECTORY include/franka_timeout_handler TYPE INCLUDE)
install(TARGETS Franka_timeout_handler EXPORT Franka_timeout_handler-config)
install(EXPORT Franka_timeout_handler-config DESTINATION lib/cmake/Franka_timeout_handler)
install(FILES "${PROJECT_SOURCE_DIR}/Franka_timeout_handler-config-version.cmake" DESTINATION lib/cmake/Franka_timeout_handler)

# Franka_timeout_handler_python
if (pybind11_FOUND)
	add_library(Franka_timeout_handler_python SHARED source/wrappers.cpp)
	if ("${Franka_timeout_handler_omit_include_directories}" MATCHES "yes")
		target_compile_definitions(Franka_timeout_handler_python PRIVATE FRANKA_TIMEOUT_HANDLER_WRAPPERS_INSTALLED)
	endif()
	target_include_directories(Franka_timeout_handler_python PRIVATE ${pybind11_INCLUDE_DIRS})
	target_link_libraries(Franka_timeout_handler_python PRIVATE Franka_timeout_handler)
	set_target_properties(Franka_timeout_handler_python PROPERTIES PREFIX "")
	set_target_properties(Franka_timeout_handler_python PROPERTIES OUTPUT_NAME "franka_timeout_handler.cpython-${python_VERSION_MAJOR}${python_VERSION_MINOR}-${CMAKE_LIBRARY_ARCHITECTURE}")
	install(TARGETS Franka_timeout_handler_python DESTINATION "lib/python${python_VERSION_MAJOR}.${python_VERSION_MINOR}/dist-packages")
endif()

# franka_timeout_handler_hold
add_executable(Franka_timeout_handler_hold example/hold.cpp)
target_link_libraries(Franka_timeout_handler_hold PUBLIC Franka_timeout_handler)
set_target_properties(Franka_timeout_handler_hold PROPERTIES OUTPUT_NAME "franka_timeout_handler_hold")
install(TARGETS Franka_timeout_handler_hold)

# franka_timeout_handler_hold.py
configure_file(example/hold.py franka_timeout_handler_hold.py)
install(PROGRAMS example/hold.py RENAME franka_timeout_handler_hold.py TYPE BIN)

# franka_timeout_handler_wave
add_executable(Franka_timeout_handler_wave example/wave.cpp)
target_link_libraries(Franka_timeout_handler_wave PUBLIC Franka_timeout_handler)
set_target_properties(Franka_timeout_handler_wave PROPERTIES OUTPUT_NAME "franka_timeout_handler_wave")
install(TARGETS Franka_timeout_handler_wave)

# franka_timeout_handler_wave.py
configure_file(example/wave.py franka_timeout_handler_wave.py)
install(PROGRAMS example/wave.py RENAME franka_timeout_handler_wave.py TYPE BIN)
