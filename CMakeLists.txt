cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

################################################################################
# Setup the build mode
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "=====================================================")
message(STATUS "configuring build for PyDart")
message(STATUS "=====================================================")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release")
ENDIF(NOT CMAKE_BUILD_TYPE)

message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

IF(CMAKE_BUILD_TYPE MATCHES "Release")
  message(STATUS "Set the release mode")
  ADD_DEFINITIONS(-DNDEBUG )
  ADD_DEFINITIONS(-DBOOST_UBLAS_NDEBUG )
ENDIF (CMAKE_BUILD_TYPE MATCHES "Release")

IF(APPLE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flat_namespace -undefined suppress")
ELSEIF(UNIX)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-enum-compare")
ENDIF()
MESSAGE(STATUS "CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

################################################################################
# Find the required packages
### SWIG
find_package(SWIG REQUIRED)
message(STATUS "SWIG_FOUND = ${SWIG_FOUND}")
message(STATUS "SWIG_USE_FILE = ${SWIG_USE_FILE}")
include(${SWIG_USE_FILE})
### PythonLibs
find_package(PythonLibs REQUIRED)
message(STATUS "PYTHON_INCLUDE_DIR = ${PYTHON_INCLUDE_DIR}")
include_directories(${PYTHON_INCLUDE_DIR})
message(STATUS "PYTHON_LIBRARIES = ${PYTHON_LIBRARIES}")

### PythonLibs
find_package(NumPy REQUIRED)
message(STATUS "NUMPY_INCLUDE_DIRS = ${NUMPY_INCLUDE_DIRS}")
include_directories(${NUMPY_INCLUDE_DIRS})

### Eigen
find_package(Eigen REQUIRED)
message(STATUS "Eigen_INCLUDE_DIRS = ${Eigen_INCLUDE_DIRS}")
include_directories(${Eigen_INCLUDE_DIRS})
### OpenGL
find_package(OpenGL)
if (OPENGL_FOUND)
  message(STATUS "OPENGL FOUND")
else (OPENGL_FOUND)
  message(STATUS "OPENGL NOT FOUND")
endif ()
### GLUT
find_package(GLUT)
if (GLUT_FOUND)
  message(STATUS "GLUT FOUND")
  message(STATUS "GLUT_LIBRARIES = ${GLUT_LIBRARIES}")
else (GLUT_FOUND)
  message(STATUS "GLUT NOT FOUND")
endif ()

################################################################################
# Configure the project
#### Directories
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/pydart")

include_directories(".")
include_directories("${CMAKE_SOURCE_DIR}")
include_directories("${CMAKE_SOURCE_DIR}/pydart")
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
#### Projectfiles
file(GLOB pydart_py "pydart/*.py")
file(GLOB pydart_srcs "pydart/*.cpp")
file(GLOB pydart_hdrs "pydart/*.h")
file(GLOB pydart_interfaces "pydart/pydart_api.i")
message(STATUS "pydart_py = ${pydart_py}")
message(STATUS "pydart_hdrs = ${pydart_hdrs}")
message(STATUS "pydart_srcs = ${pydart_srcs}")
#### SWIG configurations
set(CMAKE_SWIG_FLAGS "")
set_source_files_properties(${pydart_interfaces} PROPERTIES CPLUSPLUS ON)
set_source_files_properties(${pydart_interfaces} PROPERTIES SWIG_FLAGS "-includeall")
#### SWIG commands
swig_add_module(pydart_api python ${pydart_py} ${pydart_srcs} ${pydart_hdrs} ${pydart_interfaces})
# SWIG_ADD_MODULE(pydart python ${pydart_interfaces} ${pydart_srcs})
swig_link_libraries(pydart_api ${PYTHON_LIBRARIES})
swig_link_libraries(pydart_api dart4 dart-core4 ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

# ADD_CUSTOM_COMMAND(TARGET _pydart_api
#   PRE_BUILD 
#   COMMAND echo 'Copy pydart to build directory'
#   COMMAND  ${CMAKE_COMMAND} -E copy_if_different
#   "${CMAKE_SOURCE_DIR}/pydart/pydart.py" "${CMAKE_BINARY_DIR}/")

ADD_CUSTOM_TARGET(pydart
  COMMAND echo 'Copy pydart.py to build directory if different'
  COMMAND  ${CMAKE_COMMAND} -E copy_if_different
  "${CMAKE_SOURCE_DIR}/pydart/pydart.py" "${CMAKE_BINARY_DIR}/")
ADD_DEPENDENCIES(_pydart_api pydart)
