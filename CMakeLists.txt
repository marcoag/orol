### ---[ OROL global CMake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "possible configurations" FORCE)

# In case the user does not setup CMAKE_BUILD_TYPE, assume it's Rel WithDebInfo
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type default to RelWithDebInfo, set to Release to improve performance" FORCE)
endif("${CMAKE_BUILD_TYPE}" STREQUAL "")

project(OROL)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

### ---[ Find universal dependencies
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

# ---[ Release/Debug specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_definitions("-DBOOST_DISABLE_ASSERTS -DEIGEN_NO_DEBUG")
endif()

# Update the documentation string of CMAKE_BUILD_TYPE for GUIs
SET(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel Maintainer."
    FORCE)

include(${OROL_SOURCE_DIR}/cmake/orol_targets.cmake)

# ---[ Unix/Darwin/Windows specific flags
if(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unknown-pragmas -Wconversion -fno-strict-aliasing -Wno-format-extra-args -Wno-sign-compare -Wno-invalid-offsetof")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O4")

  # Enable -Wabi for GCC > 4.3, and -Wno-deprecated for GCC < 4.3 
  # to disable a lot of warnings which are not fixable
  execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if (GCC_VERSION VERSION_GREATER 4.3)
    message(STATUS "-- GCC > 4.3 found, enabling -Wabi")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi")
  else()
    message(STATUS "-- GCC < 4.3 found, enabling -Wno-deprecated")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")
  endif ()


  if(NOT ANDROID)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  endif(NOT ANDROID)
endif()

# ---[ Project folders
option(USE_PROJECT_FOLDERS "Use folders to organize OROL projects in an IDE." OFF)
mark_as_advanced(USE_PROJECT_FOLDERS)
if(USE_PROJECT_FOLDERS)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif(USE_PROJECT_FOLDERS)

include(${OROL_SOURCE_DIR}/cmake/orol_utils.cmake)
set(OROL_VERSION 0.0.1 CACHE STRING "OROL version")
DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

set(OROL_OUTPUT_LIB_DIR ${OROL_BINARY_DIR}/${LIB_INSTALL_DIR})
set(OROL_OUTPUT_BIN_DIR ${OROL_BINARY_DIR}/${BIN_INSTALL_DIR})
make_directory(${OROL_OUTPUT_LIB_DIR})
make_directory(${OROL_OUTPUT_BIN_DIR})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OROL_OUTPUT_LIB_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OROL_OUTPUT_BIN_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OROL_OUTPUT_LIB_DIR}")

# Add an "uninstall" target
# configure_file("${OROL_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
#                "${OROL_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
# add_custom_target(uninstall "${CMAKE_COMMAND}" -P
#                   "${OROL_BINARY_DIR}/uninstall_target.cmake")

### ---[ Find universal dependencies
# the gcc-4.2.1 coming with MacOS X is not compatible with the OpenMP pragmas we use, so disabling OpenMP for it
if((NOT APPLE) OR (NOT CMAKE_COMPILER_IS_GNUCXX) OR (GCC_VERSION VERSION_GREATER 4.2.1))
  find_package(OpenMP)
endif()
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  message (STATUS "Found OpenMP")
else(OPENMP_FOUND)
  message (STATUS "Not found OpenMP")
endif()

# OROL (required)
find_package(PCL 1.7)
include_directories(${PCL_INCLUDE_DIRS})

# OROL QMat
include(${OROL_SOURCE_DIR}/cmake/modules/FindQMat.cmake)

# Find QT4
find_package(Qt4 REQUIRED)
if (Qt4_FOUND)
  include(${QT_USE_FILE})
  add_definitions("-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED")
endif (Qt4_FOUND)


#Find Doxygen and html help compiler if any
find_package(Doxygen)
if(DOXYGEN_FOUND)
  find_package(HTMLHelp)
endif(DOXYGEN_FOUND)

### ---[ Create the config.h file
# set(orol_config_h_in "${CMAKE_CURRENT_SOURCE_DIR}/orol_config.h.in")
# set(orol_config_h "${CMAKE_CURRENT_BINARY_DIR}/include/orol/orol_config.h")
# configure_file(${orol_config_h_in} ${orol_config_h})
# OROL_ADD_INCLUDES(${orol_config_h})
# include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

### ---[ Set up for tests
enable_testing()

### ---[ Add the libraries subdirectories
include(${OROL_SOURCE_DIR}/cmake/orol_targets.cmake)

collect_subproject_directory_names(${OROL_SOURCE_DIR} "CMakeLists.txt" OROL_MODULES_NAMES OROL_MODULES_DIRS doc)
set(OROL_MODULES_NAMES_UNSORTED ${OROL_MODULES_NAMES})
topological_sort(OROL_MODULES_NAMES OROL_ _DEPENDS)
sort_relative(OROL_MODULES_NAMES_UNSORTED OROL_MODULES_NAMES OROL_MODULES_DIRS)
foreach(subdir ${OROL_MODULES_DIRS})
  add_subdirectory(${OROL_SOURCE_DIR}/${subdir})  
endforeach(subdir)

### ---[ Documentation
#add_subdirectory(doc)
