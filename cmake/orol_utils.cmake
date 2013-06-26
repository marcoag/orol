###############################################################################
# Pull the component parts out of the version number.
macro(DISSECT_VERSION)
    # Find version components
    string(REGEX REPLACE "^([0-9]+).*" "\\1"
        PCL_MAJOR_VERSION "${PCL_VERSION}")
    string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1"
        PCL_MINOR_VERSION "${PCL_VERSION}")
    string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1"
        PCL_REVISION_VERSION "${PCL_VERSION}")
    string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.[0-9]+(.*)" "\\1"
        PCL_CANDIDATE_VERSION "${PCL_VERSION}")
endmacro(DISSECT_VERSION)

###############################################################################
# Get the operating system information. Generally, CMake does a good job of
# this. Sometimes, though, it doesn't give enough information. This macro will
# distinguish between the UNIX variants. Otherwise, use the CMake variables
# such as WIN32 and APPLE and CYGWIN.
# Sets OS_IS_64BIT if the operating system is 64-bit.
# Sets LINUX if the operating system is Linux.
macro(GET_OS_INFO)
    string(REGEX MATCH "Linux" OS_IS_LINUX ${CMAKE_SYSTEM_NAME})
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(OS_IS_64BIT TRUE)
    else(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(OS_IS_64BIT FALSE)
    endif(CMAKE_SIZEOF_VOID_P EQUAL 8)
endmacro(GET_OS_INFO)

###############################################################################
# Set the destination directories for installing stuff.
# Sets LIB_INSTALL_DIR. Install libraries here.
# Sets BIN_INSTALL_DIR. Install binaries here.
# Sets INCLUDE_INSTALL_DIR. Install include files here, preferably in a
# subdirectory named after the library in question (e.g.
# "registration/blorgle.h")
macro(SET_INSTALL_DIRS)
  if (NOT DEFINED LIB_INSTALL_DIR)
    set(LIB_INSTALL_DIR "lib")
  endif (NOT DEFINED LIB_INSTALL_DIR)
    set(INCLUDE_INSTALL_ROOT
        "include/${PROJECT_NAME_LOWER}-${PCL_MAJOR_VERSION}.${PCL_MINOR_VERSION}")
    set(INCLUDE_INSTALL_DIR "${INCLUDE_INSTALL_ROOT}/orol")
    set(DOC_INSTALL_DIR "share/doc/${PROJECT_NAME_LOWER}-${PCL_MAJOR_VERSION}.${PCL_MINOR_VERSION}")
    set(BIN_INSTALL_DIR "bin")
    set(PKGCFG_INSTALL_DIR "${LIB_INSTALL_DIR}/pkgconfig")
    if(WIN32)
        set(PCLCONFIG_INSTALL_DIR "cmake")
    else(WIN32)
        set(PCLCONFIG_INSTALL_DIR "share/${PROJECT_NAME_LOWER}-${PCL_MAJOR_VERSION}.${PCL_MINOR_VERSION}")
    endif(WIN32)
endmacro(SET_INSTALL_DIRS)