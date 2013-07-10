###############################################################################
# Add a library target.
# _name The library name.
# _component The part of OROL that this library belongs to.
# ARGN The source files for the library.
macro(OROL_ADD_LIBRARY _name _component)
    add_library(${_name} ${OROL_LIB_TYPE} ${ARGN})

    set_target_properties(${_name} PROPERTIES
        VERSION ${OROL_VERSION}
        SOVERSION ${OROL_MAJOR_VERSION}.${OROL_MINOR_VERSION}
        DEFINE_SYMBOL "OROLAPI_EXPORTS")
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${_name} PROPERTIES FOLDER "Libraries")
    endif(USE_PROJECT_FOLDERS)

    install(TARGETS ${_name}
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT orol_${_component}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT orol_${_component}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT orol_${_component})

endmacro(OROL_ADD_LIBRARY)

###############################################################################
# Add a set of include files to install.
# _component The part of OROL that the install files belong to.
# _subdir The sub-directory for these include files.
# ARGN The include files.
macro(OROL_ADD_INCLUDES _component _subdir)
    install(FILES ${ARGN} DESTINATION ${INCLUDE_INSTALL_DIR}/${_subdir}
        COMPONENT orol_${_component})
endmacro(OROL_ADD_INCLUDES)

##############################################################################
# Collect subdirectories from dirname that contains filename and store them in
#  varname.
# WARNING If extra arguments are given then they are considered as exception 
# list and varname will contain subdirectories of dirname that contains 
# fielename but doesn't belong to exception list.
# dirname IN parent directory
# filename IN file name to look for in each subdirectory of parent directory
# varname OUT list of subdirectories containing filename
# exception_list OPTIONAL and contains list of subdirectories not to account
macro(collect_subproject_directory_names dirname filename names dirs)
    file(GLOB globbed RELATIVE "${dirname}" "${dirname}/*/${filename}")
    if(${ARGC} GREATER 3)
        set(exclusion_list ${ARGN})
        foreach(file ${globbed})
            get_filename_component(dir ${file} PATH)
            list(FIND exclusion_list ${dir} excluded)
            if(excluded EQUAL -1)
                set(${dirs} ${${dirs}} ${dir})
            endif(excluded EQUAL -1)
        endforeach()
    else(${ARGC} GREATER 3)
        foreach(file ${globbed})
            get_filename_component(dir ${file} PATH)
            set(${dirs} ${${dirs}} ${dir})
        endforeach(file)      
    endif(${ARGC} GREATER 3)
    foreach(subdir ${${dirs}})
        file(STRINGS ${dirname}/${subdir}/CMakeLists.txt name REGEX "set.*SUBSYS_NAME .*\\)$")
        string(REGEX REPLACE "set.*SUBSYS_NAME" "" name "${name}")
        string(REPLACE ")" "" name "${name}")
        string(STRIP "${name}" name)
#       message(STATUS "setting ${subdir} component name to ${name}")
        set(${names} ${${names}} ${name})
        file(STRINGS ${dirname}/${subdir}/CMakeLists.txt DEPENDENCIES REGEX "set.*SUBSYS_DEPS .*\\)")
        string(REGEX REPLACE "set.*SUBSYS_DEPS" "" DEPENDENCIES "${DEPENDENCIES}")
        string(REPLACE ")" "" DEPENDENCIES "${DEPENDENCIES}")
        string(STRIP "${DEPENDENCIES}" DEPENDENCIES)
        string(REPLACE " " ";" DEPENDENCIES "${DEPENDENCIES}")
        if(NOT("${DEPENDENCIES}" STREQUAL ""))
            list(REMOVE_ITEM DEPENDENCIES "#")
            string(TOUPPER "OROL_${name}_DEPENDS" SUBSYS_DEPENDS)
            set(${SUBSYS_DEPENDS} ${DEPENDENCIES})
            foreach(dependee ${DEPENDENCIES})
                string(TOUPPER "OROL_${dependee}_DEPENDIES" SUBSYS_DEPENDIES)
                set(${SUBSYS_DEPENDIES} ${${SUBSYS_DEPENDIES}} ${name})
            endforeach(dependee)
        endif(NOT("${DEPENDENCIES}" STREQUAL ""))
    endforeach(subdir)
endmacro()

###############################################################################
# Add an executable target.
# _name The executable name.
# _component The part of OROL that this library belongs to.
# ARGN the source files for the library.
macro(OROL_ADD_EXECUTABLE _name _component)
    add_executable(${_name} ${ARGN})
    # must link explicitly against boost.
    if(UNIX AND NOT ANDROID)
      target_link_libraries(${_name} ${Boost_LIBRARIES} pthread m ${CLANG_LIBRARIES})
    else()
      target_link_libraries(${_name} ${Boost_LIBRARIES})
    endif()
    #
    # Only link if needed
    if(WIN32 AND MSVC)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF
                                                DEBUG_OUTPUT_NAME ${_name}${CMAKE_DEBUG_POSTFIX}
                                                RELEASE_OUTPUT_NAME ${_name}${CMAKE_RELEASE_POSTFIX})
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
      endif()
    elseif(__COMPILER_PATHSCALE)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    else()
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
    endif()
    #
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${_name} PROPERTIES FOLDER "Tools and demos")
    endif(USE_PROJECT_FOLDERS)

    set(OROL_EXECUTABLES ${OROL_EXECUTABLES} ${_name})
    install(TARGETS ${_name} RUNTIME DESTINATION ${BIN_INSTALL_DIR}
        COMPONENT pcl_${_component})
endmacro(OROL_ADD_EXECUTABLE)

###############################################################################
# Get the include directory name of a subsystem - return _name if not set
# _var Destination variable.
# _name Name of the subsystem.
macro(OROL_GET_SUBSYS_INCLUDE_DIR _var _name)
    GET_IN_MAP(${_var} OROL_SUBSYS_INCLUDE ${_name})
    if(NOT ${_var})
      set (${_var} ${_name})
    endif(NOT ${_var})
endmacro(OROL_GET_SUBSYS_INCLUDE_DIR)

###############################################################################
# Make one subsystem depend on one or more other subsystems, and disable it if
# they are not being built.
# _var The cumulative build variable. This will be set to FALSE if the
#   dependencies are not met.
# _name The name of the subsystem.
# ARGN The subsystems and external libraries to depend on.
macro(OROL_SUBSYS_DEPEND _var _name)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs DEPS EXT_DEPS OPT_DEPS)
    cmake_parse_arguments(SUBSYS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
    if(SUBSYS_DEPS)
        SET_IN_GLOBAL_MAP(OROL_SUBSYS_DEPS ${_name} "${SUBSYS_DEPS}")
    endif(SUBSYS_DEPS)
    if(SUBSYS_EXT_DEPS)
        SET_IN_GLOBAL_MAP(OROL_SUBSYS_EXT_DEPS ${_name} "${SUBSYS_EXT_DEPS}")
    endif(SUBSYS_EXT_DEPS)
    if(SUBSYS_OPT_DEPS)
        SET_IN_GLOBAL_MAP(OROL_SUBSYS_OPT_DEPS ${_name} "${SUBSYS_OPT_DEPS}")
    endif(SUBSYS_OPT_DEPS)
    set(${test}  ("${subsys_status}" STREQUAL ""))
          message(${test} " test")
    
    foreach(_dep ${SUBSYS_DEPS})
            OROL_GET_SUBSYS_INCLUDE_DIR(_include_dir ${_dep})
            include_directories(${PROJECT_SOURCE_DIR}/${_include_dir}/include)
    endforeach(_dep)

endmacro(OROL_SUBSYS_DEPEND)
