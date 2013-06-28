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