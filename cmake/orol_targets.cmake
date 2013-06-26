###############################################################################
# Add a library target.
# _name The library name.
# _component The part of PCL that this library belongs to.
# ARGN The source files for the library.
macro(OROL_ADD_LIBRARY _name _component)
    add_library(${_name} ${PCL_LIB_TYPE} ${ARGN})

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
# _component The part of PCL that the install files belong to.
# _subdir The sub-directory for these include files.
# ARGN The include files.
macro(OROL_ADD_INCLUDES _component _subdir)
    install(FILES ${ARGN} DESTINATION ${INCLUDE_INSTALL_DIR}/${_subdir}
        COMPONENT orol_${_component})
endmacro(OROL_ADD_INCLUDES)