set(SUBSYS_NAME apps)
set(SUBSYS_DESC "Application examples/samples")
set(SUBSYS_DEPS shapes fitting)

#OROL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
OROL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS)

include_directories (${CMAKE_CURRENT_BINARY_DIR})
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/include)

OROL_ADD_EXECUTABLE(orol_rectangular_fitting_demo ${SUBSYS_NAME} src/rectangular_fitting_demo.cpp)
target_link_libraries(orol_rectangular_fitting_demo orol_shapes orol_fitting )


