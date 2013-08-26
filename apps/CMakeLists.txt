set(SUBSYS_NAME apps)
set(SUBSYS_DESC "Application examples/samples")
set(SUBSYS_DEPS shapes fitting visual)

#OROL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
OROL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS)

include_directories (${CMAKE_CURRENT_BINARY_DIR})
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/include)

OROL_ADD_EXECUTABLE(orol_naive_rectangular_prism_fitting ${SUBSYS_NAME} src/naive_rectangular_prism_fitting.cpp)
target_link_libraries(orol_naive_rectangular_prism_fitting orol_shapes orol_fitting orol_visual)

OROL_ADD_EXECUTABLE(orol_live_naive_rectangular_prism_fitting ${SUBSYS_NAME} src/live_naive_rectangular_prism_fitting.cpp)
target_link_libraries(orol_live_naive_rectangular_prism_fitting orol_shapes orol_fitting orol_visual)

OROL_ADD_EXECUTABLE(orol_mcmc_rectangular_prism_fitting ${SUBSYS_NAME} src/mcmc_rectangular_prism_fitting.cpp)
target_link_libraries(orol_mcmc_rectangular_prism_fitting orol_shapes orol_fitting orol_visual)

OROL_ADD_EXECUTABLE(orol_live_mcmc_rectangular_prism_fitting ${SUBSYS_NAME} src/live_mcmc_rectangular_prism_fitting.cpp)
target_link_libraries(orol_live_mcmc_rectangular_prism_fitting orol_shapes orol_fitting orol_visual)

