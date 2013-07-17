set(SUBSYS_NAME fitting)
set(SUBSYS_DESC "Basic shapes fitting library")
set(SUBSYS_DEPS shapes)

set(srcs
      src/naive_rect_prism_fitting.cpp
   )
   
set(incs
      include/orol/${SUBSYS_NAME}/naive_rect_prism_fitting.h
   )

OROL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS)
set(LIB_NAME orol_${SUBSYS_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

OROL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
OROL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
target_link_libraries(${LIB_NAME} ${LIB_QMAT} -lqmat ${QT_LIBRARIES} -lgsl -lgslcblas -lm ${PCL_LIBRARIES})