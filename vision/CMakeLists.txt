set(SUBSYS_NAME vision)

set(SUBSYS_DESC "Orol vision library")

set(srcs

   )
   
set(incs

   )

set(LIB_NAME orol_${SUBSYS_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

OROL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
OROL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
target_link_libraries(${LIB_NAME} ${LIB_QMAT} -lqmat ${QT_LIBRARIES} -lgsl -lgslcblas -lm )