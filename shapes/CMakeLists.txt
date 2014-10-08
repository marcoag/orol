set(SUBSYS_NAME shapes)

set(SUBSYS_DESC "Basic shapes library")

set(srcs
      src/axis.cpp
      src/cylinder.cpp
      src/rectprism.cpp
      src/vector.cpp
   )
   
set(incs
      include/orol/${SUBSYS_NAME}/axis.h
      include/orol/${SUBSYS_NAME}/cylinder.h
      include/orol/${SUBSYS_NAME}/rectprism.h
      include/orol/${SUBSYS_NAME}/vector.h
      include/orol/${SUBSYS_NAME}/codes.h      
   )

INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS(${QT_DEFINITIONS})
   
set(LIB_NAME orol_${SUBSYS_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

OROL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
OROL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
target_link_libraries(${LIB_NAME} ${LIB_QMAT} -lrobocomp_qmat ${QT_LIBRARIES} -lgsl -lgslcblas -lm )
