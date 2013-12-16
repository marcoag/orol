set(SUBSYS_NAME fitting)
set(SUBSYS_DESC "Basic shapes fitting library")
set(SUBSYS_DEPS shapes)

set(srcs
      src/naive_rect_prism_fitting.cpp
      src/mcmc_rect_prism_fitting.cpp
      src/rect_prism_cloud_particle.cpp
      src/pf_rect_prism_fitting.cpp
   )
   
set(incs
      include/orol/${SUBSYS_NAME}/fitting.h
      include/orol/${SUBSYS_NAME}/naive_rect_prism_fitting.h
      include/orol/${SUBSYS_NAME}/mcmc_rect_prism_fitting.h
      include/orol/${SUBSYS_NAME}/rect_prism_cloud_particle.h
      include/orol/${SUBSYS_NAME}/pf_rect_prism_fitting.h
   )
<<<<<<< HEAD
 set(pf_incs
      include/orol/${SUBSYS_NAME}/particle_filter/particle_filter.h
      include/orol/${SUBSYS_NAME}/particle_filter/biased_random_selector.h
    )
    
=======

set(pf_incs
      include/orol/${SUBSYS_NAME}/particle_filter/particle_filter.h
      include/orol/${SUBSYS_NAME}/particle_filter/biased_random_selector.h
   )

>>>>>>> 2e2e466b9c56587644e8d1004820c6887007e1e3
INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS(${QT_DEFINITIONS})

OROL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS)
set(LIB_NAME orol_${SUBSYS_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include/particle_filter )

<<<<<<< HEAD
OROL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${pf_incs})
OROL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
OROL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/particle_filter ${pf_incs})
=======
OROL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${pf_incs} )
OROL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
OROL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/particle_filter ${pf_incs} )
>>>>>>> 2e2e466b9c56587644e8d1004820c6887007e1e3
target_link_libraries(${LIB_NAME} ${LIB_QMAT} -lqmat ${QT_LIBRARIES} -lgsl -lgslcblas -lm ${PCL_LIBRARIES})
