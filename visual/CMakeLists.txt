set(SUBSYS_NAME visual)

set(SUBSYS_DESC "Orol visualization library")

set(srcs
    src/innermodel/innermodel.cpp
    src/innermodel/innermodelManager.cpp
    src/innermodel/innermodelreader.cpp
    src/innermodel/innermodelviewer.cpp
    src/osgviewer/adapterwidget.cpp
    src/osgviewer/findnamednode.cpp
    src/osgviewer/getworldcoorofnode.cpp
    src/osgviewer/osgview.cpp
    src/osgviewer/viewerqt.cpp
    src/viewer.cpp
   )
   
set(incs
    include/orol/${SUBSYS_NAME}/innermodel/innermodel.h
    include/orol/${SUBSYS_NAME}/innermodel/innermodelManager.h
    include/orol/${SUBSYS_NAME}/innermodel/innermodelreader.h
    include/orol/${SUBSYS_NAME}/innermodel/innermodelviewer.h
    include/orol/${SUBSYS_NAME}/osgviewer/adapterwidget.h
    include/orol/${SUBSYS_NAME}/osgviewer/findnamednode.h
    include/orol/${SUBSYS_NAME}/osgviewer/getworldcoorofnode.h
    include/orol/${SUBSYS_NAME}/osgviewer/osgview.h
    include/orol/${SUBSYS_NAME}/osgviewer/viewerqt.h
    include/orol/${SUBSYS_NAME}/viewer.h
   )

set(LIB_NAME orol_${SUBSYS_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

OROL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
OROL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
target_link_libraries(${LIB_NAME} ${LIB_QMAT} -lqmat ${QT_LIBRARIES} -lgsl -lgslcblas -lm )