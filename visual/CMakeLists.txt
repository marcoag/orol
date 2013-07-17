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

# ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB -DQT_XML_LIB -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED )
# FIND_PACKAGE( Qt4 REQUIRED )
# set( QT_USE_QTXML true )
# set( QT_USE_QTOPENGL true )
# QT4_WRAP_CPP( MOC_SOURCES ${srcs} )
#    
set(LIB_NAME orol_${SUBSYS_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include/innermodel ${CMAKE_CURRENT_SOURCE_DIR}/include/osgviewer)
# 
# OROL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${MOC_SOURCES} ${srcs} ${incs})
# OROL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
# 
# target_link_libraries( ${LIB_NAME} ${LIBS} ${LIB_QMAT} -lqmat ${QT_LIBRARIES} ${PCL_USER_LIBRARIES} -lrobocomp_innermodel -lqmat -losgGA -losgText -losgDB -losgUtil -losg -lgsl -lblas  -losgViewer -lgslcblas -lm  ${OSG_LIBRARY} ${OSGVIEWER_LIBRARY})


set(CMAKE_CXX_FLAGS "-Wall -Wfatal-errors -fopenmp -frounding-math" )

include_directories(.)

find_package(PCL 1.6 REQUIRED COMPONENTS common )
find_package(CGAL QUIET COMPONENTS Core )
include_directories(${PCL_INCLUDE_DIRS})
include_directories(/usr/include/pcl-1.6 /usr/include/vtk-5.8 /opt/robocomp/include/ /usr/include/ni/ /usr/include/boost/ /usr/include/ )
link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib /opt/robocomp/lib)
add_definitions(${PCL_DEFINITIONS})

ADD_DEFINITIONS( -msse2 -O3 )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")

# OpenMP
find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
elseif(OPENMP_FOUND)
  message(FATAL shit)
endif(OPENMP_FOUND)


# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB -DQT_XML_LIB -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED )
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTXML TRUE )
SET( QT_USE_QTOPENGL TRUE )

INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${incs} )

# PCL
SET ( PCL_USER_LIBRARIES  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} -lpcl_search -lpcl_common -lpcl_io -lpcl_filters -lpcl_kdtree -lpcl_octree -lpcl_segmentation -lgsl -lblas -lpcl_visualization -lpcl_sample_consensus -lpcl_features -lvtkCommon -lvtkFiltering)

OROL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${MOC_SOURCES} ${srcs} ${incs})
OROL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/innermodel ${SUBSYS_NAME}/osgviewer ${incs})

TARGET_LINK_LIBRARIES( ${LIB_NAME} ${LIBS} ${QT_LIBRARIES} ${PCL_USER_LIBRARIES} -lrobocomp_innermodel -lqmat -losgGA -losgText -losgDB -losgUtil -losg -losgViewer -lgsl -lblas -lm -lgslcblas )