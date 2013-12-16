#ifndef VIEWER_H
#define VIEWER_H

#include <QWidget>

#include <pcl/io/openni_grabber.h>

// #include <orol/visual/rcdraw.h>
#include <orol/visual/osgviewer/osgview.h>
#include <orol/visual/innermodel/innermodel.h>
#include <orol/visual/innermodel/innermodelviewer.h>
#include <orol/visual/innermodel/innermodelManager.h>

#include <string>

using namespace std;

typedef pcl::PointXYZRGBA PointT;

class Viewer: public QWidget
{
Q_OBJECT

  QTimer timer;
  OsgView *world3D;
  InnerModelViewer *innermodelviewer;
  InnerModel *innermodel;
  InnerModelManager *innermodelmanager;
  QMutex innermodelMutex;
  
//   QFrame *frameRGB;
//   
//   QImage *qImgRGB;
//   RCDraw *drawRGB ;
//   osg::ref_ptr<osg::Image> osgImage;
  
public:
  Viewer(string innermodelMap);
  ~Viewer();
  
  void setPointCloud(pcl::PointCloud<PointT>::Ptr cloud);
  void resizeEvent(QResizeEvent * event);
  
  void setPose(std::string item,  QVec t,  QVec r,  QVec s);
  void setScale(std::string item, float scaleX,float scaleY, float scaleZ);
  
<<<<<<< HEAD
  void showImage( uint32_t width, uint32_t height, uint8_t *rgb_image);
=======
  void showImage( int32_t width, int32_t height, uint8_t *rgb_image);
>>>>>>> 2e2e466b9c56587644e8d1004820c6887007e1e3

public slots:
  void update();
 

};
#endif