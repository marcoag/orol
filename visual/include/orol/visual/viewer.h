#ifndef VIEWER_H
#define VIEWER_H

#include <QWidget>

#include <pcl/io/openni_grabber.h>

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
  
public:
  Viewer(string innermodelMap);
  ~Viewer();
  bool event(QEvent *e);
  
  void addPointCloud(pcl::PointCloud<PointT>::Ptr cloud);
  void resizeEvent(QResizeEvent * event);

public slots:
  void update();
 

};
#endif