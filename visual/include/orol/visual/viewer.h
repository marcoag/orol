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

class Viewer: public QWidget
{
Q_OBJECT

  QTimer timer;
  OsgView *world3D;
  InnerModelManager *innerModelManager;
  QMutex innermodelMutex;
  
public:
  Viewer();
  ~Viewer();
  
    void resizeEvent(QResizeEvent * event);

public slots:
  void update();
 

};
#endif