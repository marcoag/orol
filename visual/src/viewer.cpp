#include <orol/visual/viewer.h>

Viewer::Viewer(string innermodelMap) : QWidget()
{
  innermodel = new InnerModel(innermodelMap);
  
  QGLFormat fmt;
  fmt.setDoubleBuffer(true);
  QGLFormat::setDefaultFormat(fmt);
  world3D = new OsgView(this);
  world3D->init();
  
  innermodelviewer = new InnerModelViewer(innermodel, "root", world3D->getRootGroup());
  
  world3D->getRootGroup()->addChild(innermodelviewer);
  world3D->show();
  world3D->setHomePosition(QVecToOSGVec(QVec::vec3(0,2000,-2000)), QVecToOSGVec(QVec::vec3(0,0,6000)), QVecToOSGVec(QVec::vec3(0,8000,-2000)), false);
  this->show();
  
  innermodelmanager = new InnerModelManager(innermodel, innermodelviewer);
  
  connect (&timer, SIGNAL(timeout()),this,SLOT(update()));
  timer.start(10);
}


Viewer::~Viewer()
{
  delete(world3D);
  delete(innermodel);
  delete(innermodelviewer);
}

void Viewer::addPointCloud(pcl::PointCloud<PointT>::Ptr cloud)
{
  
  innermodelMutex.lock();
  innermodelmanager->setPointCloudData("cloud",cloud);
  innermodelMutex.unlock();
  
}

void Viewer::update()
{
    world3D->update();
}

void Viewer::resizeEvent(QResizeEvent * event)
{
  world3D->autoResize();
}