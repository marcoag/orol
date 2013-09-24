#include <orol/visual/viewer.h>

Viewer::Viewer(string innermodelMap) : QWidget()
{
  innermodel = new InnerModel(innermodelMap);
  
//   frameRGB = new QFrame(this);
//   frameRGB->setObjectName(QString::fromUtf8("frameRGB"));
//   frameRGB->setGeometry(QRect(10, 0, 640, 480));
//   frameRGB->setMinimumSize(QSize(640, 480));
//   frameRGB->setFrameShape(QFrame::StyledPanel);
//   frameRGB->setFrameShadow(QFrame::Raised);
  
//   qImgRGB = new QImage ( 640, 480, QImage::Format_RGB888); 
//   drawRGB = new RCDraw ( 640, 480, qImgRGB, this->frameRGB);
//   
//   osgImage = new osg::Image(); 
//   osgImage.get()->allocateImage(640,480, 1, GL_RGB, GL_UNSIGNED_BYTE, 1);
  
  QGLFormat fmt;
  fmt.setDoubleBuffer(true);
  QGLFormat::setDefaultFormat(fmt);
  world3D = new OsgView(this);
  world3D->init();
  
  innermodelviewer = new InnerModelViewer(innermodel, "root", world3D->getRootGroup());
  
  world3D->getRootGroup()->addChild(innermodelviewer);

  world3D->show();
  world3D->setHomePosition(QVecToOSGVec(QVec::vec3(0,2000,-2000)), QVecToOSGVec(QVec::vec3(0,0,6000)), QVecToOSGVec(QVec::vec3(0,8000,-2000)), false);
  //world3D->setHomePosition(osg::Vec3(0,0,0),osg::Vec3(0.f,0.,-4000.),osg::Vec3(0.0f,1.f,0.0f), false);  
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

void Viewer::showImage( int32_t width, int32_t height, uint8_t  *rgb_image)
{
//   cout<<"About to show image"<<endl;
//   //memcpy(qImgRGB->bits(),rgb_image,640*480*3);
//   osgImage.get()->setImage(640,480,1,GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, rgb_image, osg::Image::NO_DELETE, 1);
//   world3D->setImageHUD(osgImage);
//   world3D->update();
//     
//   //qImgRGB->loadFromData( rgb_image, 640*480*3 );
//   memcpy(qImgRGB->bits(),rgb_image,640*480*3);    
//   drawRGB->update();
//   
//   cout<<"Showed image"<<endl;
  innermodelMutex.lock();
  innermodelmanager->setImageOnPlane(width, height, rgb_image, "back" );
  innermodelMutex.unlock();
//   cout<<(uint8_t)*rgb_image<<endl;
// 
//   innermodelviewer->planesHash["back"]->updateBuffer(rgb_image, width, height);
  
}

void Viewer::setPointCloud(pcl::PointCloud<PointT>::Ptr cloud)
{
  
  innermodelMutex.lock();
  innermodelmanager->setPointCloudData("cloud",cloud);
  innermodelMutex.unlock();
  
}

void Viewer::setPose(std::string item,  QVec t,  QVec r,  QVec s)
{
  innermodelMutex.lock();
  innermodelmanager->setPose( item, t, r, s );
  innermodelMutex.unlock();
  world3D->update();
}

void Viewer::setScale(std::string item, float scaleX,float scaleY, float scaleZ)
{
  innermodelMutex.lock();
  innermodelmanager->setScale(item, scaleX, scaleY, scaleZ);
  innermodelMutex.unlock();
  world3D->update();
}

void Viewer::update()
{
    world3D->update();
}

void Viewer::resizeEvent(QResizeEvent * event)
{
  world3D->autoResize();
}