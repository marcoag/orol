#include <orol/shapes/rectprism.h>
#include <orol/fitting/naive_rect_prism_fitting.h>
#include <orol/visual/viewer.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGBA PointT;

//creates a sintetic cube 
pcl::PointCloud<PointT>::Ptr sinteticCubeCloud(int Wx, int Wy, int Wz, int res)
{
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>());
  //Faces front and back
  for(float x=0; x<=Wx; x=x+res)
  {
    for(float y=0; y<=Wx; y=y+res)
    {
      //face front (x=0)
      pcl::PointXYZRGBA p;
      p.x = x;//+RectPrismCloudParticle::getRandom(10);
      p.y = y;//+RectPrismCloudParticle::getRandom(10);
      p.z = 0;//+RectPrismCloudParticle::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloud->push_back(p);
      p.x = x;
      p.y = y;
      p.z = Wz;  
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloud->push_back(p);
    }
  }
  
  //Faces up and down
  for(float x=0; x<=Wx; x=x+res)
  {
    for(float z=0; z<=Wz; z=z+res)
    {
      //face front (x=0)
      pcl::PointXYZRGBA p;
      p.x = x;//+RectPrismCloudParticle::getRandom(10);
      p.y = 0;//+RectPrismCloudParticle::getRandom(10);
      p.z = z;//+RectPrismCloudParticle::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloud->push_back(p);
      p.x = x;//+RectPrismCloudParticle::getRandom(10);
      p.y = Wy;//+RectPrismCloudParticle::getRandom(10);;
      p.z = z;//+RectPrismCloudParticle::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloud->push_back(p);
    }
  }
  
  //Faces right and left
  for(float y=0; y<=Wy; y=y+res)
  {
    for(float z=0; z<=Wz; z=z+res)
    {
      //face front (x=0)
      pcl::PointXYZRGBA p;
      p.x = 0;//+RectPrismCloudParticle::getRandom(10);
      p.y = y;//+RectPrismCloudParticle::getRandom(10);
      p.z = z;//+RectPrismCloudParticle::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloud->push_back(p);
      p.x = Wx;//+RectPrismCloudParticle::getRandom(10);
      p.y = y;//+RectPrismCloudParticle::getRandom(10);
      p.z = z;//+RectPrismCloudParticle::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloud->push_back(p);
    }
  }
  return cloud;
}
  
//moves the cloud to X, Y, Z and rotate
void moveACloud(pcl::PointCloud<PointT>::Ptr cloud2move, float X, float Y, float Z)
{
  Eigen::Matrix4f TransMat; 
  TransMat <<       1,    0,   0,  X, 
                    0,    -0.4161,   -0.9093,  Y, 
                    0,    0.9093,   -0.4161,  Z, 
                    0,    0,   0,  1; 
                    
  pcl::transformPointCloud(*cloud2move,*cloud2move,TransMat );
}

void translateClouds(pcl::PointCloud<PointT>::Ptr c_dest, 
                     const pcl::PointCloud<PointT>::ConstPtr &c_org )
{
  c_dest->clear();
  for(pcl::PointCloud<PointT>::const_iterator it = c_org->begin(); it != c_org->end(); it++)
  {
    pcl::PointXYZRGBA p;
    p.x=-it->x*1000;
    p.y=-it->y*1000;
    p.z=it->z*1000;
    p.r=it->r;
    p.g=it->g;
    p.b=it->b;
    c_dest->push_back(p);
  }
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (c_dest);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*c_dest);
}

class fitterViewer
{
  public:
    fitterViewer():v(new Viewer("cubeCloud.xml"))
    ,cloud_buff(new pcl::PointCloud<pcl::PointXYZRGBA>)
    {}
    
    void fit_cb (const boost::shared_ptr<RectPrism>  &shape)
    {
      v->setPose("cube_0_t", shape->getCenter(), shape->getRotation(), shape->getWidth() );
      v->setScale("cube_0", shape->getWidth()(0)/2, shape->getWidth()(1)/2, shape->getWidth()(2)/2);
    }
    
    void cloud_cb (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      
      cloud_mutex.lock();      
      translateClouds(cloud_buff, cloud);
      //*cloud_buff=*cloud;
      v->setPointCloud(cloud_buff);
 
      
      cloud_mutex.unlock();
      
    }
    
    void run(pcl::PointCloud<PointT>::Ptr cloud)
    {

       
//        boost::shared_ptr<Viewer> v_local(new Viewer("cubeCloud.xml"));
//        v=v_local;
      
       boost::shared_ptr<RectPrism> shape(new RectPrism());
       naiveRectangularPrismFitting* fitter = new naiveRectangularPrismFitting( cloud );

       boost::function<void (const boost::shared_ptr<RectPrism>&)> f =
         boost::bind (&fitterViewer::fit_cb, this, _1);

       fitter->registerCallback (f);

       fitter->start ();
       
       //openni grabber:
       pcl::Grabber* interface = new pcl::OpenNIGrabber();
       
       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f_kinect =
         boost::bind (&fitterViewer::cloud_cb, this, _1);
         
      interface->registerCallback(f_kinect);

      interface->start ();

    }
    
    boost::shared_ptr<Viewer> v;
    QMutex cloud_mutex;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_buff;
    
};

  
int main (int argc, char* argv[])
{ 
  
  QApplication app(argc, argv);
  //Create sintetic cube
  pcl::PointCloud<PointT>::Ptr cloud2fit = sinteticCubeCloud (100,100,400,50);
  //create fitter
  fitterViewer f;
  
  f.run(cloud2fit);
  
  app.exec();
}