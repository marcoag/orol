#include <orol/shapes/rectprism.h>
#include <orol/fitting/mcmc_rect_prism_fitting.h>
#include <orol/visual/viewer.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

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

class fitterViewer
{
  public:
    inline fitterViewer() {}
    
    void fit_cb (const boost::shared_ptr<RectPrism>  &shape)
    {
      v->setPose("cube_0_t", shape->getCenter(), shape->getRotation(), shape->getWidth() );
      v->setScale("cube_0", shape->getWidth()(0)/2, shape->getWidth()(1)/2, shape->getWidth()(2)/2);
    }
    
    void run(int argc, char* argv[], pcl::PointCloud<PointT>::Ptr cloud)
    {
       QApplication app(argc, argv);
       
       boost::shared_ptr<Viewer> v_local(new Viewer("cubeCloud.xml"));
       v=v_local;
       v->setPointCloud(cloud);
      
       boost::shared_ptr<RectPrism> shape(new RectPrism());
       mcmcRectangularPrismFitting* fitter = new mcmcRectangularPrismFitting( cloud, QVec::vec3(5,5,5), QVec::vec3(5,5,5), QVec::vec3(0.1,0.1,0.1));

       boost::function<void (const boost::shared_ptr<RectPrism>&)> f =
         boost::bind (&fitterViewer::fit_cb, this, _1);

       fitter->registerCallback (f);

       fitter->start ();

       app.exec();
    }
    
    boost::shared_ptr<Viewer> v; 
    
};

  
int main (int argc, char* argv[])
{ 
  //Create sintetic cube
  pcl::PointCloud<PointT>::Ptr cloud2fit = sinteticCubeCloud (100,100,400,50);
  //create fitter
  fitterViewer f;
  
  f.run(argc, argv, cloud2fit);
  
}