#include <orol/shapes/rectprism.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZRGBA PointT;

class RectPrismFitting
{
  pcl::PointCloud<PointT>::Ptr cloud2fit;
  RectPrism rectangularPrism;

public:
  
  RectPrismFitting(): cloud2fit(new pcl::PointCloud<PointT>())
  , rectangularPrism ()
  {
    
  }

  //creates a sintetic cube 
  void useSinteticCube(int Wx, int Wy, int Wz, int res)
  {
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
        cloud2fit->push_back(p);
        p.x = x;
        p.y = y;
        p.z = Wz;  
        p.r = 0;
        p.g = 255;
        p.b = 0;
        cloud2fit->push_back(p);
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
        cloud2fit->push_back(p);
        p.x = x;//+RectPrismCloudParticle::getRandom(10);
        p.y = Wy;//+RectPrismCloudParticle::getRandom(10);;
        p.z = z;//+RectPrismCloudParticle::getRandom(10);
        p.r = 0;
        p.g = 255;
        p.b = 0;
        cloud2fit->push_back(p);
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
        cloud2fit->push_back(p);
        p.x = Wx;//+RectPrismCloudParticle::getRandom(10);
        p.y = y;//+RectPrismCloudParticle::getRandom(10);
        p.z = z;//+RectPrismCloudParticle::getRandom(10);
        p.r = 0;
        p.g = 255;
        p.b = 0;
        cloud2fit->push_back(p);
      }
    }
  }
  
  //moves the cloud to X, Y, Z and rotate
  void moveTheCloud(float X, float Y, float Z)
  {
    Eigen::Matrix4f TransMat; 
    TransMat <<       1,    0,   0,  X, 
                      0,    -0.4161,   -0.9093,  Y, 
                      0,    0.9093,   -0.4161,  Z, 
                      0,    0,   0,  1; 
                      
    pcl::transformPointCloud(*cloud2fit,*cloud2fit,TransMat ); 
  }
  
};
  
int main (int argc, char argv[])
{
  RectPrismFitting R;
  R.useSinteticCube(100,100,400,50);
  
}