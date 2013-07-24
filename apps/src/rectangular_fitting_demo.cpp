#include <orol/shapes/rectprism.h>
#include <orol/fitting/naive_rect_prism_fitting.h>
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
    inline fitterViewer():v("cubeCloud.xml") {}
    
    void fit_cb (const boost::shared_ptr<RectPrism>  &shape)
    {
      v.setPose("cube_0_t", shape->getCenter(), shape->getRotation(), shape->getWidth() );
      v.setScale("cube_0", shape->getWidth()(0)/2, shape->getWidth()(1)/2, shape->getWidth()(2)/2);
    }
    
    void run(boost::shared_ptr<RectPrism> shape, pcl::PointCloud<PointT>::Ptr cloud)
    {
       QApplication app(0,NULL);
      
       naiveRectangularPrismFitting* fitter = new naiveRectangularPrismFitting(shape, cloud);

       boost::function<void (const boost::shared_ptr<RectPrism>&)> f =
         boost::bind (&fitterViewer::fit_cb, this, _1);

       fitter->registerCallback (f);

       fitter->start ();

       app.exec();
    }
    
    Viewer v;
    
};

  
int main (int argc, char* argv[])
{
  pcl::PointCloud<PointT>::Ptr cloud2fit;
  boost::shared_ptr<RectPrism> rectangular_prism (new RectPrism()); 
  
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  
  
  //Create sintetic cube
  cloud2fit = sinteticCubeCloud (100,100,400,50);
  
  //calculate centroid, eigen values and eigen vectors
  pcl::computeMeanAndCovarianceMatrix(*cloud2fit, covariance_matrix, centroid);
  pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values);
  
  //calculate ratio to resize the eigen_value 
  float max_distance=0;
  float max_eigenvalue=0;
  if(max_eigenvalue - eigen_values(0) < 0)
    max_eigenvalue=eigen_values(0);
  if(max_eigenvalue - eigen_values(1) < 0)
    max_eigenvalue=eigen_values(1);
  if(max_eigenvalue - eigen_values(2) < 0)
    max_eigenvalue=eigen_values(2);
  for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloud2fit->points.begin (); it < cloud2fit->points.end (); ++it)
  {
    float distance=fabs(sqrt(pow((it->x)-centroid(0),2.0)+pow((it->y)-centroid(2),2.0)+pow((it->z)-centroid(1),2.0)));
    if (max_distance<distance)
      max_distance=distance;
  }
  float ratio=max_eigenvalue/max_distance;
  
  //set initial values for the rectangular prism
  rectangular_prism->setCenter(QVec::vec3(centroid(0), centroid(1), centroid(2)));
  rectangular_prism->setWidth(QVec::vec3((eigen_values(0)/ratio),(eigen_values(0)/ratio),(eigen_values(0)/ratio)));
  ///TODO make rotation dependant on the eigen_vectors
  rectangular_prism->setRotation(QVec::vec3(0,0,0));
  
//   naiveRectangularPrismFitting fitter(rectangular_prism, cloud2fit);
//    
//   rectangular_prism->getCenter().print("center");
//   rectangular_prism->getRotation().print("rotation");
//   rectangular_prism->getWidth().print("width");
//   
//   fitter.adapt();
//   
//   Viewer v(argv[1]);
//   v.addPointCloud(cloud2fit);
//   
//   //set the rectangular_prism size and pose
//   rectangular_prism->getCenter().print("center");
//   rectangular_prism->getRotation().print("rotation");
//   rectangular_prism->getWidth().print("width");
//   
//   rectangular_prism=fitter.getRectangularPrism();
//   
//   v.setPose("cube_0_t", rectangular_prism->getCenter(), rectangular_prism->getRotation(), rectangular_prism->getWidth() );
//   v.setScale("cube_0", rectangular_prism->getWidth()(0)/2, rectangular_prism->getWidth()(1)/2, rectangular_prism->getWidth()(2)/2);
//   
//   app.exec();
  
  fitterViewer f;
  f.run(rectangular_prism, cloud2fit);
  
}