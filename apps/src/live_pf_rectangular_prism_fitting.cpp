#include <orol/shapes/rectprism.h>
#include <orol/fitting/pf_rect_prism_fitting.h>
#include <orol/visual/viewer.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/openni_camera/openni_image.h>

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
//       //face front (x=0)
      PointT p;
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
      PointT p;
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
      PointT p;
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
  
  pcl::PointCloud<PointT>::Ptr cloud_clean (new pcl::PointCloud<PointT>);
  pcl::PointCloud<int> sampled_indices;
  pcl::UniformSampling<PointT> uniform_sampling;
  uniform_sampling.setInputCloud (c_org);
  uniform_sampling.setRadiusSearch (0.01f);
  uniform_sampling.compute (sampled_indices);
  pcl::copyPointCloud (*c_org, sampled_indices.points, *cloud_clean);
  
  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
  pcl::SampleConsensusModelPlane<PointT>::Ptr
    model_s(new pcl::SampleConsensusModelPlane<PointT> (cloud_clean));
  //Ransac
  pcl::RandomSampleConsensus<PointT> ransac (model_s);
  ransac.setDistanceThreshold (0.05);
  ransac.computeModel();
  ransac.getInliers(inliers->indices);
  
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud_clean);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter(*cloud_clean);
  std::vector< int > nanindexes;
  pcl::removeNaNFromPointCloud(*cloud_clean,*cloud_clean,nanindexes);
  
      //cluster extraction
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud_clean);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.1); 
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_clean);
  ec.extract (cluster_indices);
  
  //Get the biggest
  pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
  for (std::vector<int>::const_iterator pit = cluster_indices.begin()->indices.begin (); pit != cluster_indices.begin()->indices.end (); pit++)
    cloud_cluster->points.push_back (cloud_clean->points[*pit]);
  
  
  c_dest->clear();
  for(pcl::PointCloud<PointT>::const_iterator it = cloud_cluster->begin(); it != cloud_cluster->end(); it++)
  {
    PointT p;
    p.x=-it->x*1000;
    p.y=-it->y*1000;
    p.z=it->z*1000;
    p.r=it->r;
    p.g=it->g;
    p.b=it->b;
    c_dest->push_back(p);
  }
  
  cout<<"Size: "<<c_dest->size()<<endl;

}

class fitterViewer
{
  public:
    fitterViewer()
    : v(new Viewer("cubeCloud.xml"))
    , cloud_buff(new pcl::PointCloud<PointT>)
    , cloud_to_show(new pcl::PointCloud<PointT>)
    , new_cloud_available_flag(false)
    , rgb_buffer(0)
    , first_image(true)
    {}
    
    ~fitterViewer()
    {
      interface->stop();
      fitter->stop();
    }
    
    void fit_cb (RectPrism  shape)
    {
      v->setPose("cube_0_t", shape.getCenter(), shape.getRotation(), shape.getWidth() );
      v->setScale("cube_0", shape.getWidth()(0)/2, shape.getWidth()(1)/2, shape.getWidth()(2)/2);
      
      if (new_cloud_available_flag==true)
      {
        fitter->setPointCloud(cloud_buff);
        new_cloud_available_flag=false;
      }
      
    }
    
    void image_cb (const boost::shared_ptr<openni_wrapper::Image> &image)
    {
      
      if (first_image)
      {
        rgb_buffer = new unsigned char [image->getWidth () * image->getHeight () * 3];
        first_image=false;
      }
      
      image->fillRGB(image->getWidth(), image->getHeight(), rgb_buffer);
      
      v->showImage(image->getWidth(), image->getHeight(), rgb_buffer);
      
    }
    
    void cloud_cb (const pcl::PointCloud<PointT>::ConstPtr &cloud)
    {
      cloud_mutex.lock();
      if (new_cloud_available_flag==false)
      {
        

        
/*        cloud_to_show->clear();
        for(pcl::PointCloud<PointT>::const_iterator it = cloud->begin(); it != cloud->end(); it++)
        {
          PointT p;
          p.x=-it->x*1000;
          p.y=-it->y*1000;
          p.z=it->z*1000;
          p.r=it->r;
          p.g=it->g;
          p.b=it->b;
          cloud_to_show->push_back(p);
        }
        
        v->setPointCloud(cloud_to_show);     */
        translateClouds(cloud_buff, cloud);
        new_cloud_available_flag=true;
      }
      cloud_mutex.unlock();
    }
    
    void run()
    {
      
       boost::shared_ptr<RectPrism> shape(new RectPrism());
       //openni grabber:
       interface = new pcl::OpenNIGrabber();
       
       boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f_kinect =
         boost::bind (&fitterViewer::cloud_cb, this, _1);
         
      interface->registerCallback(f_kinect);
      
      //Image callback:
      boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> f_image =
         boost::bind (&fitterViewer::image_cb, this, _1);
      
      interface->registerCallback(f_image);
      
      interface->start ();
      
      
      // Wait for the first frame:
      while(!new_cloud_available_flag) 
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
      //new_cloud_available_flag = false;
      
      fitter = new PfRectPrismFitting( 100,  cloud_buff );
         
      boost::function<void (RectPrism)> f =
        boost::bind (&fitterViewer::fit_cb, this, _1);

      fitter->registerCallback (f);

      fitter->start ();
       

    }
    
    boost::shared_ptr<Viewer> v;
    QMutex cloud_mutex;
    pcl::PointCloud<PointT>::Ptr cloud_buff;
    pcl::PointCloud<PointT>::Ptr cloud_to_show;
    boost::shared_ptr<openni_wrapper::Image> image;
    
    PfRectPrismFitting* fitter;
    pcl::Grabber* interface;
    
    
    bool new_cloud_available_flag;
    
    unsigned char *rgb_buffer;
    bool first_image;
};

  
int main (int argc, char* argv[])
{ 
  
  QApplication app(argc, argv);
  //Create sintetic cube
  //pcl::PointCloud<PointT>::Ptr cloud2fit = sinteticCubeCloud (100,100,400,50);
  //create fitter
  fitterViewer f;
  
  f.run();
  
  app.exec();
}