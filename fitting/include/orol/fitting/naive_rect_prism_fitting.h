#include <orol/shapes/rectprism.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZRGBA PointT;

class naiveRectangularPrismFitting
{
  pcl::PointCloud<PointT>::Ptr pointCloud2Fit;
  RectPrism *shape2Fit;
  float weight;
  
public:
  naiveRectangularPrismFitting(RectPrism *shape, pcl::PointCloud<PointT>::Ptr cloud) { shape2Fit=shape; pointCloud2Fit=cloud; }
  //Get and set cloud
  inline void setCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) { pointCloud2Fit=cloud; }
  inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud () { return pointCloud2Fit; }
  //get and set rectangular prism
  inline void setRectangularPrism(RectPrism *shape) { shape2Fit=shape; }
  inline RectPrism* getRectangularPrism() { return shape2Fit; }
  
  //calcualte weight of the cloud and rectangular prism fitting
  float computeWeight();
  //do a step of the naive_fitting
  void adapt();
  
  
protected:
  void incTranslation(int index);
  void incWidth(int index);
  void incRotation(int index);
  
};