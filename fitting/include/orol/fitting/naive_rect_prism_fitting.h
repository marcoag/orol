#include <orol/shapes/rectprism.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZRGBA PointT;

class naiveRectangularPrismFitting
{
  pcl::PointCloud<PointT>::Ptr pointCloud2Fit;
  RectPrism shape2Fit;
  float weight;
  
public:
  void setCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) { pointCloud2Fit=cloud; }
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud () { return pointCloud2Fit; }
  
  float computeWeight();
  void adapt();
  
protected:
  void incTranslation(int index);
  void incWidth(int index);
  void incRotation(int index);
  
};