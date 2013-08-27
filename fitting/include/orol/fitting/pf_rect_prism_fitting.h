#ifndef OROL_PF_RECT_PRISM_FITTING
#define OROL_PF_RECT_PRISM_FITTING

#include <orol/shapes/rectprism.h>
#include <orol/fitting/fitting.h>

#include <boost/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

#include <orol/fitting/rect_prism_cloud_particle.h>
#include <orol/shapes/rectprism.h>
#include <orol/fitting/fitting.h>

/** \brief Fitting for rectangular prism
  * \author Marco A. Gutierrez <marcog@unex.es>
  * \ingroup fitting
  */

typedef pcl::PointXYZRGBA PointT;

class PfRectangularPrismFitting: public fitting
{
  // Define callback signature typedefs
  typedef void (sig_cb_fitting_addapt) (const boost::shared_ptr<RectPrism>&);

  pcl::PointCloud<PointT>::Ptr pointCloud2Fit;
  RectPrismCloudPFInputData input;
  RCParticleFilter<RectPrismCloudPFInputData, int, RectPrismCloudParticle, RCParticleFilter_Config> *pf;
  RCParticleFilter_Config c;
  RectPrismCloudParticle best_particle;
  
  boost::thread captured_thread;
  mutable boost::mutex capture_mutex;
  boost::signals2::signal<sig_cb_fitting_addapt>* fitting_signal;
  
  
  
public:
  PfRectangularPrismFitting (int num_particles, pcl::PointCloud<PointT>::Ptr cloud);
  ~PfRectangularPrismFitting();
  //Get and set cloud
  inline void setCloud (pcl::PointCloud<PointT>::Ptr cloud) { pointCloud2Fit=cloud; }
  inline pcl::PointCloud<PointT>::Ptr getCloud () { return pointCloud2Fit; }
  
  inline boost::shared_ptr<RectPrism> get_best_shape () { return best_particle.get_shape(); }
  
protected:
  void initRectangularPrism ();
  void captureThreadFunction ();
  float getRandom(float var);
  
};

#endif