#include <orol/fitting/pf_rect_prism_fitting.h>

/**
  * \brief Default constructor
  */
PfRectangularPrismFitting::PfRectangularPrismFitting (int num_particles, pcl::PointCloud<PointT>::Ptr cloud)
{ 
  c.particles=num_particles;
  
  pointCloud2Fit=cloud; 
  
  captured_thread = boost::thread (&PfRectangularPrismFitting::captureThreadFunction, this);
  fitting_signal = createSignal<sig_cb_fitting_addapt> ();
  
  input.cloud_target=cloud;
  
  pf = new RCParticleFilter<RectPrismCloudPFInputData, int, RectPrismCloudParticle, RCParticleFilter_Config> (&c, input, 0);
}

void PfRectangularPrismFitting::captureThreadFunction ()
{
  while (true)
  {

    // Lock before checking running flag
    boost::unique_lock<boost::mutex> capture_lock (capture_mutex);
    if(running)
    {

      pf->step(input, 0, false, -1);
      best_particle=pf->getBest();
      
      cout<<"PESO:"<<best_particle.getWeight()<<endl;
      
      // Check for shape slots
      if (num_slots<sig_cb_fitting_addapt> () > 0 )
        fitting_signal->operator() (get_best_shape ());
      
    }
    capture_lock.unlock ();
  }
}

float PfRectangularPrismFitting::getRandom(float var)
{
  double U = double(rand())/RAND_MAX;
  double V = double(rand())/RAND_MAX;
  return sqrt(-2*log(U))*cos(2.*M_PIl*V)*var;
}