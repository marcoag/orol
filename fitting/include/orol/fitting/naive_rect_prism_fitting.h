#ifndef OROL_NAIVE_RECT_PRISM_FITTING
#define OROL_NAIVE_RECT_PRISM_FITTING

#include <orol/shapes/rectprism.h>

#include <boost/thread.hpp>
#include <boost/signals2.hpp>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZRGBA PointT;

/** \brief Fitting for rectangular prism
  * \author Marco A. Gutierrez <marcog@unex.es>
  * \ingroup io
  */
class naiveRectangularPrismFitting
{
  // Define callback signature typedefs
  typedef void (sig_cb_fitting_addapt) (const boost::shared_ptr<RectPrism>&);

  pcl::PointCloud<PointT>::Ptr pointCloud2Fit;
  boost::shared_ptr<RectPrism> shape2Fit;
  float weight;
  boost::thread captured_thread;
  bool running;
  mutable boost::mutex capture_mutex;
  boost::signals2::signal<sig_cb_fitting_addapt>* fitting_signal;
  
  template<typename T> int 
  num_slots () const;
  std::map<std::string, boost::signals2::signal_base*> signals_;
  std::map<std::string, std::vector<boost::signals2::connection> > connections;
  std::map<std::string, std::vector<boost::signals2::shared_connection_block> > shared_connections;
  
public:
  naiveRectangularPrismFitting (boost::shared_ptr<RectPrism> shape, pcl::PointCloud<PointT>::Ptr cloud);
  //Get and set cloud
  inline void setCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) { cout<<"setcloud"<<endl; pointCloud2Fit=cloud; }
  inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud () { return pointCloud2Fit; }
  //get and set rectangular prism
  inline void setRectangularPrism (boost::shared_ptr<RectPrism> shape) { shape2Fit=shape; }
  inline boost::shared_ptr<RectPrism> getRectangularPrism () { return shape2Fit; }
  
  template<typename T> boost::signals2::connection
  registerCallback (const boost::function<T> & callback);
  
  inline bool isRunning () { return running; }
  
  inline void start () { running=true; }
  inline void stop () { running=false; }
  
  //calcualte weight of the cloud and rectangular prism fitting
  float computeWeight();
  //do a step of the naive_fitting
  void adapt();
  
protected:
  
  template<typename T> boost::signals2::signal<T>* createSignal ();
  void captureThreadFunction ();
  void incTranslation (int index);
  void incWidth (int index);
  void incRotation (int index);
  
};

/// move to parent in the future:

template<typename T> int
naiveRectangularPrismFitting::num_slots () const
{
  typedef boost::signals2::signal<T> Signal;

  // see if we have a signal for this type
  std::map<std::string, boost::signals2::signal_base*>::const_iterator signal_it = signals_.find (typeid (T).name ());
  if (signal_it != signals_.end ())
  {
    Signal* signal = dynamic_cast<Signal*> (signal_it->second);
    return (static_cast<int> (signal->num_slots ()));
  }
  return (0);
}


template<typename T> boost::signals2::signal<T>*
naiveRectangularPrismFitting::createSignal ()
{
  typedef boost::signals2::signal<T> Signal;

  if (signals_.find (typeid (T).name ()) == signals_.end ())
  {
    Signal* signal = new Signal ();
    signals_[typeid (T).name ()] = signal;
    return (signal);
  }
  return (0);

}

template<typename T> boost::signals2::connection
naiveRectangularPrismFitting::registerCallback (const boost::function<T> & callback)
{
  typedef boost::signals2::signal<T> Signal;
  if (signals_.find (typeid (T).name ()) == signals_.end ())
  {

    std::cout << "no callback for type:" << typeid (T).name ();
  }
  Signal* signal = dynamic_cast<Signal*> (signals_[typeid (T).name ()]);
  boost::signals2::connection ret = signal->connect (callback);

  connections[typeid (T).name ()].push_back (ret);
  shared_connections[typeid (T).name ()].push_back (boost::signals2::shared_connection_block (connections[typeid (T).name ()].back (), false));
 //signalsChanged ();
  return (ret);
}

#endif