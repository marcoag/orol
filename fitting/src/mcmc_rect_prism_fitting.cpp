#include <orol/fitting/mcmc_rect_prism_fitting.h>

mcmcRectangularPrismFitting::mcmcRectangularPrismFitting( pcl::PointCloud<PointT>::Ptr cloud, QVec C, QVec S, QVec R  )
: shape2Fit(new RectPrism())
, bestFit(new RectPrism())
{ 
  //initialize weights
  weight=0;
  bestweight=0;
  
  pointCloud2Fit=cloud; 
  
  varianceC=C;
  varianceS=S;
  varianceR=R;
  
  captured_thread = boost::thread (&mcmcRectangularPrismFitting::captureThreadFunction, this);
  fitting_signal = createSignal<sig_cb_fitting_addapt> ();
  
  //initialize Rectangular prism to cloud
  initRectangularPrism();
}

void mcmcRectangularPrismFitting::initRectangularPrism ()
{
  
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  
  //calculate centroid, eigen values and eigen vectors
  pcl::computeMeanAndCovarianceMatrix(*pointCloud2Fit, covariance_matrix, centroid);
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
  for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = pointCloud2Fit->points.begin (); it < pointCloud2Fit->points.end (); ++it)
  {
    float distance=fabs(sqrt(pow((it->x)-centroid(0),2.0)+pow((it->y)-centroid(2),2.0)+pow((it->z)-centroid(1),2.0)));
    if (max_distance<distance)
      max_distance=distance;
  }
  float ratio=max_eigenvalue/max_distance;
  
  //set initial values for the rectangular prism
  shape2Fit->setCenter(QVec::vec3(centroid(0), centroid(1), centroid(2)));
  shape2Fit->setWidth(QVec::vec3((eigen_values(0)/ratio),(eigen_values(0)/ratio),(eigen_values(0)/ratio)));

//   shape2Fit->setCenter(QVec::vec3(440,0,0));
//   shape2Fit->setWidth(QVec::vec3(100,100,400));
  shape2Fit->setRotation(QVec::vec3(0,0,0));
}

void mcmcRectangularPrismFitting::captureThreadFunction ()
{
  while (true)
  {

    // Lock before checking running flag
    boost::unique_lock<boost::mutex> capture_lock (capture_mutex);
    if(running)
    {
      adapt ();
      // Check for shape slots
      if (num_slots<sig_cb_fitting_addapt> () > 0 )
        fitting_signal->operator() (getRectangularPrism ());
      
    } 
    capture_lock.unlock ();
  }
}
  
float mcmcRectangularPrismFitting::computeWeight()
{
  weight=0.;
  //estimate normals
  pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
  ne.setInputCloud (pointCloud2Fit);
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
  ne.setSearchMethod (tree);
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (5);
  // Compute the features
  ne.compute (*cloud_normals);

  int normalint =0;
  for( pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = pointCloud2Fit->begin(); it != pointCloud2Fit->end(); it++ )
  {
    
    QVec point = QVec::vec3(it->x, it->y, it->z);
    QVec normal = QVec::vec3( cloud_normals->points[normalint].normal_x,cloud_normals->points[normalint].normal_y, cloud_normals->points[normalint].normal_z);


    double dist = shape2Fit->distance(point,normal);
    weight += dist*dist;
    normalint++;
  }
  
  weight /= pointCloud2Fit->points.size();

  return weight;
}

void mcmcRectangularPrismFitting::adapt()
{
  //MarkovChainStepOnAll();
  MarkovChainStepOnOne();

}

void mcmcRectangularPrismFitting::MarkovChainStepOnAll()
{
  //incs on each stuff
  QVec translationInc = QVec::vec3(getRandom(varianceC(0)),getRandom(varianceC(1)),getRandom(varianceC(2)));
  QVec rotationInc    = QVec::vec3(getRandom(varianceR(0)),getRandom(varianceR(1)),getRandom(varianceR(2)));
  QVec widthInc       = QVec::vec3(getRandom(varianceS(0)),getRandom(varianceS(1)),getRandom(varianceS(2)));
  
  float currentWeight=weight;
  float nextWeight;
  QVec translation = shape2Fit->getCenter();
  QVec rotation = shape2Fit->getRotation();
  QVec width = shape2Fit->getWidth();
  
  //do change and get weight
  shape2Fit->setCenter(QVec::vec3(translation(0)+translationInc(0),translation(1)+translationInc(1),translation(2)+translationInc(2)));
  shape2Fit->setRotation(QVec::vec3(rotation(0)+rotationInc(0),rotation(1)+rotationInc(1),rotation(2)+rotationInc(2)));
  shape2Fit->setWidth(QVec::vec3(width(0)+widthInc(0),width(1)+widthInc(1),width(2)+widthInc(2)));  
  computeWeight();
  nextWeight=weight;
  
  //undo
  shape2Fit->setCenter(QVec::vec3(translation(0)-translationInc(0),translation(1)-translationInc(1),translation(2)-translationInc(2)));
  shape2Fit->setRotation(QVec::vec3(rotation(0)-rotationInc(0),rotation(1)-rotationInc(1),rotation(2)-rotationInc(2)));
  shape2Fit->setWidth(QVec::vec3(width(0)-widthInc(0),width(1)-widthInc(1),width(2)-widthInc(2)));   
  computeWeight();
  
  //we get it for sure
  if(nextWeight>weight)
  {
    shape2Fit->setCenter(QVec::vec3(translation(0)+translationInc(0),translation(1)+translationInc(1),translation(2)+translationInc(2)));
    shape2Fit->setRotation(QVec::vec3(rotation(0)+rotationInc(0),rotation(1)+rotationInc(1),rotation(2)+rotationInc(2)));
    shape2Fit->setWidth(QVec::vec3(width(0)+widthInc(0),width(1)+widthInc(1),width(2)+widthInc(2))); 
    computeWeight();
    //if better than best update best
    if(weight>bestweight)
    {
      bestFit->setCenter(shape2Fit->getCenter());
      bestFit->setRotation(shape2Fit->getRotation());
      bestFit->setWidth(shape2Fit->getWidth());
      bestweight=weight;
    }
  }
  //ifnot get it with probability nextweight/weight
  else
  {
    float probability=nextWeight/weight;
    if ( (((float) rand())/(float) RAND_MAX) < probability)
    {
      shape2Fit->setCenter(QVec::vec3(translation(0)+translationInc(0),translation(1)+translationInc(1),translation(2)+translationInc(2)));
      shape2Fit->setRotation(QVec::vec3(rotation(0)+rotationInc(0),translation(1)+rotationInc(1),rotation(2)+rotationInc(2)));
      shape2Fit->setWidth(QVec::vec3(width(0)+widthInc(0),width(1)+widthInc(1),width(2)+widthInc(2))); 
      computeWeight();
    }
  }
}

void mcmcRectangularPrismFitting::MarkovChainStepOnOne()
{
  //incs on each stuff
  int selection = rand()%3;
  QVec translationInc = QVec::vec3(getRandom(varianceC(0)),getRandom(varianceC(1)),getRandom(varianceC(2)));
  QVec rotationInc    = QVec::vec3(getRandom(varianceR(0)),getRandom(varianceR(1)),getRandom(varianceR(2)));
  QVec widthInc       = QVec::vec3(getRandom(varianceS(0)),getRandom(varianceS(1)),getRandom(varianceS(2)));
  
  float currentWeight=weight;
  float nextWeight;
  QVec translation = shape2Fit->getCenter();
  QVec rotation = shape2Fit->getRotation();
  QVec width = shape2Fit->getWidth();
  
  //do change and get weight
  if(selection==0)
    shape2Fit->setCenter(translation+translationInc);
  if(selection==1)
    shape2Fit->setRotation(rotation+rotationInc);
  if(selection==2)
    shape2Fit->setWidth(width+widthInc);  
  computeWeight();
  nextWeight=weight;
  
  //undo
  if(selection==0)
    shape2Fit->setCenter(translation-translationInc);
  if(selection==1)
    shape2Fit->setRotation(rotation-rotationInc);
  if(selection==2)
    shape2Fit->setWidth(width-widthInc);   
  computeWeight();

  //we get it for sure
  if(nextWeight>weight)
  {
    if(selection==0)
      shape2Fit->setCenter(translation+translationInc);
    if(selection==1)
      shape2Fit->setRotation(rotation+rotationInc);
    if(selection==2)
      shape2Fit->setWidth(width+widthInc); 
    computeWeight();
    //if better than best update best
    if(weight>bestweight)
    {
      if(selection==0)
        bestFit->setCenter(shape2Fit->getCenter());
      if(selection==1)
        bestFit->setRotation(shape2Fit->getRotation());
      if(selection==2)
        bestFit->setWidth(shape2Fit->getWidth());
      bestweight=weight;
    }
  }
  //ifnot get it with probability nextweight/weight
  else
  {
    float probability=nextWeight/weight;
    if ( (((float) rand())/(float) RAND_MAX) < probability)
    {
      if(selection==0)
        shape2Fit->setCenter(translation+translationInc);
      if(selection==1)
        shape2Fit->setRotation(rotation+rotationInc);
      if(selection==2)
        shape2Fit->setWidth(width+widthInc); 
      computeWeight();
    }
  }
}


float mcmcRectangularPrismFitting::getRandom(float var)
{
  double U = double(rand())/RAND_MAX;
  double V = double(rand())/RAND_MAX;
  return sqrt(-2*log(U))*cos(2.*M_PIl*V)*var;
}
