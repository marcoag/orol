#include <orol/fitting/naive_rect_prism_fitting.h>

float naiveRectangularPrismFitting::computeWeight()
{
  
  float weight=0.;
  //estimate normals
  pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
  ne.setInputCloud (pointCloud2Fit);
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
  ne.setSearchMethod (tree);
  // Output pointCloud2Fitsets
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

    double dist = shape2Fit.distance(point,normal);
    
    weight += dist;
    normalint++;
  }
  
  weight /= pointCloud2Fit->points.size();
  
  weight = 1./(this->weight+0.1);
  
  return weight;
}

void naiveRectangularPrismFitting::adapt()
{
  incTranslation(0);
  
  switch(rand()%9)
  {
    //x
    case 0:
      incTranslation(0);
      break;
    //y
    case 1:
      incTranslation(1);
      break;
    //z
    case 2:
      incTranslation(2);
      break;
    //Wx
    case 3:
      incWidth(0);
      break;
    //Wy
    case 4:
      incWidth(1);
      break;  
    //Wz
    case 5:
      incWidth(2);
      break;
    //Rx
    case 6:
      incRotation(0);
      break;
    //Ry
    case 7:
      incRotation(1);
      break;
    //Rz
    case 8:
      incRotation(2);
      break;       
  }
}

void naiveRectangularPrismFitting::incTranslation(int index)
{
  QVec auxvec;
  float positiveWeight, negativeWeight, transformedWeight;
  QVec width = shape2Fit.getWidth();
  float inc = width(index)/40;
  auxvec = shape2Fit.getCenter();
   
//   //decide direction
  auxvec(index)=auxvec(index)+inc;
  shape2Fit.setCenter(auxvec);
  
  positiveWeight=computeWeight();
  
  //undo and sobstract a quarter = 2 quarters
  auxvec(index)=auxvec(index)-(inc*2);
  shape2Fit.setCenter(auxvec);
  
  negativeWeight=computeWeight();
  
  //undo changes
  auxvec(index)=auxvec(index)+inc;
  shape2Fit.setCenter(auxvec);
  
  //cout<<"Negative: "<<negativeWeight<<" Positive: "<<positiveWeight<<endl;
  
  //if negative is good go with it
  if(negativeWeight>positiveWeight)
  {
    transformedWeight=negativeWeight;
    inc*=-1;
  }
  else
    transformedWeight=positiveWeight;
    
  
//  cout<<"QQQQQQTransofredWeight: "<<transformedWeight<<" weight"<<this->weight<<endl;
  while(transformedWeight>weight)
  {
   // cout<<"TransofredWeight: "<<transformedWeight<<" weight"<<this->weight<<endl;
    auxvec(index)=auxvec(index)+inc;
    shape2Fit.setCenter(auxvec);
    this->weight=transformedWeight;
    
    //calculate futureWeight
    auxvec(index)=auxvec(index)+inc;
    shape2Fit.setCenter(auxvec);
    transformedWeight=computeWeight();
    auxvec(index)=auxvec(index)-inc;
    shape2Fit.setCenter(auxvec);
  }
  
  this->weight=computeWeight();
  cout<<this->weight<<endl;
}


void naiveRectangularPrismFitting::incWidth(int index)
{
  QVec auxvec;
  float positiveWeight, negativeWeight, transformedWeight;
  auxvec = shape2Fit.getWidth();
  float inc = auxvec(index)/40;;
  

  //decide direction
  auxvec(index)=auxvec(index)+inc;
  shape2Fit.setWidth(auxvec);
  
  positiveWeight=computeWeight();
  
  //undo and sobstract a quarter = 2 quarters
  auxvec(index)=auxvec(index)-(inc*2);
  shape2Fit.setWidth(auxvec);
  
  negativeWeight=computeWeight();
  
  //undo changes
  auxvec(index)=auxvec(index)+inc;
  shape2Fit.setWidth(auxvec);
  
  //if negative is good go with it
  if(negativeWeight>positiveWeight)
  {
    transformedWeight=negativeWeight;
    inc*=-1;
  }
  else
    transformedWeight=positiveWeight;
    
  while(transformedWeight>weight)
  {
    auxvec(index)=auxvec(index)+inc;
    shape2Fit.setWidth(auxvec);
    weight=transformedWeight;
    
    //calculate futureWeight
    auxvec(index)=auxvec(index)+inc;
    shape2Fit.setWidth(auxvec);
    transformedWeight=computeWeight();
    auxvec(index)=auxvec(index)-inc;
    shape2Fit.setWidth(auxvec);
  }
}

void naiveRectangularPrismFitting::incRotation(int index)
{
  QVec auxvec;
  float positiveWeight, negativeWeight, transformedWeight;
  auxvec = shape2Fit.getRotation();
  float inc = 0.02;
  

  //decide direction
  auxvec(index)=auxvec(index)+inc;
  shape2Fit.setRotation(auxvec);
  
  positiveWeight=computeWeight();
  
  //undo and sobstract a quarter = 2 quarters
  auxvec(index)=auxvec(index)-(inc*2);
  shape2Fit.setRotation(auxvec);
  
  negativeWeight=computeWeight();
  
  //undo changes
  auxvec(index)=auxvec(index)+inc;
  shape2Fit.setRotation(auxvec);
  
  //if negative is good go with it
  if(negativeWeight>positiveWeight)
  {
    transformedWeight=negativeWeight;
    inc*=-1;
  }
  else
    transformedWeight=positiveWeight;
    
  while(transformedWeight>weight)
  {
    auxvec(index)=auxvec(index)+inc;
    shape2Fit.setRotation(auxvec);
    weight=transformedWeight;
  //  cout<<"current: " <<weight<<endl;
    
    //calculate futureWeight
    auxvec(index)=auxvec(index)+inc;
    shape2Fit.setRotation(auxvec);
    transformedWeight=computeWeight();
    auxvec(index)=auxvec(index)-inc;
    shape2Fit.setRotation(auxvec);
    
    //cout<<"tranformedWeight: "<<transformedWeight<<endl;
  }
}
