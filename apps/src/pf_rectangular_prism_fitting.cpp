#include <QtCore>

#include <orol/shapes/rectprism.h>
#include <orol/visual/viewer.h>
#include <orol/fitting/pf_rect_prism_fitting.h>

typedef pcl::PointXYZRGBA PointT;

pcl::PointCloud<PointT>::Ptr sinteticCubeCloud(int Wx, int  Wy, int Wz, int res)
{
  pcl::PointCloud<PointT>::Ptr cloud_cup(new pcl::PointCloud<PointT>());

  //Rot3D r(0.5, 0.2, 0.2);
  //Faces front and back
  for(float x=0; x<=Wx; x=x+res)
  {
    for(float y=0; y<=Wy; y=y+res)
    {
      //face front (x=0)
      PointT p;
      p.x = x;
      p.y = y;
      p.z = 0;
      p.r = 0;
      p.g = 255;
      p.b = 0;          
      cloud_cup->push_back(p);
      p.x = x;
      p.y = y;
      p.z = Wz;   
      p.r = 0;
      p.g = 255;
      p.b = 0;          
      cloud_cup->push_back(p);
    }
  }
  //Faces up and down
  for(float x=0; x<=Wx; x=x+res)
  {
    for(float z=0; z<=Wz; z=z+res)
    {
      //face front (x=0)
      PointT p;
      p.x = x;
      p.y = 0;
      p.z = z;
      p.r = 0;
      p.g = 255;
      p.b = 0;          
      cloud_cup->push_back(p);
      p.x = x;
      p.y = Wy;
      p.z = z;   
      p.r = 0;
      p.g = 255;
      p.b = 0;          
      cloud_cup->push_back(p);
    }
  }
  //Faces right and left
  for(float y=0; y<=Wy; y=y+res)
  {
    for(float z=0; z<=Wz; z=z+res)
    {
      //face front (x=0)
      PointT p;
      p.x = 0;
      p.y = y;
      p.z = z;
      p.r = 0;
      p.g = 255;
      p.b = 0;          
      cloud_cup->push_back(p);
      p.x = Wx;
      p.y = y;
      p.z = z;  
      p.r = 0;
      p.g = 255;
      p.b = 0;          
      cloud_cup->push_back(p);
    }
  }
  
  return cloud_cup;
}

class fitterViewer
{
public:

  fitterViewer()
  :v(new Viewer("cubeCloud.xml")) 
  { }
  
  void fit_cb (RectPrism  shape)
  {
    v->setPose("cube_0_t", shape.getCenter(), shape.getRotation(), shape.getWidth() );
    v->setScale("cube_0", shape.getWidth()(0)/2, shape.getWidth()(1)/2, shape.getWidth()(2)/2);
  }
  
  void run(pcl::PointCloud<PointT>::Ptr cloudToFit)
  {
    v->setPointCloud(cloudToFit);
    
    //fix this linking error with rectprism if not here:
    boost::shared_ptr<RectPrism> shape(new RectPrism());
    
    fitter = new PfRectPrismFitting( 30,  cloudToFit );
      
    boost::function<void (RectPrism)> f =
      boost::bind (&fitterViewer::fit_cb, this, _1);
         
    fitter->registerCallback (f);
      
    fitter->start ();
  }

  void resizeEvent(QResizeEvent * event);
  
private:
  
  pcl::PointCloud<PointT>::Ptr cloudToFit;
  boost::shared_ptr<Viewer> v;
  PfRectPrismFitting *fitter;
  
};

int main(int argc, char* argv[])
{ 

   QApplication app(argc, argv);
   
   fitterViewer f;
   f.run(sinteticCubeCloud(120,400,200,10));
   
   app.exec();
  
  return 0;
}


