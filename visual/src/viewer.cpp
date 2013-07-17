#include <orol/visual/viewer.h>

Viewer::Viewer() : QWidget()
{
  world3D=NULL;
}


Viewer::~Viewer()
{
  delete(world3D);
}


void Viewer::update()
{
    world3D->update();
}

void Viewer::resizeEvent(QResizeEvent * event)
{
  world3D->autoResize();
}