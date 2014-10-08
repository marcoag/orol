#ifndef INNERMODELVIEWER_H
#define INNERMODELVIEWER_H

#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/GraphicsWindow>
#include <osgGA/TrackballManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <QtGui>
#include <QtOpenGL/QGLWidget>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osg/Shape>
#include <osg/Point>
#include <osg/Quat>
#include <osg/Image>
#include <osg/ShapeDrawable>
#include <osg/Camera>
#include <osg/Geometry>
#include <osg/LineSegment>
#include <osg/TexMat>
#include <osgUtil/IntersectVisitor>

#include <orol/visual/osgviewer/adapterwidget.h>
#include <orol/visual/osgviewer/findnamednode.h>
#include <orol/visual/osgviewer/viewerqt.h>
#include <orol/visual/osgviewer/getworldcoorofnode.h>
#include <orol/visual/innermodel/innermodel.h>

#include <qmat/QMatAll>
#include <QHash>

class InnerModelViewer;

osg::Vec3 QVecToOSGVec(const QVec &vec);
osg::Vec4 htmlStringToOsgVec4(QString color);
QString osgVec4ToHtmlString(osg::Vec4 color);
osg::Matrix QMatToOSGMat4(const RTMat &nodeB);


struct IMVCamera
{
	osg::Image *rgb;
	osg::Image *d;
	InnerModelRGBD *RGBDNode;
	osgViewer::Viewer *viewerCamera;
	osgGA::TrackballManipulator *manipulator;
	QString id;
};

struct IMVLaser
{
	InnerModelLaser *laserNode;
	osg::Switch *osgNode;
	QString id;
};

struct IMVMesh
{
	osg::Node * osgmeshes;
	osg::MatrixTransform * osgmeshPaths;
	osg::MatrixTransform * meshMts;
};

class IMVPlane : public osg::Geode
{
	friend class InnerModelViewer;
public:
	IMVPlane(InnerModelPlane *plane, std::string imagenEntrada, osg::Vec4 valoresMaterial, float transparencia);
	void updateBuffer(uint8_t *data_, int32_t width_, int32_t height_)
	{
		data = data_;
		width = width_;
		height = height_;
		dirty = true;
	}
	void performUpdate()
	{
		static uint8_t *backData = NULL;

		if (dirty)
		{
			if (backData != data)
				image->setImage(width, height, 3, GL_RGB8 ,GL_RGB, GL_UNSIGNED_BYTE, data, osg::Image::NO_DELETE, 1);
			else
				image->dirty();

			dirty = false;
			backData = data;
		}
	}

// protected:
	uint8_t *data;
	bool dirty;
	int32_t width, height;
	osg::Texture2D* texture;
	osg::ShapeDrawable *planeDrawable;
	osg::Image *image;
};

class IMVPointCloud : public osg::Geode
{
public:
	IMVPointCloud(std::string id_) : osg::Geode()
	{
		/// ID
		id = id_;
		/// Default point size
		pointSize = 1.;
		/// Vertices
		points = new osg::Vec3Array;
		points->resize(64);
		cloudVertices = new osg::Vec3Array;
		*cloudVertices = *points;
		/// Colors
		colors = new osg::Vec4Array;
		colors->resize(64);
		colorsArray = new osg::Vec4Array;
		*colorsArray = *colors;

		/// Index array
		colorIndexArray = new osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 4, 4>;
		colorIndexArray->resize(64);
		/// DrawArrays
		arrays = new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, cloudVertices->size());
		/// Geometry
		cloudGeometry = new osg::Geometry();
		cloudGeometry->setVertexArray(cloudVertices);
		cloudGeometry->addPrimitiveSet(arrays);
		cloudGeometry->setColorArray(colorsArray);
		cloudGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
		/// Geode

		addDrawable(cloudGeometry);

	}

	void update()
	{
		if (points->size() != colors->size()) throw "points->size() != colors->size()";
		/// Geode 1
		removeDrawable(cloudGeometry);

		/// Arrays
		cloudVertices = new osg::Vec3Array;
		*cloudVertices = *points;
		colorsArray = new osg::Vec4Array;
		*colorsArray = *colors;
		colorIndexArray = new osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 4, 4>;
		for (uint i=0; i<points->size(); i++) colorIndexArray->push_back(i);
		/// DrawArrays
		arrays = new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, cloudVertices->size());
		/// Geometry
		cloudGeometry = new osg::Geometry();
		cloudGeometry->setVertexArray(cloudVertices);
		cloudGeometry->addPrimitiveSet(arrays);
		cloudGeometry->getOrCreateStateSet()->setAttribute( new osg::Point(pointSize), osg::StateAttribute::ON );
		cloudGeometry->setColorArray(colorsArray);
		cloudGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
		/// Geode 2
		addDrawable(cloudGeometry);
	}

	float getPointSize() { return pointSize; }
	void setPointSize(float p) { pointSize = p; }

	std::string id;
	osg::Vec3Array *points;
	osg::Vec4Array *colors;

protected:
	osg::Vec3Array *cloudVertices;
	osg::Vec4Array *colorsArray;
	osg::Geometry *cloudGeometry;
	osg::TemplateIndexArray <unsigned int, osg::Array::UIntArrayType,4,4> *colorIndexArray;
	osg::DrawArrays *arrays;
	float pointSize;
};

class InnerModelViewer : public osg::Switch
{
public:
	enum CameraView { BACK_POV, FRONT_POV, LEFT_POV, RIGHT_POV, TOP_POV };

	InnerModelViewer(InnerModel *im, QString root="root", osg::Group *parent=NULL);

	~InnerModelViewer();
	
	///Return geode id if no geode, return null.
	osg::Geode* getGeode(QString id);
	void update();
	
	void reloadMesh(QString id);


// 	void recursiveConstructor(InnerModelNode *node, osg::Group *parent, QHash<QString, osg::MatrixTransform *> &mtsHash, QHash<QString, osg::Node *> &osgmeshesHash, QHash<QString, osg::MatrixTransform *> &osgmeshPatsHash);
	void recursiveConstructor(InnerModelNode* node, osg::Group* parent, QHash< QString, osg::MatrixTransform* >& mtsHash, QHash< QString, IMVMesh >& meshHash);

protected:
	void setOSGMatrixTransformForPlane(osg::MatrixTransform *mt, InnerModelPlane *plane);
	InnerModel *innerModel;
	
public:
	//QHash<QString, osg::Node *> osgmeshes;
	QHash<QString, osg::PolygonMode *> osgmeshmodes;
// 	QHash<QString, osg::MatrixTransform *> osgmeshPats;
	QHash<QString, osg::MatrixTransform *> mts;
	QHash<QString, osg::MatrixTransform *> planeMts;
// 	QHash<QString, osg::MatrixTransform *> meshMts;
	QHash<QString, IMVMesh> meshHash;
	QHash<QString, IMVPointCloud *> pointCloudsHash;
	QHash<QString, IMVPlane *> planesHash;
	QHash<QString, IMVCamera> cameras;
	QHash<QString, IMVLaser> lasers;


	void setMainCamera(osgGA::TrackballManipulator *manipulator, CameraView pov) const;
};

#endif
