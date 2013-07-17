/*
 *    Copyright (C) 2010-2013 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INNERMODEL_H
#define INNERMODEL_H

#include <stdexcept>
#include <typeinfo>
#include <stdint.h>

#include <QMutexLocker>
#include <QHash>


#include <QMat/qmat.h>
#include <QMat/qvec.h>
#include <QMat/qcamera.h>
#include <QMat/qrtmat.h>
#include <QMat/qfundamental.h>

class InnerModel;
#include <orol/visual/innermodel/innermodelreader.h>

using namespace RMat;

class InnerModelReader;

class InnerModelNode;
class InnerModelTransform;
class InnerModelJoint;
class InnerModelPrismaticJoint;
class InnerModelDifferentialRobot;
class InnerModelPlane;
class InnerModelCamera;
class InnerModelRGBD;
class InnerModelIMU;
class InnerModelLaser;
class InnerModelMesh;
class InnerModelPointCloud;


class InnerModelException: public std::runtime_error
{
public:
	InnerModelException(const std::string &reason) : runtime_error(std::string("InnerModelException: ") + reason)
	{
		std::cout << reason << std::endl;
	}
};

struct AttributeType{
	QString type;
	QString value;
};

class InnerModelNode : public RTMat
{
public:
	InnerModelNode(QString id_, InnerModelNode *parent_=NULL) : RTMat()
	{
		fixed = true;
		parent = parent_;
		if (parent)
			level = parent->level+1;
		else
			level = 0;
		id = id_;
		attributes.clear();
	}
	void treePrint(QString s, bool verbose=false);
	virtual void print(bool verbose) = 0;
	virtual void update() = 0;
	virtual void save(QTextStream &out, int tabs) = 0;
	void setParent(InnerModelNode *parent_) { parent = parent_; level = parent->level+1; }
	void addChild(InnerModelNode *child) { children.append(child); child->parent = this; }
	void setFixed(bool f=true) { fixed = f; }
	bool isFixed() { return fixed; }
	void updateChildren() { foreach(InnerModelNode *i, children) i->update(); }

//protected:
	QString id;
	int level;
	bool fixed;
	InnerModelNode *parent;
	QList<InnerModelNode *> children;
	QHash<QString,AttributeType> attributes;
};


class InnerModel
{
public:

	/// (Con/De)structors
	InnerModel();
  InnerModel(std::string xmlFilePath);
  InnerModel(const InnerModel &original);
 	~InnerModel();
	friend class InnerModelReader;
	bool save(QString path);

	/// Auto update method
	void update();
	void setUpdateRotationPointers(QString rotationId, float *x, float *y, float *z);
	void setUpdateTranslationPointers(QString translationId, float *x, float *y, float *z);
	void setUpdatePlanePointers(QString planeId, float *nx, float *ny, float *nz, float *px, float *py, float *pz);
	void setUpdateTransformPointers(QString transformId, float *tx, float *ty, float *tz, float *rx, float *ry, float *rz);
	void cleanupTables();

	/// Manual update method
	void updateTransformValues(QString transformId, float tx, float ty, float tz, float rx, float ry, float rz, QString parentId="");
	void updateTranslationValues(QString transformId, float tx, float ty, float tz, QString parentId="");
	void updateRotationValues(QString transformId, float rx, float ry, float rz,QString parentId="");
	void updateJointValue(QString jointId, float angle);
	void updatePrismaticJointPosition(QString jointId, float position);
	void updatePlaneValues(QString planeId, float nx, float ny, float nz, float px, float py, float pz);




	/// Model construction methods
	void setRoot(InnerModelNode *node);
	InnerModelTransform *newTransform(QString id, QString engine, InnerModelNode *parent, float tx=0, float ty=0, float tz=0, float rx=0, float ry=0, float rz=0, float mass=0);
	InnerModelJoint *newJoint(QString id, InnerModelTransform* parent, float lx = 0, float ly = 0, float lz = 0, float hx = 0, float hy = 0, float hz = 0,  float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, float min=-INFINITY, float max=INFINITY, uint32_t port = 0, std::string axis = "z", float home=0);
	InnerModelPrismaticJoint *newPrismaticJoint(QString id, InnerModelTransform* parent, float min=-INFINITY, float max=INFINITY, float value=0, float offset=0, uint32_t port = 0, std::string axis = "z", float home=0);
	InnerModelDifferentialRobot *newDifferentialRobot(QString id, InnerModelTransform* parent, float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, uint32_t port = 0);
	InnerModelCamera *newCamera(QString id, InnerModelNode *parent, float width, float height, float focal);
	InnerModelRGBD *newRGBD(QString id, InnerModelNode *parent, float width, float height, float focal, uint32_t port = 0, QString ifconfig="");
	InnerModelIMU *newIMU(QString id, InnerModelNode *parent, uint32_t port = 0);
	InnerModelLaser *newLaser(QString id, InnerModelNode *parent, uint32_t port = 0, uint32_t min=0, uint32_t max=30000, float angle = M_PIl, uint32_t measures = 360, QString ifconfig="");
	InnerModelPlane *newPlane(QString id, InnerModelNode *parent, QString texture, float width, float height, float depth, int repeat, float nx=0, float ny=0, float nz=0, float px=0, float py=0, float pz=0);
	InnerModelMesh *newMesh(QString id, InnerModelNode *parent, QString path, float scale, int render, float tx, float ty, float tz, float rx, float ry, float rz);
	InnerModelMesh *newMesh(QString id, InnerModelNode *parent, QString path, float scalex, float scaley, float scalez, int render, float tx, float ty, float tz, float rx, float ry, float rz);
	InnerModelPointCloud *newPointCloud(QString id, InnerModelNode *parent);

	InnerModelTransform *getTransform(const QString &id);
	InnerModelJoint *getJoint(const QString &id);
	InnerModelPrismaticJoint *getPrismaticJoint(const QString &id);
	InnerModelDifferentialRobot *getDifferentialRobot(const QString &id);
	InnerModelCamera *getCamera(QString id);
	InnerModelRGBD *getRGBD(QString id);
	InnerModelIMU *getIMU(QString id);
	InnerModelLaser *getLaser(QString id);
	InnerModelPlane *getPlane(const QString &id);
	InnerModelMesh *getMesh(const QString &id);
	InnerModelPointCloud *getPointCloud(const QString &id);

	/// Information retrieval methods
	QVec transform(const QString & destId, const QVec &origVec, const QString & origId);
	QVec project(QString reference, QVec origVec, QString cameraId);
	QVec backProject(const QString &cameraId, const QVec &coord) ;//const;
	void imageCoordToAngles(const QString &cameraId, QVec coord, float &pan, float &tilt, const QString & anglesRefS);
	QVec anglesToImageCoord(const QString &cameraId, float pan, float tilt, const QString & anglesRefS);
	QVec imageCoordPlusDepthTo(QString cameraId, QVec coord, float depth, QString to);
	QVec projectFromCameraToPlane(const QString &to, const QVec &coord, const QString &cameraId, const QVec &vPlane, const float &dist); 
	QVec horizonLine(QString planeId, QString cameraId, float heightOffset=0.);

	/// Matrix transformation retrieval methods
	RTMat getTransformationMatrix(const QString &destId, const QString &origId);
	QMat getRotationMatrixTo(const QString &to, const QString &from);
	QVec getTranslationVectorTo(const QString &to, const QString &from);
	QMat getHomographyMatrix(QString destCamera, QString plane, QString sourceCamera);
	QMat getAffineHomographyMatrix(QString destCamera, QString plane, QString sourceCamera);
	QMat getPlaneProjectionMatrix(QString virtualCamera, QString plane, QString sourceCamera);

	/// Misc
	void print(QString s="") { treePrint(s, true); }
	void treePrint(QString s="", bool verbose=false) { root->treePrint(QString(s), verbose); }

	/// Robex Base specific getters
	QVec robotToWorld(const QVec & vec);
	/**
	 * @brief Returns the x,y,z coordinates of the center of reference of the robot. No angle is computed
	 *
	 * @return QVec
	 **/
	QVec robotInWorld(); 
	float getBaseX();
	float getBaseZ();
	float getBaseAngle(); 
	float getBaseRadius(); 
	float getCameraFocal(const QString &cameraId ) const;
	int getCameraWidth( QString cameraId );
	int getCameraHeight(const QString &cameraId ) const;
	int getCameraSize(const QString &cameraId ) const;
	void getFundamental(float h[3][3]) const;
	QMat getFundamental() const;

	inline QVec getBaseCoordinates() { return transform("world", QVec::vec3(0,0,0), "base");};
	inline T getBaseLine() const { return 0;/*return rightCamTranslationToLeftCam().vectorNormL2(); */ }
	/**
	 * @brief Return the current pose (x,z,angle) of the robot in the world reference frame.
	 *
	 * @return QVec
	 **/
	QVec getBaseOdometry();
		
	/// Stereo computations
	void updateStereoGeometry( const QString &firstCam, const QString & secondCam );
	QVec compute3DPointInCentral(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right);
	QVec compute3DPointInRobot(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right);
	QVec compute3DPointFromImageCoords(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem);
	QVec compute3DPointFromImageAngles(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem);	

	/// Setters for model parameters

	/// Laser stuff
	QVec laserToWorld( const QString &laserId , const QVec &p) { return laserTo("world", laserId, p); }
	QVec laserToWorld( const QString &laserId , float r, float alfa) { return laserTo("world", laserId, r, alfa);	}
	QVec laserTo(const QString &dest, const QString & laserId , const QVec &p);
	QVec laserTo(const QString &dest, const QString & laserId , float r, float alfa);
	QVec laserToRefFrame(const QString & laserId , float r, float alpha, const QString & refFrame) { return laserTo(refFrame, laserId, r, alpha); }
	QVec worldToLaser( const QString & laserId , const QVec & p);
	QVec laserToBase( const QString & laserId , float r, float alfa);
	
	/// Frustrum
	struct TPlano { QVec n; float d; };
	struct TFrustrum { TPlano left; TPlano top; TPlano right; TPlano down; TPlano near; TPlano far;};
	TFrustrum frustrumLeft, frustrumThird, frustrumRight;
	
	/// Clonning
	InnerModel cloneFake( const QVec & basePose) const;

	QList<QString> getIDKeys() {return hash.keys(); }
	InnerModelNode *getNode(const QString & id) const { if (hash.contains(id)) return hash[id]; else return NULL;}
	void removeSubTree(InnerModelNode *item, QStringList *l);
	void removeNode(const QString & id)  { hash.remove(id); }
	void getSubTree(InnerModelNode *node, QStringList *l);
	/// Set debug level
	int debugLevel(int level=-1) { static int debug_level=0; if (level>-1) debug_level=level; return debug_level; } 

	InnerModelNode *getRoot() { return root; }
	



protected:
	QMutex *mutex;
	InnerModelNode *root;
	QHash<QString, InnerModelNode *> hash;
	QHash<QPair<QString, QString>, RTMat> localHashTr;
	QHash<QPair<QString, QString>, QMat> localHashRot;
	void setLists(const QString &origId, const QString &destId);
	QList<InnerModelNode *> listA, listB;
	
	QFundamental fundamental;
	QEssential	essential;
	
};


class InnerModelTransform : public InnerModelNode
{
public:
	InnerModelTransform(QString id_, QString engine_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float mass_, InnerModelNode *parent_=NULL) : InnerModelNode(id_, parent_)
	{
		engine = engine_;
		set(rx_, ry_, rz_, tx_, ty_, tz_);
		mass = mass_;
		backtX = tx_;
		backtY = ty_;
		backtZ = tz_;
		backrX = rx_;
		backrY = ry_;
		backrZ = rz_;
		rx = ry = rz = tx = ty = tz = NULL;
		gui_translation = gui_rotation = true;
	}
	void print(bool verbose)
	{
		printf("Transform: %s\n", qPrintable(id));
		if (verbose)
		{
			((QMat *)this)->print(qPrintable(id));
			getTr().print(id+"_T");
			//extractAnglesR().print(id+"_R");
		}
	}
	void save(QTextStream &out, int tabs)
	{
		QList<InnerModelNode*>::iterator c;
		
		if (id == "root")
		{
			for (int i=0; i<tabs; i++) out << "\t";
			out << "<innermodel>\n";
			for (c=children.begin(); c!=children.end(); c++) (*c)->save(out, tabs+1);
			for (int i=0; i<tabs; i++) out << "\t";
			out << "</innermodel>\n";
		}
		else
		{
			for (int i=0; i<tabs; i++) out << "\t";
			if (gui_translation and not gui_rotation)
				out << "<translation id=\"" << id << "\" tx=\""<< QString::number(backtX, 'g', 10) <<"\" ty=\""<< QString::number(backtY, 'g', 10) <<"\" tz=\""<< QString::number(backtZ, 'g', 10) <<"\">\n";
			else if (gui_rotation and not gui_translation)
				out << "<rotation id=\"" << id << "\" tx=\""<< QString::number(backrX, 'g', 10) <<"\" ty=\""<< QString::number(backrY, 'g', 10) <<"\" tz=\""<< QString::number(backrZ, 'g', 10) <<"\">\n";
			else
				out << "<transform id=\"" << id << "\" tx=\""<< QString::number(backtX, 'g', 10) <<"\" ty=\""<< QString::number(backtY, 'g', 10) <<"\" tz=\""<< QString::number(backtZ, 'g', 10) <<"\"  rx=\""<< QString::number(backrX, 'g', 10) <<"\" ry=\""<< QString::number(backrY, 'g', 10) <<"\" rz=\""<< QString::number(backrZ, 'g', 10) <<"\">\n";
						
			for (c=children.begin(); c!=children.end(); c++)
				(*c)->save(out, tabs+1);

			for (int i=0; i<tabs; i++) out << "\t";
			if (gui_translation and not gui_rotation )
				out << "</translation>\n";
			else if (gui_rotation and not gui_translation)
				out << "</rotation>\n";
			else
				out << "</transform>\n";
		}
	}
	void setUpdatePointers(float *tx_, float *ty_, float *tz_, float *rx_, float *ry_, float *rz_)
	{
		tx = tx_;
		ty = ty_;
		tz = tz_;
		rx = rx_;
		ry = ry_;
		rz = rz_;
		fixed = false;
	}
	void setUpdateTranslationPointers(float *tx_, float *ty_, float *tz_)
	{
		tx = tx_;
		ty = ty_;
		tz = tz_;
		fixed = false;
	}
	void setUpdateRotationPointers(float *rx_, float *ry_, float *rz_)
	{
		rx = rx_;
		ry = ry_;
		rz = rz_;
		fixed = false;
	}
	void update()
	{
		if (!fixed)
		{
			if (tx) backtX = *tx;
			if (ty) backtY = *ty;
			if (tz) backtZ = *tz;
			if (rx) backrX = *rx;
			if (ry) backrY = *ry;
			if (rz) backrZ = *rz;
			set(backrX, backrY, backrZ, backtX, backtY, backtZ);
		}
		updateChildren();
	}
	void update(float tx_, float ty_, float tz_, float rx_, float ry_, float rz_)
	{
		backrX = rx_; backrY = ry_; backrZ = rz_;
		backtX = tx_; backtY = ty_; backtZ = tz_;
		set(backrX, backrY, backrZ, backtX, backtY, backtZ);
		fixed = true;
	}
	float *tx, *ty, *tz;
	float *rx, *ry, *rz;
	float mass;
	float backtX, backtY, backtZ;
	float backrX, backrY, backrZ;
	bool gui_translation, gui_rotation;
	QString engine;
};



class InnerModelJoint : public InnerModelTransform
{
public:
	InnerModelJoint(QString id_, float lx_, float ly_, float lz_, float hx_, float hy_, float hz_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float min_=-INFINITY, float max_=INFINITY, uint32_t port_=0,std::string axis_="z", float home_=0, InnerModelTransform *parent_=NULL) : InnerModelTransform(id_,QString("static"),tx_,ty_,tz_,rx_,ry_,rz_, 0, parent_)
	{
// 		set(rx_, ry_, rz_, tx_, ty_, tz_);
		backlX = lx_;
		backlY = ly_;
		backlZ = lz_;
		backhX = hx_;
		backhY = hy_;
		backhZ = hz_;
		min = min_;
		max = max_;
		home = home_;
		hx = hy = hz =lx = ly = lz = NULL;
		port = port_;
		axis = axis_;
		if (axis == "x")
		{
			update(min, 0, 0, max, 0, 0);
		}
		else if (axis == "y")
		{
			update(0, min, 0, 0, max, 0);
		}
		else if (axis == "z")
		{
			update(0, 0, min, 0, 0, max);
		}
		else
		{
			qFatal("internal error, no such axis %s\n", axis.c_str());
		}
	}
	void print(bool verbose)
	{
		printf("Joint: %s\n", qPrintable(id));
		if (verbose)
		{
			((QMat *)this)->print(qPrintable(id));
			getTr().print(id+"_T");
			//extractAnglesR().print(id+"_R");
		}
	}
	void save(QTextStream &out, int tabs)
	{
		for (int i=0; i<tabs; i++) out << "\t";
		out << "### joints cannot be saved yet ###\n";
	}
	void setUpdatePointers(float *lx_, float *ly_, float *lz_, float *hx_, float *hy_, float *hz_)
	{
		lx = lx_;
		ly = ly_;
		lz = lz_;
		hx = hx_;
		hy = hy_;
		hz = hz_;
		fixed = false;
	}

	void update()
	{
		if (!fixed)
		{
			if (lx) backtX = *tx;
			if (ly) backtY = *ty;
			if (lz) backtZ = *tz;
			if (rx) backhX = *hx;
			if (ry) backhY = *hy;
			if (rz) backhZ = *hz;
		}
		updateChildren();
	}
	void update(float lx_, float ly_, float lz_, float hx_, float hy_, float hz_)
	{
		backhX = hx_; backhY = hy_; backhZ = hz_;
		backlX = lx_; backlY = ly_; backlZ = lz_;
		fixed = true;
	}
	float getAngle() { return backrZ; }
	float setAngle(float angle)
	{
		float ret;
		if (angle <= max and angle >= min)
		{
			ret = angle;
		}
		else if (angle >= max)
		{
			ret = max;
		}
		else if (angle <= min)
		{
			ret = min;
		}
		backrZ = ret;
		if (axis == "x")
		{
			set(ret,0,0, 0,0,0);
		}
		else if (axis == "y")
		{
			set(0,ret,0, 0,0,0);
		}
		else if (axis == "z")
		{
			set(0,0,ret, 0,0,0);
		}
		else
		{
			qFatal("internal error, no such axis %s\n", axis.c_str());
		}
		return ret;
	}
	
	QVec unitaryAxis()
	{
		if( axis == "x") return QVec::vec3(1,0,0);
		if( axis == "y") return QVec::vec3(0,1,0);
		if( axis == "z") return QVec::vec3(0,0,1);
		return QVec::zeros(3);
	}
	
	float *lx, *ly, *lz;
	float *hx, *hy, *hz;
	float backlX, backlY, backlZ;
	float backhX, backhY, backhZ;
	float min, max;
	float home;
	uint32_t port;
	std::string axis;
};

class InnerModelPrismaticJoint : public InnerModelTransform
{
public:
	InnerModelPrismaticJoint(QString id_, float min_, float max_, float val_, float offset_, uint32_t port_=0, std::string axis_="z", float home_=0, InnerModelTransform *parent_=NULL) : InnerModelTransform(id_,QString("static"),0,0,0,0,0,0, 0, parent_)
	{
		min = min_;
		max = max_;
		port = port_;
		axis = axis_;
		home = home_;
		offset = offset_;
		fixed = false;
		setPosition(val_);
	}
	void print(bool verbose)
	{
		printf("Prismatic Joint: %s\n", qPrintable(id));
		if (verbose)
		{
			((QMat *)this)->print(qPrintable(id));
			getTr().print(id+"_T");
		}
	}
	void save(QTextStream &out, int tabs)
	{
		for (int i=0; i<tabs; i++) out << "\t";
		out << "### joints cannot be saved yet ###\n";
	}

	void update()
	{
		updateChildren();
	}
	float getPosition() { return value; }
	float setPosition(float v)
	{
		float ret;
		if (v <= max and v >= min)
		{
			ret = v;
		}
		else
		{
			if (v > max)
				ret = max;
			else
				ret = min;
		}
		value = v = ret;
		if (axis == "x")
		{
			set(0,0,0, v+offset,0,0);
		}
		else if (axis == "y")
		{
			set(0,0,0, 0,v+offset,0);
		}
		else if (axis == "z")
		{
			set(0,0,0, 0,0,v+offset);
		}
		else
		{
			qFatal("internal error, no such axis %s\n", axis.c_str());
		}
		return ret;
	}
	float value, offset;
	float min, max;
	float home;
	uint32_t port;
	std::string axis;
};


class InnerModelDifferentialRobot : public InnerModelTransform
{
public:
	InnerModelDifferentialRobot(QString id_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, uint32_t port_=0, InnerModelTransform *parent_=NULL) : InnerModelTransform(id_,QString("static"),tx_,ty_,tz_,rx_,ry_,rz_, 0, parent_)
	{
		port = port_;
	}
	uint32_t port;
};



class InnerModelPlane : public InnerModelNode
{
public:
	InnerModelPlane(QString id_, QString texture_, float width_, float height_,float depth_, int repeat_, float nx_, float ny_, float nz_, float px_, float py_, float pz_, InnerModelNode *parent_=NULL) : InnerModelNode(id_, parent_)
	{
		normal = QVec::vec3(nx_, ny_, nz_);
		point = QVec::vec3(px_, py_, pz_);
		nx = ny = nz = px = py = pz = NULL;
		texture = texture_;
		width = width_;
		height = height_;
		depth = depth_;
		repeat = repeat_;
	}
	void print(bool verbose)
	{
		if (verbose) normal.print(QString("Plane: ")+id);
	}
	void save(QTextStream &out, int tabs)
	{
		for (int i=0; i<tabs; i++) out << "\t";
		out << "<plane id=\"" << id << "\" texture=\"" << texture << "\" repeat=\"" << QString::number(repeat, 'g', 10) << "\" nx=\"" << QString::number(normal(0), 'g', 10) << "\" ny=\"" << QString::number(normal(1), 'g', 10) << "\" nz=\"" << QString::number(normal(2), 'g', 10) << "\" px=\"" << QString::number(point(0), 'g', 10) << "\" py=\"" << QString::number(point(1), 'g', 10) << "\" pz=\"" << QString::number(point(2), 'g', 10) << "\" />\n";
	}
	void setUpdatePointers(float *nx_, float *ny_, float *nz_, float *px_, float *py_, float *pz_)
	{
		nx = nx_;
		ny = ny_;
		nz = nz_;
		px = px_;
		py = py_;
		pz = pz_;
		nx = ny = nz = px = py = pz = NULL;
		fixed = false;
	}
	void update()
	{
		if (!fixed)
		{
			update(nx!=NULL?*nx:normal(0), ny!=NULL?*ny:normal(1), nz!=NULL?*nz:normal(2), px!=NULL?*px:point(0), py!=NULL?*py:point(1), pz!=NULL?*pz:point(2));
		}
		updateChildren();
	}
	void update(float nx_, float ny_, float nz_, float px_, float py_, float pz_)
	{
		normal(0) = nx_;
		normal(1) = ny_;
		normal(2) = nz_;
		point(0) = px_;
		point(1) = py_;
		point(2) = pz_;
		fixed = true;
	}
	QVec normal, point;
	QString texture;
	float width, height,depth;
	int repeat;
	float *nx, *ny, *nz;
	float *px, *py, *pz;
};

class InnerModelCamera : public InnerModelNode
{
public:
	InnerModelCamera(QString id_, float width_, float height_, float focal_, InnerModelNode *parent_=NULL) : InnerModelNode(id_, parent_)
	{
		camera = Cam(focal, focal, width/2., height/2.);
		camera.setSize(width, height);
		camera.print(id_);
		width = width_;
		height = height_;
		focal = focal_;
	}
	void print(bool verbose)
	{
		if (verbose) camera.print(QString("Camera: ")+id);
	}
	void save(QTextStream &out, int tabs)
	{
		for (int i=0; i<tabs; i++) out << "\t";
		out << "<camera id=\"" << id << "\" width=\"" << camera.getWidth() << "\" height=\"" << camera.getHeight() << "\" focal=\"" << QString::number(camera.getFocal(), 'g', 10) << "\" />\n";
	}
	Cam camera;
	void update()
	{
		if (fixed)
		{
		}
		updateChildren();
	}
	float width, height, focal;
};

class InnerModelRGBD : public InnerModelCamera
{
public:
	InnerModelRGBD(QString id_, float width, float height, float focal, uint32_t _port, QString _ifconfig, InnerModelNode *parent_=NULL) : InnerModelCamera(id_, width, height, focal, parent_)
	{
		port = _port;
		ifconfig = _ifconfig;
	}
	void save(QTextStream &out, int tabs)
	{
		for (int i=0; i<tabs; i++) out << "\t";
		out << "<rgbd id=\"" << id << "\" width=\"" << camera.getWidth() << "\" height=\"" << camera.getHeight() << "\" focal=\"" << QString::number(camera.getFocal(), 'g', 10) << "\" />\n";
	}
	uint32_t port;
	QString ifconfig;
};

class InnerModelIMU : public InnerModelNode
{
public:
	InnerModelIMU(QString id_, uint32_t _port, InnerModelNode *parent_=NULL) : InnerModelNode(id_, parent_)
	{
		port = _port;
	}
	void save(QTextStream &out, int tabs)
	{
		for (int i=0; i<tabs; i++) out << "\t";
		out << "<imu id=\"" << id << "\" />\n";
	}
	void print(bool verbose)
	{
		if (verbose) printf("IMU.");
	}
	void update()
	{
		if (fixed)
		{
		}
		updateChildren();
	}
	uint32_t port;
};

class InnerModelLaser : public InnerModelNode
{
public:
	InnerModelLaser(QString id_, uint32_t _port, uint32_t _min, uint32_t _max, float _angle, uint32_t _measures, QString _ifconfig, InnerModelNode *parent_=NULL) : InnerModelNode(id_, parent_)
	{
		port = _port;
		min = _min;
		max = _max;
		measures = _measures;
		angle = _angle;
		ifconfig = _ifconfig;
	}
	void save(QTextStream &out, int tabs)
	{
		for (int i=0; i<tabs; i++) out << "\t";
		out << "<laser id=\"" << id << "\" />\n";
	}
	void print(bool verbose)
	{
		if (verbose) printf("LASER.");
	}
	void update()
	{
		if (fixed)
		{
		}
		updateChildren();
	}
	uint32_t port;
	uint32_t min, max;
	float angle;
	uint32_t measures;
	QString ifconfig;
};



class InnerModelMesh : public InnerModelNode
{
public:
	enum RenderingModes { NormalRendering=0, WireframeRendering=1};
	
	InnerModelMesh(QString id_, QString meshPath_, float scale, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, InnerModelNode *parent_=NULL) : InnerModelNode(id_, parent_)
	{
		InnerModelMesh(id_,meshPath_,scale,scale,scale,render_,tx_,ty_,tz_,rx_,ry_,rz_,parent_);
	}
	InnerModelMesh(QString id_, QString meshPath_, float scalex_, float scaley_, float scalez_, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, InnerModelNode *parent_=NULL) : InnerModelNode(id_, parent_)
	{
		id = id_;
		render = render_;
		meshPath = meshPath_;
		scalex = scalex_;
		scaley = scaley_;
		scalez = scalez_;
		tx = tx_;
		ty = ty_;
		tz = tz_;
		rx = rx_;
		ry = ry_;
		rz = rz_;
	}
	void save(QTextStream &out, int tabs)
	{
		for (int i=0; i<tabs; i++) out << "\t";
		out << "<mesh id=\""<<id<<"\"" <<" file=\"" << meshPath << "\" scale=\"" << QString::number(scalex, 'g', 10) << ","<< QString::number(scaley, 'g', 10)<< ","<< QString::number(scalez, 'g', 10) << "\" tx=\"" << QString::number(tx, 'g', 10) << "\" ty=\"" << QString::number(ty, 'g', 10) << "\" tz=\"" << QString::number(tz, 'g', 10) << "\" rx=\"" << QString::number(rx, 'g', 10) << "\" ry=\"" << QString::number(ry, 'g', 10) << "\" rz=\"" << QString::number(rz, 'g', 10) << "\" />\n";
	}
	void print(bool verbose)
	{
		if (verbose) printf("Mesh: %s\n", qPrintable(id));
	}
	void update()
	{
		if (fixed)
		{
		}
		updateChildren();
	}
	void setScale(float x, float y, float z) { scalex=x; scaley=y; scalez=z; }
	RenderingModes render;
	bool normalRendering() const { return render == NormalRendering; }
	bool wireframeRendering() const { return render == WireframeRendering; }
	QString meshPath;
	float scalex, scaley, scalez;
	float tx, ty, tz;
	float rx, ry, rz;
};

class InnerModelPointCloud : public InnerModelNode
{
public:
	InnerModelPointCloud(QString id_, InnerModelNode *parent_=NULL) : InnerModelNode(id_, parent_)
	{
		id = id_;
	}
	void save(QTextStream &out, int tabs)
	{
		for (int i=0; i<tabs; i++) out << "\t";
		out << "<pointcloud id=\""<<id<<"\"/>\n";
	}
	void print(bool verbose)
	{
		if (verbose) printf("Point Cloud: %s\n", qPrintable(id));
	}
	void update()
	{
		if (fixed)
		{
		}
		updateChildren();
	}
};

#endif
