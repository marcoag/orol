#include <orol/visual/innermodel/innermodel.h>

void InnerModelNode::treePrint(QString s, bool verbose)
{
	printf("%s%s l(%d) [%d]\n", qPrintable(s), qPrintable(id), level, children.size());
	QList<InnerModelNode*>::iterator i;
	for (i=children.begin(); i!=children.end(); i++)
	{
		if (verbose)
			(*i)->print(verbose);
		(*i)->treePrint(s+QString("  "), verbose);
		
	}
}


/// (Con/De)structors
InnerModel::InnerModel(std::string xmlFilePath)
{
  mutex = new QMutex(QMutex::Recursive);
  if (not InnerModelReader::load(QString::fromStdString(xmlFilePath), this))
  {
    qFatal("InnerModelReader::load error using file %s\n", xmlFilePath.c_str());
  }
}

InnerModel::InnerModel()
{
	// Set Mutex
	mutex = new QMutex(QMutex::Recursive);
	// Set Root node
	InnerModelTransform *root = new InnerModelTransform("root", "static", 0, 0, 0, 0, 0, 0, 0);
	setRoot(root);
	hash["root"] = root;

	// How to use:
	//   InnerModelTransform *tr = innerModel->newTransform("name", parent, rx, ry, rz, px, py, pz);
	//   parent->addChild(tr);
}

InnerModel::InnerModel(const InnerModel &original){}

InnerModel::~InnerModel(){}
///Remove sub tree and return sa list with his id
void InnerModel::removeSubTree(InnerModelNode *node, QStringList *l)
{
	QList<InnerModelNode*>::iterator i;
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{
		removeSubTree(*i,l);
	}
	node->parent->children.removeOne(node);
	l->append(node->id);
	removeNode(node->id);
}

///get sub tree and return a list with his id
void InnerModel::getSubTree(InnerModelNode *node, QStringList *l)
{
	QList<InnerModelNode*>::iterator i;
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{
		getSubTree(*i,l);
	}
	//node->parent->children.removeOne(node);
	l->append(node->id);
	//removeNode(node->id);
}


bool InnerModel::save(QString path)
{
	QFile file(path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		return false;

	QTextStream out(&file);
	root->save(out, 0);
	file.close();
	return true;
}


InnerModel InnerModel::cloneFake(const QVec & basePose) const
{
	InnerModel rob( *this );
	rob.updateTransformValues("base", basePose(0), 0, basePose(1), 0, basePose(2), 0 );
	return rob;
}


/// Auto update method
void InnerModel::update()
{
	root->update();
	cleanupTables();
}

void InnerModel::cleanupTables()
{
	 QMutexLocker l(mutex);
	 localHashTr.clear();
	 localHashRot.clear();
}

void InnerModel::setUpdateRotationPointers(QString rotationId, float *x, float *y, float *z)
{
	InnerModelTransform *aux;
	if ((aux=dynamic_cast<InnerModelTransform *>(hash[rotationId])) != NULL)
		aux->setUpdateRotationPointers(x, y, z);
	else if (hash[rotationId] == NULL)
		qDebug() << "There is no such" << rotationId << "node";
	else
		qDebug() << "Dynamic cast error from" << rotationId << "to InnerModelTransform. " << typeid(hash[rotationId]).name();
}

void InnerModel::setUpdateTranslationPointers(QString translationId, float *x, float *y, float *z)
{
	InnerModelTransform *aux;
	if ((aux=dynamic_cast<InnerModelTransform *>(hash[translationId])) != NULL)
		aux->setUpdateTranslationPointers(x, y, z);
	else if (hash[translationId] == NULL)
		qDebug() << "There is no such" << translationId << "node";
	else
		qDebug() << "Dynamic cast error from" << translationId << "to InnerModelTransform. " << typeid(hash[translationId]).name();
}

void InnerModel::setUpdateTransformPointers(QString transformId, float *tx, float *ty, float *tz, float *rx, float *ry, float *rz)
{
	InnerModelTransform *aux;
	if ((aux=dynamic_cast<InnerModelTransform *>(hash[transformId])) != NULL)
		aux->setUpdatePointers(tx, ty, tz,rx,ry,rz);
	else if (hash[transformId] == NULL)
		qDebug() << "There is no such" << transformId << "node";
}

void InnerModel::setUpdatePlanePointers(QString planeId, float *nx, float *ny, float *nz, float *px, float *py, float *pz)
{
	InnerModelPlane *aux;
	if ((aux=dynamic_cast<InnerModelPlane *>(hash[planeId])) != NULL)
		aux->setUpdatePointers(nx, ny, nz, px, py, pz);
	else if (hash[planeId] == NULL)
		qDebug() << "There is no such" << planeId << "node";
	else
		qDebug() << "Dynamic cast error from" << planeId << "to InnerModelPlane";
}

void InnerModel::updateTransformValues(QString transformId, float tx, float ty, float tz, float rx, float ry, float rz, QString parentId)
{
	cleanupTables();
	
	InnerModelTransform *aux = dynamic_cast<InnerModelTransform *>(hash[transformId]);
	if (aux != NULL)
	{
		if (parentId!="")
		{
			InnerModelTransform *auxParent = dynamic_cast<InnerModelTransform *>(hash[parentId]);
			if (auxParent!=NULL)
			{
				RTMat Tbi;
				Tbi.setTr( tx,ty,tz);
				Tbi.setR ( rx,ry,rz);
				
				///Tbp Inverse = Tpb. This gets Tpb directly. It's the same
				RTMat Tpb= getTransformationMatrix ( getNode ( transformId)->parent->id,parentId );
				///New Tpi
				RTMat Tpi=Tpb*Tbi;

				QVec angles =Tpi.extractAnglesR();
				QVec tr=Tpi.getTr();
				
				rx=angles.x();ry=angles.y();rz=angles.z();
				tx=tr.x();ty=tr.y();tz=tr.z();
			}
			else if (hash[parentId] == NULL)
				qDebug() << "There is no such" << parentId << "node";
			else
				qDebug() << "?????";
		}
		//always update
		aux->update(tx,ty,tz,rx,ry,rz);
	}
	else if (hash[transformId] == NULL)
		qDebug() << "There is no such" << transformId << "node";
	else
		qDebug() << "?????";
}

void InnerModel::updatePlaneValues(QString planeId, float nx, float ny, float nz, float px, float py, float pz)
{
	cleanupTables();

	InnerModelPlane *plane = dynamic_cast<InnerModelPlane *>(hash[planeId]);
	if (plane != NULL)
	{
		plane->update(nx, ny, nz, px, py, pz);
	}
	else if (hash[planeId] == NULL)
		qDebug() << "There is no such" << planeId << "node";
	else
		qDebug() << "?????";
}

void InnerModel::updateTranslationValues(QString transformId, float tx, float ty, float tz, QString parentId)
{
	cleanupTables();

	InnerModelTransform *aux = dynamic_cast<InnerModelTransform *>(hash[transformId]);
	if (aux != NULL)
	{
		if (parentId!="")
			updateTransformValues(transformId, tx,ty,tz,0.,0.,0.,parentId);
		else
			aux->update(tx,ty,tz,aux->backrX,aux->backrY,aux->backrZ);
	}
	else if (hash[transformId] == NULL)
		qDebug() << "There is no such" << transformId << "node";
	else
		qDebug() << "?????";
}

void InnerModel::updateRotationValues(QString transformId, float rx, float ry, float rz, QString parentId)
{
	cleanupTables();

	InnerModelTransform *aux = dynamic_cast<InnerModelTransform *>(hash[transformId]);
	if (aux != NULL)
	{
		if (parentId!="")
		{
			updateTransformValues(transformId,0.,0.,0.,rx,ry,rz,parentId);
		}
		else
			aux->update(aux->backtX,aux->backtY,aux->backtZ,rx,ry,rz);
	}
	else if (hash[transformId] == NULL)
		qDebug() << "There is no such" << transformId << "node";
	else
		qDebug() << "?????";
}

void InnerModel::updateJointValue(QString jointId, float angle)
{
	cleanupTables();

	InnerModelJoint *j = dynamic_cast<InnerModelJoint *>(hash[jointId]);
	if (j != NULL)
	{
		j->setAngle(angle);
	}
	else if (hash[jointId] == NULL)
		qDebug() << "There is no such" << jointId << "node";
	else
		qDebug() << "?????";
}

void InnerModel::updatePrismaticJointPosition(QString jointId, float pos)
{
	cleanupTables();

	InnerModelPrismaticJoint *j = dynamic_cast<InnerModelPrismaticJoint *>(hash[jointId]);
	if (j != NULL)
	{
		j->setPosition(pos);
	}
	else if (hash[jointId] == NULL)
		qDebug() << "There is no such" << jointId << "node";
	else
		qDebug() << "?????";
}



/// Model construction methods
void InnerModel::setRoot(InnerModelNode *node)
{
	root = node;
	hash["root"] = root;
	root->parent=NULL;
}

InnerModelTransform *InnerModel::newTransform(QString id, QString engine, InnerModelNode *parent, float tx, float ty, float tz, float rx, float ry, float rz, float mass)
{
	if (hash.contains(id)) qFatal("InnerModel::newTransform: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelTransform *newnode = new InnerModelTransform(id, engine, tx, ty, tz, rx, ry, rz, mass, parent);
	hash[id] = newnode;
	return newnode;
}

InnerModelJoint *InnerModel::newJoint(QString id, InnerModelTransform *parent,float lx, float ly, float lz,float hx, float hy, float hz, float tx, float ty, float tz, float rx, float ry, float rz, float min, float max, uint32_t port,std::string axis, float home)
{
	if (hash.contains(id)) qFatal("InnerModel::newJoint: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelJoint *newnode = new InnerModelJoint(id,lx,ly,lz,hx,hy,hz, tx, ty, tz, rx, ry, rz, min, max, port, axis, home, parent);
	hash[id] = newnode;
	return newnode;
}

InnerModelPrismaticJoint *InnerModel::newPrismaticJoint(QString id, InnerModelTransform *parent, float min, float max, float value, float offset, uint32_t port,std::string axis, float home)
{
	if (hash.contains(id)) qFatal("InnerModel::newPrismaticJoint: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelPrismaticJoint *newnode = new InnerModelPrismaticJoint(id, min, max, value, offset, port, axis, home, parent);
	hash[id] = newnode;
	return newnode;
}


InnerModelDifferentialRobot *InnerModel::newDifferentialRobot(QString id, InnerModelTransform *parent, float tx, float ty, float tz, float rx, float ry, float rz, uint32_t port)
{
	if (hash.contains(id)) qFatal("InnerModel::newDifferentialrobot: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelDifferentialRobot *newnode = new InnerModelDifferentialRobot(id, tx, ty, tz, rx, ry, rz, port, parent);
	hash[id] = newnode;
	return newnode;
}

InnerModelCamera *InnerModel::newCamera(QString id, InnerModelNode *parent, float width, float height, float focal)
{
	if (hash.contains(id)) qFatal("InnerModel::newCamera: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelCamera *newnode = new InnerModelCamera(id, width, height, focal, parent);
	hash[id] = newnode;
	return newnode;
}

InnerModelRGBD *InnerModel::newRGBD(QString id, InnerModelNode *parent, float width, float height, float focal, uint32_t port, QString ifconfig)
{
	if (hash.contains(id)) qFatal("InnerModel::newRGBD: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelRGBD *newnode = new InnerModelRGBD(id, width, height, focal, port, ifconfig, parent);
	hash[id] = newnode;
	return newnode;
}

InnerModelIMU *InnerModel::newIMU(QString id, InnerModelNode *parent, uint32_t port)
{
	if (hash.contains(id)) qFatal("InnerModel::newIMU: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
// 	printf("newIMU id=%s  parentId=%s port=%d\n", id.toStdString().c_str(), parent->id.toStdString().c_str(), port);
	InnerModelIMU *newnode = new InnerModelIMU(id, port, parent);
	hash[id] = newnode;
	return newnode;
}

InnerModelLaser *InnerModel::newLaser(QString id, InnerModelNode *parent, uint32_t port, uint32_t min, uint32_t max, float angle, uint32_t measures, QString ifconfig)
{
	if (hash.contains(id)) qFatal("InnerModel::newLaser: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
// 	printf("newLaser id=%s  parentId=%s port=%d min=%d max=%d angle=%f measures=%d\n", id.toStdString().c_str(), parent->id.toStdString().c_str(), port, min, max, angle, measures);
	InnerModelLaser *newnode = new InnerModelLaser(id, port, min, max, angle, measures, ifconfig, parent);
	hash[id] = newnode;
	return newnode;
}

InnerModelPlane *InnerModel::newPlane(QString id, InnerModelNode *parent, QString texture, float width, float height, float depth, int repeat, float nx, float ny, float nz, float px, float py, float pz)
{
	if (hash.contains(id)) qFatal("InnerModel::newPlane: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelPlane *newnode = new InnerModelPlane(id, texture, width, height, depth, repeat, nx, ny, nz, px, py, pz, parent);
	hash[id] = newnode;
	return newnode;
}

InnerModelMesh *InnerModel::newMesh(QString id, InnerModelNode *parent, QString path, float scalex, float scaley, float scalez, int render, float tx, float ty, float tz, float rx, float ry, float rz)
{
	if (hash.contains(id)) qFatal("InnerModel::newMesh: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelMesh *newnode = new InnerModelMesh(id, path, scalex, scaley, scalez, (InnerModelMesh::RenderingModes)render, tx, ty, tz, rx, ry, rz, parent);
	hash[id] = newnode;
	return newnode;
}
InnerModelMesh *InnerModel::newMesh(QString id, InnerModelNode *parent, QString path, float scale, int render, float tx, float ty, float tz, float rx, float ry, float rz)
{
	return newMesh(id,parent,path,scale,scale,scale,render,tx,ty,tz,rx,ry,rz);
}

InnerModelPointCloud *InnerModel::newPointCloud(QString id, InnerModelNode *parent)
{
	if (hash.contains(id)) qFatal("InnerModel::newPointCloud: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelPointCloud *newnode = new InnerModelPointCloud(id, parent);
	hash[id] = newnode;
	printf("Inserted point cloud %s ptr(%p), on node %s\n", id.toStdString().c_str(), newnode, parent->id.toStdString().c_str());
	return newnode;
}


InnerModelTransform *InnerModel::getTransform(const QString &id)
{
	InnerModelTransform *tr = dynamic_cast<InnerModelTransform *>(hash[id]);
	if (not tr)
	{
		if (not hash[id])
			qFatal("No such transform %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a transform", id.toStdString().c_str());
	}
	return tr;
}

InnerModelJoint *InnerModel::getJoint(const QString &id)
{
	InnerModelJoint *tr = dynamic_cast<InnerModelJoint *>(hash[id]);
	if (not tr)
	{
		if (not hash[id])
			qFatal("No such joint %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a joint", id.toStdString().c_str());
	}
	return tr;
}

InnerModelPrismaticJoint *InnerModel::getPrismaticJoint(const QString &id)
{
	InnerModelPrismaticJoint *tr = dynamic_cast<InnerModelPrismaticJoint *>(hash[id]);
	if (not tr)
	{
		if (not hash[id])
			qFatal("No such joint %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a prismatic joint", id.toStdString().c_str());
	}
	return tr;
}

InnerModelCamera *InnerModel::getCamera(const QString id)
{
	InnerModelCamera *camera = dynamic_cast<InnerModelCamera *>(hash[id]);
	if (not camera)
	{
		if (not hash[id])
			qFatal("No such camera %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a camera", id.toStdString().c_str());
	}
	return camera;
}

InnerModelRGBD *InnerModel::getRGBD(const QString id)
{
	InnerModelRGBD *camera = dynamic_cast<InnerModelRGBD *>(hash[id]);
	if (not camera)
	{
		if (not hash[id])
			qFatal("No such camera %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a camera", id.toStdString().c_str());
	}
	return camera;
}

InnerModelIMU *InnerModel::getIMU(const QString id)
{
	InnerModelIMU *imu = dynamic_cast<InnerModelIMU *>(hash[id]);
	if (not imu)
	{
		if (not hash[id])
			qFatal("No such innertial unit %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be an innertial unit", id.toStdString().c_str());
	}
	return imu;
}

InnerModelLaser *InnerModel::getLaser(const QString id)
{
	InnerModelLaser *laser = dynamic_cast<InnerModelLaser *>(hash[id]);
	if (not laser)
	{
		if (not hash[id])
			qFatal("No such laser %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be an laser", id.toStdString().c_str());
	}
	return laser;
}

InnerModelPlane *InnerModel::getPlane(const QString &id)
{
	InnerModelPlane *plane = dynamic_cast<InnerModelPlane *>(hash[id]);
	if (not plane)
	{
		if (not hash[id])
			qFatal("No such plane %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a plane", id.toStdString().c_str());
	}
	return plane;
}

InnerModelMesh *InnerModel::getMesh(const QString &id)
{
	InnerModelMesh *mesh = dynamic_cast<InnerModelMesh *>(hash[id]);
	if (not mesh)
	{
		if (not hash[id])
			qFatal("No such mesh %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a mesh", id.toStdString().c_str());
	}
	return mesh;
}

InnerModelPointCloud *InnerModel::getPointCloud(const QString &id)
{
	InnerModelPointCloud *pointcloud = dynamic_cast<InnerModelPointCloud *>(hash[id]);
	if (not pointcloud)
	{
		if (not hash[id])
			qFatal("No such pointcloud %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a pointcloud", id.toStdString().c_str());
	}
	return pointcloud;
}

InnerModelDifferentialRobot *InnerModel::getDifferentialRobot(const QString &id)
{
	InnerModelDifferentialRobot *diff = dynamic_cast<InnerModelDifferentialRobot *>(hash[id]);
	if (not diff)
	{
		if (not hash[id])
			qFatal("No such differential robot %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a differential robot", id.toStdString().c_str());
	}
	return diff;
}


/// Stereo geometry

/**
 * \brief Computes essential and fundamental matrices for the pair of cameras stated in the parameters
 * @param firstCam id of the firt camera
 * @param secondCam id of the second camera
 */
// TODO COMPROBAR
void InnerModel::updateStereoGeometry(const QString &firstCam, const QString &secondCam)
{
	this->essential.set(this->getRotationMatrixTo(secondCam, firstCam), this->getTranslationVectorTo(secondCam, firstCam) );
	this->fundamental.set( essential, dynamic_cast<InnerModelCamera *>(hash[firstCam])->camera, dynamic_cast<InnerModelCamera *>(hash[secondCam])->camera );
}

/**
 * \brief Computes de 3D triangulation of two correspondent image points in robot reference system
 * @param left 2D image reference system point
 * @param right 2D image reference system point
 * @return 3D point in robot(base) reference system
 */
// TODO COMPROBAR
QVec InnerModel::compute3DPointInCentral(const QString & firstCam, const QVec & first, const QString & secondCam, const QVec & second)
{
	T detA, a/*, b*/, c;

	QVec pI = this->getRotationMatrixTo("central", "firstCam") * static_cast<InnerModelCamera *>(hash[firstCam])->camera.getRayHomogeneous( first );
	QVec pD = this->getRotationMatrixTo("central", "secondCam") * static_cast<InnerModelCamera *>(hash[secondCam])->camera.getRayHomogeneous( second );
// 	QVec pI = (centralToLeftMotor.getR().transpose() * leftMotorToLeftCamera.getR().transpose()) * leftCamera.getRayHomogeneous( left );
// 	QVec pD = (centralToRightMotor.getR().transpose() * rightMotorToRightCamera.getR().transpose()) * rightCamera.getRayHomogeneous( right );

	QVec n = QVec::vec3( pI(1)-pD(1) , -pI(0)+pD(0) , pI(0)*pD(1)-pD(0)*pI(1) );

	QMat A(3,3);
	A(0,0)=pI(0);  A(0,1)=-pD(0);  A(0,2)=n(0);
	A(1,0)=pI(1);  A(1,1)=-pD(1);  A(1,2)=n(1);
	A(2,0)=1;      A(2,1)=-1;      A(2,2)=n(2);

	detA = A(0,0)*(A(1,1)*A(2,2)-A(1,2)*A(2,1))-A(0,1)*(A(1,0)*A(2,2)-A(1,2)*A(2,0))+A(0,2)*(A(1,0)*A(2,1)-A(1,1)*A(2,0));

	float baseLine = this->getBaseLine(); //NOT IMPLEMENTED
	a = baseLine*(-pD(1)*n(2)+n(1))/detA;
// 	b = baseLine*(pI(1)*n(2)-n(1))/detA;
	c = baseLine*(-pI(1)+pD(1))/detA;

	QVec res(3);
	res(0) = (a*pI(0)-(baseLine/2.))+(c*n(0))/2.;
	res(1) = a*pI(1) + c*n(1)/2.;
	res(2) = a*pI(2) + c*n(2)/2.;

	return res;
}

//
// TODO COMPROBAR
QVec InnerModel::compute3DPointInRobot(const QString & firstCamera, const QVec & left, const QString & secondCamera, const QVec & right)
{
	return transform("central", this->compute3DPointInCentral( firstCamera, left, secondCamera, right), "base");
}

QVec InnerModel::compute3DPointFromImageCoords(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem)
{
	QVec pI(3), pD(3), n(3), ray(3), T(3), TI(3), TD(3), pR(0), abc(3);
	QMat A(3,3);

	ray = backProject(firstCamera, left);
	pI = getRotationMatrixTo(refSystem, firstCamera)*ray;


	pI(0)=pI(0)/pI(2);
	pI(1)=pI(1)/pI(2);
	pI(2)=1.;

	ray = backProject(secondCamera, right);
	pD = getRotationMatrixTo(refSystem, secondCamera)*ray;

	pD(0)=pD(0)/pD(2);
	pD(1)=pD(1)/pD(2);
	pD(2)=1.;

	n = pI ^ pD;

	A(0,0)=pI(0);  A(0,1)=-pD(0);  A(0,2)=n(0);
	A(1,0)=pI(1);  A(1,1)=-pD(1);  A(1,2)=n(1);
	A(2,0)=pI(2);  A(2,1)=-pD(2);  A(2,2)=n(2);

	TI = getTranslationVectorTo(refSystem, firstCamera).fromHomogeneousCoordinates();
	TD = getTranslationVectorTo(refSystem, secondCamera).fromHomogeneousCoordinates();
	T = TD - TI ;

	abc = (A.invert())*T;

	pR = (pI*abc(0));
	pR = pR + TI;
	pR = (n*(abc(2)/2)) + pR;

	return pR;

}

QVec InnerModel::compute3DPointFromImageAngles(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem)
{
	QVec pI(3), pD(3), n(3), ray(3), T(3), TI(3), TD(3), pR(0), abc(3);
	QMat A(3,3);

	ray(0) = tan(left(0));
	ray(1) = tan(left(1));
	ray(2) = 1.;
	pI = ray;//getRotationMatrixTo(refSystem, firstCamera)*ray;


	pI(0)=pI(0)/pI(2);
	pI(1)=pI(1)/pI(2);
	pI(2)=1.;

	ray(0) = tan(right(0));
	ray(1) = tan(right(1));
	ray(2) = 1.;
	pD = ray;//getRotationMatrixTo(refSystem, secondCamera)*ray;

	pD(0)=pD(0)/pD(2);
	pD(1)=pD(1)/pD(2);
	pD(2)=1.;


	n = pI ^ pD;

	A(0,0)=pI(0);  A(0,1)=-pD(0);  A(0,2)=n(0);
	A(1,0)=pI(1);  A(1,1)=-pD(1);  A(1,2)=n(1);
	A(2,0)=pI(2);  A(2,1)=-pD(2);  A(2,2)=n(2);

	TI = getTranslationVectorTo(refSystem, firstCamera).fromHomogeneousCoordinates();
	TD = getTranslationVectorTo(refSystem, secondCamera).fromHomogeneousCoordinates();
	T = TD - TI;

	abc = (A.invert())*T;

	pR = (pI*abc(0));
	pR = pR + TI;
	pR = (n*(abc(2)/2)) + pR;

	return pR;

}


/// Information retrieval methods
QVec InnerModel::transform(const QString &destId, const QVec &initVec, const QString &origId)
{	
	return (getTransformationMatrix(destId, origId) * initVec.toHomogeneousCoordinates()).fromHomogeneousCoordinates();
}

QVec InnerModel::project(QString reference, QVec origVec, QString cameraId)
{
	origVec = transform(cameraId, origVec, reference);

	QVec pc;
	InnerModelCamera *camera=NULL;

	camera = dynamic_cast<InnerModelCamera *>(hash[cameraId]);
	if (not camera)
		qFatal("No such %s camera", qPrintable(cameraId));

	 pc = camera->camera.project(origVec);

	return QVec::vec3(pc(0), pc(1), origVec.norm2());
}


/**
 * \brief Retro-projection function, defines a line in the camera reference system which can be parametrized by the depth s with the expression:
 * p = s*[ (u-u0) / alfaU ; (v-v0) / alfaV ; 1] being alfaU and alfaV the horizontal and vertical focals of the camera (in pixels)
 * p has value 1 in the z axis, the one going out the camera.
 * @param cameraId name of camara to be used as known in innermodel tree
 * @param coord  point in image coordinates
 * @return a line en camera reference system that can be parametrized by the depth s
 */
QVec InnerModel::backProject( const QString &cameraId, const QVec &	coord) //const
{
	if(hash.contains(cameraId))
	{
		QVec p = static_cast<InnerModelCamera *>(hash[cameraId])->camera.getRayHomogeneous(coord);
		return p;
	}
	return QVec();
}

void InnerModel::imageCoordToAngles(const QString &cameraId, QVec coord, float &pan, float &tilt, const QString & anglesRefS)
{
	QVec ray = backProject(cameraId, coord);

	QVec finalRay = getRotationMatrixTo(anglesRefS, cameraId)*ray;

	pan = atan2(finalRay(0), finalRay(2));
	tilt = atan2(finalRay(1), finalRay(2));

}

QVec InnerModel::anglesToImageCoord(const QString &cameraId, float pan, float tilt, const QString & anglesRefS)
{
 	QVec p(3), ray(3);

	p(0) = tan(pan);
	p(1) = tan(tilt);
	p(2) = 1;

 	ray = getRotationMatrixTo(cameraId, anglesRefS) * p;
 	ray(0)=ray(0)/ray(2);
	ray(1)=ray(1)/ray(2);
	ray(2)=1;

	return project(cameraId, ray, cameraId);

}

QVec InnerModel::imageCoordPlusDepthTo(QString cameraId, QVec coord, float depth, QString reference)
{
	//We obtain a 3D line (a,b,1) in camera reference system that can be parametrized in depth to obtain a point at "depth" from the camera.
	QVec p = backProject( cameraId, coord ) * depth;
	//Now we transform it to requested node of the robot.
	if(p.size()>0)
		return transform(reference, p, cameraId);
	return p;
}

QVec InnerModel::projectFromCameraToPlane(const QString &to, const QVec &coord, const QString &cameraId, const QVec &vPlane, const float &dist)
{
 	QMat mSystem(3,3);
 	QVec tIndep(3);
	QVec pCam(3);
	QVec res(3);
	float dxz, dyz;

	pCam(0) = -coord(0)+getCameraWidth(cameraId)/2;
	pCam(1) = -(coord(1)-getCameraHeight(cameraId)/2);
	pCam(2) = getCameraFocal(cameraId);
	QVec pDest = transform(to, pCam, cameraId);
	QVec pCent = transform(to, QVec::vec3(0,0,0), cameraId);
	QVec direc = pDest-pCent;
	dxz = direc(0)/direc(2);
	dyz = direc(1)/direc(2);

	res(2) = dist + vPlane(0)*(dxz*pCent(2)-pCent(0)) + vPlane(1)*(dyz*pCent(2)-pCent(1));
	res(2) = res(2)/(vPlane(0)*dxz+vPlane(1)*dyz+vPlane(2));
	res(0)=dxz*(res(2)-pCent(2))+pCent(0);
	res(1)=dyz*(res(2)-pCent(2))+pCent(1);

/*	res.print("res");

	mSystem(0,0) = vPlane(0);         mSystem(0,1) = vPlane(1);         mSystem(0,2) = vPlane(2);
	mSystem(1,0) = 0;                 mSystem(1,1) = pCent(2)-pDest(2); mSystem(1,2) = pDest(1)-pCent(1);
	mSystem(2,0) = pDest(2)-pCent(2); mSystem(2,1) = 0;                 mSystem(2,2) = pCent(0)-pDest(0);
	tIndep(0) = dist;
	tIndep(1) = pCent(2)*(pDest(1)-pCent(1))+pCent(1)*(pCent(2)-pDest(2));
	tIndep(2) = pCent(0)*(pDest(2)-pCent(2))+pCent(2)*(pCent(0)-pDest(0));

 	return (mSystem.invert())*tIndep;*/
 	return res;
}


//
// bool InnerModel::check3DPointInsideFrustrum(QString cameraId, QVec coor)
// {
// }

/**
 * \brief Returns a 3D vector (A,B,C) containing the horizon line for the specified camera+plane in the form 'Ax + By + C = 0'.
 *
 * <p>
 * Returns a 3D vector (A,B,C) containing the horizon line in the form Ax + By + C = 0. For general lines, it will also work as 'y = Ax + C' (not for vertical lines, which are a very rare case).
 * You can check B to know if the returned vector is a regular line:
 * </p>
 * <p>
 * QVec horizon = innerModel->horizonLine("floor", "mycamera", );
 * <br>
 * if (horizon(1) == 0) printf("Vertical horizon.\n");
 * <br>
 * else printf("Regular horizon.\n");
 * </p>
 */
QVec InnerModel::horizonLine(QString planeId, QString cameraId, float heightOffset)
{
	QMutexLocker l(mutex);
// 	printf("-------------------------------------- cam:%s plane:%s\n", qPrintable(cameraId), qPrintable(planeId));
	// Get camera and plane pointers
	InnerModelPlane *plane = getPlane(planeId);
	InnerModelCamera *camera = getCamera(cameraId);
	// Transform rotate plane normal vector to camera reference system
	QMat rtm = getRotationMatrixTo(cameraId, planeId);
	QVec vec = QVec::vec3(plane->normal(0), plane->normal(1), plane->normal(2));
	QVec normal = rtm*vec;
	if (normal(1) <= 0.0000002) throw false;

	// Create two points
	QVec p1=QVec::vec3(0., 0., 0.), p2=QVec::vec3(0., 0., 0.);
	// Move both points forward
	p1(2) = p2(2) =  1000.;
	if (normal(1) > 0.0000001) p1(1) = p2(1) = p1(2)*normal(2)/normal(1);
	// Move points left/right-wards
	if (normal(1) > 0.0000001) p1(1) -=  200.*normal(0)/normal(1);
	p1(0) =  200.;
	if (normal(1) > 0.0000001) p2(1) -= -200.*normal(0)/normal(1);
	p2(0) = -200.;
	// Project points
	p1 = project(cameraId, p1, cameraId);
	p2 = project(cameraId, p2, cameraId);
	// Compute image line
	double dx=p2(0)-p1(0);
	double dy=p2(1)-p1(1);

	if (abs(dx) <= 1)
	{
		if (abs(dy) <= 1) qFatal("Degenerated camera");
		return QVec::vec3(-1, 0, p1(0));
	}
	else
	{
		return QVec::vec3(dy/dx, -1, camera->camera.getHeight()-(p1(1)-(dy*p1(0)/dx))+heightOffset);
	}
}


/// Matrix transformation retrieval methods
RTMat InnerModel::getTransformationMatrix(const QString &to, const QString &from)
{
	RTMat ret;

	QMutexLocker l(mutex);
	if (localHashTr.contains(QPair<QString, QString>(to, from)))
	{
		ret = localHashTr[QPair<QString, QString>(to, from)];
	}
	else
	{
		setLists(from, to);
		foreach (InnerModelNode *i, listA)
		{
			ret = ((RTMat)(*i)).operator*(ret);
		}
		foreach (InnerModelNode *i, listB)
		{
			ret = i->invert() * ret;
		}
 		localHashTr[QPair<QString, QString>(to, from)] = ret;
	}
	return RTMat(ret);
}

QMat InnerModel::getRotationMatrixTo(const QString &to, const QString &from)
{
	QMat rret = QMat::identity(3);

	QMutexLocker l(mutex);
	
	if (localHashRot.contains(QPair<QString, QString>(to, from)))
	{
		rret = localHashRot[QPair<QString, QString>(to, from)];
	}
	else
	{
		setLists(from, to);
		InnerModelTransform *tf=NULL;

		foreach (InnerModelNode *i, listA)
		{
			if ((tf=dynamic_cast<InnerModelTransform *>(i))!=NULL)
			{
				rret = tf->getR() * rret;
			}
		}
		foreach (InnerModelNode *i, listB)
		{
			if ((tf=dynamic_cast<InnerModelTransform *>(i))!=NULL)
			{
				rret = tf->getR().transpose() * rret;
			}
		}
		localHashRot[QPair<QString, QString>(to, from)] = rret;
	}

	return rret;
}

QVec InnerModel::getTranslationVectorTo(const QString &to, const QString &from)
{
	QMat m = this->getTransformationMatrix(to, from);
	return m.getCol(3);
}

QMat InnerModel::getHomographyMatrix(QString virtualCamera, QString plane, QString sourceCamera)
{
	QVec planeN = getPlane(plane)->normal;
	planeN = getRotationMatrixTo(sourceCamera, plane)*planeN;
	QVec planePoint = transform(sourceCamera, getPlane(plane)->point, plane);

	QMat R  = getRotationMatrixTo(virtualCamera, sourceCamera);
	QMat t  = transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
	QMat n  = QMat(planeN);
	QMat K1 = getCamera(sourceCamera)->camera;
	QMat K2 = getCamera(virtualCamera)->camera;

	double d = -(planePoint*planeN);
	QMat H = K2 * ( R - ((t*n.transpose()) / d) ) * K1.invert();
	return H;
}




QMat InnerModel::getAffineHomographyMatrix(QString virtualCamera, QString plane, QString sourceCamera)
{
	QVec planeN = getPlane(plane)->normal;
	planeN = getRotationMatrixTo(sourceCamera, plane)*planeN;
	QVec planePoint = transform(sourceCamera, getPlane(plane)->point, plane);

	QMat R  = getRotationMatrixTo(virtualCamera, sourceCamera);
	QMat t  = transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
	QMat n  = QMat(planeN);
	QMat K1 = getCamera(sourceCamera)->camera;

	double d = -(planePoint*planeN);
	QMat H = ( R - ((t*n.transpose()) / d) ) * K1.invert();
	for (int r=0;r<2;r++)
		for (int c=0;c<3;c++)
				H(r,c) = H(r,c) * 1000.;
	return H;
}

QMat InnerModel::getPlaneProjectionMatrix(QString virtualCamera, QString plane, QString sourceCamera)
{
	QVec planeN = getPlane(plane)->normal;
	planeN = getRotationMatrixTo(sourceCamera, plane)*planeN;
	QVec planePoint = transform(sourceCamera, getPlane(plane)->point, plane);

	QMat R  = getRotationMatrixTo(virtualCamera, sourceCamera);
	QMat t  = transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
	QMat n  = QMat(planeN);
	QMat K1 = getCamera(sourceCamera)->camera;

	double d = -(planePoint*planeN);
	QMat H = ( R - ((t*n.transpose()) / d) ) * K1.invert();
	QMat HFinal(4,3);
	HFinal.inject(H, 0, 0);
	HFinal = HFinal*(1000*1000);
	HFinal(3,0)=1000*H(2,0);
	HFinal(3,1)=1000*H(2,1);
	HFinal(3,2)=1000*H(2,2);
	return HFinal;
}


void InnerModel::setLists(const QString & origId, const QString & destId)
{
	InnerModelNode *a=hash[origId], *b=hash[destId];
	if (!a)
		throw InnerModelException("Cannot find node: \""+ origId.toStdString()+"\"");
	if (!b)
		throw InnerModelException("Cannot find node: "+ destId.toStdString()+"\"");
	
	int minLevel = a->level<b->level? a->level : b->level;
	listA.clear();
	while (a->level >= minLevel)
	{
		listA.push_back(a);
		if(a->parent == NULL)
		{
// 			qFatal("InnerModel::setLists: It wouldn't be here!!!!");
			break;
		}
		a=a->parent;
	}

	listB.clear();
	while (b->level >= minLevel)
	{
		listB.push_front(b);
		if(b->parent == NULL)
		{
// 			qFatal("InnerModel::setLists: It wouldn't be here!!!!");
			break;
		}
		b=b->parent;
	}
	while (b!=a)
	{
		listA.push_back(a);
		listB.push_front(b);
		a = a->parent;
		b = b->parent;
	}
}

/// Robex Base specific getters

float InnerModel::getCameraFocal(const QString & cameraId) const
{
	return static_cast<InnerModelCamera *>(getNode(cameraId))->camera.getFocal();
}

int InnerModel::getCameraWidth(QString cameraId)
{
	return getCamera(cameraId)->camera.getWidth();
}

int InnerModel::getCameraHeight(const QString & cameraId) const
{
 	return static_cast<InnerModelCamera *>(getNode(cameraId))->camera.getHeight();
}

int InnerModel::getCameraSize(const QString & cameraId) const
{
	return static_cast<InnerModelCamera *>(getNode(cameraId))->camera.getSize();
}

 /**
 * \brief Returns current copy of fundamenta matrix as a float h[3][3] array
 * @param h[][] [3][3] preallocates array of floats to contain de fundamental matrix
 */
void InnerModel::getFundamental(float h[3][3]) const
{
	h[0][0] = fundamental(0,0);
	h[0][1] = fundamental(0,1);
	h[0][2] = fundamental(0,2);
	h[1][0] = fundamental(1,0);
	h[1][1] = fundamental(1,1);
	h[1][2] = fundamental(1,2);
	h[2][0] = fundamental(2,0);
	h[2][1] = fundamental(2,1);
	h[2][2] = fundamental(2,2);
}

QMat InnerModel::getFundamental() const
{
	return fundamental;
}

//
QVec InnerModel::robotToWorld(const QVec & vec)
{
	return transform("world", vec, "base");
}

//
QVec InnerModel::robotInWorld()
{
	return transform("world", QVec::vec3(0,0,0), "base");
}

//
float InnerModel::getBaseX()
{
	return transform("world", QVec::vec3(0,0,0), "base")(0);
}

//
float InnerModel::getBaseZ()
{
	return transform("world", QVec::vec3(0,0,0), "base")(2);
}

//
float InnerModel::getBaseAngle()
{
	return getRotationMatrixTo("world", "base").extractAnglesR()(1);
}

float InnerModel::getBaseRadius() //OJO Get from XML file
{
	return 200.f;
}

//
QVec InnerModel::getBaseOdometry()
{
	QVec res = transform("world", QVec::vec3(0,0,0), "base");
	res(1) = res(2);
	res(2) = getRotationMatrixTo("world", "base").extractAnglesR()(1);
	return res;
}

////////////////////////////////////////////
// LASER
////////////////////////////////////////////
/**
 * \brief Local laser measure with index i in laser array is converted to Any RS
 * @param i indexof laser array
 * @return 3Dpoint in World RS
 */
QVec InnerModel::laserTo(const QString &dest, const QString & laserId, const QVec &p)
{
	return transform(dest, p, laserId);
}

/**
 * \brief Local laser measure of range r and angle alfa is converted to Any RS
 * @param r range measure
 * @param alfa angle measure
 * @return 3-vector of x,y,z coordinates un WRS
 */
QVec InnerModel::laserTo(const QString &dest, const QString & laserId , float r, float alpha)
{
	QVec p(3);
	p(0) = r * sin(alpha);
	p(1) = 0 ;
	p(2) = r * cos(alpha);
	return transform(dest, p , laserId);
}


/**
 * \brief Converts a 3D point en WRS to Laser reference system
 * @param world 3D coordinates of a world point as a 3-vector
 * @return 3D coordinates of given point seen from Laser reference system
 */
//
QVec InnerModel::worldToLaser(const QString & laserId, const QVec & p)
{
	return transform(laserId, p, "world");
}

/**
 * \brief Converts a 3D point in Laser (range,angle) coordinates to Robot reference system
 * @param r range measure
 * @param alfa angle measure
 * @return 3-vector of 3D point in Robot reference system
 */
//
QVec InnerModel::laserToBase(const QString & laserId, float r, float alpha)
{
	QVec p(3);
	p(0) = r * sin ( alpha ) ;
	p(2) = r * cos ( alpha ) ;
	p(1) = 0 ; // Laser reference system
	return transform("base", p , laserId);
}

