#include <orol/visual/innermodel/innermodelreader.h>

InnerModelReader::InnerModelReader()
{

}

bool InnerModelReader::load(const QString &file, InnerModel *model)
{
 	qDebug() << "InnerModelReader: reading" << qPrintable(file);
	QDomDocument doc("mydocument");
	QFile fich(file);
	if (!fich.open(QIODevice::ReadOnly))
	{
		qDebug() << "Can't open" << qPrintable(file);
		return false;
	}

	QString errorMsg;
	int errorLine, errorColumn;
	if (!doc.setContent(&fich, &errorMsg, &errorLine, &errorColumn)) 
	{
		qDebug() << "Can't set document content from" << qPrintable(file);
		qDebug() << "line:" << errorLine << "  column:" << errorColumn;
		qDebug() << "error:" << errorMsg;
		fich.close();
		return false;
	}

	QDomElement root = doc.documentElement();
	if (root.tagName().toLower() != QString("innerModel").toLower())
		qFatal("<innerModel> tag missing.");
	InnerModelTransform *r = new InnerModelTransform(QString("root"), QString("static"), 0, 0, 0, 0, 0, 0, 0);
	model->setRoot(r);
	r->parent = NULL;
	recursive(root, model, model->root);

	fich.close();
	return true;
}



InnerModelReader::~InnerModelReader()
{
}


void InnerModelReader::recursive(QDomNode parentDomNode, InnerModel *model, InnerModelNode *imNode)
{
	QMap<QString, QStringList> nodeAttributes;

	if (  parentDomNode.toElement().tagName().toLower() != QString("innermodel").toLower()
	  and parentDomNode.toElement().tagName().toLower() != QString("transform").toLower()
	  and parentDomNode.toElement().tagName().toLower() != QString("rotation").toLower()
	  and parentDomNode.toElement().tagName().toLower() != QString("translation").toLower()
	  and parentDomNode.toElement().tagName().toLower() != QString("joint").toLower()
	  and parentDomNode.toElement().tagName().toLower() != QString("prismaticjoint").toLower()
	  and parentDomNode.toElement().tagName().toLower() != QString("differentialrobot").toLower()
	)
		if (not parentDomNode.firstChild().isNull())
			qFatal("Only <innerModel>, <transform>, <translation>, <rotation> <prismaticjoint> or <joint> nodes may have childs. Line %d (%s)", parentDomNode.lineNumber(), qPrintable(parentDomNode.toElement().tagName()));

	InnerModelNode *node;
	QDomElement e;
	for (QDomNode domNode = parentDomNode.firstChild(); not domNode.isNull(); domNode = domNode.nextSibling())
	{
		e = domNode.toElement();

		if (not e.isNull())
		{
			// Check there are no unknown node attributes
			try
			{
				QStringList validAttrs;
				if (not getValidNodeAttributes().contains(e.tagName().toLower()))
				{
					qFatal("Line %d: <%s> is not a valid tag name", domNode.lineNumber(), e.tagName().toStdString().c_str());
				}
				validAttrs = getValidNodeAttributes()[e.tagName().toLower()];
				for (uint i=0; i < e.attributes().length(); i++)
				{
					if (not validAttrs.contains(e.attributes().item(i).toAttr().name()))
					{
						throw e.attributes().item(i).toAttr().name();
					}
				}
			}
			catch (QString attr)
			{
				qFatal("InnerModelReader::load(): '%s' is not a valid %s attribute. Error in line %d", attr.toStdString().c_str(), e.tagName().toStdString().c_str(), domNode.lineNumber());
			}
			catch (...)
			{
				qFatal("Error in line %d", domNode.lineNumber());
			}

			// Once we know there are no unknown attributes, try to create the 
			if (e.tagName().toLower() == "rotation")
			{
				QString ngn = e.attribute("engine", "static");
				if (ngn != "static" and ngn != "bullet") qFatal("Error in line %d: %s is not a valid physics engine.", domNode.lineNumber(), ngn.toStdString().c_str());
				InnerModelTransform *tr = model->newTransform(e.attribute("id"), e.attribute("engine", "static"), imNode, 0., 0., 0., e.attribute("rx", "0").toFloat(), e.attribute("ry", "0").toFloat(), e.attribute("rz", "0").toFloat(), e.attribute("mass", "0").toFloat());
				tr->gui_translation = false;
				imNode->addChild(tr);
				node = tr;
			}
			else if (e.tagName().toLower() == "translation")
			{
				QString ngn = e.attribute("engine", "static");
				if (ngn != "static" and ngn != "bullet") qFatal("Error in line %d: %s is not a valid physics engine.", domNode.lineNumber(), ngn.toStdString().c_str());
				InnerModelTransform *tr = model->newTransform(e.attribute("id"), e.attribute("engine", "static"), imNode, e.attribute("tx", "0").toFloat(), e.attribute("ty", "0").toFloat(), e.attribute("tz", "0").toFloat(), 0., 0., 0., e.attribute("mass", "0").toFloat());
				tr->gui_rotation = false;
				imNode->addChild(tr);
				node = tr;
			}
			else if (e.tagName().toLower() == "transform")
			{
				QString ngn = e.attribute("engine", "static");
				if (ngn != "static" and ngn != "bullet") qFatal("Error in line %d: %s is not a valid physics engine.", domNode.lineNumber(), ngn.toStdString().c_str());
				InnerModelTransform *tr = model->newTransform(e.attribute("id"), e.attribute("engine", "static"),  imNode, e.attribute("tx", "0").toFloat(), e.attribute("ty", "0").toFloat(), e.attribute("tz", "0").toFloat(), e.attribute("rx", "0").toFloat(), e.attribute("ry", "0").toFloat(), e.attribute("rz", "0").toFloat(), e.attribute("mass", "0").toFloat());
				imNode->addChild(tr);
				node = tr;
			}
			else if (e.tagName().toLower() == "joint")
			{
				InnerModelTransform * im = dynamic_cast<InnerModelTransform *>(imNode );
				InnerModelJoint *jr = model->newJoint(e.attribute("id"),im, e.attribute("lx", "0").toFloat(), e.attribute("ly", "0").toFloat(), e.attribute("lz", "0").toFloat(),
				  e.attribute("hx", "0").toFloat(), e.attribute("hy", "0").toFloat(), e.attribute("hz", "0").toFloat(),
				  e.attribute("tx", "0").toFloat(), e.attribute("ty", "0").toFloat(), e.attribute("tz", "0").toFloat(), e.attribute("rx", "0").toFloat(), e.attribute("ry", "0").toFloat(), e.attribute("rz", "0").toFloat(), e.attribute("min", "-inf").toDouble(), e.attribute("max", "inf").toDouble(), e.attribute("port", "0").toInt(),e.attribute("axis","z").toStdString(), e.attribute("home", "0").toDouble());
				imNode->addChild(jr);
				node = jr;
			}
			else if (e.tagName().toLower() == "prismaticjoint")
			{
				InnerModelTransform * im = dynamic_cast<InnerModelTransform *>(imNode );
				InnerModelPrismaticJoint *jr = model->newPrismaticJoint(e.attribute("id"),im, e.attribute("min", "-inf").toDouble(), e.attribute("max", "inf").toDouble(), e.attribute("position", "0").toDouble(), e.attribute("offset", "0").toDouble(), e.attribute("port", "0").toInt(),e.attribute("axis","z").toStdString(), e.attribute("home", "0").toDouble());
				imNode->addChild(jr);
				node = jr;
			}
			else if (e.tagName().toLower() == "differentialrobot")
			{
				InnerModelTransform * im = dynamic_cast<InnerModelTransform *>(imNode );
				InnerModelDifferentialRobot *dr = model->newDifferentialRobot(e.attribute("id"), im, e.attribute("tx", "0").toFloat(), e.attribute("ty", "0").toFloat(), e.attribute("tz", "0").toFloat(), e.attribute("rx", "0").toFloat(), e.attribute("ry", "0").toFloat(), e.attribute("rz", "0").toFloat(), e.attribute("port", "0").toInt());
				imNode->addChild(dr);
				node = dr;
			}
			else if (e.tagName().toLower() == "camera")
			{
				InnerModelCamera *cam = model->newCamera(e.attribute("id"), imNode, e.attribute("width", "0").toFloat(), e.attribute("height", "0").toFloat(), e.attribute("focal", "0").toFloat());
				imNode->addChild(cam);
				node = cam;
			}
			else if (e.tagName().toLower() == "rgbd")
			{
				InnerModelRGBD *cam = model->newRGBD(e.attribute("id"), imNode, e.attribute("width", "0").toFloat(), e.attribute("height", "0").toFloat(), e.attribute("focal", "0").toFloat(), e.attribute("port", "0").toInt(), e.attribute("ifconfig", ""));
				imNode->addChild(cam);
				node = cam;
			}
			else if (e.tagName().toLower() == "imu")
			{
				InnerModelIMU *imu = model->newIMU(e.attribute("id"), imNode, e.attribute("port", "0").toInt());
				printf("IMU: %s, port %d\n", imu->id.toStdString().c_str(), imu->port);
				imNode->addChild(imu);
				node = imu;
			}
			else if (e.tagName().toLower() == "laser")
			{
				InnerModelLaser *laser = model->newLaser(e.attribute("id"), imNode, e.attribute("port", "0").toInt(), e.attribute("min").toInt(), e.attribute("max").toInt(), e.attribute("angle").toFloat(), e.attribute("measures").toInt(), e.attribute("ifconfig"));
				printf("laser: %s, port %d\n", laser->id.toStdString().c_str(), laser->port);
				imNode->addChild(laser);
				node = laser;
			}
			else if (e.tagName().toLower() == "mesh")
			{
				int render;
				if (e.attribute("render", "normal").toLower() == "wireframe")
					render = InnerModelMesh::WireframeRendering;
				else if (e.attribute("render", "normal").toLower() == "normal")
					render = InnerModelMesh::NormalRendering;
				else qFatal("Rendering mode not valid \"render=%s\"\n", e.attribute("render", "normal").toStdString().c_str());
				QString scale = e.attribute("scale", "1");
				QStringList li = scale.split(",");
				float scalex = li[0].toFloat();
				float scaley;
				float scalez;
				if (li.size() == 2) scaley = li[1].toFloat();
				else if (li.size() == 3){ scaley = li[1].toFloat(); scalez = li[2].toFloat(); }
				else if (li.size() == 1){ scaley = scalex; scalez = scalex;}
				
				else qFatal("too many numbers in mesh definition");
				InnerModelMesh *mesh = model->newMesh(e.attribute("id"), imNode, e.attribute("file"), scalex, scaley, scalez, render, e.attribute("tx").toFloat(), e.attribute("ty").toFloat(), e.attribute("tz").toFloat(), e.attribute("rx").toFloat(), e.attribute("ry").toFloat(), e.attribute("rz").toFloat());
				imNode->addChild(mesh);
				node = mesh;
			}
			else if (e.tagName().toLower() == "pointcloud")
			{
				InnerModelPointCloud *pointcloud = model->newPointCloud(e.attribute("id"), imNode);
				imNode->addChild(pointcloud);
				node = pointcloud;
			}
			else if (e.tagName().toLower() == "plane")
			{
				QString size = e.attribute("size", "2500");
				QStringList li = size.split(",");
				float width = li[0].toFloat();
				float height;
				float depth;
				if (li.size() == 2) height = li[1].toFloat();
				else if (li.size() == 3){ height = li[1].toFloat(); depth = li[2].toFloat(); }
				else if (li.size() == 1){ height = width; depth = 0.05;}
				else qFatal("too many numbers in plane definition");
				InnerModelPlane *plane = model->newPlane(e.attribute("id"), imNode, e.attribute("texture", ""), width, height, depth, e.attribute("repeat", "1000").toInt(), e.attribute("nx", "0").toFloat(), e.attribute("ny", "0").toFloat(), e.attribute("nz", "0").toFloat(), e.attribute("px", "0").toFloat(), e.attribute("py", "0").toFloat(), e.attribute("pz", "0").toFloat());
				imNode->addChild(plane);
				node = plane;
			}
			else if (e.tagName().toLower() == "innermodel")
			{
				qFatal("Tag <innerModel> can only be the root tag.");
				return;
			}
			else
			{
				qFatal("%s is not a valid tag name.\n", qPrintable(e.tagName()));
				return;
			}
			recursive(domNode, model, node);
		}
	}
}

QMap<QString, QStringList> InnerModelReader::getValidNodeAttributes()
{
	QStringList temporalList;
	QMap<QString, QStringList>	nodeAttributes;

	temporalList.clear();
	nodeAttributes["innermodel"] = temporalList;
	
	temporalList.clear();
	temporalList << "id" << "rx" << "ry" << "rz" << "engine" << "mass";
	nodeAttributes["rotation"] = temporalList;
	
	temporalList.clear();
	temporalList << "id" << "tx" << "ty" << "tz" << "engine" << "mass";
	nodeAttributes["translation"] = temporalList;
	
	temporalList.clear();
	temporalList << "id" << "tx" << "ty" << "tz" << "rx" << "ry" << "rz" << "engine" << "mass";
	nodeAttributes["transform"] = temporalList;
	
	temporalList.clear();
	temporalList << "id" << "lx" << "ly" << "lz" << "hx" << "hy" << "hz" <<  "tx" << "ty" << "tz" << "rx" << "ry" << "rz" << "min" << "max" << "port" << "axis" << "home";
	nodeAttributes["joint"] = temporalList;
	
	temporalList.clear();
	temporalList << "id" << "min" << "max" << "position" << "offset" << "port" << "axis" << "home";
	nodeAttributes["prismaticjoint"] = temporalList;
	
	temporalList.clear();
	temporalList << "id" << "tx" << "ty" << "tz" << "rx" << "ry" << "rz" << "port";
	nodeAttributes["differentialrobot"] = temporalList;
	
	temporalList.clear();
	temporalList << "id" << "width" << "height" << "focal" << "port" << "ifconfig";
	nodeAttributes["rgbd"] = temporalList;

	temporalList.clear();
	temporalList << "id" << "port";
	nodeAttributes["imu"] = temporalList;

	temporalList.clear();
	temporalList << "id" << "port" << "port" << "min" << "max" << "angle" << "measures" << "ifconfig";
	nodeAttributes["laser"] = temporalList;

	temporalList.clear();
	temporalList << "id" << "width" << "height" << "focal";
	nodeAttributes["camera"] = temporalList;
	
	temporalList.clear();
	temporalList << "id" << "file" << "scale" << "render" << "tx" << "ty" << "tz" << "rx" << "ry" << "rz";
	nodeAttributes["mesh"] = temporalList;

	temporalList.clear();
	temporalList << "id";
	nodeAttributes["pointcloud"] = temporalList;
	
	temporalList.clear();
	temporalList << "id" << "texture" << "repeat" << "size" << "nx" << "ny" << "nz" << "px" << "py" << "pz";
	nodeAttributes["plane"] = temporalList;
	
	return nodeAttributes;
}

