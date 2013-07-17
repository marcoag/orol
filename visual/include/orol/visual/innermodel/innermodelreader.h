#ifndef INNERMODELREADER_H
#define INNERMODELREADER_H

#include <QtXml/QtXml>

class InnerModelNode;
class InnerModelReader;
#include <orol/visual/innermodel/innermodel.h>

class InnerModelReader
{
public:
	InnerModelReader();
	~InnerModelReader();
	static bool load(const QString &file, InnerModel *model);
private:
	static void recursive(QDomNode parentDomNode, InnerModel *model, InnerModelNode *imNode);
	static QMap<QString, QStringList> getValidNodeAttributes();
};

#endif
