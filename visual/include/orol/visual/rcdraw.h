#ifndef RCDRAW_H
#define RCDRAW_H

#include <QtGui>
#include <stdint.h>

#include <qmat/QMatAll>
#include <QGLWidget>


/**
	@author authorname <authormail>

  The constructor will automatically call show() method.

*/
class RCDraw : public QGLWidget
{
Q_OBJECT
public:
	RCDraw(int _width, int _height, uchar *img=NULL, QWidget *parent = 0);
	RCDraw(int _width, int _height, uchar *img=NULL, QImage::Format format=QImage::Format_Indexed8, QWidget *parent = 0);
	RCDraw(int _width, int _height, QImage *img=NULL, QWidget *parent = 0);
	RCDraw(int _width, int _height, QWidget *parent = 0);
	RCDraw(QImage *img, QWidget *parent = 0);
	RCDraw(QWidget *parent = 0);
	RCDraw(const QRect & win_, QWidget *parent = 0);
	~RCDraw();
	void init();
	void setImage(QImage *img);
	void paintEvent(QPaintEvent *);
	void setWindow(const QRect & win_) { effWin = win = win_; }
	void drawSquare(const QRect &, const QColor &,  bool fill=false, int id= -1, float rads=0, float width=0);
	void drawSquare(const QPoint &, int sideX, int sideY, const QColor &,  bool fill=false , int id= -1, float rads=0, float width=0);
	void drawSquare(const QPointF &, float sideX, float sideY, const QColor &,  bool fill=false , int id= -1, float rads=0, float width=0);
	void drawLine(const QLine &line, const QColor & c, float width=0);
	void drawLine(const QLineF &line, const QColor & c, float width=0);
	void drawLineOnTop(const QLine &line, const QColor & c, float width=0);
	void drawLineOnTop(const QLineF &line, const QColor & c, float width=0);
	void drawLineFOnTop(const QLineF &line, const QColor & c, float width=0);
	void drawPolyLine(const QVector<QPoint> & pline, const QColor & c, int width=1);
	void drawPolyLine(const QVector<int> & xs, const QVector<int> & ys, const QColor & c, int width=1);
	void drawGrad(const QLine &line, const QColor & c, const QColor & c1, float width=0);
	void drawEllipse(const QRectF &, const QColor &, bool fill= false, int id =-1, float rads=0);
	void drawEllipse(const QRect &, const QColor &, bool fill= false, int id =-1, float rads=0);
	void drawEllipse(const QPointF &, float radiusX, float radiusY, const QColor &, bool fill=false, int id =-1, float rads=0);
	void drawEllipse(const QPoint &, int radiusX, int radiusY, const QColor &, bool fill=false, int id =-1, float rads=0);
	void draw2DRoiOnFloor(const QPoint & center, const QMat & cov, const QColor & col, bool fill, int id);
	void draw3DRoiOnFloor( const QVec & center, const QMat & cov, const QColor &, bool fill=false, int id =-1 );
	void drawAxis(const QColor &, int w);
	void drawPerimeter(const QColor &c, int width, int margin);
	void drawPerimeter(const QColor &c, int width);
	void drawCrossHair(const QColor &c);
	void drawText(const QPointF & pos, const QString & text, int size, const QColor & color, bool centered=false, const QColor bgc=QColor(127,127,127,0));
	void drawText(const QPoint& pos, const QString & text, int size, const QColor & color, bool centered=false, const QColor bgc=QColor(127,127,127,0)) { drawText(QPointF(pos), text, size, color, centered, bgc); }
	void scaleImage(float sscale) { imageScale = sscale; setFixedSize(sscale*width, sscale*height); }

	void setDrawAxis(bool f) { DRAW_AXIS = f;}
	void setDrawPerimeter(bool f) { DRAW_PERIMETER = f;}
	QRectF getWindow() { return win;}

	uint32_t getWidth() { return width; }
	uint32_t getHeight() { return height; }
	bool autoResize();
	uchar *imageBuffer() { if (qimg != NULL) return qimg->bits(); return NULL; }
	void removeImage() { if (qimg != NULL) delete qimg; qimg = NULL; }
	

protected:
	float imageScale;
	bool invertedVerticalAxis;
	struct TRect
	{
		QRectF rect;
		QColor color;
		int id;
		float ang;
		bool fill;
		float width;
	};

	struct TEllipse
	{
		QRectF rect;
		QPointF center;
		float rx, ry;
		QColor color;
		int id;
		bool fill;
		float ang;
	};

	struct TLine
	{
		QLineF line;
		QColor color;
		float width;
	};

	struct TGrad
	{
		QLine line;
		QColor color;
		QColor color1;
		float width;
	};

	struct TText
	{
		QPointF pos;
		int size;
		QColor color;
		QString text;
		float width;
		bool centered;
		QColor bgc;
	};

	int width, height;
	QRectF win;
	QRectF effWin;
	QQueue<TRect> squareQueue;
	QQueue<TLine> lineQueue;
	QQueue<TLine> lineOnTopQueue;
	QQueue<TEllipse> ellipseQueue;
	QQueue<TGrad> gradQueue;
	QQueue<TText> textQueue;

	QImage *qimg;
	QVector<QRgb> ctable; //For gray conversion
	QPoint inicio, actual;
	bool translating;
	QPointF backPos;
	bool DRAW_AXIS, DRAW_PERIMETER;

	QLinearGradient linGrad;

	QVec visibleCenter;

signals:
  	void iniMouseCoor(QPoint p);
	void endMouseCoor(QPoint p);
	void newCoor(QPointF p);
	void endCoor(QPointF p);
	void newLeftCoor(QPointF p);
	void newRightCoor(QPointF p);
	void press(QPointF p);
	void release(QPointF p);
protected:
	void wheelEvent(QWheelEvent *);
	void mousePressEvent(QMouseEvent *);
	void mouseMoveEvent(QMouseEvent *);
	void mouseReleaseEvent(QMouseEvent *);
};

#endif
