#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QtOpenGL/qgl.h>
//#include <QtWidgets/

class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(QWidget* parent=0);
    ~GLWidget();

    GLfloat xRot,yRot,zRot;

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width,int height);
//    void resize();

    void keyPressEvent(QKeyEvent *e);
//    void timerEvent(QTimerEvent *);

protected:
//    bool fullScreen;

//    GLfloat rQuad;
//    GLfloat xRot,yRot,zRot;
    GLfloat xRot_c,yRot_c,zRot_c;
    GLfloat zoom;
//    GLuint texture[1];

public slots:
    void AttitudeGet(float x,float y,float z);

};


#endif // GLWIDGET_H
