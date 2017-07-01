#include "glwidget.h"
#include <GL/glu.h>
#include <QKeyEvent>

GLWidget::GLWidget(QWidget* parent)
    : QGLWidget(parent)
{

    setWindowTitle("3Dshow");

//    setGeometry(100,100,640,480);

    xRot = yRot = zRot = 0;
    xRot_c = 15;
    yRot_c = zRot_c = 0;
    zoom = -5.0;

//    startTimer(5);
}

GLWidget::~GLWidget()
{}

//void GLWidget::timerEvent(QTimerEvent *)
//{
//    xRot += xRot_v;
//    yRot += yRot_v;
//    zRot += zRot_v;

//    updateGL();
//}

void GLWidget::initializeGL()
{
    glShadeModel(GL_SMOOTH);

    glClearColor(0.0,0.0,0.0,0.0);

    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glHint(GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST);
}

//*********************************************************paintGL()
void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    glTranslatef(0,0,zoom);
//    glRotatef(rQuad,1,1,0);

    glRotatef(yRot_c,0,1,0);glRotatef(yRot,0,1,0);
    glRotatef(xRot_c,1,0,0);glRotatef(xRot,1,0,0);
    glRotatef(zRot_c,0,0,1);glRotatef(zRot,0,0,1);



//    glBegin(GL_TRIANGLES);
//    glColor3f(1.0,0.0,0.0);
//    glVertex3f(0,0,0);
//    glColor3f(0.0,1.0,0.0);
//    glVertex3f(0,1,0);
//    glColor3f(0.0,0.0,1.0);
//    glVertex3f(1,0,0);
//    glEnd();


    glBegin(GL_QUADS);

    glColor3f(0,1,0);
    glVertex3f(1,0.25,-1);
    glVertex3f(1,0.25,1);
    glVertex3f(-1,0.25,1);
    glVertex3f(-1,0.25,-1);

    glColor3f(0,0,1);
    glVertex3f(1,0.25,1);
    glVertex3f(1,0.25,-1);
    glVertex3f(1,-0.25,-1);
    glVertex3f(1,-0.25,1);

    glColor3f(1,0,0);
    glVertex3f(1,-0.25,-1);
    glVertex3f(1,-0.25,1);
    glVertex3f(-1,-0.25,1);
    glVertex3f(-1,-0.25,-1);

    glColor3f(1,0,1);
    glVertex3f(-1,0.25,1);
    glVertex3f(-1,0.25,-1);
    glVertex3f(-1,-0.25,-1);
    glVertex3f(-1,-0.25,1);

    glColor3f(1,1,0);
    glVertex3f(-1,0.25,1);
    glVertex3f(1,0.25,1);
    glVertex3f(1,-0.25,1);
    glVertex3f(-1,-0.25,1);

    glColor3f(0,1,1);
    glVertex3f(-1,0.25,-1);
    glVertex3f(1,0.25,-1);
    glVertex3f(1,-0.25,-1);
    glVertex3f(-1,-0.25,-1);

    glEnd();

//    renderText(20,100,"Hello");
    glColor3f(1,1,1);
    renderText(10,10,QString::number(xRot));
    renderText(10,20,QString::number(zRot));
    renderText(10,30,QString::number(yRot));
}

void GLWidget::resizeGL(int w, int h)
{
    if(h == 0)
        h = 1;

    glViewport(0,0,(GLint)w,(GLint)h);

    glMatrixMode(GL_PROJECTION);

    glLoadIdentity();

    gluPerspective(45.0,(GLfloat)w/(GLfloat)h,0.1,100.0);

    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();

//    GLfloat zNear = 0.1;
//    GLfloat zFar = 100.0;
//    GLfloat aspect = (GLfloat)width/(GLfloat)height;
//    GLfloat fH = tan(GLfloat(90.0/360.0*3.14159))*zNear;
//    GLfloat fW = fH * aspect;
//    glFrustum(-fW, fW, -fH, fH, zNear, zFar);
}


void GLWidget::keyPressEvent(QKeyEvent *e)
{
    switch(e->key())
    {
    case Qt::Key_Escape:
        close();
        break;
    case Qt::Key_Up:
        xRot_c -= 1;
        updateGL();
        break;
    case Qt::Key_Down:
        xRot_c += 1;
        updateGL();
        break;
    case Qt::Key_Right:
        yRot_c += 1;
        updateGL();
        break;
    case Qt::Key_Left:
        yRot_c -= 1;
        updateGL();
        break;
    }
}


void GLWidget::AttitudeGet(float x, float y, float z)
{
    xRot = x;
    yRot = z;
    zRot = -y;
    updateGL();
}













