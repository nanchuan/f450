#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QPainter>

#include "win_qextserialport.h"

#include "glwidget.h"

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

private slots:
//    void on_pushButton_clicked();

//    void on_horizontalSlider_sliderMoved(int position);

//    void on_horizontalSlider_sliderPressed();

//    void on_horizontalSlider_sliderReleased();

//    void on_horizontalSlider_actionTriggered(int action);

//    void on_horizontalSlider_valueChanged(int value);

    void readMyCom();

    void on_pushButton_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_2_clicked();

    void on_verticalSlider_2_valueChanged(int value);

    void on_horizontalSlider_valueChanged(int value);

    void on_pushButton_5_clicked();

    void on_verticalSlider_valueChanged(int value);

    void on_pushButton_6_clicked();

//    void on_radioButton_clicked(bool checked);

//    void on_radioButton_2_clicked(bool checked);

//    void on_radioButton_3_clicked(bool checked);

    void on_checkBox_clicked(bool checked);

    void on_checkBox_2_clicked(bool checked);

    void on_checkBox_3_clicked(bool checked);

    void on_pushButton_7_clicked();

    void on_textBrowser_textChanged();

    void on_lineEdit_2_editingFinished();

    void on_lineEdit_3_editingFinished();

private:
    Ui::Dialog *ui;

    Win_QextSerialPort *myCom;

    void paintEvent(QPaintEvent *);

    QPixmap pix;
    QPoint lastPoint,lastPoint2,lastPoint3;
    QPoint endPoint,endPoint2,endPoint3;
    bool point1,point_bool_2,point_bool_3;

    int tim/*,tim_a*/;
    int scdat,max_int,chang_h,scdat_2,scdat_3;
//    float max_temp;
    bool cl,cl2,cl3;
//    QString rx;
    long tmp,tmp2;
//    UINT8 * rxbuf;

    GLWidget opgl;

signals:
    void AttitudeRead(float x,float y,float z);

};

#endif // DIALOG_H
