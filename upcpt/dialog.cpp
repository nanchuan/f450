#include "dialog.h"
#include "ui_dialog.h"


Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);

    setWindowTitle("串口示波器 —— By Luyuexin !");

    myCom = new Win_QextSerialPort("COM4",QextSerialBase::EventDriven);
//    myCom->open(QIODevice::ReadWrite);
    myCom->setBaudRate(BAUD115200);
    myCom->setDataBits(DATA_8);
    myCom->setParity(PAR_NONE);
    myCom->setStopBits(STOP_1);
    myCom->setFlowControl(FLOW_OFF);
//    myCom->setTimeout(500);
//    myCom->close();
    connect(myCom,SIGNAL(readyRead()),this,SLOT(readMyCom()));

    ui->pushButton_3->setEnabled(false);

    pix = QPixmap(600,200);
    pix.fill(Qt::black);

    tim = 0;/*tim_a=1;*/
    cl = true;cl2 = false;cl3 = false;
    chang_h = 0;

//    ui->textBrowser->backward();

    ui->horizontalSlider->setMinimum(1);
    ui->horizontalSlider->setMaximum(10);
    ui->horizontalSlider->setValue(5);
//    ui->lcdNumber->display(5);
//    ui->lcdNumber->display(tim_a);
    ui->verticalSlider_2->setMinimum(10);
    ui->verticalSlider_2->setMaximum(32768);
    ui->verticalSlider_2->setValue(500);
//    ui->verticalSlider->setMinimum(1);
//    ui->verticalSlider->setMaximum(10);
//    update();
    ui->verticalSlider->setMinimum(-32768);
    ui->verticalSlider->setMaximum(32768);
    ui->verticalSlider->setValue(0);
    ui->lineEdit_2->setText("0");

//    ui->radioButton->setChecked(true);
    ui->checkBox->setChecked(true);
    point1 = true;
//    ui->radioButton_2->setChecked(false);
    ui->checkBox_2->setChecked(false);
    point_bool_2 = false;
//    ui->radioButton_3->setChecked(false);
    ui->checkBox_3->setChecked(false);
    point_bool_3 = false;

    tmp = 0;tmp2 = 0;

    connect(this,SIGNAL(AttitudeRead(float,float,float)),
            &opgl,SLOT(AttitudeGet(float,float,float)));

}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::readMyCom()
{
    QByteArray temp = myCom->readAll();
    ui->textBrowser->insertPlainText(temp.toHex()+' '+'/'+'\n');


//    rxbuf = temp.toHex();
    if(temp[0].operator ==(0x88))
    {
        scdat = (temp[26]*256) | (temp[27]&(0xff));
        ui->label_2->setText(QString::number(scdat));
        lastPoint = endPoint;

        scdat_2 = (temp[28]*256) | (temp[29]&(0xff));//
        ui->label_6->setText(QString::number(scdat_2));
        lastPoint2 = endPoint2;

        scdat_3 = (temp[30]*256) | (temp[31]&(0xff));
        ui->label_8->setText(QString::number(scdat_3));
        lastPoint3 = endPoint3;

//        tmp += scdat;
//        tmp2++;
//        ui->lineEdit->setText(QString::number(tmp/tmp2));

        ui->lcdNumber_2->display((temp[9]*256) | (temp[10]&(0xff)));
        ui->lcdNumber_3->display((temp[11]*256) | (temp[12]&(0xff)));
        ui->lcdNumber_4->display((temp[13]*256) | (temp[14]&(0xff)));
        ui->lcdNumber_5->display((temp[15]*256) | (temp[16]&(0xff)));

        ui->lcdNumber_6->display((temp[17]&(0xff)));
        ui->lcdNumber_7->display((temp[18]&(0xff)));
        ui->lcdNumber_8->display((temp[19]&(0xff)));
        ui->lcdNumber_9->display((temp[20]&(0xff)));

//        ui->lcdNumber_10->display((temp[24]*256) | (temp[25]&(0xff)));
//        ui->lcdNumber_11->display((temp[26]*256) | (temp[27]&(0xff)));
//        ui->lcdNumber_12->display((temp[28]*256) | (temp[29]&(0xff)));
//        ui->lcdNumber_13->display((temp[30]*256) | (temp[31]&(0xff)));


        emit AttitudeRead(((temp[1]*256) | (temp[2]&(0xff)))/10.0,
                ((temp[3]*256) | (temp[4]&(0xff)))/10.0,
                ((temp[5]*256) | (temp[6]&(0xff)))/10.0);


        tim += ui->horizontalSlider->value();
        if(tim>600)
        {
            cl = true;
            cl2 = true;
        }
        else
        {
            endPoint.setX(tim);
            endPoint.setY(100- (scdat- chang_h)/(max_int/100.0));

            endPoint2.setX(tim);
            endPoint2.setY(100- (scdat_2- chang_h)/(max_int/100.0));

            endPoint3.setX(tim);
            endPoint3.setY(100- (scdat_3- chang_h)/(max_int/100.0));

            cl3 = true;
            update();
        }
    }

    if(temp[0].operator ==(0x84))
    {
        ui->lcdNumber_14->display((temp[1]&(0xff)));
        ui->lcdNumber_15->display((temp[2]&(0xff)));
        ui->lcdNumber_16->display((temp[3]&(0xff)));

        ui->lcdNumber_18->display((temp[4]&(0xff)));
        ui->lcdNumber_19->display((temp[5]&(0xff)));
        ui->lcdNumber_20->display((temp[6]&(0xff)));

        ui->lcdNumber_22->display((temp[7]&(0xff)));
        ui->lcdNumber_23->display((temp[8]&(0xff)));
        ui->lcdNumber_24->display((temp[9]&(0xff)));
    }
}


void Dialog::paintEvent(QPaintEvent *)
{
    QPainter pt(&pix);

    if(cl)
    {
        cl = false;

        pix.fill(Qt::black);
        lastPoint.setX(0);
        lastPoint2.setX(0);
        lastPoint3.setX(0);

        tim = 0;
        endPoint.setX(0);
        endPoint2.setX(0);
        endPoint3.setX(0);

        pt.setPen(Qt::darkBlue);

        pt.drawLine(0,50,600,50);
        pt.drawLine(0,150,600,150);
        pt.drawLine(75,0,75,200);
        pt.drawLine(150,0,150,200);
        pt.drawLine(225,0,225,200);
        pt.drawLine(375,0,375,200);
        pt.drawLine(450,0,450,200);
        pt.drawLine(525,0,525,200);

        pt.setPen(Qt::blue);
        pt.drawLine(0,100,600,100);
        pt.drawLine(300,0,300,200);
    }

    if(cl2)
    {
        cl2 = false;

        pt.setPen(Qt::black);
        pt.setBrush(Qt::black);
        pt.drawRect(0,0,35,9);
        pt.drawRect(0,90,35,9);
        pt.drawRect(0,40,35,9);
        pt.drawRect(0,140,35,9);
        pt.drawRect(0,190,35,9);
//        pt.drawText(0,100,"      ");
//        pt.drawText(0,10,"      ");
//        pt.drawText(0,50,"      ");
//        pt.drawText(0,150,"      ");
//        pt.drawText(0,200,"      ");

//        pt.setPen(Qt::blue);
//        pt.drawText(0,100,QString::number((int)(chang_h*max_int/100.0)));
//        pt.drawText(0,10,QString::number((int)(max_int*(1.0+ chang_h/100.0))));
//        pt.drawText(0,50,QString::number((int)(max_int*(0.5+ chang_h/100.0))));
//        pt.drawText(0,150,QString::number((int)(max_int*(-0.5+ chang_h/100.0))));
//        pt.drawText(0,200,QString::number((int)(max_int*(-1.0+ chang_h/100.0))));
        pt.setPen(Qt::blue);
        pt.drawText(0,100,QString::number((int)(+chang_h)));
        pt.drawText(0,10,QString::number((int)(max_int+ chang_h)));
        pt.drawText(0,50,QString::number((int)(max_int/2.0+ chang_h)));
        pt.drawText(0,150,QString::number((int)(-max_int/2.0+ chang_h)));
        pt.drawText(0,200,QString::number((int)(-max_int+ chang_h)));
    }

    if(cl3)
    {
        cl3 = false;
        if(point1)
        {
            pt.setPen(Qt::white);
            pt.drawLine(lastPoint,endPoint);
        }
        if(point_bool_2)
        {
            pt.setPen(Qt::red);
            pt.drawLine(lastPoint2,endPoint2);
        }
        if(point_bool_3)
        {
            pt.setPen(Qt::yellow);
            pt.drawLine(lastPoint3,endPoint3);
        }
    }


    QPainter painter(this);

    painter.drawPixmap(10,10,pix);
}





void Dialog::on_pushButton_clicked()
{
    myCom->open(QIODevice::ReadWrite);
    ui->pushButton->setEnabled(false);
    ui->pushButton_3->setEnabled(true);

    cl = true;
    cl2 = true;
}

void Dialog::on_pushButton_3_clicked()
{
    myCom->close();
    ui->pushButton_3->setEnabled(false);
    ui->pushButton->setEnabled(true);
}

void Dialog::on_pushButton_4_clicked()
{
    myCom->write(ui->lineEdit->text().toLocal8Bit());
}

void Dialog::on_pushButton_2_clicked()
{
    ui->textBrowser->clear();
    cl = true;
    cl2 = true;

    endPoint.setX(tim);
    endPoint2.setX(tim);
    endPoint3.setX(tim);
}

void Dialog::on_verticalSlider_2_valueChanged(int value)
{
    max_int = value;
    ui->lineEdit_3->setText(QString::number(value));

    cl2 = true;
    update();
}

void Dialog::on_horizontalSlider_valueChanged(int value)
{
    ui->lcdNumber->display(value);
}

void Dialog::on_pushButton_5_clicked()
{
    ui->verticalSlider->setValue(ui->lineEdit_2->text().toInt());
}

void Dialog::on_verticalSlider_valueChanged(int value)
{
    chang_h = value;
    ui->lineEdit_2->setText(QString::number(value));

    cl2 = true;
    update();
}

void Dialog::on_pushButton_6_clicked()
{
    ui->verticalSlider_2->setValue(ui->lineEdit_3->text().toInt());
}

//void Dialog::on_radioButton_clicked(bool checked)
//{
//    point1 = checked;
//}

//void Dialog::on_radioButton_2_clicked(bool checked)
//{
//    point_bool_2 = checked;
//}

//void Dialog::on_radioButton_3_clicked(bool checked)
//{
//    point_bool_3 = checked;
//}

void Dialog::on_checkBox_clicked(bool checked)
{
    point1 = checked;
}

void Dialog::on_checkBox_2_clicked(bool checked)
{
    point_bool_2 = checked;
}

void Dialog::on_checkBox_3_clicked(bool checked)
{
    point_bool_3 = checked;
}

void Dialog::on_pushButton_7_clicked()
{
    opgl.show();
    opgl.setGeometry(30,100,640,480);
//    ui->pushButton_7->setEnabled(false);
}

void Dialog::on_textBrowser_textChanged()
{
    ui->textBrowser->moveCursor(QTextCursor::End);
}

void Dialog::on_lineEdit_2_editingFinished()
{
    ui->verticalSlider->setValue(ui->lineEdit_2->text().toInt());
}

void Dialog::on_lineEdit_3_editingFinished()
{
    ui->verticalSlider_2->setValue(ui->lineEdit_3->text().toInt());
}
