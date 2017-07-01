#-------------------------------------------------
#
# Project created by QtCreator 2014-03-07T18:12:10
#
#-------------------------------------------------

QT       += core gui

QT += opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = upcpt
TEMPLATE = app


SOURCES += main.cpp\
        dialog.cpp \
    qextserialbase.cpp \
    win_qextserialport.cpp \
    glwidget.cpp

HEADERS  += dialog.h \
    qextserialbase.h \
    win_qextserialport.h \
    glwidget.h

FORMS    += dialog.ui
