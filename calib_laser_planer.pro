QT += core
QT -= gui

CONFIG += c++11

TARGET = calib_laser_planer
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

HEADERS += \
    Get3DLinearFunc.h \
    Get2DLinearFunc.h

SOURCES += main.cpp \
    Get3DLinearFunc.cpp \
    Get2DLinearFunc.cpp

INCLUDEPATH += \
/usr/local/opencv2.4.13/include \

LIBS +=  \
/usr/local/opencv2.4.13/lib/libopencv_*.so \
