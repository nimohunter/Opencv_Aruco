QT += core
QT -= gui

CONFIG += c++11

TARGET = ReStructuion
CONFIG += console
CONFIG -= app_bundle
INCLUDEPATH += .
INCLUDEPATH += /usr/local/include/

LIBS += -L/usr/local/lib/ -lopencv_core -lopencv_imgcodecs -lopencv_imgproc -lopencv_highgui -lopencv_aruco -lopencv_videoio -lopencv_calib3d -lopencv_video

TEMPLATE = app

SOURCES += main.cpp \
    NimoPoseDetect/camerastaticdata.cpp \
    NimoPoseDetect/objectstaticdata.cpp \
    NimoPoseDetect/utils.cpp \
    NimoPoseDetect/kalman.cpp \
    NimoCMultiThread/CThread.cpp \
    NimoCMultiThread/COperatingSystem.cpp \
    NimoCMultiThread/COperatingSystemFactory.cpp \
    NimoCMultiThread/CLinuxOperatingSystem.cpp  \
    NimoCMultiThread/CCountingSem.cpp \
    NimoCMultiThread/CLinuxCountingSem.cpp \
    NimoCMultiThread/CMutex.cpp \
    NimoCMultiThread/CLinuxMutex.cpp \
    NimoCMultiThread/CMsgQueue.cpp \
    NimoCMultiThread/CLinuxMsgQueue.cpp \
    NimoCMultiThread/PnPThread.cpp \
    NimoCMultiThread/SocketThread.cpp \
    NimoCMultiThread/transdata.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    NimoPoseDetect/camerastaticdata.h \
    NimoPoseDetect/objectstaticdata.h \
    NimoPoseDetect/utils.h \
    NimoPoseDetect/kalman.h \
    NimoCMultiThread/CThread.h \
    NimoCMultiThread/COperatingSystem.h \
    NimoCMultiThread/COperatingSystemFactory.h \
    NimoCMultiThread/CLinuxOperatingSystem.h  \
    NimoCMultiThread/CCountingSem.h \
    NimoCMultiThread/CLinuxCountingSem.h \
    NimoCMultiThread/CMutex.h \
    NimoCMultiThread/CLinuxMutex.h \
    NimoCMultiThread/CMsgQueue.h \
    NimoCMultiThread/CLinuxMsgQueue.h \
    NimoCMultiThread/PnPThread.h \
    NimoCMultiThread/SocketThread.h \
    NimoCMultiThread/transdata.h
