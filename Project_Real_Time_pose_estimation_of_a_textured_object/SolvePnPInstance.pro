QT += core
QT -= gui

CONFIG += c++11

TARGET = testOpencv
CONFIG += console
CONFIG -= app_bundle
INCLUDEPATH += .
INCLUDEPATH += /usr/local/include/

LIBS += -L/usr/local/lib/ -lopencv_core -lopencv_imgcodecs -lopencv_imgproc \
-lopencv_highgui -lopencv_videoio -lopencv_calib3d -lopencv_video -lopencv_features2d \
-lopencv_flann -lopencv_objdetect


TEMPLATE = app

SOURCES += \
main_detection.cpp \
#        main_registration.cpp \
        CsvReader.cpp \
        CsvWriter.cpp \
        ModelRegistration.cpp \
        Mesh.cpp \
        Model.cpp \
        PnPProblem.cpp\
        Utils.cpp\
        RobustMatcher.cpp\
#    testvideo.cpp

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
        CsvReader.h \
        CsvWriter.h \
        ModelRegistration.h \
        Mesh.h \
        Model.h \
        PnPProblem.h\
        Utils.h\
        RobustMatcher.h\
