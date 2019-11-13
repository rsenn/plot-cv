TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += link_pkgconfig

QMAKE_CXXFLAGS -= -Wall
QMAKE_CXXFLAGS_WARN_ON -= -Wall
QMAKE_CXXFLAGS_WARN_ON += -Wno-unused-variable  -Wno-unused-parameter

SOURCES += \
    src/CannyWebcam1.cpp
INCLUDEPATH +=  /usr/local/include \
                /usr/include \
                 /opt/opencv4/include/ \
                 /opt/opencv4/include/opencv4



LIBS += -L/opt/opencv4/lib -lopencv_core \
        -lopencv_highgui \
        -lopencv_videoio \
        -lopencv_imgproc \
        -lopencv_imgcodecs \
