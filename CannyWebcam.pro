TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += link_pkgconfig

QMAKE_CXXFLAGS -= -Wall
QMAKE_CXXFLAGS_WARN_ON -= -Wall
QMAKE_CXXFLAGS_WARN_ON += -Wno-unused-variable  -Wno-unused-parameter

SOURCES += src/color.cpp src/data.cpp src/geometry.cpp src/js.cpp src/jsbindings.cpp src/line.cpp src/matrix.cpp src/plot-cv.cpp src/polygon.cpp
HEADERS += src/color.h src/data.h src/geometry.h src/js.h src/jsbindings.h src/line.h src/matrix.h src/plot-cv.h src/polygon.h src/psimpl.h src/simple_svg_1.0.0.hpp src/simple_svg_writer.h

INCLUDEPATH +=  /usr/local/include \
                /usr/include \
                 /opt/opencv4/include/ \
                 /opt/opencv4/include/opencv4



LIBS += -L/opt/opencv4/lib -lopencv_core \
        -lopencv_highgui \
        -lopencv_videoio \
        -lopencv_imgproc \
        -lopencv_imgcodecs \
