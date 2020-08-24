QT += core
QT -= gui

CONFIG += c++11
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = app
TARGET = batest

SOURCES += main.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

INCLUDEPATH += ../batotp

CONFIG( debug, debug|release ) {
  unix{
    DESTDIR = ../Debug
    LIBS += -L../Debug
    PRE_TARGETDEPS += ../Debug/libbatotp.a
  }
  windows{
    DESTDIR = ../x64/Debug
    LIBS += -L../x64/Debug
    PRE_TARGETDEPS += ../x64/Debug/batotp.lib
  }
} else {
  unix{
    DESTDIR = ../Release
    LIBS += -L../Release
    PRE_TARGETDEPS += ../Release/libbatotp.a
  }
  windows{
    DESTDIR = ../x64/Release
    LIBS += -L../x64/Release
    PRE_TARGETDEPS += ../x64/Release/batotp.lib
  }
}
LIBS += -lbatotp
