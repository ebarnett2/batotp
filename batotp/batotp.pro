QT += core
QT -= gui

TARGET = batotp

CONFIG += c++11
CONFIG += staticlib

CONFIG += console
CONFIG -= app_bundle

CONFIG( debug, debug|release ) {
  unix{
    DESTDIR = ../Debug
  }
  windows{
    DESTDIR = ../x64/Debug
  }
  OBJECTS_DIR = obj/Debug
} else {
  unix{
    DESTDIR = ../Release
  }
  windows{
    DESTDIR = ../x64/Release
  }
  OBJECTS_DIR = obj/Release
}

INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += ../../eigenlib

TEMPLATE = lib

SOURCES += spline.cpp \
    robot.cpp \
    ba.cpp \
    util.cpp

HEADERS += \
    config.h \
    spline.h \
    robot.h \
    ba.h \
    util.h

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
