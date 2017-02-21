#-------------------------------------------------
#
# Project created by QtCreator 2017-02-21T02:19:22
#
#-------------------------------------------------

QT       += testlib

QT       -= gui

TARGET = tst_testsdi12test
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += tst_testsdi12test.cpp \
    ../sdi12.cpp
DEFINES += SRCDIR=\\\"$$PWD/\\\"

INCLUDEPATH += ../Timbre

HEADERS += \
    usart.h
