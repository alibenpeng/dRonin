TEMPLATE = lib
TARGET = Alibenpeng
include(../../gcsplugin.pri)
include(../../plugins/uavobjects/uavobjects.pri)
include(../../plugins/coreplugin/coreplugin.pri)

OTHER_FILES += Alibenpeng.pluginspec

HEADERS += \
    alibenpengplugin.h \
    myfirstfc.h

SOURCES += \
    alibenpengplugin.cpp \
    myfirstfc.cpp

RESOURCES += \
    alibenpeng.qrc
