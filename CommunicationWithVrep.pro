QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
# --------- config opencv --------
INCLUDEPATH += D:/Soft/Opencv4/opencv/build/include
CONFIG(debug, debug|release) {
LIBS += D:/Soft/Opencv4/opencv/build/x64/vc14/lib/opencv_world410d.lib
}
else {
LIBS += D:/Soft/Opencv4/opencv/build/x64/vc14/lib/opencv_world410.lib
}

INCLUDEPATH += 'D:/Documents/WorkSpace/QT/VREP/CommunicationWithVrep/3rdParty/programming/common'
DEPENDPATH += 'D:/Documents/WorkSpace/QT/VREP/CommunicationWithVrep/3rdParty/programming/common'

INCLUDEPATH += 'D:/Documents/WorkSpace/QT/VREP/CommunicationWithVrep/3rdParty/programming/include'
DEPENDPATH += 'D:/Documents/WorkSpace/QT/VREP/CommunicationWithVrep/3rdParty/programming/include'

INCLUDEPATH += 'D:/Documents/WorkSpace/QT/VREP/CommunicationWithVrep/3rdParty/programming/remoteApi'
DEPENDPATH += 'D:/Documents/WorkSpace/QT/VREP/CommunicationWithVrep/3rdParty/programming/remoteApi'


SOURCES += \
        main.cpp \
    GetVrepDataImpl.cpp \
    Vrep.cpp \
    test_func.cpp \
    VrepInterface.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/3rdParty/release2/ -lremoteApi
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/3rdParty/release2/ -lremoteApid

INCLUDEPATH += $$PWD/3rdParty/release2
DEPENDPATH += $$PWD/3rdParty/release2

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/3rdParty/release2/libremoteApi.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/3rdParty/release2/libremoteApid.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/3rdParty/release2/remoteApi.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/3rdParty/release2/remoteApid.lib

HEADERS += \
    GetVrepDataImpl.h \
    TypeDef.h \
    UseOpencv.h \
    Vrep.h \
    VrepConfig.h \
    VrepInterface.h \
    testfunc.h

win32:CONFIG(release, debug|release): LIBS += -L'C:/Program Files/V-REP3/V-REP_PRO_EDU/programming/remoteApiBindings/lib/x64/release/' -lremoteApiSharedLib-64
else:win32:CONFIG(debug, debug|release): LIBS += -L'C:/Program Files/V-REP3/V-REP_PRO_EDU/programming/remoteApiBindings/lib/x64/debug/' -lremoteApiSharedLib-64

INCLUDEPATH += 'C:/Program Files/V-REP3/V-REP_PRO_EDU/programming/remoteApiBindings/lib/x64/Release'
DEPENDPATH += 'C:/Program Files/V-REP3/V-REP_PRO_EDU/programming/remoteApiBindings/lib/x64/Release'

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += 'C:/Program Files/V-REP3/V-REP_PRO_EDU/programming/remoteApiBindings/lib/x64/release/libremoteApiSharedLib-64.a'
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += 'C:/Program Files/V-REP3/V-REP_PRO_EDU/programming/remoteApiBindings/lib/x64/debug/libremoteApiSharedLib-64.a'
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += 'C:/Program Files/V-REP3/V-REP_PRO_EDU/programming/remoteApiBindings/lib/x64/release/remoteApiSharedLib-64.lib'
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += 'C:/Program Files/V-REP3/V-REP_PRO_EDU/programming/remoteApiBindings/lib/x64/debug/remoteApiSharedLib-64.lib'
