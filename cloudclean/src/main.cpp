#include "app.h"
#include <ctime>
#include <cstdlib>
#include <QCoreApplication>
#include <QtDebug>
#include <QtGlobal>

#ifdef WIN32
    #include <Windows.h>
#endif

void customMessageHandler(QtMsgType type, const QMessageLogContext& context, const QString &msg)
{
    QString txt;
    /*
    switch (type) {
    case QtDebugMsg:
        txt = QString("Debug: %1").arg(msg);
        break;
    case QtWarningMsg:
        txt = QString("Warning: %1").arg(msg);
    break;
    case QtCriticalMsg:
        txt = QString("Critical: %1").arg(msg);
    break;
    case QtFatalMsg:
        txt = QString("Fatal: %1").arg(msg);
        abort();
    }
    */

    txt = QString("MESSAGE (%1:%2 %3): %4\n").arg(context.file).arg(context.line).arg(context.function).arg(msg);

    QFile outFile("debuglog.txt");
    outFile.open(QIODevice::WriteOnly | QIODevice::Append);
    QTextStream ts(&outFile);
    ts << txt << endl;

#ifdef WIN32
    OutputDebugString(txt.toLocal8Bit().constData());
#else
    //printf("MESSAGE (%s:%u %s): %s\n", context.file, context.line, context.function, msg.constData());
    printf("%s", txt.toLocal8Bit().constData());
    fflush(stdout);
#endif


}

int main(int argc, char* argv[])
{
    srand (time(NULL));
    qInstallMessageHandler(customMessageHandler);
    App app(argc,argv);
    return app.exec();
}

