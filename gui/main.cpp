#include <QApplication>
#include <QtSerialPort/QSerialPortInfo>
#include <QStringList>

#include "picoashacomm.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    PicoAshaComm picoASHA;

    picoASHA.showUI();
    return app.exec();
}
