#ifndef TEST_H
#define TEST_H

#include <QtPlugin>
#include <QObject>
#include "interfaces/interfaces.h"

class Test : public QObject, public TestPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(TestPluginInterface)
public:
    Test();
};

#endif // TEST_H
