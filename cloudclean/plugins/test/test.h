#ifndef TEST_H
#define TEST_H

#include <QObject>
#include "testinterface.h"

class Test : public QObject, public TestPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(TestPluginInterface)
public:
    Test();
    ~Test();
};

#endif // TEST_H
