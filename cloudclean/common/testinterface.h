#ifndef TESTINTERFACE_H
#define TESTINTERFACE_H

#include <QtPlugin>

class TestPluginInterface
{
public:
    TestPluginInterface();
    virtual ~TestPluginInterface(){}
};

Q_DECLARE_INTERFACE(TestPluginInterface, "za.co.circlingthesun.cloudclean.testplugininterface/1.0")

#endif // TESTINTERFACE_H
