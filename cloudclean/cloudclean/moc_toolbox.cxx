/****************************************************************************
** Meta object code from reading C++ file 'toolbox.h'
**
** Created: Thu Nov 29 16:54:06 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "toolbox.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'toolbox.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Toolbox[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      16,    9,    8,    8, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Toolbox[] = {
    "Toolbox\0\0widget\0setSettingsWidget(QWidget*)\0"
};

void Toolbox::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Toolbox *_t = static_cast<Toolbox *>(_o);
        switch (_id) {
        case 0: _t->setSettingsWidget((*reinterpret_cast< QWidget*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Toolbox::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Toolbox::staticMetaObject = {
    { &QDockWidget::staticMetaObject, qt_meta_stringdata_Toolbox,
      qt_meta_data_Toolbox, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Toolbox::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Toolbox::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Toolbox::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Toolbox))
        return static_cast<void*>(const_cast< Toolbox*>(this));
    return QDockWidget::qt_metacast(_clname);
}

int Toolbox::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDockWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
