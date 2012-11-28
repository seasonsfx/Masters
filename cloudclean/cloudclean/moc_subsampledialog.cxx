/****************************************************************************
** Meta object code from reading C++ file 'subsampledialog.h'
**
** Created: Thu Nov 29 01:28:17 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "subsampledialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'subsampledialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SubsampleDialog[] = {

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
      25,   21,   17,   16, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SubsampleDialog[] = {
    "SubsampleDialog\0\0int\0val\0valueChanged(int)\0"
};

void SubsampleDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SubsampleDialog *_t = static_cast<SubsampleDialog *>(_o);
        switch (_id) {
        case 0: { int _r = _t->valueChanged((*reinterpret_cast< int(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SubsampleDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SubsampleDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_SubsampleDialog,
      qt_meta_data_SubsampleDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SubsampleDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SubsampleDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SubsampleDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SubsampleDialog))
        return static_cast<void*>(const_cast< SubsampleDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int SubsampleDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
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
