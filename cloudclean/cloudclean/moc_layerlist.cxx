/****************************************************************************
** Meta object code from reading C++ file 'layerlist.h'
**
** Created: Thu Nov 29 16:54:06 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "layerlist.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'layerlist.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_LayerList[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      16,   11,   10,   10, 0x05,
      70,   68,   10,   10, 0x05,
      87,   10,   10,   10, 0x05,

 // slots: signature, parameters, type, tag, flags
     108,  100,   10,   10, 0x0a,
     153,  139,   10,   10, 0x0a,
     189,  183,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_LayerList[] = {
    "LayerList\0\0mode\0"
    "selectModeChanged(QAbstractItemView::SelectionMode)\0"
    "i\0selectLayer(int)\0updateView()\0indices\0"
    "deleteLayers(std::vector<int>)\0"
    "layersToMerge\0mergeLayers(std::vector<int>)\0"
    "index\0selectModeChanged(int)\0"
};

void LayerList::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        LayerList *_t = static_cast<LayerList *>(_o);
        switch (_id) {
        case 0: _t->selectModeChanged((*reinterpret_cast< QAbstractItemView::SelectionMode(*)>(_a[1]))); break;
        case 1: _t->selectLayer((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->updateView(); break;
        case 3: _t->deleteLayers((*reinterpret_cast< std::vector<int>(*)>(_a[1]))); break;
        case 4: _t->mergeLayers((*reinterpret_cast< std::vector<int>(*)>(_a[1]))); break;
        case 5: _t->selectModeChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData LayerList::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject LayerList::staticMetaObject = {
    { &QAbstractListModel::staticMetaObject, qt_meta_stringdata_LayerList,
      qt_meta_data_LayerList, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &LayerList::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *LayerList::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *LayerList::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_LayerList))
        return static_cast<void*>(const_cast< LayerList*>(this));
    return QAbstractListModel::qt_metacast(_clname);
}

int LayerList::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QAbstractListModel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void LayerList::selectModeChanged(QAbstractItemView::SelectionMode _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void LayerList::selectLayer(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void LayerList::updateView()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}
QT_END_MOC_NAMESPACE
