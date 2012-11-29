/****************************************************************************
** Meta object code from reading C++ file 'layerview.h'
**
** Created: Fri Nov 30 01:21:51 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "layerview.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'layerview.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_LayerView[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      11,   10,   10,   10, 0x05,

 // slots: signature, parameters, type, tag, flags
      29,   24,   10,   10, 0x0a,
      88,   80,   10,   10, 0x0a,
     138,  136,   10,   10, 0x0a,
     155,   10,   10,   10, 0x0a,
     170,   10,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_LayerView[] = {
    "LayerView\0\0updateView()\0mode\0"
    "setSelectionMode(QAbstractItemView::SelectionMode)\0"
    "sel,des\0selectionChanged(QItemSelection,QItemSelection)\0"
    "i\0selectLayer(int)\0deleteLayers()\0"
    "mergeLayers()\0"
};

void LayerView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        LayerView *_t = static_cast<LayerView *>(_o);
        switch (_id) {
        case 0: _t->updateView(); break;
        case 1: _t->setSelectionMode((*reinterpret_cast< QAbstractItemView::SelectionMode(*)>(_a[1]))); break;
        case 2: _t->selectionChanged((*reinterpret_cast< const QItemSelection(*)>(_a[1])),(*reinterpret_cast< const QItemSelection(*)>(_a[2]))); break;
        case 3: _t->selectLayer((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->deleteLayers(); break;
        case 5: _t->mergeLayers(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData LayerView::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject LayerView::staticMetaObject = {
    { &QDockWidget::staticMetaObject, qt_meta_stringdata_LayerView,
      qt_meta_data_LayerView, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &LayerView::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *LayerView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *LayerView::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_LayerView))
        return static_cast<void*>(const_cast< LayerView*>(this));
    return QDockWidget::qt_metacast(_clname);
}

int LayerView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDockWidget::qt_metacall(_c, _id, _a);
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
void LayerView::updateView()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE
