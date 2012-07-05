/********************************************************************************
** Form generated from reading UI file 'layerview.ui'
**
** Created: Thu Jul 5 11:02:18 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LAYERVIEW_H
#define UI_LAYERVIEW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDockWidget>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QListView>
#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LayerView
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QListView *listView;
    QHBoxLayout *horizontalLayout;
    QPushButton *delete_button;
    QPushButton *merge_button;

    void setupUi(QDockWidget *LayerView)
    {
        if (LayerView->objectName().isEmpty())
            LayerView->setObjectName(QString::fromUtf8("LayerView"));
        LayerView->resize(251, 469);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        listView = new QListView(dockWidgetContents);
        listView->setObjectName(QString::fromUtf8("listView"));

        verticalLayout_2->addWidget(listView);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        delete_button = new QPushButton(dockWidgetContents);
        delete_button->setObjectName(QString::fromUtf8("delete_button"));

        horizontalLayout->addWidget(delete_button);

        merge_button = new QPushButton(dockWidgetContents);
        merge_button->setObjectName(QString::fromUtf8("merge_button"));

        horizontalLayout->addWidget(merge_button);


        verticalLayout_2->addLayout(horizontalLayout);

        LayerView->setWidget(dockWidgetContents);

        retranslateUi(LayerView);

        QMetaObject::connectSlotsByName(LayerView);
    } // setupUi

    void retranslateUi(QDockWidget *LayerView)
    {
        LayerView->setWindowTitle(QApplication::translate("LayerView", "Layers", 0, QApplication::UnicodeUTF8));
        delete_button->setText(QApplication::translate("LayerView", "Delete", 0, QApplication::UnicodeUTF8));
        merge_button->setText(QApplication::translate("LayerView", "Merge", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class LayerView: public Ui_LayerView {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LAYERVIEW_H
