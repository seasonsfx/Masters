/********************************************************************************
** Form generated from reading UI file 'layerview.ui'
**
** Created: Tue Jul 17 03:04:20 2012
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
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QTableView>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LayerView
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QTableView *listView;
    QHBoxLayout *horizontalLayout;
    QPushButton *delete_button;
    QPushButton *merge_button;
    QVBoxLayout *verticalLayout;
    QLabel *selection_mode_label;
    QComboBox *selection_mode_combo;

    void setupUi(QDockWidget *LayerView)
    {
        if (LayerView->objectName().isEmpty())
            LayerView->setObjectName(QString::fromUtf8("LayerView"));
        LayerView->resize(251, 514);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        listView = new QTableView(dockWidgetContents);
        listView->setObjectName(QString::fromUtf8("listView"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(listView->sizePolicy().hasHeightForWidth());
        listView->setSizePolicy(sizePolicy);

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

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        selection_mode_label = new QLabel(dockWidgetContents);
        selection_mode_label->setObjectName(QString::fromUtf8("selection_mode_label"));

        verticalLayout->addWidget(selection_mode_label);

        selection_mode_combo = new QComboBox(dockWidgetContents);
        selection_mode_combo->setObjectName(QString::fromUtf8("selection_mode_combo"));

        verticalLayout->addWidget(selection_mode_combo);


        verticalLayout_2->addLayout(verticalLayout);

        LayerView->setWidget(dockWidgetContents);

        retranslateUi(LayerView);

        QMetaObject::connectSlotsByName(LayerView);
    } // setupUi

    void retranslateUi(QDockWidget *LayerView)
    {
        LayerView->setWindowTitle(QApplication::translate("LayerView", "Layers", 0, QApplication::UnicodeUTF8));
        delete_button->setText(QApplication::translate("LayerView", "Delete", 0, QApplication::UnicodeUTF8));
        merge_button->setText(QApplication::translate("LayerView", "Merge", 0, QApplication::UnicodeUTF8));
        selection_mode_label->setText(QApplication::translate("LayerView", "Selection mode:", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class LayerView: public Ui_LayerView {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LAYERVIEW_H
