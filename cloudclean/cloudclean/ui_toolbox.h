/********************************************************************************
** Form generated from reading UI file 'toolbox.ui'
**
** Created: Thu Nov 29 01:28:17 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TOOLBOX_H
#define UI_TOOLBOX_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDockWidget>
#include <QtGui/QHeaderView>
#include <QtGui/QScrollArea>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Toolbox
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout;
    QScrollArea *settingsArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *content;

    void setupUi(QDockWidget *Toolbox)
    {
        if (Toolbox->objectName().isEmpty())
            Toolbox->setObjectName(QString::fromUtf8("Toolbox"));
        Toolbox->resize(283, 493);
        Toolbox->setMinimumSize(QSize(96, 200));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout = new QVBoxLayout(dockWidgetContents);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        settingsArea = new QScrollArea(dockWidgetContents);
        settingsArea->setObjectName(QString::fromUtf8("settingsArea"));
        settingsArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 263, 448));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(scrollAreaWidgetContents->sizePolicy().hasHeightForWidth());
        scrollAreaWidgetContents->setSizePolicy(sizePolicy);
        verticalLayout_2 = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        content = new QVBoxLayout();
        content->setObjectName(QString::fromUtf8("content"));
        content->setSizeConstraint(QLayout::SetDefaultConstraint);

        verticalLayout_2->addLayout(content);

        settingsArea->setWidget(scrollAreaWidgetContents);

        verticalLayout->addWidget(settingsArea);

        Toolbox->setWidget(dockWidgetContents);

        retranslateUi(Toolbox);

        QMetaObject::connectSlotsByName(Toolbox);
    } // setupUi

    void retranslateUi(QDockWidget *Toolbox)
    {
        Toolbox->setWindowTitle(QApplication::translate("Toolbox", "Toolbox", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Toolbox: public Ui_Toolbox {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TOOLBOX_H
