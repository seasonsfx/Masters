/********************************************************************************
** Form generated from reading UI file 'settings.ui'
**
** Created: Wed Aug 22 22:34:16 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SETTINGS_H
#define UI_SETTINGS_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Settings
{
public:
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QLabel *label_2;
    QDoubleSpinBox *eigen2;
    QDoubleSpinBox *eigen3;
    QDoubleSpinBox *eigen1;
    QDoubleSpinBox *deviation;
    QLabel *label_3;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout;
    QLabel *label;
    QSpinBox *kConnect;
    QLabel *label_4;
    QSpinBox *kPCA;
    QSpacerItem *verticalSpacer;

    void setupUi(QWidget *Settings)
    {
        if (Settings->objectName().isEmpty())
            Settings->setObjectName(QString::fromUtf8("Settings"));
        Settings->resize(272, 388);
        verticalLayout = new QVBoxLayout(Settings);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        groupBox = new QGroupBox(Settings);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_2->addWidget(label_2, 0, 0, 1, 1);

        eigen2 = new QDoubleSpinBox(groupBox);
        eigen2->setObjectName(QString::fromUtf8("eigen2"));
        eigen2->setSingleStep(0.1);
        eigen2->setValue(1);

        gridLayout_2->addWidget(eigen2, 1, 1, 1, 1);

        eigen3 = new QDoubleSpinBox(groupBox);
        eigen3->setObjectName(QString::fromUtf8("eigen3"));
        eigen3->setSingleStep(0.1);
        eigen3->setValue(0.5);

        gridLayout_2->addWidget(eigen3, 2, 1, 1, 1);

        eigen1 = new QDoubleSpinBox(groupBox);
        eigen1->setObjectName(QString::fromUtf8("eigen1"));
        eigen1->setSingleStep(0.1);
        eigen1->setValue(1);

        gridLayout_2->addWidget(eigen1, 0, 1, 1, 1);

        deviation = new QDoubleSpinBox(groupBox);
        deviation->setObjectName(QString::fromUtf8("deviation"));
        deviation->setMaximum(1);
        deviation->setSingleStep(0.01);
        deviation->setValue(0.1);

        gridLayout_2->addWidget(deviation, 3, 1, 1, 1);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_2->addWidget(label_3, 3, 0, 1, 1);


        verticalLayout->addWidget(groupBox);

        groupBox_2 = new QGroupBox(Settings);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        gridLayout = new QGridLayout(groupBox_2);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label = new QLabel(groupBox_2);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 1, 0, 1, 1);

        kConnect = new QSpinBox(groupBox_2);
        kConnect->setObjectName(QString::fromUtf8("kConnect"));
        kConnect->setMinimum(1);
        kConnect->setValue(4);

        gridLayout->addWidget(kConnect, 1, 1, 1, 1);

        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 2, 0, 1, 1);

        kPCA = new QSpinBox(groupBox_2);
        kPCA->setObjectName(QString::fromUtf8("kPCA"));
        kPCA->setMinimum(4);
        kPCA->setValue(20);

        gridLayout->addWidget(kPCA, 2, 1, 1, 1);

        gridLayout->setColumnStretch(0, 1);
        gridLayout->setColumnStretch(1, 7);

        verticalLayout->addWidget(groupBox_2);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        retranslateUi(Settings);

        QMetaObject::connectSlotsByName(Settings);
    } // setupUi

    void retranslateUi(QWidget *Settings)
    {
        Settings->setWindowTitle(QApplication::translate("Settings", "Form", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("Settings", "Threshold", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("Settings", "Eigen ratio ", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("Settings", "Deviation", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("Settings", "Neighbours", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("Settings", "K connectivity", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("Settings", "K PCA", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Settings: public Ui_Settings {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETTINGS_H
