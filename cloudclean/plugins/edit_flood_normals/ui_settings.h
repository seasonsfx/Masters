/********************************************************************************
** Form generated from reading UI file 'settings.ui'
**
** Created: Fri Oct 12 14:34:30 2012
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
    QLabel *label_3;
    QDoubleSpinBox *minNoise;
    QDoubleSpinBox *maxNoise;
    QLabel *label_4;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout;
    QSpinBox *kConnect;
    QLabel *label;
    QLabel *label_2;
    QSpinBox *noiseK;
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
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_2->addWidget(label_3, 0, 0, 1, 1);

        minNoise = new QDoubleSpinBox(groupBox);
        minNoise->setObjectName(QString::fromUtf8("minNoise"));
        minNoise->setAccelerated(true);
        minNoise->setMaximum(1);
        minNoise->setSingleStep(0.01);
        minNoise->setValue(0.1);

        gridLayout_2->addWidget(minNoise, 0, 1, 1, 1);

        maxNoise = new QDoubleSpinBox(groupBox);
        maxNoise->setObjectName(QString::fromUtf8("maxNoise"));
        maxNoise->setAccelerated(true);
        maxNoise->setMinimum(0);
        maxNoise->setMaximum(1);
        maxNoise->setSingleStep(0.01);
        maxNoise->setValue(1);

        gridLayout_2->addWidget(maxNoise, 1, 1, 1, 1);

        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_2->addWidget(label_4, 1, 0, 1, 1);


        verticalLayout->addWidget(groupBox);

        groupBox_2 = new QGroupBox(Settings);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        gridLayout = new QGridLayout(groupBox_2);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        kConnect = new QSpinBox(groupBox_2);
        kConnect->setObjectName(QString::fromUtf8("kConnect"));
        kConnect->setMinimum(1);
        kConnect->setValue(4);

        gridLayout->addWidget(kConnect, 1, 1, 1, 1);

        label = new QLabel(groupBox_2);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 1, 0, 1, 1);

        label_2 = new QLabel(groupBox_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 2, 0, 1, 1);

        noiseK = new QSpinBox(groupBox_2);
        noiseK->setObjectName(QString::fromUtf8("noiseK"));
        noiseK->setMinimum(2);
        noiseK->setValue(20);

        gridLayout->addWidget(noiseK, 2, 1, 1, 1);

        gridLayout->setColumnStretch(0, 1);

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
        label_3->setText(QApplication::translate("Settings", "Min noise", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("Settings", "Max noise", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("Settings", "Neighbours", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("Settings", "K connectivity", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("Settings", "K Noise Calc", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Settings: public Ui_Settings {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETTINGS_H
