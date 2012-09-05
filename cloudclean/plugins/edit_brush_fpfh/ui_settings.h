/********************************************************************************
** Form generated from reading UI file 'settings.ui'
**
** Created: Wed Aug 22 01:14:43 2012
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
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QRadioButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Settings
{
public:
    QVBoxLayout *verticalLayout;
    QGroupBox *threshBox;
    QVBoxLayout *verticalLayout_2;
    QRadioButton *euclidianRad;
    QDoubleSpinBox *euclidian;
    QRadioButton *cosineRad;
    QDoubleSpinBox *cosine;
    QRadioButton *intensityRad;
    QDoubleSpinBox *intensity;
    QGroupBox *neighbourBox;
    QVBoxLayout *verticalLayout_3;
    QRadioButton *kRad;
    QSpinBox *k;
    QRadioButton *fixedRad;
    QDoubleSpinBox *fixed;
    QRadioButton *dynRad;
    QDoubleSpinBox *dyn;
    QSpacerItem *verticalSpacer;

    void setupUi(QWidget *Settings)
    {
        if (Settings->objectName().isEmpty())
            Settings->setObjectName(QString::fromUtf8("Settings"));
        Settings->resize(209, 481);
        verticalLayout = new QVBoxLayout(Settings);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        threshBox = new QGroupBox(Settings);
        threshBox->setObjectName(QString::fromUtf8("threshBox"));
        threshBox->setMinimumSize(QSize(0, 0));
        verticalLayout_2 = new QVBoxLayout(threshBox);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        euclidianRad = new QRadioButton(threshBox);
        euclidianRad->setObjectName(QString::fromUtf8("euclidianRad"));
        euclidianRad->setChecked(true);

        verticalLayout_2->addWidget(euclidianRad);

        euclidian = new QDoubleSpinBox(threshBox);
        euclidian->setObjectName(QString::fromUtf8("euclidian"));
        euclidian->setMinimum(1);
        euclidian->setMaximum(200);
        euclidian->setSingleStep(1);
        euclidian->setValue(25);

        verticalLayout_2->addWidget(euclidian);

        cosineRad = new QRadioButton(threshBox);
        cosineRad->setObjectName(QString::fromUtf8("cosineRad"));

        verticalLayout_2->addWidget(cosineRad);

        cosine = new QDoubleSpinBox(threshBox);
        cosine->setObjectName(QString::fromUtf8("cosine"));
        cosine->setMouseTracking(true);
        cosine->setAccelerated(true);
        cosine->setMaximum(1);
        cosine->setSingleStep(0.01);
        cosine->setValue(0.1);

        verticalLayout_2->addWidget(cosine);

        intensityRad = new QRadioButton(threshBox);
        intensityRad->setObjectName(QString::fromUtf8("intensityRad"));

        verticalLayout_2->addWidget(intensityRad);

        intensity = new QDoubleSpinBox(threshBox);
        intensity->setObjectName(QString::fromUtf8("intensity"));
        intensity->setMaximum(1);
        intensity->setSingleStep(0.01);

        verticalLayout_2->addWidget(intensity);


        verticalLayout->addWidget(threshBox);

        neighbourBox = new QGroupBox(Settings);
        neighbourBox->setObjectName(QString::fromUtf8("neighbourBox"));
        verticalLayout_3 = new QVBoxLayout(neighbourBox);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        kRad = new QRadioButton(neighbourBox);
        kRad->setObjectName(QString::fromUtf8("kRad"));
        kRad->setChecked(true);

        verticalLayout_3->addWidget(kRad);

        k = new QSpinBox(neighbourBox);
        k->setObjectName(QString::fromUtf8("k"));
        k->setMinimum(1);
        k->setValue(4);

        verticalLayout_3->addWidget(k);

        fixedRad = new QRadioButton(neighbourBox);
        fixedRad->setObjectName(QString::fromUtf8("fixedRad"));

        verticalLayout_3->addWidget(fixedRad);

        fixed = new QDoubleSpinBox(neighbourBox);
        fixed->setObjectName(QString::fromUtf8("fixed"));
        fixed->setMinimum(0.05);
        fixed->setSingleStep(0.05);

        verticalLayout_3->addWidget(fixed);

        dynRad = new QRadioButton(neighbourBox);
        dynRad->setObjectName(QString::fromUtf8("dynRad"));

        verticalLayout_3->addWidget(dynRad);

        dyn = new QDoubleSpinBox(neighbourBox);
        dyn->setObjectName(QString::fromUtf8("dyn"));
        dyn->setMinimum(0.1);
        dyn->setSingleStep(0.1);

        verticalLayout_3->addWidget(dyn);


        verticalLayout->addWidget(neighbourBox);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        retranslateUi(Settings);

        QMetaObject::connectSlotsByName(Settings);
    } // setupUi

    void retranslateUi(QWidget *Settings)
    {
        Settings->setWindowTitle(QApplication::translate("Settings", "Form", 0, QApplication::UnicodeUTF8));
        threshBox->setTitle(QApplication::translate("Settings", "Threshold function", 0, QApplication::UnicodeUTF8));
        euclidianRad->setText(QApplication::translate("Settings", "Euclidian", 0, QApplication::UnicodeUTF8));
        cosineRad->setText(QApplication::translate("Settings", "Cosine", 0, QApplication::UnicodeUTF8));
        intensityRad->setText(QApplication::translate("Settings", "Intensity", 0, QApplication::UnicodeUTF8));
        neighbourBox->setTitle(QApplication::translate("Settings", "Neigbours", 0, QApplication::UnicodeUTF8));
        kRad->setText(QApplication::translate("Settings", "K", 0, QApplication::UnicodeUTF8));
        fixedRad->setText(QApplication::translate("Settings", "Fixed radius (m?)", 0, QApplication::UnicodeUTF8));
        dynRad->setText(QApplication::translate("Settings", "Dynamic radius coef", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Settings: public Ui_Settings {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETTINGS_H
