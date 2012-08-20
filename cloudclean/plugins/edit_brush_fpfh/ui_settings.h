/********************************************************************************
** Form generated from reading UI file 'settings.ui'
**
** Created: Mon Aug 20 02:31:20 2012
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
#include <QtGui/QLabel>
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
    QGroupBox *radioGroup;
    QRadioButton *euclid;
    QRadioButton *cosine;
    QLabel *label_2;
    QDoubleSpinBox *thresholdSpinner;
    QLabel *label;
    QSpinBox *neighboursSpinner;
    QSpacerItem *verticalSpacer;

    void setupUi(QWidget *Settings)
    {
        if (Settings->objectName().isEmpty())
            Settings->setObjectName(QString::fromUtf8("Settings"));
        Settings->resize(205, 300);
        verticalLayout = new QVBoxLayout(Settings);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        radioGroup = new QGroupBox(Settings);
        radioGroup->setObjectName(QString::fromUtf8("radioGroup"));
        radioGroup->setMinimumSize(QSize(0, 80));
        euclid = new QRadioButton(radioGroup);
        euclid->setObjectName(QString::fromUtf8("euclid"));
        euclid->setGeometry(QRect(0, 30, 151, 22));
        euclid->setChecked(true);
        cosine = new QRadioButton(radioGroup);
        cosine->setObjectName(QString::fromUtf8("cosine"));
        cosine->setGeometry(QRect(0, 50, 141, 22));

        verticalLayout->addWidget(radioGroup);

        label_2 = new QLabel(Settings);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout->addWidget(label_2);

        thresholdSpinner = new QDoubleSpinBox(Settings);
        thresholdSpinner->setObjectName(QString::fromUtf8("thresholdSpinner"));
        thresholdSpinner->setMouseTracking(true);
        thresholdSpinner->setAccelerated(true);
        thresholdSpinner->setMaximum(200);

        verticalLayout->addWidget(thresholdSpinner);

        label = new QLabel(Settings);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        neighboursSpinner = new QSpinBox(Settings);
        neighboursSpinner->setObjectName(QString::fromUtf8("neighboursSpinner"));
        neighboursSpinner->setMinimum(1);
        neighboursSpinner->setValue(4);

        verticalLayout->addWidget(neighboursSpinner);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        retranslateUi(Settings);

        QMetaObject::connectSlotsByName(Settings);
    } // setupUi

    void retranslateUi(QWidget *Settings)
    {
        Settings->setWindowTitle(QApplication::translate("Settings", "Form", 0, QApplication::UnicodeUTF8));
        radioGroup->setTitle(QApplication::translate("Settings", "Distance function", 0, QApplication::UnicodeUTF8));
        euclid->setText(QApplication::translate("Settings", "Euclidian", 0, QApplication::UnicodeUTF8));
        cosine->setText(QApplication::translate("Settings", "Cosine", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("Settings", "Threshold", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("Settings", "Neighbours", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Settings: public Ui_Settings {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETTINGS_H
