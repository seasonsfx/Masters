/********************************************************************************
** Form generated from reading UI file 'subsampledialog.ui'
**
** Created: Thu Nov 29 16:54:06 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SUBSAMPLEDIALOG_H
#define UI_SUBSAMPLEDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QSpinBox>

QT_BEGIN_NAMESPACE

class Ui_SubsampleDialog
{
public:
    QDialogButtonBox *buttonBox;
    QLabel *label;
    QSpinBox *spinBox;

    void setupUi(QDialog *SubsampleDialog)
    {
        if (SubsampleDialog->objectName().isEmpty())
            SubsampleDialog->setObjectName(QString::fromUtf8("SubsampleDialog"));
        SubsampleDialog->resize(280, 98);
        buttonBox = new QDialogButtonBox(SubsampleDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(10, 50, 251, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        label = new QLabel(SubsampleDialog);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 10, 191, 31));
        spinBox = new QSpinBox(SubsampleDialog);
        spinBox->setObjectName(QString::fromUtf8("spinBox"));
        spinBox->setGeometry(QRect(200, 10, 60, 27));
        spinBox->setMinimum(0);
        spinBox->setMaximum(32);
        spinBox->setSingleStep(2);
        spinBox->setValue(0);

        retranslateUi(SubsampleDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), SubsampleDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), SubsampleDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(SubsampleDialog);
    } // setupUi

    void retranslateUi(QDialog *SubsampleDialog)
    {
        SubsampleDialog->setWindowTitle(QApplication::translate("SubsampleDialog", "Dialog", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SubsampleDialog", "Subsample cloud by factor:", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SubsampleDialog: public Ui_SubsampleDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SUBSAMPLEDIALOG_H
