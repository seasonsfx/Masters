// <Copyright Holder>. Copyright (C) <Copyright Year(s)>. <License>.
#ifndef HEADER_SRC_APP_H_INCLUDED
#define HEADER_SRC_APP_H_INCLUDED

#include <QtCore>
#include <QtGui>
#include <QApplication>
#include <QProgressBar>
#include <QStatusBar>
#include <QTabWidget>
#include <boost/shared_ptr.hpp>
#include "gui/mainwindow.h"
#include "gui/glwidget.h"
#include "model/datamodel.h"

class App : public QApplication
{
        Q_OBJECT
    public:
        App(int& argc, char** argv);
        ~App();
        
        App* INSTANCE();

        QString getProjectName();
        QString getProjectCodeName();
        QString getProjectVendorID();
        QString getProjectVendorName();
        QString getProjectID();
        int getProjectMajorVersion();
        int getProjectMinorVersion();
        int getProjectPatchVersion();
        QString getProjectVersion();
        QString getProjectCopyrightYears();
        QString getProjectInvocation();

    public slots:
        void loadImage(QImage image);

    private:
        void initGUI();
        void printHelpMessage();
        void printVersionMessage();
        void printVersionTripletMessage();
        void printApplicationIdentifier();
        void setPreference(const std::string& key, const std::string& val);
        void unsetPreference(const std::string& key);
        void printPreference(const std::string& key)const;
        void printAllPreferences()const;
        std::string getKeyName(const std::string& key)const;
        std::string getKeyRepr(const std::string& key)const;
        std::string convert(const QString& str)const;
        QString convert(const std::string& str)const;
        
        static App* _instance;
        QString _invocation;
        std::shared_ptr<MainWindow> mainwindow_;
        std::shared_ptr<DataModel> model_;

        GLWidget * glwidget_;
        QStatusBar * statusbar_;
        QProgressBar *progressbar_;
        QTabWidget * tabs_;
        QLabel * imageLabel;
};

#endif
