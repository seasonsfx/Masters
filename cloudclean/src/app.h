// <Copyright Holder>. Copyright (C) <Copyright Year(s)>. <License>.
#ifndef HEADER_SRC_APP_H_INCLUDED
#define HEADER_SRC_APP_H_INCLUDED

#include <QtCore>
#include <QtGui>
#include <QApplication>

#include <boost/shared_ptr.hpp>
#include "gui/mainwindow.h"
#include "gui/glwidget.h"
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/flatview.h"
#include "pluginsystem/pluginmanager.h"

#include "gui/layerlistview.h"
#include "gui/cloudlistview.h"
#include "gui/gldata.h"

class QProgressBar;
class QStatusBar;
class QTabWidget;
class QUndoStack;

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
        bool notify(QObject *receiver, QEvent *event);

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
        std::shared_ptr<CloudList> cl_;
        std::shared_ptr<LayerList> ll_;
        std::shared_ptr<GLData> gld_;

        CloudListView * clv_;
        LayerListView * llv_;

        GLWidget * glwidget_;
        MainWindow * mainwindow_;
        QStatusBar * statusbar_;
        QProgressBar *progressbar_;
        QTabWidget * tabs_;
        FlatView * flatview_;
        PluginManager * pm_;
        QUndoStack * undostack_;
};

#endif
