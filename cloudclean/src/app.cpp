#include "app.h"
#include "appinfo.h"
#include <iostream>
#include <cstdlib>
#include <cctype>
#include <exception>
#include <stdexcept>
#include <memory>
#include <thread>

#include <QProgressBar>
#include <QStatusBar>
#include <QTabWidget>
#include <QUndoStack>
#include <QDesktopWidget>
#include <QGridLayout>

#include "actionmanager.h"
#include "gui/mainwindow.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "model/layerlist.h"
#include "model/cloudlist.h"

#include "pluginsystem/pluginmanager.h"

#ifdef _WIN32
#   define INFINITY (DBL_MAX+DBL_MAX)
#   define NAN (INFINITY-INFINITY)
#endif

inline bool isNaN(float val){
    return (val != val);
}

namespace {
    bool matches_option(const QString& givenoption, const QString& expectedoption, int mindashes=1, int maxdashes=2) {
        int dashes = 0;
        if ( givenoption.length() > 0 ) {
            while ((dashes<givenoption.length())&&(givenoption[dashes]=='-')) {
                dashes++;
            }
        }
        if ( (dashes < mindashes) || (dashes > maxdashes) ) {
            return false;
        }
        QString substr=givenoption.right(givenoption.length()-dashes);
        return (expectedoption.compare(substr,Qt::CaseInsensitive)==0);
    }
}

inline std::ostream& operator<<(std::ostream& out, const QString& str) {
    QByteArray a = str.toUtf8();
    out << a.constData();
    return out;
}

App::App(int& argc, char** argv) : QApplication(argc,argv),
    _invocation(argv[0]) {
    // Enforce singleton property
    if ( _instance ) {
        throw std::runtime_error("Only one instance of App allowed.");
    }

    // Remember if we are done
    bool done = false;

    // Set the singleton instance to this
    _instance = this;

    // Set the application properties
    setApplicationName(APPLICATION_NAME);
    setApplicationVersion(APPLICATION_VERSION_STRING);
    setOrganizationName(APPLICATION_VENDOR_NAME);
    setOrganizationDomain(APPLICATION_VENDOR_URL);
    
    // Parse the commandline
    int idx = 1;
    while ( idx < argc ) {
        QString arg(argv[idx]);
        if ( matches_option(arg, "help", 0) || matches_option(arg, "h")
             || matches_option(arg, "?",0) ) {
            printHelpMessage();
            std::exit(0);
        } else if ( matches_option(arg, "version",0) ) {
            printVersionMessage();
            std::exit(0);
        } else if ( matches_option(arg, "version-triplet") ) {
            printVersionTripletMessage();
            std::exit(0);
        } else if ( matches_option(arg, "prefset") ) {
            // Verify that there is another argument
            if ( (idx+1) >= argc ) {
                qDebug() << "Option \"" << arg << "\" requires a parameter.";
                std::exit(1);
            }
            
            // Increment the index
            idx++;
            
            // Get the next parameter
            std::string param(argv[idx]);
            
            // Determine if there is an equals sign
            // If there is, set the preference;
            // Otherwise, remove the preference
            size_t eqidx = param.find('=');
            if ( eqidx != std::string::npos ) {
                std::string key = param.substr(0, eqidx);
                std::string val = param.substr(eqidx+1);
                setPreference(key,val);
            } else {
                unsetPreference(param);
            }
            done = true;
        } else if ( matches_option(arg, "prefdel") ) {
            // Verify that there is another argument
            if ( (idx+1) >= argc ) {
                qDebug() << "Option \"" << arg << "\" requires a parameter.";
                std::exit(1);
            }
            
            // Increment the index
            idx++;
            
            // Get the next parameter
            std::string param(argv[idx]);
            
            // Remove the preference
            unsetPreference(param);
            done = true;
        } else if ( matches_option(arg, "preflist") ) {
            printAllPreferences();
            done = true;
        } else if ( matches_option(arg, "prefget") ) {
            // Verify that there is another argument
            if ( (idx+1) >= argc ) {
                qDebug() << "Option \"" << arg << "\" requires a parameter.";
                std::exit(1);
            }
            
            // Increment the index
            idx++;
            
            // Get the next parameter
            std::string param(argv[idx]);
            
            // Print the preference
            printPreference(param);
            done = true;
        } else if ( matches_option(arg, "appid") || matches_option(arg, "application-identifier") ) {
            printApplicationIdentifier();
            std::exit(0);
        } else {
            qDebug() << "Unrecognized option: \"" << arg << "\". Ignoring";
        }
        idx++;
    }
    
    if ( done ) {
        std::exit(0);
    }

    undostack_ = new QUndoStack();

    // initialise data model
    ll_.reset(new LayerList());
    cl_.reset(new CloudList(undostack_));

    initGUI();

    QAction * undo = undostack_->createUndoAction(0);
    QAction * redo = undostack_->createRedoAction(0);
    am_->addAction(undo, "Edit");
    am_->addAction(redo, "Edit");

    QAction * deselect = new QAction(tr("Deselect all"), mainwindow_);
    connect(deselect, SIGNAL(triggered()), cl_.get(), SLOT(deselectAllPoints()));

    am_->addAction(deselect, "Edit");


    qRegisterMetaType<std::shared_ptr<PointCloud> >("std::shared_ptr<PointCloud>");
    qRegisterMetaType<std::shared_ptr<Layer> >("std::shared_ptr<Layer>");
    // link up signals to model
    connect(cl_.get(), SIGNAL(cloudUpdate(std::shared_ptr<PointCloud>)), gld_.get(), SLOT(reloadCloud(std::shared_ptr<PointCloud>)));
    connect(cl_.get(), SIGNAL(updated()), glwidget_, SLOT(update()));
    connect(cl_.get(), SIGNAL(updatedActive(std::shared_ptr<PointCloud>)), flatview_, SLOT(setCloud(std::shared_ptr<PointCloud>)));
    connect(cl_.get(), SIGNAL(progressUpdate(int)), progressbar_, SLOT(setValue(int)));
    connect(gld_.get(), SIGNAL(update()), glwidget_, SLOT(update()));
    connect(gld_.get(), SIGNAL(update()), flatview_, SLOT(update()));
    connect(ll_.get(), SIGNAL(layerUpdate(std::shared_ptr<Layer>)), gld_.get(), SLOT(reloadColorLookupBuffer()));
    connect(ll_.get(), SIGNAL(lookupTableUpdate()), gld_.get(), SLOT(reloadColorLookupBuffer()));

    // reload the lookup table
    gld_->reloadColorLookupBuffer();

    pm_ = new PluginManager(glwidget_, flatview_, cl_.get(), ll_.get(), am_);
    pm_->loadPlugins();
    pm_->initializePlugins();

    // load a cloud
    std::function<void (const char *)> loadcloud = [&] (const char * fname) {

        std::shared_ptr<PointCloud> pc = cl_->loadFile(fname);

        // make a selection
        std::vector<PointFlags> & flags = pc->flags_;
        for(uint i = 0; i < flags.size()/5; i++){
            flags[i] = PointFlags::selected;
        }


        // create layers with colors
        std::shared_ptr<Layer> layers[3];
        layers[0] = ll_->addLayer("Test1");
        layers[1] = ll_->addLayer("Test2");
        layers[2] = ll_->addLayer("Test3");

        // make five labels
        for(int i = 0; i < 5; i++)
            ll_->newLabelId(layers[i%3]);


        // label the cloud
        std::vector<int16_t> & labels = pc->labels_;
        for(uint i = 1; i <= labels.size(); i++){
            int l = 1+5*(float(i)/labels.size());
            labels[i] = l;
        }

        qRegisterMetaType<std::shared_ptr<PointCloud> >("std::shared_ptr<PointCloud>");
        QMetaObject::invokeMethod(flatview_, "setCloud", Q_ARG(std::shared_ptr<PointCloud>, pc));

        glwidget_->update();
        QMetaObject::invokeMethod(progressbar_, "setRange", Q_ARG(int, 0), Q_ARG(int, 100));
        QMetaObject::invokeMethod(progressbar_, "reset");


        qDebug() << "Loaded";
        qDebug() << "Size: " << pc->size();


    };

    //std::thread(loadcloud, "/home/rickert/trees.ptx").detach();
    std::thread(loadcloud, "/home/rickert/Workspace/uscans/2011.06.11-10.23.54.zfs_cy.ptx").detach();
    //std::thread(loadcloud, "/home/rickert/Workspace/uscans/Petra_Top_xf.ptx").detach();
    //std::thread(loadcloud, "/home/rickert/Petra_Top_xf.ptx").detach();
}

App::~App() {
    delete mainwindow_;
    delete am_;
}
App* App::INSTANCE() {
    return _instance;
}

void App::initGUI() {
    mainwindow_ = new MainWindow();
    am_ = new ActionManager(mainwindow_->menuBar());
    statusbar_ = mainwindow_->statusBar();

    progressbar_ = new QProgressBar();
    QLabel *size = new QLabel( tr("  999999kB  ") );

    size->setMinimumSize( size->sizeHint() );
    size->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
    size->setText( tr("%1kB ").arg(0) );
    size->setToolTip( tr("The memory used for the current document.") );

    progressbar_->setTextVisible( false );
    progressbar_->setRange( 0, 100 );
    statusbar_->addWidget( progressbar_, 1 );
    statusbar_->addWidget( size );
    statusbar_->showMessage( tr("Ready"), 2000 );

    tabs_ = new QTabWidget(mainwindow_);


    QGLFormat base_format;
    base_format.setVersion(3, 3);
    base_format.setProfile(QGLFormat::CompatibilityProfile);
    base_format.setSampleBuffers(true);


    glwidget_ = new GLWidget(base_format, cl_, ll_, tabs_);
    flatview_ = new FlatView(base_format, cl_, ll_, tabs_, glwidget_);

    QGLContext * ctx = const_cast<QGLContext *>(glwidget_->context());
    gld_.reset(new GLData(ctx, cl_, ll_));
    glwidget_->setGLD(gld_);
    flatview_->setGLD(gld_);

    clv_ = new CloudListView(ll_, cl_, mainwindow_);
    mainwindow_->addDockWidget(Qt::RightDockWidgetArea, clv_);

    llv_ = new LayerListView(ll_, cl_, mainwindow_);
    mainwindow_->addDockWidget(Qt::RightDockWidgetArea, llv_);

    //QAction * show_cloudlist = new QAction("Show Cloudlist", 0);
    //QAction * show_layerlist = new QAction("Show Layerlist", 0);

    //connect(show_cloudlist, SIGNAL(toggled(bool)), clv_, SLOT(show()));
    //connect(show_layerlist, SIGNAL(toggled(bool)), llv_, SLOT(show()));

    am_->addAction(clv_->toggleViewAction(), "View");
    am_->addAction(llv_->toggleViewAction(), "View");

    tabs_->addTab(glwidget_, "3D View");
    tabs_->addTab(flatview_, "2D View");
    mainwindow_->setCentralWidget(tabs_);
    mainwindow_->setVisible(true);
}

void App::printHelpMessage() {
    std::cout << "Usage: " << getProjectInvocation() << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "    --help                       Displays this help message." << std::endl;
    std::cout << "    --version                    Prints the program version." << std::endl;
    std::cout << "    --version-triplet            Prints the undecorated program version." << std::endl;
    std::cout << "    --appid                      Prints the unique application identifier." << std::endl;
    std::cout << "    --prefset <key>=<val>        Sets the given preference." << std::endl;
    std::cout << "    --prefdel <key>              Unsets the given preference." << std::endl;
    std::cout << "    --prefget <key>              Prints the given preference." << std::endl;
    std::cout << "    --preflist                   Lists all preferences that are set." << std::endl;
}

void App::printVersionMessage() {
    std::cout << getProjectName() << " v" << getProjectVersion() << std::endl;
    std::cout << getProjectVendorName() << "; Copyright (C) " << getProjectCopyrightYears();
}

void App::printVersionTripletMessage() {
    std::cout << getProjectVersion() << std::endl;
}

void App::printApplicationIdentifier() {
    std::cout << getProjectID() << std::endl;
}

QString App::getProjectName() {
    return APPLICATION_NAME;
}


QString App::getProjectCodeName() {
    return APPLICATION_CODENAME;
}

QString App::getProjectVendorID() {
    return APPLICATION_VENDOR_ID;
}

QString App::getProjectVendorName() {
    return APPLICATION_VENDOR_NAME;
}

QString App::getProjectID() {
    return APPLICATION_ID;
}

int App::getProjectMajorVersion() {
    return APPLICATION_VERSION_MAJOR;
}

int App::getProjectMinorVersion() {
    return APPLICATION_VERSION_MINOR;
}

int App::getProjectPatchVersion() {
    return APPLICATION_VERSION_PATCH;
}

QString App::getProjectVersion() {
    return APPLICATION_VERSION_STRING;
}

QString App::getProjectCopyrightYears() {
    return APPLICATION_COPYRIGHT_YEARS;
}

QString App::getProjectInvocation() {
    return _invocation;
}

std::string App::getKeyName(const std::string& key) const {
    std::string result(key);
    for ( size_t i = 0; i < result.size(); i++ ) {
        if ( (result[i]=='/') || (result[i]=='\\') ) {
            result[i] = '.';
        }
    }
    return result;
}

std::string
App::getKeyRepr(const std::string& key) const {
    std::string result(key);
    for ( size_t i = 0; i < result.size(); i++ ) {
        if ( (result[i]=='/') || (result[i]=='\\') ) {
            result[i] = '/';
        }
    }
    return result;
}

void App::setPreference(const std::string& key, const std::string& val) {
    QSettings settings;
    std::string keyrep(getKeyRepr(key));
    QString qkeyrep(keyrep.c_str());
    QString qval(val.c_str());
    settings.setValue(qkeyrep,qval);
    settings.sync();
}

void App::unsetPreference(const std::string& key) {
    QSettings settings;
    std::string keyrep(getKeyRepr(key));
    QString qkeyrep(keyrep.c_str());
    settings.beginGroup(qkeyrep);
    if ( (settings.childGroups().length()!=0) || (settings.childKeys().length()!=0) ) {
        settings.setValue("", "");
    } else {
        settings.remove("");
    }
    settings.endGroup();
    settings.sync();
}

void App::printPreference(const std::string& key) const {
    QSettings settings;
    std::string keyrep(getKeyRepr(key));
    QString qkeyrep(keyrep.c_str());
    QString result="undefined";
    if ( settings.contains(qkeyrep) ) {
        result=settings.value(qkeyrep,QString("undefined")).toString();
    }
    std::cout << result << std::endl;
}

void App::printAllPreferences() const {
    QSettings settings;
    QStringList keys = settings.allKeys();
    for ( QStringList::const_iterator it = keys.begin(); it != keys.end(); ++it ) {
        QString qkeystr = *it;
        QString qvalstr = settings.value(qkeystr).toString();
        
        if ( ! qvalstr.isEmpty() ) {
            std::string key=getKeyName(convert(qkeystr));
            std::cout << key << "=" << qvalstr << std::endl;
        }
    }
}

std::string App::convert(const QString& str) const {
    QByteArray data = str.toUtf8();
    std::string result(data.constData());
    return result;
}

QString App::convert(const std::string& str) const {
    QString result(str.c_str());
    return result;
}

bool App::notify(QObject * receiver, QEvent * event){
    try {
        return QApplication::notify(receiver, event);
    }
    catch(std::exception& e) {
        qDebug() << "Exception thrown:" << e.what();
        exit(1);
    }
    return true;
}

App* App::_instance = 0;
