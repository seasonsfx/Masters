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

    core_ = new Core();
    pm_ = new PluginManager(core_);
    pm_->loadPlugins();
    pm_->initializePlugins();

    QAction * disable_plugins = new QAction(this);
    core_->mw_->addAction(disable_plugins);
    connect(disable_plugins, SIGNAL(toggled(bool)), pm_, SIGNAL(endEdit()));
    disable_plugins->setShortcut(QKeySequence(Qt::Key_Escape));
    //disable_plugins->setShortcutContext(Qt::ApplicationShortcut);

    // load a cloud
    std::function<void (const char *)> loadcloud = [&] (const char * fname) {

        std::shared_ptr<PointCloud> pc = core_->cl_->loadFile(fname);

        // make a selection
        std::vector<PointFlags> & flags = pc->flags_;
        for(uint i = 0; i < flags.size()/5; i++){
            flags[i] = PointFlags::selected;
        }


        // create layers with colors
        std::shared_ptr<Layer> layers[3];
        layers[0] = core_->ll_->addLayer("Test1");
        layers[1] = core_->ll_->addLayer("Test2");
        layers[2] = core_->ll_->addLayer("Test3");

        // make five labels
        for(int i = 0; i < 5; i++){
            layers[i%3]->addLabel(core_->ll_->newLabelId());
        }


        // label the cloud
        std::vector<int16_t> & labels = pc->labels_;
        for(uint i = 1; i <= labels.size(); i++){
            int l = 1+5*(float(i)/labels.size());
            labels[i] = l;
        }

        qDebug() << "Loaded";
        qDebug() << "Size: " << pc->size();

    };

    //std::thread(loadcloud, "/home/rickert/trees.ptx").detach();
    //std::thread(loadcloud, "/home/rickert/Workspace/uscans/2011.06.11-10.23.54.zfs_cy.ptx").detach();
    //std::thread(loadcloud, "/home/rickert/Workspace/uscans/Petra_Top_xf.ptx").detach();
    //std::thread(loadcloud, "/home/rickert/Petra_Top_xf.ptx").detach();
}

App::~App() {
    delete core_;
    delete pm_;
}
App* App::INSTANCE() {
    return _instance;
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
