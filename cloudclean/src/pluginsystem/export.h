#ifndef PLUGINSYSTEM_EXPORT_H
    #define PLUGINSYSTEM_EXPORT_H
    #if defined _WIN32 || defined __CYGWIN__
        #ifdef gui_EXPORTS
            #define PLUGINSYS_API Q_DECL_EXPORT
        #else
            #define PLUGINSYS_API Q_DECL_IMPORT
        #endif
    #else
        #define PLUGINSYS_API __attribute__ ((visibility ("default")))
    #endif
#endif
