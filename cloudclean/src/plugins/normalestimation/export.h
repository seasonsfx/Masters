#ifndef PLUGINS_NORMALESTIMATION_EXPORT_H
    #define PLUGINS_NORMALESTIMATION_EXPORT_H
    #if defined _WIN32 || defined __CYGWIN__
        #ifdef gui_EXPORTS
            #define NE_API Q_DECL_EXPORT
        #else
            #define NE_API Q_DECL_IMPORT
        #endif
    #else
        #define NE_API __attribute__ ((visibility ("default")))
    #endif
#endif  // PLUGINS_NORMALESTIMATION_EXPORT_H
