#ifndef UTILITIES_EXPORT_H
    #define UTILITIES_EXPORT_H
    #if defined _WIN32 || defined __CYGWIN__
        #ifdef gui_EXPORTS
            #define UTIL_API Q_DECL_EXPORT
        #else
            #define UTIL_API Q_DECL_IMPORT
        #endif
    #else
        #define UTIL_API __attribute__ ((visibility ("default")))
    #endif
#endif
