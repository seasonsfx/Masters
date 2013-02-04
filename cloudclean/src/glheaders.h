#ifdef _WIN32
#include <GL/glew.h>
#else
#define GL3_PROTOTYPES
#include <gl3.h>
#endif
#include <GL/glu.h>

static GLenum gl_err = 0;
#define CE() gl_err = glGetError();\
if (gl_err != GL_NO_ERROR) {\
    const char* err_str = reinterpret_cast<const char *>(gluErrorString(gl_err));\
    QString errString(err_str);\
    qDebug() << "GL Error:" << errString << "on line" << __LINE__ << "of" << __FILE__;\
    abort();\
}

#define RC(CODE) if(CODE == -1){\
    qDebug() << "Function call failed on line" << __LINE__ << "of" << __FILE__;\
    abort();}
