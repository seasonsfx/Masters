#ifndef GLHEADERS_H_
#define GLHEADERS_H_
#ifdef _WIN32
#include <GL/glew.h>
#else
#define GL3_PROTOTYPES
#include <gl3.h>
#endif
#include <GL/glu.h>
#include <QDebug>
#include <QString>

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

#define ERR_MSG(err) QString(QString(err) + QString(" @ Line ") + QString::number(__LINE__) +  QString(" of ") +  QString(__FILE__))

class CheckSucc{
 public:
    CheckSucc(QString msg){
        succ = true;
        msg_ = msg;
    }

    CheckSucc operator = (bool rhs){
        succ = rhs;
        if(!rhs)
            qDebug() << msg_;
        return *this;
    }

    CheckSucc operator = (int rhs){
        if(rhs == -1){
            succ = false;
            qDebug() << msg_;
        }
        return *this;
    }

    bool operator == (bool rhs){
        return this->succ == rhs;
    }


 private:
    bool succ;
    QString msg_;
};

#define IF_FAIL(msg) CheckSucc(QString(QString(msg) + QString(" @ Line ") + QString::number(__LINE__) +  QString(" of ") +  QString(__FILE__)))

#endif  // GLHEADERS_H_
