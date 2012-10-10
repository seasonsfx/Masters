#include "utilities.h"
#include <GL/glu.h>
#include "QDebug"
#include <cstdlib>

// Helper function to get error string
// *********************************************************************
const char* oclErrorString(cl_int error)
{
    static const char* errorString[] = {
        "CL_SUCCESS",
        "CL_DEVICE_NOT_FOUND",
        "CL_DEVICE_NOT_AVAILABLE",
        "CL_COMPILER_NOT_AVAILABLE",
        "CL_MEM_OBJECT_ALLOCATION_FAILURE",
        "CL_OUT_OF_RESOURCES",
        "CL_OUT_OF_HOST_MEMORY",
        "CL_PROFILING_INFO_NOT_AVAILABLE",
        "CL_MEM_COPY_OVERLAP",
        "CL_IMAGE_FORMAT_MISMATCH",
        "CL_IMAGE_FORMAT_NOT_SUPPORTED",
        "CL_BUILD_PROGRAM_FAILURE",
        "CL_MAP_FAILURE",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "CL_INVALID_VALUE",
        "CL_INVALID_DEVICE_TYPE",
        "CL_INVALID_PLATFORM",
        "CL_INVALID_DEVICE",
        "CL_INVALID_CONTEXT",
        "CL_INVALID_QUEUE_PROPERTIES",
        "CL_INVALID_COMMAND_QUEUE",
        "CL_INVALID_HOST_PTR",
        "CL_INVALID_MEM_OBJECT",
        "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR",
        "CL_INVALID_IMAGE_SIZE",
        "CL_INVALID_SAMPLER",
        "CL_INVALID_BINARY",
        "CL_INVALID_BUILD_OPTIONS",
        "CL_INVALID_PROGRAM",
        "CL_INVALID_PROGRAM_EXECUTABLE",
        "CL_INVALID_KERNEL_NAME",
        "CL_INVALID_KERNEL_DEFINITION",
        "CL_INVALID_KERNEL",
        "CL_INVALID_ARG_INDEX",
        "CL_INVALID_ARG_VALUE",
        "CL_INVALID_ARG_SIZE",
        "CL_INVALID_KERNEL_ARGS",
        "CL_INVALID_WORK_DIMENSION",
        "CL_INVALID_WORK_GROUP_SIZE",
        "CL_INVALID_WORK_ITEM_SIZE",
        "CL_INVALID_GLOBAL_OFFSET",
        "CL_INVALID_EVENT_WAIT_LIST",
        "CL_INVALID_EVENT",
        "CL_INVALID_OPERATION",
        "CL_INVALID_GL_OBJECT",
        "CL_INVALID_BUFFER_SIZE",
        "CL_INVALID_MIP_LEVEL",
        "CL_INVALID_GLOBAL_WORK_SIZE",
    };

    const int errorCount = sizeof(errorString) / sizeof(errorString[0]);

    const int index = -error;

    return (index >= 0 && index < errorCount) ? errorString[index] : "";

}

void inline clError(const char * msg, cl_int err){
    if(err)
        qWarning() << msg << oclErrorString(err);
}


void inline  glError(const char * msg){
    int err = glGetError();
    if(err){
        printf("%s : %s\n", msg , gluErrorString(err));
    }
}


inline void proj(float* mat, float* point){

    float p0[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    p0[0] += mat[0] * point[0];
    p0[1] += mat[1] * point[0];
    p0[2] += mat[2] * point[0];
    p0[3] += mat[3] * point[0];

    p0[0] += mat[4] * point[1];
    p0[1] += mat[5] * point[1];
    p0[2] += mat[6] * point[1];
    p0[3] += mat[7] * point[1];

    p0[0] += mat[8] * point[2];
    p0[1] += mat[9] * point[2];
    p0[2] += mat[10] * point[2];
    p0[3] += mat[11] * point[2];

    p0[0] += mat[12];
    p0[1] += mat[13];
    p0[2] += mat[14];
    p0[3] += mat[15];

    point[0] = p0[0]/ p0[3];
    point[1] = p0[1]/ p0[3];
    point[2] = p0[2]/ p0[3];
    point[3] = p0[3]/ p0[3];
}

