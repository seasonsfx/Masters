#ifndef HELPERS_H
#define HELPERS_H

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#include "gl_global.h"
#include "cloudclean_global.h"

// Helper function to get error string
// *********************************************************************
DLLSPEC const char* oclErrorString(cl_int error);

DLLSPEC void clError(const char * msg, cl_int err);

DLLSPEC void glError(const char * msg);

DLLSPEC void proj(float* mat, float* point);

#endif // HELPERS_H
