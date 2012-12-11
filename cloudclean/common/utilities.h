#ifndef HELPERS_H
#define HELPERS_H

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#include "../cloudclean/glarea.h"
#include <GL/glu.h>

// Helper function to get error string
// *********************************************************************
const char* oclErrorString(cl_int error);

void clError(const char * msg, cl_int err);

void glError(const char * msg);

void proj(float* mat, float* point);

#endif // HELPERS_H
