#ifndef HELPERS_H
#define HELPERS_H

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

// Helper function to get error string
// *********************************************************************
const char* oclErrorString(cl_int error);

void inline clError(const char * msg, cl_int err);

void inline  glError(const char * msg);

inline void proj(float* mat, float* point);

#endif // HELPERS_H
