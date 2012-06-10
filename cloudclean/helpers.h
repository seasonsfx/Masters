#ifndef HELPERS_H
#define HELPERS_H

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

typedef struct {
double r;       // percent
double g;       // percent
double b;       // percent
} rgb;

typedef struct {
double h;       // angle in degrees
double s;       // percent
double v;       // percent
} hsv;

static hsv      rgb2hsv(rgb in);
static rgb      hsv2rgb(hsv in);

hsv rgb2hsv(rgb in)
{
hsv         out;
double      min, max, delta;

min = in.r < in.g ? in.r : in.g;
min = min  < in.b ? min  : in.b;

max = in.r > in.g ? in.r : in.g;
max = max  > in.b ? max  : in.b;

out.v = max;                                // v
delta = max - min;
if( max > 0.0 ) {
    out.s = (delta / max);                  // s
} else {
    // r = g = b = 0                        // s = 0, v is undefined
    out.s = 0.0;
    out.h = NAN;                            // its now undefined
    return out;
}
if( in.r >= max )                           // > is bogus, just keeps compilor happy
    out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
else
if( in.g >= max )
    out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
else
    out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

out.h *= 60.0;                              // degrees

if( out.h < 0.0 )
    out.h += 360.0;

return out;
}


rgb hsv2rgb(hsv in)
{
double      hh, p, q, t, ff;
long        i;
rgb         out;

if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
    if(isnan(in.h)) {   // in.h == NAN
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    // error - should never happen
    out.r = 0.0;
    out.g = 0.0;
    out.b = 0.0;
    return out;
}
hh = in.h;
if(hh >= 360.0) hh = 0.0;
hh /= 60.0;
i = (long)hh;
ff = hh - i;
p = in.v * (1.0 - in.s);
q = in.v * (1.0 - (in.s * ff));
t = in.v * (1.0 - (in.s * (1.0 - ff)));

switch(i) {
case 0:
    out.r = in.v;
    out.g = t;
    out.b = p;
    break;
case 1:
    out.r = q;
    out.g = in.v;
    out.b = p;
    break;
case 2:
    out.r = p;
    out.g = in.v;
    out.b = t;
    break;

case 3:
    out.r = p;
    out.g = q;
    out.b = in.v;
    break;
case 4:
    out.r = t;
    out.g = p;
    out.b = in.v;
    break;
case 5:
default:
    out.r = in.v;
    out.g = p;
    out.b = q;
    break;
}
return out;
}

#endif // HELPERS_H
