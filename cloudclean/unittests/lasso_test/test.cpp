#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <CL/cl.h>

#define __global 

//#include "../helpers.h"

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


class float2
{
    public:
        float x, y;
};

int rand(){
    return 699;
}

float cross2D(float2 lineA, float2 lineB, float2 other)
{
    float2 dA, dB;
    dA.x = lineA.x - other.x; dA.y = lineA.y - other.y;
    dB.x = lineB.x - other.x; dB.y = lineB.y - other.y;

    return dA.x*dB.y - dA.y*dB.x;
}

int side(float a)
{
    if(a < -1e-6)
        return -1;
    if(a > 1e-6)
        return 1;
    return 0;
}

bool oppositeSides(float2 lineA, float2 lineB, float2 pointC, float2 pointD)
{
    float crossC = cross2D(lineA, lineB, pointC);
    float crossD = cross2D(lineA, lineB, pointD);

    int sideC = side(crossC);
    int sideD = side(crossD);

    return sideC != sideD;
}

float pointDistance(float2 pointA, float2 pointB)
{
    float dx = pointA.x - pointB.x;
    float dy = pointA.y - pointB.y;

    return sqrt(dx*dx + dy*dy);
}

bool pointOnLineSegment(float2 lineA, float2 lineB, float2 pointC)
{
    float lineLength = pointDistance(lineA, lineB);
    float viaPoint = pointDistance(lineA, pointC) + pointDistance(lineB, pointC);

    return fabs(viaPoint - lineLength) < 1e-6;
}

bool intersects(float2 lineA, float2 lineB, float2 lineC, float2 lineD)
{
    if(oppositeSides(lineA, lineB, lineC, lineD) && oppositeSides(lineC, lineD, lineA, lineB))
        return true; /// Lines intersect in obvious manner

    /// Line segments either don't intersect or are parallel
    if(pointOnLineSegment(lineA, lineB, lineC) || pointOnLineSegment(lineA, lineB, lineD) ||
       pointOnLineSegment(lineC, lineD, lineA) || pointOnLineSegment(lineC, lineD, lineB))
        return true;

    return false;
}

float randomAngle()
{
    return 2.0*M_PI*(rand() % 10000)/10000.0;
}

float2 randomLineSegment(float2 origin)
{
    float angle = randomAngle();
    float2 endPoint;
    endPoint.x = 10.0*cos(angle) + origin.x;
    endPoint.y = 10.0*sin(angle) + origin.y;
    return endPoint;
}

bool pointInsidePolygon(__global float2* polygon, int n, float2 point)
{
    while(true)
    {
        float2 endPoint = randomLineSegment(point);

        for(int i = 0; i < n; ++i)
            if(pointOnLineSegment(point, endPoint, polygon[i]))
                continue;

        int hits = 0;

        for(int i = 0; i < n; ++i)
            if(intersects(polygon[i], polygon[(i + 1) % n], point, endPoint))
                ++hits;

        return (hits % 2 == 1);
    }
}

float2 f2(float* a){
    float2 b;
    b.x = a[0];
    b.y = a[1];
    return b;
}

////////////////////////////////////////////////


int main() {

    cl_int result;
    cl_platform_id platform;
    cl_device_id device;
    cl_uint platforms, devices;

    // Fetch the Platform and Device IDs; we only want one.
    result=clGetPlatformIDs(1, &platform, &platforms);
    result=clGetDeviceIDs(platform, CL_DEVICE_TYPE_ALL, 1, &device, &devices);
    cl_context_properties properties[]={
        CL_CONTEXT_PLATFORM, (cl_context_properties)platform,
        0};
    // Note that nVidia's OpenCL requires the platform property
    cl_context context=clCreateContext(properties, 1, &device, NULL, NULL, &result);
    cl_command_queue cmd_queue = clCreateCommandQueue(context, device, 0, &result);


    // Setup OpenCL
    //if(result != CL_SUCCESS)
    //    std::cout << "CL object create failed:" << oclErrorString(result);

    // Load the program source into memory
    std::ifstream file("../../lasso.cl");
    std::string prog(std::istreambuf_iterator<char>(file), (std::istreambuf_iterator<char>()));
    file.close();
    const char* source = prog.c_str();
    const size_t kernelsize = prog.length()+1;
    int err;
    cl_program program = clCreateProgramWithSource(context, 1, (const char**) &source,
                                 &kernelsize, &err);
    //clError("Create program failed: ", err);

    // Build the program executable
    err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
    if (err != CL_SUCCESS) {

        //clError("Build failed: ", err);

        size_t len;
        char buffer[8096];

        std::cerr << "Error: Failed to build program executable!" << std::endl;
        clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG,
                           sizeof(buffer), buffer, &len);
        std::cerr << buffer << std::endl;
        exit(1);
    }

    // Create the compute kernel in the program
    cl_kernel kernel = clCreateKernel(program, "lasso", &err);
        if (!kernel || err != CL_SUCCESS) {
        std::cerr << "Error: Failed to create compute kernel!" << std::endl;
        exit(1);
    }

    const size_t worksize = 3;
    
    const int point_n = 3;
    float points[4*point_n] = {
            1.0f, 0.0f, 0.0f, 0.0f, 
            0.0f, 0.0f, 0.0f, 0.0f, 
            0.0f, 0.0f, 0.0f, 0.0f
    };
    
    int sidx[3] = {0,1,2};
    int didx[3] = {-11111,-11111,-11111};

    const int lasso_n = 4;
    float lasso[2*lasso_n] = {
                -0.5f, -0.5f,
                -0.5f, 0.5f,
                0.5f, 0.5f,
                0.5f, -0.5f,
    };

    float mat[16] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f, 
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
    };

    cl_mem cl_points = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(points), &points, &result);
    cl_mem cl_sidx = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, sizeof(sidx), &sidx, &result);
    cl_mem cl_didx = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, sizeof(didx), &didx, &result);
    cl_mem cl_lasso = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(lasso), &lasso, &result);
    

    if (result != CL_SUCCESS)
        printf("ERR 0: %s\n", oclErrorString(result));

    result = clSetKernelArg(kernel, 0, sizeof(cl_points), &cl_points);
    result = clSetKernelArg(kernel, 1, sizeof(cl_sidx), &cl_sidx);
    result = clSetKernelArg(kernel, 2, sizeof(cl_didx), &cl_didx);
    result = clSetKernelArg(kernel, 3, sizeof(cl_lasso), &cl_lasso);
    result = clSetKernelArg(kernel, 4, sizeof(lasso_n), &lasso_n);
    result = clSetKernelArg(kernel, 5, sizeof(mat), &mat);

    if (result != CL_SUCCESS)
        printf("ERR 0.1: %s\n", oclErrorString(result));

    result = clEnqueueNDRangeKernel(cmd_queue, kernel, 1, NULL, &worksize, &worksize, 0, NULL, NULL);

    if (result != CL_SUCCESS)
        printf("ERR 1: %s\n", oclErrorString(result));

    bool out_val[worksize];
    result=clEnqueueReadBuffer(cmd_queue, cl_didx, CL_FALSE, 0, sizeof(didx), didx, 0, NULL, NULL);
    result=clEnqueueReadBuffer(cmd_queue, cl_sidx, CL_FALSE, 0, sizeof(didx), sidx, 0, NULL, NULL);

    if (result != CL_SUCCESS)
        printf("ERR 2: %s\n", oclErrorString(result));

    result=clFinish(cmd_queue);

    if (result != CL_SUCCESS)
        printf("ERR 3: %s\n", oclErrorString(result));    

    printf("Results:\n");
    for(int i = 0; i < worksize; i++){
        printf("sidx: %d \n", sidx[i]);
        printf("didx: %d \n", didx[i]);
    }
    

}