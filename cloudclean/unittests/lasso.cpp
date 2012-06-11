#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>

#include <CL/cl.h>
//#include "../helpers.h"

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
    std::ifstream file("../lasso.cl");
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
    cl_kernel kernel = clCreateKernel(program, "dist", &err);
        if (!kernel || err != CL_SUCCESS) {
        std::cerr << "Error: Failed to create compute kernel!" << std::endl;
        exit(1);
    }

    const size_t worksize = 1;

    cl_mem out=clCreateBuffer(context, CL_MEM_WRITE_ONLY, worksize, NULL, &result);
    float p1[2] = {0.0f, 0.0f};
    float p2[2] = {0.0f, 1.0f};

    result = clSetKernelArg(kernel, 0, sizeof(float)*2, (void*)&p1);
    result = clSetKernelArg(kernel, 0, sizeof(float)*2, (void*)&p2);

    result=clEnqueueNDRangeKernel(cmd_queue, kernel, 1, NULL, &worksize, &worksize, 0, NULL, NULL);

    float out_val[1];
    result=clEnqueueReadBuffer(cmd_queue, out, CL_FALSE, 0, worksize, out_val, 0, NULL, NULL);

    printf("Result: &f", *out_val);

    result=clFinish(cmd_queue);

}


// Allocate memory for the kernel to work with
//cl_mem mem1, mem2;
//mem1=clCreateBuffer(context, CL_MEM_READ_ONLY, worksize, NULL, &result);
//mem2=clCreateBuffer(context, CL_MEM_WRITE_ONLY, worksize, NULL, &result);


// Send input data to OpenCL (async, don't alter the buffer!)
//result=clEnqueueWriteBuffer(cmd_queue, mem1, CL_FALSE, 0, worksize, buf, 0, NULL, NULL);
// Perform the operation
//result=clEnqueueNDRangeKernel(cmd_queue, k_rot13, 1, NULL, &worksize, &worksize, 0, NULL, NULL);
// Read the result back into buf2
//result=clEnqueueReadBuffer(cmd_queue, mem2, CL_FALSE, 0, worksize, buf2, 0, NULL, NULL);
// Await completion of all the above
