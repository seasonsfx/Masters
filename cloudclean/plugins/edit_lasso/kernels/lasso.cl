inline float4 proj(float16 mat, float4 point){

    float4 out = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
    point.w = 1;


    out.x += mat.s0 * point.x;
    out.y += mat.s1 * point.x;
    out.z += mat.s2 * point.x;
    out.w += mat.s3 * point.x;

    out.x += mat.s4 * point.y;
    out.y += mat.s5 * point.y;
    out.z += mat.s6 * point.y;
    out.w += mat.s7 * point.y;

    out.x += mat.s8 * point.z;
    out.y += mat.s9 * point.z;
    out.z += mat.sA * point.z;
    out.w += mat.sB * point.z;

    out.x += mat.sC;
    out.y += mat.sD;
    out.z += mat.sE;
    out.w += mat.sF;

    out.x/= out.w;
    out.y/= out.w;
    out.z/= out.w;
    out.w/= out.w;

    return out;
}

int rand(int value)
{
    const int a = 1103515245;
    const int c = 12345;

    return (a*value) + c;
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

float randomAngle(int* lastRandom)
{
    *lastRandom = rand(*lastRandom);
    return 2.0f*GPU_PI*(*lastRandom % 10000)/10000.0f;
}

float2 randomLineSegment(float2 origin, int* lastRandom)
{
    float angle = randomAngle(lastRandom);
    float2 endPoint;
    endPoint.x = 10000.0f*cos(angle) + origin.x;
    endPoint.y = 10000.0f*sin(angle) + origin.y;
    return endPoint;
}

bool pointInsidePolygon(__global float2* polygon, int n, float2 point)
{
    int lastRandom = rand(get_global_id(0));

    while(true)
    {
        float2 endPoint = randomLineSegment(point, &lastRandom);

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

__kernel void lasso (__global float4* points, __global int* source_indices, __global int* dest_indices, __global float2* polygon, int n, float16 mat)
{
    unsigned int idx = get_global_id(0);

    // Perspecive projection
    float4 vertex = proj(mat, points[source_indices[idx]]);

    bool in_lasso = pointInsidePolygon(polygon, n, vertex.xy);

    if(in_lasso){
        dest_indices[idx] = source_indices[idx];
        source_indices[idx] = -1;
        return;
    }
    else{
        dest_indices[idx] = -1;
        return;
    }

}

// Unit test kernels

__kernel void proj_test(float16 mat, __global float4* point, __global float4* out){
    unsigned int idx = get_global_id(0);

    float4 t;

    //out[idx] = point[idx];

    out[idx] = proj(mat, point[idx]);
}

__kernel void intersects_test(float2 origin, float2 dest, float2 p1, float2 p2, __global bool* out){
    unsigned int idx = get_global_id(0);
    out[idx] = intersects(origin, dest, p1, p2);
}

__kernel void pointInsidePolygon_test(__global float2* polygon, int n, float4 point, __global bool* out){
    unsigned int idx = get_global_id(0);
    out[idx] = pointInsidePolygon(polygon, n, point.xy);
}
