inline float4 proj(float16 mat, float4 point){

    float4 p0 = (float4)(0.0f, 0.0f, 0.0f, 0.0f);

    p0.x += mat.s0 * point.x;
    p0.y += mat.s1 * point.x;
    p0.z += mat.s2 * point.x;
    p0.w += mat.s3 * point.x;

    p0.x += mat.s4 * point.y;
    p0.y += mat.s5 * point.y;
    p0.z += mat.s6 * point.y;
    p0.w += mat.s7 * point.y;

    p0.x += mat.s8 * point.z;
    p0.y += mat.s9 * point.z;
    p0.z += mat.sA * point.z;
    p0.w += mat.sB * point.z;

    p0.x += mat.sC;
    p0.y += mat.sD;
    p0.z += mat.sE;
    p0.w += mat.sF;

    p0.x/= p0.w;
    p0.y/= p0.w;
    p0.z/= p0.w;

    return p0;
}

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

__kernel void lasso (__global float4* points, __global int* source_indices, __global int* dest_indices, __global float2* lasso, int lasso_line_count, float16 mat)
{
    unsigned int idx = get_global_id(0);

    // Perspecive projection
    float4 vertex = proj(mat, points[idx]);

    float2 point = vertex.xy;

    if(pointInsidePolygon(lasso, lasso_line_count, point)){
        dest_indices[idx] = source_indices[idx];
        source_indices[idx] = 0;
    }
    else{
        dest_indices[idx] = 0;
    }
}


__kernel void proj_test(float16 mat, float4 point, __global float4* out){
    unsigned int idx = get_global_id(0);
    out[idx] = proj(mat, point);
}

__kernel void intersects_test(float2 origin, float2 dest, float2 p1, float2 p2, __global bool* out){
    unsigned int idx = get_global_id(0);
    out[idx] = intersects(origin, dest, p1, p2);
}

/*
__kernel void intersects_test(float2 origin, float2 dest, float2 p1, float2 p2, __global float2* out){
    unsigned int idx = get_global_id(0);
    float2[idx*2] = origin;//intersects(origin, dest, p1, p2);
    float2[idx*2+1] = dest;
}
*/
