inline float4 matMul(float16 mat, float4 point){

    float4 p0 = (float4)(0.0f, 0.0f, 0.0f, 0.0f);

    p0.x += mat.s0 * points[idx].x;
    p0.y += mat.s1 * points[idx].x;
    p0.z += mat.s2 * points[idx].x;
    p0.w += mat.s3 * points[idx].x;

    p0.x += mat.s4 * points[idx].y;
    p0.y += mat.s5 * points[idx].y;
    p0.z += mat.s6 * points[idx].y;
    p0.w += mat.s7 * points[idx].y;

    p0.x += mat.s8 * points[idx].z;
    p0.y += mat.s9 * points[idx].z;
    p0.z += mat.sA * points[idx].z;
    p0.w += mat.sB * points[idx].z;

    p0.x += mat.sC;
    p0.y += mat.sD;
    p0.z += mat.sE;
    p0.w += mat.sF;

    p0.x/= p0.w;
    p0.y/= p0.w;
    p0.z/= p0.w;

    return p0;
}

inline float dist(float2 a, float2 b){
    sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

inline bool rayIntercepts(float2 origin, float slope, float2 p1, float2 p2){
    float2 intercept;

    float ray_c = origin.y/(origin.x*slope);
    line_slope = (p1.y-p2.y)/(p1.x-p2.x);
    line_c = p1.y - (p1.x*line_slope);
    intercept.x =  (line_c - ray_c) / (slope - line_slope);

    // if x is NaN then its parallel
    if(intercept.x != intercept.x)
        return false;
    intercept.y = line_slope * intercept.x + line_c;

    // Check if point is in the right direction
    if(intercept.x < origin.x)
        return false;

    // is the intercept between two end points?
    if(dist(p1, intercept) + dist(p2, intercept) < dist(p1, p2))
        return true;
}

__kernel void lassok(__global float4* points, __global int* source_indices, __global int* dest_indices, __global float2* lasso, int lasso_line_count, float16 mat)
{
    unsigned int idx = get_global_id(0);

    // Perspecive projection
    float4 p0 = matMul(mat, points[idx]);

    // 2d ray casting to deterine if point is in lasso

    int hits = 0;

    // generate line to cast - todo check vertices
    float ray_slope = 0.0f;

    // for every lasso line
    for(int ii = 0; ii < lasso_line_count; ii++){
        int i = ii%(lasso_line_count-1); // Wrap around for last line
        if(rayIntercepts(p0, ray_slope, lasso[i], lasso[i+1]))
            hits++;
    }

    // if hits are even then the point is inside
    if(hits%2!=0){
        dest_indices[idx] = source_indices[idx];
        source_indices[idx] = 0;
    }
    else{
        dest_indices[idx] = 0;
    }
}
