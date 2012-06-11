inline float dist(float2 a, float2 b){
    sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

__kernel void dim(__global float4* points, __global int* source_indices, __global int* dest_indices, __global float2* lasso, int lasso_line_count, float16 mat)
{
    unsigned int idx = get_global_id(0);

    // Perspecive projection
    float4 point = (float4)(0.0f, 0.0f, 0.0f, 0.0f);

    point.x += mat.s0 * points[idx].x;
    point.y += mat.s1 * points[idx].x;
    point.z += mat.s2 * points[idx].x;
    point.w += mat.s3 * points[idx].x;

    point.x += mat.s4 * points[idx].y;
    point.y += mat.s5 * points[idx].y;
    point.z += mat.s6 * points[idx].y;
    point.w += mat.s7 * points[idx].y;

    point.x += mat.s8 * points[idx].z;
    point.y += mat.s9 * points[idx].z;
    point.z += mat.sA * points[idx].z;
    point.w += mat.sB * points[idx].z;

    point.x += mat.sC * points[idx].w;
    point.y += mat.sD * points[idx].w;
    point.z += mat.sE * points[idx].w;
    point.w += mat.sF * points[idx].w;

    point.x/= point.w;
    point.y/= point.w;
    point.z/= point.w;

    // 2d ray casting to deterine if point is in lasso

    int hits = 0;

    // generate line to cast - todo check vertices
    float m1 = 0.1f;
    float c1 = point.y/(point.x*m1);

    // lasso line
    float2 p1;
    float2 p2;
    float m2;
    float c2;

    float2 p3; //intersection point

    // for every lasso line
    for(int ii = 0; ii < lasso_line_count; ii++){
        int i = ii%(lasso_line_count-1); // Wrap around for last line
        p1 = lasso[i];
        p2 = lasso[i+1];
        m2 = (p1.y-p2.y)/(p1.x - p2.x);
        c2 = p1.y/(p1.x*m2);
        p3.x =  (c2 - c1) / (m1 - m2);
        // if x is NaN then its paralelle
        if(p3.x != p3.x)
            continue;
        p3.y = m2 * p3.x + c2;
        // is the intercept between two end points?
        if(dist(p1, p3) + dist(p2, p3) < dist(p1, p2))
            hits++;
    }


    // if hits are even then the point is inside
    if(hits%2==0){
        dest_indices[idx] = source_indices[idx];
        source_indices[idx] = 0;
    }
    else{
        dest_indices[idx] = 0;
    }
}
