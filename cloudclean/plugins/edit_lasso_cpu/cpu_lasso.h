#ifndef CPU_LASSO_H
#define CPU_LASSO_H

struct float2{
    float x, y;
};

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
    /*if(pointOnLineSegment(lineA, lineB, lineC) || pointOnLineSegment(lineA, lineB, lineD) ||
       pointOnLineSegment(lineC, lineD, lineA) || pointOnLineSegment(lineC, lineD, lineB))
        return true;
    */
    return false;
}

float randomAngle(int* lastRandom)
{
    *lastRandom = rand(*lastRandom);
    return 2.0f*M_PI*(*lastRandom % 10000)/10000.0f;
}

float2 randomLineSegment(float2 origin, int* lastRandom)
{
    float angle = randomAngle(lastRandom);
    float2 endPoint;
    endPoint.x = 10000.0f*cos(angle) + origin.x;
    endPoint.y = 10000.0f*sin(angle) + origin.y;
    return endPoint;
}

bool pointInsidePolygon(float2* polygon, int n, float2 point)
{
    int lastRandom = rand(1);

    while(true)
    {
        float2 endPoint = randomLineSegment(point, &lastRandom);

        for(int i = 0; i < n; ++i)
            if(pointOnLineSegment(point, endPoint, polygon[i]))
                continue;

        int hits = 0;

         //qDebug("CHECK!");

        for(int i = 0; i < n; ++i){

            //qDebug("POLY: (%f, %f) -- (%f, %f), (%f, %f) -- (%f, %f)", polygon[i].x, polygon[i].y, polygon[(i + 1) % n].x, polygon[(i + 1) % n].y, point.x, point.y, endPoint.x, endPoint.y);

            if(intersects(polygon[i], polygon[(i + 1) % n], point, endPoint)){
                ++hits;
                //qDebug("HIT %d!", hits);
            }
        }

        if(hits % 2 == 1){
            //qDebug("Point (%f, %f) is inside. Hits = %d", point.x, point.y, hits);
        }

        return (hits % 2 == 1);
    }
}


#endif // CPU_LASSO_H
