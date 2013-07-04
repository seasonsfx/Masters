#include "plugins/snake/algo.h"
#include <vector>
#include <memory>
#include <utilities/cv.h>
#include <Eigen/Dense>

/*F///////////////////////////////////////////////////////////////////////////////////////
//    Name:      icvSnake8uC1R
//    Purpose:
//    Context:
//    Parameters:
//               img - source image,
//               imgStep - its step in bytes,
//               roi - size of ROI,
//               points - pointer to snake points array
//               n - size of points array,
//               alpha - pointer to coefficient of continuity energy,
//               beta - pointer to coefficient of curvature energy,
//               gamma - pointer to coefficient of image energy,
//               coeffUsage - if CV_VALUE - alpha, beta, gamma point to single value
//                            if CV_MATAY - point to arrays
//               criteria - termination criteria.
//               scheme - image energy scheme
//                         if _CV_SNAKE_IMAGE - image intensity is energy
//                         if _CV_SNAKE_GRAD  - magnitude of gradient is energy
//    Returns:
//F*/


template <class T>
T _max(T a, T b) {
    return (a>b?a:b);
}

template <class T>
T _min(T a, T b) {
    return (a<b?a:b);
}

bool snake_iteration(std::shared_ptr<const std::vector<float> > img,
               int w,
               int h,
               std::vector<Eigen::Vector2f> points,
               int win_w,
               int win_h,
               float alpha,
               float beta,
               float gamma)
{
    int n = points.size();
    int neighbors = win_h * win_w;

    int centerx = win_w >> 1;
    int centery = win_h >> 1;

    int iteration = 0;
    bool converged = false;

    /* check bad arguments */
    assert(points.size() > 2);
    assert((h > 0) && (w > 0));
    assert((win_h > 0) && (win_h & 1));
    assert((win_w > 0) && (win_w & 1));

    //float invn = 1 / ((float) points.size());

    std::vector<float> Econt(neighbors);
    std::vector<float> Ecurv(neighbors);
    std::vector<float> Eimg(neighbors);
    std::vector<float> E(neighbors);

    while( !converged )
    {
        float ave_d = 0;
        int moved = 0;

        converged = false;
        iteration++;
        /* compute average distance */
        for(int i = 1; i < n; i++ )
        {
            int diffx = points[i - 1].x() - points[i].x();
            int diffy = points[i - 1].y() - points[i].y();

            ave_d += sqrt( (float) (diffx * diffx + diffy * diffy) );
        }
        ave_d += sqrt( (float) ((points[0].x() - points[n - 1].x()) *
                                  (points[0].x() - points[n - 1].x()) +
                                  (points[0].y() - points[n - 1].y()) * (points[0].y() - points[n - 1].y())));

        ave_d /= n;
        /* average distance computed */
        for(int i = 0; i < n; i++ )
        {
            /* Calculate Econt */
            float maxEcont = 0;
            float maxEcurv = 0;
            float maxEimg = 0;
            float minEcont = FLT_MAX;
            float minEcurv = FLT_MAX;
            float minEimg = FLT_MAX;
            float Emin = FLT_MAX;

            int offsetx = 0;
            int offsety = 0;
            float tmp;

            /* compute bounds */
            int left = _min( int(points[i].x()), win_w >> 1 );
            int right = _min( w - 1 - int(points[i].x()), win_w >> 1 );
            int upper = _min( int(points[i].y()), win_h >> 1 );
            int bottom = _min( h - 1 - int(points[i].y()), win_h >> 1 );

            maxEcont = 0;
            minEcont = FLT_MAX;
            for(int j = -upper; j <= bottom; j++ )
            {
                for(int k = -left; k <= right; k++ )
                {
                    int diffx, diffy;

                    if( i == 0 )
                    {
                        diffx = points[n - 1].x() - (points[i].x() + k);
                        diffy = points[n - 1].y() - (points[i].y() + j);
                    }
                    else
                    {
                        diffx = points[i - 1].x() - (points[i].x() + k);
                        diffy = points[i - 1].y() - (points[i].y() + j);
                    }

                    float energy = (float) fabs(ave_d - sqrt( (float) (diffx * diffx + diffy * diffy) ));

                    Econt[(j + centery) * win_w + k + centerx] = energy;

                    maxEcont = _max( maxEcont, energy );
                    minEcont = _min( minEcont, energy );
                }
            }
            tmp = maxEcont - minEcont;
            tmp = (tmp == 0) ? 0 : (1 / tmp);
            for(int k = 0; k < neighbors; k++ )
            {
                Econt[k] = (Econt[k] - minEcont) * tmp;
            }

            /*  Calculate Ecurv */
            maxEcurv = 0;
            minEcurv = FLT_MAX;
            for(int j = -upper; j <= bottom; j++ )
            {
                for(int k = -left; k <= right; k++ )
                {
                    int tx, ty;
                    float energy;

                    if(i == 0 )
                    {
                        tx = points[n - 1].x() - 2 * (points[i].x() + k) + points[i + 1].x();
                        ty = points[n - 1].y() - 2 * (points[i].y() + j) + points[i + 1].y();
                    }
                    else if( i == n - 1 )
                    {
                        tx = points[i - 1].x() - 2 * (points[i].x() + k) + points[0].x();
                        ty = points[i - 1].y() - 2 * (points[i].y() + j) + points[0].y();
                    }
                    else
                    {
                        tx = points[i - 1].x() - 2 * (points[i].x() + k) + points[i + 1].x();
                        ty = points[i - 1].y() - 2 * (points[i].y() + j) + points[i + 1].y();
                    }
                    Ecurv[(j + centery) * win_w + k + centerx] = energy =
                        (float) (tx * tx + ty * ty);
                    maxEcurv = _max( maxEcurv, energy );
                    minEcurv = _min( minEcurv, energy );
                }
            }
            tmp = maxEcurv - minEcurv;
            tmp = (tmp == 0) ? 0 : (1 / tmp);
            for(int k = 0; k < neighbors; k++ )
            {
                Ecurv[k] = (Ecurv[k] - minEcurv) * tmp;
            }

            /* Calculate Eimg */
            for(int j = -upper; j <= bottom; j++ )
            {
                for(int k = -left; k <= right; k++ )
                {
                    float energy = (*img)[(points[i].y() + j) * w + points[i].x() + k];

                    Eimg[(j + centery) * win_w + k + centerx] = energy;

                    maxEimg = _max( maxEimg, energy );
                    minEimg = _min( minEimg, energy );
                }
            }

            tmp = (maxEimg - minEimg);
            tmp = (tmp == 0) ? 0 : (1 / tmp);

            for(int k = 0; k < neighbors; k++ )
            {
                Eimg[k] = (minEimg - Eimg[k]) * tmp;
            }

            /* Find Minimize point in the neighbors */
            for(int k = 0; k < neighbors; k++ )
            {
                E[k] = alpha * Econt[k] + beta * Ecurv[k] + gamma * Eimg[k];
            }
            Emin = FLT_MAX;
            for(int j = -upper; j <= bottom; j++ )
            {
                for(int k = -left; k <= right; k++ )
                {

                    if( E[(j + centery) * win_w + k + centerx] < Emin )
                    {
                        Emin = E[(j + centery) * win_w + k + centerx];
                        offsetx = k;
                        offsety = j;
                    }
                }
            }

            if( offsetx || offsety )
            {
                points[i].x() += offsetx;
                points[i].y() += offsety;
                moved++;
            }
        }
        converged = (moved == 0);
    }

}
