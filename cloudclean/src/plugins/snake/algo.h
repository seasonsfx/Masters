#ifndef ALGO_H
#define ALGO_H

#include <vector>
#include <memory>
#include <Eigen/Dense>

bool snake_iteration(std::shared_ptr<const std::vector<float> > img,
               int w,
               int h,
               std::vector<Eigen::Vector2f> points,
               int win_w,
               int win_h,
               float alpha,
               float beta,
               float gamma);

#endif  // ALGO_H
