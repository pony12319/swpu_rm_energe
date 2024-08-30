#ifndef PNP_SOLVER_HPP_
#define PNP_SOLVER_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <eigen3/Eigen/Dense>

#include <array>
#include <vector>

struct Armor
{
    Armor() = default;
    Armor(cv::Point2f &lt_input, cv::Point2f &lb_input, cv::Point2f &rb_input, cv::Point2f &rt_input)
    {
      lt = lt_input;
      lb = lb_input;
      rb = rb_input;
      rt = rt_input;
      center.x = (lt.x + lb.x + rb.x + rt.x) * 0.25;
      center.y = (lt.y + lb.y + rb.y + rt.y) * 0.25;
    }
    void setSize();

    cv::Point2f lt, lb, rb, rt;
    cv::Point2f center;

};

class PnPSolver
{
public:
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  cv::Mat solvePnP(const Armor & armor);

  cv::Mat rvec, tvec;

private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  const double ArmorWidth = 320 ; // mm //135 230
  const double ArmorHeight = 70; //映射平行

  // Four vertices of armor in 3d
  std::vector<cv::Point3f> armor_points_;

};

#endif  //PNP_SOLVER_HPP_
