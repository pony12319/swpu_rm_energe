#include "pitch_yaw_solve/pnp_solver.hpp"

PnPSolver::PnPSolver(
    const std::array<double, 9> &camera_matrix, const std::vector<double> &dist_coeffs)
    : camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
      dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  float half_x = ArmorWidth / 2000.0;
  float half_y = ArmorHeight / 2000.0;

  // Start from bottom left in clockwise order
  armor_points_.emplace_back(cv::Point3f(-half_x, half_y, 0));
  armor_points_.emplace_back(cv::Point3f(-half_x, -half_y, 0));
  armor_points_.emplace_back(cv::Point3f(half_x, -half_y, 0));
  armor_points_.emplace_back(cv::Point3f(half_x, half_y, 0));

}

//返回解算后的yaw pitch 度
cv::Mat PnPSolver::solvePnP(const Armor &armor)
{

    std::vector<cv::Point2f> image_armor_points;

    // Fill in image points
    image_armor_points.emplace_back(armor.lb);
    image_armor_points.emplace_back(armor.lt);
    image_armor_points.emplace_back(armor.rt);
    image_armor_points.emplace_back(armor.rb);

    // Solve pnp
    std::vector<cv::Point3f> object_points;
    object_points = armor_points_;

    bool success = cv::solvePnP(
        object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
        cv::SOLVEPNP_IPPE);

    if (success)
    {
        // double x = tvec.at<double>(0);
        // double y = tvec.at<double>(1);
        // double z = tvec.at<double>(2);

        double yaw = std::atan2(tvec.at<double>(1), tvec.at<double>(0));
        if(yaw < 0)
            yaw += 2*CV_PI;
        yaw = yaw / CV_PI * 180;

        double pitch = std::atan2(tvec.at<double>(2), std::pow(tvec.at<double>(0)*tvec.at<double>(0) + tvec.at<double>(1)*tvec.at<double>(1), 0.5));
        if(pitch < 0)
            pitch += 2*CV_PI;
        pitch = pitch / CV_PI * 180;

        cv::Mat results = (cv::Mat_<double>(2, 1) << yaw, pitch);

        return results;
    }
    else
    {
        std::cerr<<" pnp error"<<std::endl;
        return (cv::Mat_<double>(2, 1) << -1, -1);
    }

}

