#ifndef PREDICT_POINTS_HPP
#define PREDICT_POINTS_HPP

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "pitch_yaw_solve/kalmank.hpp"

class PrePoints
{
    public:
         PrePoints();
         std::vector<cv::Point2f> runPre();
         void initPara(const double a, const double w, const double phi, const int direction, const int even_pace, const double speed_input);
         void addData(std::vector<cv::Point2f> points_input, double time_input);
         void setSize();

         Eigen::MatrixXd a_omega_phi_direction;//解算出来的参数

         int if_even_pace = -1;//是否匀速
         double speed = -1;

         kalman kal;

    private:
        
        const double delta_t = 0.3;  //预测t0+delta_t后的位置 s    未测试！！！！！！！！！！！！！

        double img_width;//图像中矩形宽度
        double img_height;//图像中矩形高度
        double radius;//图像中半径

        std::vector<cv::Point2f> points_now; //r lt lb rb rt  (5)
        double time_now; //s
        std::vector<cv::Point2f> points_pre; // lt_new lb_new rb_new rt_new (4)
        double time_new; //s
        
};


#endif //PREDICT_POINTS_HPP