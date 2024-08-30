#ifndef KALMANK_HPP
#define KALMANK_HPP

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>


class kalman
{
public:
    Eigen::Matrix<double, 4, 1> X;        // x y omega phi 
    Eigen::Matrix<double, 4, 1> A;
    Eigen::Matrix<double, 4, 4> Jac;

    Eigen::Matrix<double, 4, 4> Q;        // 系统噪声
    Eigen::Matrix<double, 2, 2> R;        // 观测噪声
    Eigen::Matrix<double, 4, 4> P;        // 协方差矩阵
    Eigen::Matrix<double, 4, 2> K;        // kalman gain
    Eigen::Matrix<double, 2, 4> H;        // 转换矩阵

    double a, w;//pso解算参数

    int rotate_direction = 0;//旋转方向 1-逆时针 -1-顺时针 0-未知
    Eigen::Matrix<double, 2, 1> time_temp;
    Eigen::Matrix<double, 2, 1> measurements;
    int interation_times = 0;

    kalman();
    Eigen::Matrix<double, 2, 1> runPredict(Eigen::Matrix<double, 4, 1> updated_X, double t, double delta_t);
    void kalmaninit(Eigen::Matrix<double, 4, 1> initial_value);
    void set_a_w(double a_input, double w_input);
    void predict(double t, double dt);
    void update();
};

#endif // KALMANK_HPP
