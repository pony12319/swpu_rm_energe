#ifndef PSO_HPP
#define PSO_HPP

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <iostream>
#include <cmath>

class pso
{
    public:
        pso();
        void addData(double time, double angle);
        double convert(double rad);
        bool ifJump(double angle_last, double angle_now);
        Eigen::VectorXd fit();
        Eigen::MatrixXd runpso();
        void initial();
        Eigen::Index minIndex(const Eigen::VectorXd data);
        void v_update();
        void x_update();

        //正反转
        int rotate_direction = 0; //1-逆时针 -1-顺时针 0-未知

        //参数
        const double E = 0.000001; //精度
        const int maxnum = 100;   //最大迭代次数
        const int narvs = 3;        //自变量个数 a omega phi
        const int particlesize = 500; //粒子个数
        const int c1 = 2; //个体学习因子
        const int c2 = 2; //群体学习因子
        double w = 0.6; //惯性因子
        const double v_max = 1.5; //最大飞行速度

        //收集到的时间-角度数据
        const size_t data_size = 26; //需要的数据量
        std::vector<double> m_times;
        std::vector<double> m_angles;

        //存放数据的容器
        Eigen::MatrixXd x;
        Eigen::MatrixXd v;
        Eigen::MatrixXd pbest;
        Eigen::VectorXd fitness_p;
        Eigen::MatrixXd gbest;
        double fitness_g = 0;
        cv::Mat img;
        std::vector<float> best;

};

#endif //PSO_HPP