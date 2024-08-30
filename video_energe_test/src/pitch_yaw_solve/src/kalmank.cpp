#include "pitch_yaw_solve/kalmank.hpp"

kalman::kalman()
{
    this->X = Eigen::Matrix<double, 4, 1>::Zero();
    this->A = Eigen::Matrix<double, 4, 1>::Zero();
    this->Jac = Eigen::Matrix<double, 4, 4>::Zero();

    //Q R均为仿真数据！！！！！！！！！！！！！！！！！！
    this->Q = Eigen::Matrix<double, 4, 4>::Zero();
    Q << 0.01, 0.001, 0, 0,
        0.001, 0.01, 0, 0,
        0, 0, 0.01, 0.001,
        0, 0, 0.001, 0.01;

    this->R = Eigen::Matrix<double, 2, 2>::Zero();
    R << 0.0001, 0,
        0, 0.0001;

    this->P = Eigen::Matrix<double, 4, 4>::Zero();
    this->K = Eigen::Matrix<double, 4, 2>::Zero();

    this->H = Eigen::Matrix<double, 2, 4>::Zero();
    this->H(0, 0) = 1;
    this->H(1, 1) = 1;

    this->time_temp = Eigen::Matrix<double, 2, 1>::Zero();
    this->measurements = Eigen::Matrix<double, 2, 1>::Zero();
}

void kalman::kalmaninit(Eigen::Matrix<double, 4, 1> initial_value)
{
    this->X = initial_value;
}

 void kalman::set_a_w(double a_input, double w_input)
 {
    a = a_input;
    w = w_input;
 }

// 返回预测的x y坐标，传入的X是update之后的
Eigen::Matrix<double, 2, 1> kalman::runPredict(Eigen::Matrix<double, 4, 1> updated_X, double t_input, double delta_t)
{
    double x, y, omega, phi, t;
    t = t_input;
    Eigen::Matrix<double, 4, 1> X_temp = updated_X;
    Eigen::Matrix<double, 4, 1> A_temp;

    double dt = 0.01;
    int times = std::floor(delta_t / dt);

    for (int i = 0; i < times; i++)
    {
        x = X_temp(0, 0);
        y = X_temp(1, 0);
        omega = X_temp(2, 0);
        phi = X_temp(3, 0);

        A_temp << x * std::cos(omega * delta_t) - y * std::sin(omega * delta_t),
            y * std::cos(omega * delta_t) + x * std::sin(omega * delta_t),
            rotate_direction * (a * std::sin(w * (t + dt) + phi) + 2.090 - a),
            phi;

        X_temp = A_temp;
    }

    Eigen::Matrix<double, 2, 1> result;
    result << X_temp(0, 0), X_temp(1, 0);
    return result;
}

// 迭代的预测步
void kalman::predict(double t, double dt)
{
    double x, y, omega, phi;
    x = X(0, 0);
    y = X(1, 0);
    omega = X(2, 0);
    phi = X(3, 0);

    // 构造预测矩阵
    A << x * std::cos(omega * dt) - y * std::sin(omega * dt),
        y * std::cos(omega * dt) + x * std::sin(omega * dt),
        rotate_direction * (a * std::sin(w * (t + dt) + phi) + 2.090 - a),
        phi;

    Jac << std::cos(omega * dt), -std::sin(omega * dt), -dt * y * std::cos(omega * dt) - dt * x * std::sin(omega * dt), 0,
        std::sin(omega * dt), std::cos(omega * dt), dt * x * std::cos(omega * dt) - dt * y * std::sin(omega * dt), 0,
        0, 0, 0, rotate_direction * (a * std::cos(w * (t + dt) + phi)),
        0, 0, 0, 1;

    X = A;
    P = Jac * P * Jac.transpose() + Q;
    // std::cout<<"predict finish"<<std::endl;
}

// 迭代的更新步
void kalman::update()
{
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    X = X + K * (measurements - H * X);
    P = (Eigen::MatrixXd::Identity(4, 4) - K * H) * P;
    interation_times += 1;
    // std::cout<<"update finish"<<std::endl;
}
