#include "para_cal/pso.hpp"

#include <chrono>

pso::pso()
{
    x = Eigen::MatrixXd(particlesize, 3);
    v = Eigen::MatrixXd(particlesize, 3);
    pbest = Eigen::MatrixXd(particlesize, 3);
    gbest = Eigen::MatrixXd(1, 3); //gbest只有一个
    fitness_p = Eigen::VectorXd(particlesize);
}

void pso::addData(double time, double angle)
{
    m_times.push_back(time);
    m_angles.push_back(angle);
}

//把算出的弧度转换为0-360
double pso::convert(double rad)
{
    //把rad转换到0-2*pi
    rad = fmod(rad, 2*CV_PI);
    while(rad > 2*CV_PI)
        rad = rad - 2*CV_PI;
    while(rad < 0.0)
        rad = rad + 2*CV_PI;
    
    return rad/CV_PI*180;
}

//判断是否发生扇叶跳变 1-yes 0-no
bool pso::ifJump(double angle_last, double angle_now)
{
    double diff = 0.0;
    if(rotate_direction == 1) //逆时针
    {
        diff = angle_now - angle_last;
        if (diff < 0)
            diff = 360 + diff;
    }
    else //顺时针
    {
        diff = angle_now - angle_last;
        if(diff > 0)
            diff = 360 - diff;
    }

    if (std::abs(diff) >= 65.00)//这是理论值，实际上还没测试！！！！！！！！！！！！1
        return true;
    else 
        return false;
}

//返回每个粒子对于所有角度的总差值
Eigen::VectorXd pso::fit()
{
    Eigen::VectorXd diff(particlesize);
    diff.setZero();
    // std::cout<<"times: "<<m_times.size()<<" angles: "<<m_angles.size()<<std::endl;
    for(int i = 0; i<particlesize; i++)
    {
        // auto now = std::chrono::system_clock::now();
        
        for(size_t j = 0; j<m_times.size() && j<m_angles.size(); j++)
        {
            diff[i] = diff[i] + (std::abs((m_angles[j] - convert(rotate_direction * (-x(i, 0)*std::cos(x(i, 1)*m_times[j] + x(i, 2))/x(i, 1) + (2.090-x(i, 0))*m_times[j])))));
        }
        // auto now_c = std::chrono::system_clock::to_time_t(now);
        // std::cout << "Current time is: " << std::ctime(&now_c) << std::endl;
        diff[i] = 1.0 * diff[i] / m_times.size();

        // std::cout<<"diff[i]"<<diff[i]<<std::endl;
    }

    return diff;
}

void pso::initial()
{
    std::random_device rd;
    std::mt19937::result_type seed = rd();
    std::mt19937 gen(seed);

    x.setZero();
    v.setZero();
    pbest.setZero();
    gbest.setZero();
    fitness_p.setZero();

    for (int i = 0; i < particlesize; i++)
    {
        std::uniform_real_distribution<> a_dis(0.780, 1.045);
        x(i, 0) = a_dis(gen);
        std::uniform_real_distribution<> oemga_dis(1.884, 2.000);
        x(i, 1) = oemga_dis(gen);
        std::uniform_real_distribution<> phi_dis(0.0, 2*CV_PI);
        x(i, 2) = phi_dis(gen);
        for (int j = 0; j < 3; j++)
        {
            std::uniform_real_distribution<> v_dis(-v_max, v_max);
            v(i, j) = v_dis(gen);
        }
    }
}


Eigen::Index pso::minIndex(Eigen::VectorXd data)
{
    Eigen::Index minIndex;
    data.minCoeff(&minIndex);
    //std::cout<<"minval: "<<min<<" minIndex: "<<minIndex<<std::endl;

    return minIndex;
}

void pso::v_update()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    for (int i = 0; i < v.rows(); i++)
    {
        for(int j = 0; j < v.cols(); j++)
        {
            v(i, j) = w*v(i, j) + c1*dis(gen)*(pbest(i, j) - x(i, j)) + c2*dis(gen)*(gbest(0, j) - x(i, j));
            if (v(i, j) < -v_max)
                v(i, j) = -v_max;
            if (v(i, j) > v_max)
                v(i, j) = v_max;
        }
    }
}

void pso::x_update()
{
    for(int i = 0; i < x.rows(); i++)
    {
        for (int j = 0; j < x.cols(); j++)
        {
            x(i, j) = x(i, j) + v(i, j);
        }
        if (x(i, 0) < 0.780)
            x(i, 0) = 0.780;
        if (x(i, 0) > 1.045)
            x(i, 0) = 1.045;
        if (x(i, 1) < 1.884)
            x(i, 1) = 1.884;
        if (x(i, 1) > 2.000)
            x(i, 1) = 2.000;
        if (x(i, 2) < 0.000)
            x(i, 2) = 0.000;
        if (x(i, 2) > 2*CV_PI)
            x(i, 2) = 2*CV_PI;
    }
}

//返回最优 a omega phi
Eigen::MatrixXd pso::runpso()
{
    // std::cout<<"step in pso"<<std::endl;
    initial();
    // std::cout<<"inital finish"<<std::endl;
    auto temp = fit();
    // std::cout<<"fit finish"<<std::endl;
    pbest = x;
    fitness_p = temp;
    auto min_index = minIndex(fitness_p);
    gbest(0, 0) = x(min_index, 0);
    gbest(0, 1) = x(min_index, 1);
    gbest(0, 2) = x(min_index, 2);
    // std::cout<<"gbest start: "<<" "<<gbest(0, 0)<<" "<<gbest(0, 1)<<" "<<gbest(0, 2)<<std::endl;
    fitness_g = fitness_p(min_index);
    
    for (int i = 0; i < maxnum; i++)
    {
        //w = (0.9 - 0.4) * i / maxnum;
        w = i*(0.9-0.1)/maxnum + 0.1;
        auto fitness = fit();
        for(int j = 0; j < particlesize; j++)
        {
            if (fitness(j) < fitness_p(j))
            {
                fitness_p(j) = fitness(j);
                pbest(j, 0) = x(j, 0);
                pbest(j, 1) = x(j, 1);
                pbest(j, 2) = x(j, 2);
            }
        }

        auto min_index = minIndex(fitness_p);
        //std::cout<<"minIndex: "<<min_index<<std::endl;
        if(fitness_p(min_index) < fitness_g)
        {

            fitness_g = fitness_p(min_index);   
            gbest(0, 0) = x(min_index, 0);
            gbest(0, 1) = x(min_index, 1);
            gbest(0, 2) = x(min_index, 2);
        }

        v_update();
        x_update();

        if(fitness_g < E)
            break;

    }
    //std::cout<<"finish runpso"<<std::endl;


    return gbest;
}