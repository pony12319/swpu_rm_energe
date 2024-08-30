#include "pitch_yaw_solve/predict_points.hpp"

PrePoints::PrePoints()
{
    a_omega_phi_direction = Eigen::MatrixXd(4, 1);
    a_omega_phi_direction(0, 0) = -1;
    a_omega_phi_direction(1, 0) = -1;
    a_omega_phi_direction(2, 0) = -1;
    a_omega_phi_direction(3, 0) = 0;
}

std::vector<cv::Point2f> PrePoints::runPre()
{
    points_pre.clear();//先清空

    time_new = time_now + delta_t;
    double theta_new = 0;

    if(if_even_pace == 0)//非匀速
    {
        // std::cout<<"uneven_pace"<<std::endl;
        double a = a_omega_phi_direction(0, 0);
        double w = a_omega_phi_direction(1, 0);
        double phi = a_omega_phi_direction(2, 0);

        //卡尔曼初始化
        if(kal.X.isZero())
        {
            Eigen::Matrix<double, 4, 1> x_input;// x y omega phi
            kal.set_a_w(a, w);
            x_input << (points_now[1].x + points_now[2].x + points_now[3].x + points_now[4].x)* 0.25,
                    (points_now[1].y + points_now[2].y + points_now[3].y + points_now[4].y)* 0.25,
                    kal.rotate_direction*(a*std::sin(w*time_now+phi) + 2.090 - a),
                    phi;
            kal.kalmaninit(x_input);
            kal.rotate_direction = a_omega_phi_direction(3,0);
            kal.time_temp(0, 0) = time_now;
            // std::cout<<"init kal卡尔曼"<<std::endl;
        }
        else//持续迭代
        {
            kal.time_temp(1, 0) = time_now;
            auto dt = kal.time_temp(1, 0) - kal.time_temp(0, 0);
            kal.predict(time_now, dt);
            kal.measurements << 
                            (points_now[1].x + points_now[2].x + points_now[3].x + points_now[4].x)* 0.25, 
                            (points_now[1].y + points_now[2].y + points_now[3].y + points_now[4].y)* 0.25;
            kal.update();
            kal.time_temp(0, 0) = kal.time_temp(1, 0);
            // std::cout<<"interation finish"<<std::endl;
        }

        if(kal.interation_times >= 50) //设定一个迭代次数使Q R收敛 次数未测试！！！！！！！！！！！！！！！
        {
            auto xy = kal.runPredict(kal.X, time_now, delta_t);
            cv::Point2f center;
            center.x = xy(0, 0);
            center.y = xy(1, 0);

            theta_new = std::atan2(center.y-points_now[0].y, center.x-points_now[0].x);

            //后处理 解算预测的lt lb rb rt
            cv::Point2f lt;//( 0.5*h,  0.5*w) relative 从x轴开始 以cx, cy为原点
            cv::Point2f lb;//(-0.5*h,  0.5*w)
            cv::Point2f rb;//(-0.5*h, -0.5*w)
            cv::Point2f rt;//( 0.5*h, -0.5*w)

            //absolute_x = relative_x*cos(theta) - relative_y*sin(theta) + cx
            //absolute_y = relative_x*sin(theta) - relative_y*cos(theta) + cy
            lt.x = 0.5*img_height*std::cos(theta_new) - 0.5*img_width*std::sin(theta_new) + center.x;
            lt.y = 0.5*img_height*std::sin(theta_new) + 0.5*img_width*std::cos(theta_new) + center.y;
            points_pre.push_back(lt);

            lb.x = -0.5*img_height*std::cos(theta_new) - 0.5*img_width*std::sin(theta_new) + center.x;
            lb.y = -0.5*img_height*std::sin(theta_new) + 0.5*img_width*std::cos(theta_new) + center.y;
            points_pre.push_back(lb);

            rb.x = -0.5*img_height*std::cos(theta_new) + 0.5*img_width*std::sin(theta_new) + center.x;
            rb.y = -0.5*img_height*std::sin(theta_new) - 0.5*img_width*std::cos(theta_new) + center.y;
            points_pre.push_back(rb);

            rt.x = 0.5*img_height*std::cos(theta_new) + 0.5*img_width*std::sin(theta_new) + center.x;
            rt.y = 0.5*img_height*std::sin(theta_new) - 0.5*img_width*std::cos(theta_new) + center.y;
            points_pre.push_back(rt);
        }
    }
    else if(if_even_pace == 1)//匀速 匀速不需要用到卡尔曼
    {
        theta_new = kal.rotate_direction * speed * time_new;
        cv::Point2f center;
        center.x = radius * std::cos(theta_new);
        center.y = radius * std::sin(theta_new);

        //后处理 解算预测的lt lb rb rt
        cv::Point2f lt;//( 0.5*h,  0.5*w) relative 从x轴开始 以cx, cy为原点
        cv::Point2f lb;//(-0.5*h,  0.5*w)
        cv::Point2f rb;//(-0.5*h, -0.5*w)
        cv::Point2f rt;//( 0.5*h, -0.5*w)

        //absolute_x = relative_x*cos(theta) - relative_y*sin(theta) + cx
        //absolute_y = relative_x*sin(theta) - relative_y*cos(theta) + cy
        lt.x = 0.5*img_height*std::cos(theta_new) - 0.5*img_width*std::sin(theta_new) + center.x;
        lt.y = 0.5*img_height*std::sin(theta_new) + 0.5*img_width*std::cos(theta_new) + center.y;
        points_pre.push_back(lt);

        lb.x = -0.5*img_height*std::cos(theta_new) - 0.5*img_width*std::sin(theta_new) + center.x;
        lb.y = -0.5*img_height*std::sin(theta_new) + 0.5*img_width*std::cos(theta_new) + center.y;
        points_pre.push_back(lb);

        rb.x = -0.5*img_height*std::cos(theta_new) + 0.5*img_width*std::sin(theta_new) + center.x;
        rb.y = -0.5*img_height*std::sin(theta_new) - 0.5*img_width*std::cos(theta_new) + center.y;
        points_pre.push_back(rb);

        rt.x = 0.5*img_height*std::cos(theta_new) + 0.5*img_width*std::sin(theta_new) + center.x;
        rt.y = 0.5*img_height*std::sin(theta_new) - 0.5*img_width*std::cos(theta_new) + center.y;
        points_pre.push_back(rt);
    }
    
    return points_pre;

}

void PrePoints::initPara(const double a, const double w, const double phi, const int direction, const int even_pace, const double speed_input)
{
    if_even_pace = even_pace;

    a_omega_phi_direction(0, 0) = a;
    a_omega_phi_direction(1, 0) = w;
    a_omega_phi_direction(2, 0) = phi;
    a_omega_phi_direction(3, 0) = direction;

    speed = speed_input;
    kal.rotate_direction = direction;
}

void PrePoints::addData(std::vector<cv::Point2f> points_input, double time_input)
{
    points_now.clear();//先清空
    for(size_t i = 0; i < points_input.size(); i++)//五点 r lt lb rb rt
        points_now.push_back(points_input[i]);

    time_now = time_input;

    setSize();
}

void PrePoints::setSize()
{
    double w = std::sqrt(std::pow(points_now[1].x-points_now[4].x, 2) + std::pow(points_now[1].y-points_now[4].y, 2));
    double h = std::sqrt(std::pow(points_now[1].x-points_now[2].x, 2) + std::pow(points_now[1].y-points_now[2].y, 2));

    float cx = (points_now[1].x + points_now[2].x + points_now[3].x + points_now[4].x) * 0.25;
    float cy = (points_now[1].y + points_now[2].y + points_now[3].y + points_now[4].y) * 0.25;
    double r = std::sqrt(std::pow(points_now[0].x-cx, 2) + std::pow(points_now[1].y-cy, 2));

    img_width = w;
    img_height = h;
    radius = r;
}