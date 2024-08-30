#include "pitch_yaw_solve/pitch_yaw_solve_node.hpp"

namespace PYSolve
{
    PYSNode::PYSNode(const rclcpp::NodeOptions &options) : Node("py_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "py_node start!");

        //这里的相机参数随便给的！!!!!!!!!!!!!!!!
        std::array<double, 9> camera_matrix = {{
            1330.768140, 0.000000, 650.176426,
            0.000000, 1331.610058, 525.164410,
            0.000000, 0.000000, 1.000000,
        }};
        std::vector<double> distortion_coefficients = {-0.073677, 0.207124,-0.000625, -0.001595, 0.000000};

        this->pre_points = std::make_shared<PrePoints>();
        this->pnp_solver = std::make_shared<PnPSolver>(camera_matrix, distortion_coefficients);

        //坐标订阅者
        this->armor_sub = this->create_subscription<interfaces::msg::Armor>("/energe_detect/armor", rclcpp::SensorDataQoS(),
        [this](interfaces::msg::Armor::SharedPtr armor)
        {
            this->armorCallback(armor);
        });

        //参数订阅者
        this->para_sub = this->create_subscription<interfaces::msg::ParaCalcu>("energe_detect/para", rclcpp::SensorDataQoS(), 
        [this](interfaces::msg::ParaCalcu::SharedPtr para)
        {
            this->paraCallback(para);
        });
    }

    void PYSNode::paraCallback(const interfaces::msg::ParaCalcu::SharedPtr &para)
    {
        RCLCPP_INFO(this->get_logger(), "para receive success!");

        int if_even_pace_;
        int direction;
        double a, omega, phi, speed;
        if (para->even_pace.data == 1)//匀速
        {
            if_even_pace_ = 1;
            direction = para->rotate_direction.data;

            a = 0;
            omega = 0;
            phi = 0;

            speed = para->speed.data;
        }
        else if(para->even_pace.data == 0)//非匀速
        {
            if_even_pace_ = 0;
            direction = para->rotate_direction.data;

            a = para->a.data;
            omega = para->omega.data;
            phi = para->phi.data;

            speed = 0;
        }
        
        this->pre_points->initPara(a, omega, phi, direction, if_even_pace_, speed);
        // std::cout<<"initpara success"<<std::endl;
    }

    void PYSNode::armorCallback(const interfaces::msg::Armor::SharedPtr &armor)
    {
        if(pre_points->a_omega_phi_direction(0, 0) != -1)//成功订阅参数之后才开始预测
        {
            std::vector<cv::Point2f> points_input;
            cv::Point2f r, lt, lb, rb, rt;
            double time_temp = armor->header.stamp.sec + armor->header.stamp.nanosec/1000000000.0; //转换为s
            r.x = armor->sign_r.x.data;
            r.y = armor->sign_r.y.data;
            points_input.push_back(r);

            lt.x = armor->p_lt.x.data;
            lt.y = armor->p_lt.y.data;
            points_input.push_back(lt);

            lb.x = armor->p_lb.x.data;
            lb.y = armor->p_lb.y.data;
            points_input.push_back(lb);

            rb.x = armor->p_rb.x.data;
            rb.y = armor->p_rb.y.data;
            points_input.push_back(rb);

            rt.x = armor->p_rt.x.data;
            rt.y = armor->p_rt.y.data;
            points_input.push_back(rt);

            this->pre_points->addData(points_input, time_temp);
            // std::cout<<"runPre"<<std::endl;
            std::vector<cv::Point2f> results = this->pre_points->runPre();// lt lb rb rt

            if(results.size() == 4)
            {
                Armor armor_results(results[0], results[1], results[2], results[3]);

                auto py = pnp_solver->solvePnP(armor_results);

                double yaw_result = py.at<double>(0);
                double pitch_result = py.at<double>(1);

                std::cout<<"pitch: "<<pitch_result<<" yaw: "<<yaw_result<<std::endl;

                // pypub(pitch_result, yaw_result);

            }


        }

    }

    // void PYSNode::pypub(double pitch, double yaw)
    // {
    //     std::cout<<"finish"<<pitch<<" "<<yaw<<std::endl;
    //     /*
    //     我删除了ros2发布pitch yaw的部分
    //     在这里编写你和电控的发布逻辑
    //     */
    // }

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(PYSolve::PYSNode)