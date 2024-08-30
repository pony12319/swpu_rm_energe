#include "para_cal/pso_node.hpp"

namespace ParaCalcu
{
  
    PSONode::PSONode(const rclcpp::NodeOptions &options) : Node("para_cal_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "para_cal_node start!");

        para_final = Eigen::MatrixXd(1, 3);
        this->pso_calculator = std::make_shared<pso>();

        armor_sub = this->create_subscription<interfaces::msg::Armor>("/energe_detect/armor", rclcpp::SensorDataQoS(),
        [this](interfaces::msg::Armor::SharedPtr armor)
        {
            this->msgCallback(armor);
        });

        para_pub = this->create_publisher<interfaces::msg::ParaCalcu>("energe_detect/para", rclcpp::SensorDataQoS());

    }

    void PSONode::msgCallback(const interfaces::msg::Armor::SharedPtr &armor)
    {
        // std::cout<<"rec points"<<std::endl;
        double time_temp = armor->header.stamp.sec + armor->header.stamp.nanosec/1000000000.0; //转换为s

        //由装甲板四个点计算圆心
        double center_x = (armor->p_lt.x.data + armor->p_lb.x.data + armor->p_rb.x.data + armor->p_rt.x.data) * 0.25;
        double center_y = (armor->p_lt.y.data + armor->p_lb.y.data + armor->p_rb.y.data + armor->p_rt.y.data) * 0.25;

        double angle_temp = std::atan2(center_y - armor->sign_r.y.data, center_x - armor->sign_r.x.data); //解算角度在-pi~pi 需要计算到0-360之间
        if(angle_temp < 0)
            angle_temp = makeAngleRegular(angle_temp + 2*CV_PI);
        else 
            angle_temp = makeAngleRegular(angle_temp);

        // std::cout<<"angle: "<<angle_temp<<std::endl;
        //先判断旋转方向
        if(angles_judge_direction.size() < 3)
        {
            angles_judge_direction.push_back(angle_temp);
            // std::cout<<"size: "<<angles_judge_direction.size()<<std::endl;
            // std::cout<<"direction: "<<pso_calculator->rotate_direction<<std::endl;
        }
        else if(pso_calculator->rotate_direction == 0)
        {
            pso_calculator->rotate_direction = judgeDirection();
            std::cout<<"direction: "<<pso_calculator->rotate_direction<<std::endl;
        }

        //解算前提是旋转方向成功判断
        if(pso_calculator->rotate_direction != 0)
        {
            //存储的扇叶数量大于2时候，跳到下一个扇叶需重新收集数据
            if((pso_calculator->m_angles.size() >= 2) && pso_calculator->ifJump(pso_calculator->m_angles.back(), angle_temp))
            {
                pso_calculator->m_angles.clear();
                pso_calculator->m_times.clear();
            }
            else
                pso_calculator->addData(time_temp, angle_temp);

            //开始解算
            if(pso_calculator->m_angles.size() == pso_calculator->data_size)
            {
                // std::cout<<"start solve para"<<std::endl;
                interfaces::msg::ParaCalcu para;

                if(if_even_space == -1)//判断是否匀速
                {
                    // std::cout<<"even pace"<<std::endl;
                    double v1 = (pso_calculator->m_angles[1] - pso_calculator->m_angles[0]) / (pso_calculator->m_times[1] - pso_calculator->m_times[0]);
                    double v2 = (pso_calculator->m_angles[6] - pso_calculator->m_angles[5]) / (pso_calculator->m_times[6] - pso_calculator->m_times[5]);
                    double v3 = (pso_calculator->m_angles[21] - pso_calculator->m_angles[20]) / (pso_calculator->m_times[21] - pso_calculator->m_times[20]);
                    double v4 = (pso_calculator->m_angles[26] - pso_calculator->m_angles[25]) / (pso_calculator->m_times[26] - pso_calculator->m_times[25]);
                    // std::cout<<"3-1: "<<v3-v1<<" 4-2: "<<v4-v2<<std::endl;
                    if ((-0.01 < v3-v1 && v3-v1< 0.01) && (-0.01 < v4-v2 && v4-v2 < 0.01))
                    {
                        if_even_space = 1;
                        double speed = 0.25 * (v1 + v2 + v3 + v4);
                        std::cout<<"speed: "<<speed<<std::endl;
                        para.speed.data = speed;
                        para.even_pace.data = if_even_space;
                        para.rotate_direction.data = pso_calculator->rotate_direction;
                    }
                    else 
                    {
                        if_even_space = 0;
                    }
                }

                if(if_even_space != 1)//不是匀速
                {
                    // std::cout<<"start pso"<<std::endl;
                    if_even_space = 0;

                    pso_calculator->runpso();
                    para_final = pso_calculator->gbest;
                    // std::cout<<"pso finish"<<std::endl;

                    para.a.data = para_final(0, 0);
                    para.omega.data = para_final(0, 1);
                    para.phi.data = para_final(0, 2);
                    para.even_pace.data = if_even_space;
                    para.rotate_direction.data = pso_calculator->rotate_direction;
                    std::cout<<"a: "<<para.a.data<<" w: "<<para.omega.data<<" phi: "<<para.phi.data<<std::endl;
                }


                para_pub->publish(para);
                RCLCPP_INFO(this->get_logger(), "para calculate success!");
            }

        }

    }

    //把角度转换到0-360
    double PSONode::makeAngleRegular(double angle)
    {
        double angle_tmp;
        angle_tmp = fmod(angle/CV_PI*180, 360);

        return angle_tmp;
    }

    //angles_judge_direction中有三个值即可开始判断，判断旋转方向 1-逆时针 -1-顺时针 0-未知
    int PSONode::judgeDirection()
    {
        // std::cout<<"judge direction"<<std::endl;
        const double diff1 = angles_judge_direction[1] - angles_judge_direction[0];
        const double diff2 = angles_judge_direction[2] - angles_judge_direction[1];
        // std::cout<<"diff1: "<<diff1<<" diff2: "<<diff2<<std::endl;
        //跳变的情况排除
        if(diff1>0 && std::abs(diff1) < 50)//这个值我还没测试！！！！！！！！
        {
            // std::cout<<"第一次 逆时针"<<std::endl;
            return 1; //逆时针
        } 
        else if(diff1<0 && std::abs(diff1) < 50)
        {
            // std::cout<<"第一次 顺时针"<<std::endl;
            return -1; //顺时针
        } 
        else
        {   //如果第一第二个点出现在边界两边，则通过第三个点判断
            if(diff2>0 && std::abs(diff2) < 50)
            {
                // std::cout<<"第二次 逆时针"<<std::endl;
                return 1; //逆时针
            } 
            else if(diff2<0 && std::abs(diff2) < 50)
            {
                // std::cout<<"第二次 顺时针"<<std::endl;
                return -1; //顺时针
            }
            else
            {
                std::cerr<<" judge rotation error!"<<std::endl;
                return 0;
            }
        }
    }
}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ParaCalcu::PSONode)
