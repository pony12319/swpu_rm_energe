#ifndef PSO_NODE_HPP
#define PSO_NODE_HPP

#include <vector>
#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>

#include "interfaces/msg/point.hpp"
#include "interfaces/msg/armor.hpp"
#include "interfaces/msg/para_calcu.hpp"

#include "pso.hpp"

namespace ParaCalcu
{
    class PSONode : public rclcpp::Node
    {
        public:
            PSONode(const rclcpp::NodeOptions &options);
            

        private:
            void msgCallback(const interfaces::msg::Armor::SharedPtr &armor);
            double makeAngleRegular(double angle);
            int judgeDirection();

            int if_even_space = -1; //-1-未知 0-非匀速 1-匀速

            //订阅装甲板消息
            rclcpp::Subscription<interfaces::msg::Armor>::SharedPtr armor_sub;
            //发布解算好的参数
            rclcpp::Publisher<interfaces::msg::ParaCalcu>::SharedPtr para_pub;

            //创建pso实例
            std::shared_ptr<pso> pso_calculator;

            //存放最后的解算结果
            Eigen::MatrixXd para_final;
            //存放角度数据判断旋转方向
            std::vector<double> angles_judge_direction;

    };




}


#endif //PSO_NODE_HPP

