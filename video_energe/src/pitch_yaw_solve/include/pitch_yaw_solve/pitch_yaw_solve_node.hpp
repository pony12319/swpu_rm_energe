#ifndef PNP_SOLVER_NODE_HPP_
#define PNP_SOLVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>

#include "interfaces/msg/point.hpp"
#include "interfaces/msg/armor.hpp"
#include "interfaces/msg/para_calcu.hpp"

#include "pnp_solver.hpp"
#include "predict_points.hpp"
#include "kalmank.hpp"

namespace PYSolve
{
    class PYSNode : public rclcpp::Node
    {
        public:
            PYSNode(const rclcpp::NodeOptions &options);
            

        private:
            void paraCallback(const interfaces::msg::ParaCalcu::SharedPtr & para);
            void armorCallback(const interfaces::msg::Armor::SharedPtr &armor);

            // void pypub(double pitch, double yaw);

            //订阅装甲板消息
            rclcpp::Subscription<interfaces::msg::Armor>::SharedPtr armor_sub;
            //订阅解算好的参数
            rclcpp::Subscription<interfaces::msg::ParaCalcu>::SharedPtr para_sub;


            //卡尔曼实例
            std::shared_ptr<kalman> kal;
            //预测实例
            std::shared_ptr<PrePoints> pre_points;
            //pnp解算实例
            std::shared_ptr<PnPSolver> pnp_solver;
    };

}




#endif //PNP_SOLVER_NODE_HPP_