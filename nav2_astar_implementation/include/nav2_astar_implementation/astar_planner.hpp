#ifndef NAV2_ASTAR_PLANNER_HPP_
#define NAV2_ASTAR_PLANNER_HPP_

#include <string.h>
#include <memory.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_utils/robot_utils.hpp"
#include "nav2_msgs/msg/path.hpp"
#include "nav2_utils/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_astar_planner
{
    class AstarPlanner : public nav2_core::GlobalPlanner
    {
        public:
            AstarPlanner()  = default
            ~AstarPlanner() = default

            void configure(
                const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name,
                std::shared_ptr<tf2_ros::Buffer> tf,
                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

            void cleanup() override;
            void activate() override;
            void deactivate() override;

            nav2_msgs::msg::Path createPlan(
                const geometry_msgs::msg::PostStamped & start,
                const geometry_msgs::msg::PostStamped & goal) override;
        
        private:
            std::shared_ptr<tf2_ros::Buffer> tf_;
            nav2_util::LifecycleNode::SharedPtr node_;
            nav2_costmap_2d::Costmap2D * costmap_;
            std::string global_frame_, name_;
            double interpolation_resolution_;
    };    
} // namespace nav2_astar_planner
#endif