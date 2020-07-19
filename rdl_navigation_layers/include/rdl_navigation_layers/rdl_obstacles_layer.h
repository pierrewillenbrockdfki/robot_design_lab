#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseArray.h>
#include <pluginlib/class_list_macros.h>

namespace rdl_navigation_layers
{
    class RGBObstaclesLayer : public costmap_2d::Layer
    {
      public:

        RGBObstaclesLayer();

        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                  double* min_y, double* max_x, double* max_y);
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

      private:

        void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

        void rdlObstaclesCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

        double mark_x_, mark_y_;
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

        ros::Subscriber sub_obstacles_; 

        // flag to indicate that topic was received
        bool message_received_;

        geometry_msgs::PoseArray obstacle_points_;

        // if true, 8 points of a circle of radius r is added to the lethal cost position
        bool draw_circle_around_;

        double r_;
    };
}
