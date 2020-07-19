#include <rdl_obstacles_layer.h>

PLUGINLIB_EXPORT_CLASS(rdl_navigation_layers::RGBObstaclesLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace rdl_navigation_layers
{
    RGBObstaclesLayer::RGBObstaclesLayer() : mark_x_(0.0), mark_y_(0.0), r_(-1.0), draw_circle_around_(false) {}

    void RGBObstaclesLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_);
        current_ = true;

        // dynamic reconfigure allows to enable/disable the marking behaviors
        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
            &RGBObstaclesLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        // subscribe to rdl rgb detected obstacles
        sub_obstacles_ = nh.subscribe("rdl_rgb_obstacles", 10, &RGBObstaclesLayer::rdlObstaclesCallback, this);

        // initializing variable
        message_received_ = false;

        // get parameters from param server
        nh.param<bool>("draw_circle_around", draw_circle_around_, false);
        nh.param<double>("radius", r_, -1.0);
    }

    void RGBObstaclesLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
    }

    void RGBObstaclesLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                            double* min_y, double* max_x, double* max_y)
    {
        if (!enabled_)
        {
            return;
        }

        // defines the area that will need to be updated
        *min_x = std::min(*min_x, mark_x_);
        *min_y = std::min(*min_y, mark_y_);
        *max_x = std::max(*max_x, mark_x_);
        *max_y = std::max(*max_y, mark_y_);
    }

    void RGBObstaclesLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
        {
            obstacle_points_.poses.clear();
            return;
        }

        unsigned int mx;
        unsigned int my;

        for(int i = 0; i < obstacle_points_.poses.size() ; i++)
        {
            mark_x_ = obstacle_points_.poses[i].position.x;
            mark_y_ = obstacle_points_.poses[i].position.y;

            // center
            if(master_grid.worldToMap(mark_x_, mark_y_, mx, my))
            {
                master_grid.setCost(mx, my, LETHAL_OBSTACLE);
            }

            if(draw_circle_around_)
            {
                if(r_ == -1.0)
                {
                    ROS_WARN("draw circle around is set to true but no radius is set, will not draw circle around");
                    return;
                }

                // mark 8 points around the center sampling a circle of radius r

                double r = 0.2;
                double sq = std::sqrt(3.0 / 4.0); // to sample 45 degree points (2, 4, 6, 8)

                // p1 (North)
                if(master_grid.worldToMap(mark_x_, mark_y_ + r_, mx, my))
                {
                    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
                }

                // p2 (NE)
                if(master_grid.worldToMap(mark_x_ + r_ / 2.0, mark_y_ + r_ * sq, mx, my))
                {
                    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
                }

                // p3 (East)
                if(master_grid.worldToMap(mark_x_ + r_, mark_y_, mx, my))
                {
                    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
                }

                // p4 (SE)
                if(master_grid.worldToMap(mark_x_ + r_ / 2.0 , mark_y_ - r_ * sq , mx, my))
                {
                    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
                }

                // p5 (South)
                if(master_grid.worldToMap(mark_x_, mark_y_ - r_, mx, my))
                {
                    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
                }

                // p6 (SW)
                if(master_grid.worldToMap(mark_x_ - r_ / 2.0, mark_y_ - r_ * sq, mx, my))
                {
                    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
                }

                // p7 (West)
                if(master_grid.worldToMap(mark_x_ - r_, mark_y_, mx, my))
                {
                    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
                }

                // p8 (NW)
                if(master_grid.worldToMap(mark_x_ - r_ / 2.0, mark_y_ + r_ * sq, mx, my))
                {
                    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
                }
            }
        }
    }

    void RGBObstaclesLayer::rdlObstaclesCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
        message_received_ = true;
        obstacle_points_ = *msg;
    }

} // end namespace
