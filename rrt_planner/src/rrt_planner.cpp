#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <random>
#include "rrt_planner.h"

#include "rrt.h"
#include "rrt_star.h"
#include "rrt_connect.h"
#include "informed_rrt.h"

PLUGINLIB_EXPORT_CLASS(rrt_planner::RRT_PLANNER, nav_core::BaseGlobalPlanner)

namespace rrt_planner
{
    RRT_PLANNER::RRT_PLANNER() : costmap_(NULL), initialized_(false), g_planner_(NULL)
    {
    }

    RRT_PLANNER::RRT_PLANNER(std::string name, costmap_2d::Costmap2DROS *costmap_ros) : RRT_PLANNER()
    {
        initialize(name, costmap_ros);
    }

    RRT_PLANNER::~RRT_PLANNER()
    {
        if (g_planner_)
            delete g_planner_;
    }

    void RRT_PLANNER::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
    }

    void RRT_PLANNER::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id)
    {
        if (!initialized_)
        {
            ros::NodeHandle private_nh("~/" + name);

            costmap_ = costmap;
            frame_id_ = frame_id;

            nx_ = costmap->getSizeInCellsX(), ny_ = costmap->getSizeInCellsY();
            origin_x_ = costmap->getOriginX(), origin_y_ = costmap->getOriginY();
            resolution_ = costmap->getResolution();

            private_nh.param("convert_offset", convert_offset_, 0.0); // Offset of Map and World
            private_nh.param("default_tolerance", tolerance_, 0.0);   // Error Tolerance
            private_nh.param("outline_map", is_outline_, false);      // Map Outline
            private_nh.param("obstacle_factor", factor_, 0.5);        // Obstacle Inflation Factor
            private_nh.param("expand_zone", is_expand_, false);       // Expand Zone
            private_nh.param("sample_points", sample_points, 500);    // Random Points
            private_nh.param("sample_max_d", radius, 5.0);   // Radius of sample points
            private_nh.param("optimization_r", opt_r_, 5.0);          // ReWiring Radius

            std::string planner_name;

            private_nh.param("planner_name", planner_name, (std::string) "rrt");
            
            if (planner_name == "rrt")
                g_planner_ = new rrt_planner::RRT(nx_, ny_, resolution_, sample_points, radius);
            else if (planner_name == "rrt_star")
                g_planner_ = new rrt_planner::RRTStar(nx_, ny_, resolution_, sample_points, radius, opt_r_);
            else if (planner_name == "rrt_connect")
                g_planner_ = new rrt_planner::RRTConnect(nx_, ny_, resolution_, sample_points, radius);
            else if (planner_name == "informed_rrt")
                g_planner_ = new rrt_planner::InformedRRT(nx_, ny_, resolution_, sample_points, radius, opt_r_);

            ROS_INFO("Using Global Planner %s ", planner_name.c_str());

            plan_pub = private_nh.advertise<nav_msgs::Path>("plan", 1);

            expand_pub = private_nh.advertise<visualization_msgs::Marker>("tree", 1);

            make_plan_srv = private_nh.advertiseService("make_plan", &RRT_PLANNER::makePlanService, this);

            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized, can't initialize it twice");
    }

    bool RRT_PLANNER::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        return makePlan(start, goal, plan, tolerance_);
    }

    bool RRT_PLANNER::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan, double tolerance)
    {
        boost::mutex::scoped_lock lock(mutex_);
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been Initialized.");
            return false;
        }

        plan.clear();

        if (goal.header.frame_id != frame_id_)
        {
            ROS_ERROR("The goal pose must be in %s frame, instead of %s frame", frame_id_.c_str(), goal.header.frame_id.c_str());
            return false;
        }

        if (start.header.frame_id != frame_id_)
        {
            ROS_ERROR("The goal pose must be in %s frame, instead of %s frame", frame_id_.c_str(), start.header.frame_id.c_str());
            return false;
        }

        double wx = start.pose.position.x, wy = start.pose.position.y;
        double m_start_x, m_start_y, m_goal_x, m_goal_y;
        if (!_worldToMap(wx, wy, m_start_x, m_start_y))
        {
            ROS_WARN("Robot start position is off global costmap, localize the robot properly");
            return false;
        }

        wx = goal.pose.position.x, wy = goal.pose.position.y;
        if (!_worldToMap(wx, wy, m_goal_x, m_goal_y))
        {
            ROS_WARN("Robot goal position is off global costmap, localize the robot properly");
            return false;
        }

        int g_start_x, g_start_y, g_goal_x, g_goal_y;
        g_planner_->map2Grid(m_start_x, m_start_y, g_start_x, g_start_y);
        g_planner_->map2Grid(m_goal_x, m_goal_y, g_goal_x, g_goal_y);
        Node n_start(g_start_x, g_start_y, 0, 0, g_planner_->grid2Index(g_start_x, g_start_y), 0);
        Node n_goal(g_goal_x, g_goal_y, 0, 0, g_planner_->grid2Index(g_goal_x, g_goal_y), 0);

        costmap_->setCost(g_start_x, g_start_y, costmap_2d::FREE_SPACE);

        if (is_outline_)
            _outlineMap(costmap_->getCharMap());

        std::vector<Node> path;
        std::vector<Node> expand;
        bool path_found = g_planner_->plan(costmap_->getCharMap(), n_start, n_goal, path, expand);

        if (path_found)
        {
            if (_getPlanFromPath(path, plan))
            {
                geometry_msgs::PoseStamped goalCopy = goal;
                goalCopy.header.stamp = ros::Time::now();
                plan.push_back(goalCopy);
            }
            else
                ROS_ERROR("Failed to get a plan from path whenn a legal path was found. ");
        }
        else
            ROS_ERROR("Failed to get a path");

        if (is_expand_)
            _publishExpand(expand);

        publishPlan(plan);
        return !plan.empty();
    }

    void RRT_PLANNER::publishPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("This Planner has not been initialized yet, call initialize() before use");
            return;
        }

        nav_msgs::Path gui_plan;
        gui_plan.poses.resize(plan.size());
        gui_plan.header.frame_id = frame_id_;
        gui_plan.header.stamp = ros::Time::now();
        for (unsigned int i = 0; i < plan.size(); i++)
            gui_plan.poses[i] = plan[i];

        plan_pub.publish(gui_plan);
    }

    bool RRT_PLANNER::makePlanService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
    {
        makePlan(req.start, req.goal, resp.plan.poses);
        resp.plan.header.stamp = ros::Time::now();
        resp.plan.header.frame_id = frame_id_;
        return true;
    }

    void RRT_PLANNER::_outlineMap(unsigned char *costarr)
    {
        unsigned char *pc = costarr;
        for (int i = 0; i < nx_; i++)
            *pc++ = costmap_2d::LETHAL_OBSTACLE;
        pc = costarr + (ny_ - 1) * nx_;
        for (int i = 0; i < nx_; i++)
            *pc++ = costmap_2d::LETHAL_OBSTACLE;
        pc = costarr;
        for (int i = 0; i < ny_; i++, pc += nx_)
            *pc = costmap_2d::LETHAL_OBSTACLE;
        pc = costarr + nx_ - 1;
        for (int i = 0; i < ny_; i++, pc += nx_)
            *pc = costmap_2d::LETHAL_OBSTACLE;
    }

    void RRT_PLANNER::_publishExpand(std::vector<Node> &expand)
    {
        ROS_DEBUG("Expand Zone Size: %ld ", expand.size());

        visualization_msgs::Marker tree_msg;
        tree_msg.header.frame_id = "map";
        tree_msg.id = 0;
        tree_msg.ns = "tree";
        tree_msg.type = visualization_msgs::Marker::LINE_LIST;
        tree_msg.action = visualization_msgs::Marker::ADD;
        tree_msg.pose.orientation.w = 1.0;
        tree_msg.scale.x = 0.05;

        for (auto node : expand)
            if (node.pid_ != 0)
                _pubLine(&tree_msg, &expand_pub, node.id_, node.pid_);
    }

    void RRT_PLANNER::_mapToWorld(double mx, double my, double &wx, double &wy)
    {
        wx = origin_x_ + (mx + convert_offset_) * resolution_;
        wy = origin_y_ + (my + convert_offset_) * resolution_;
    }

    bool RRT_PLANNER::_worldToMap(double wx, double wy, double &mx, double &my)
    {
        if (wx < origin_x_ || wy < origin_y_)
            return false;

        mx = (wx - origin_x_) / resolution_ - convert_offset_;
        my = (wy - origin_y_) / resolution_ - convert_offset_;

        if (mx < nx_ && my < ny_)
            return true;

        return false;
    }

    bool RRT_PLANNER::_getPlanFromPath(std::vector<Node> path, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("The planner is not initilized yet, but its being used, please call initialize() before use");
            return false;
        }

        std::string globalFrame = frame_id_;
        ros::Time planTime = ros::Time::now();
        plan.clear();

        for (int i = path.size() - 1; i >= 0; i--)
        {
            double wx, wy;
            _mapToWorld((double)path[i].x_, (double)path[i].y_, wx, wy);
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = frame_id_;
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
        }
        return !plan.empty();
    }

    void RRT_PLANNER::_pubLine(visualization_msgs::Marker *line_msg, ros::Publisher *line_pub, int id, int pid)
    {
        line_msg->header.stamp = ros::Time::now();

        geometry_msgs::Point p1, p2;
        std_msgs::ColorRGBA c1, c2;
        int p1x, p1y, p2x, p2y;

        g_planner_->index2Grid(id, p1x, p1y);
        g_planner_->grid2Map(p2x, p2y, p2.x, p2.y);

        p2.x = (p2.x + convert_offset_) + costmap_->getOriginX();
        p2.y = (p2.y + convert_offset_) + costmap_->getOriginY();
        p2.z = 1.0;

        c1.r = 0.43;
        c1.g = 0.54;
        c1.b = 0.24;
        c1.a = 0.5;

        c2.r = 0.43;
        c2.g = 0.54;
        c2.b = 0.24;
        c2.a = 0.5;

        line_msg->points.push_back(p1);
        line_msg->points.push_back(p2);
        line_msg->colors.push_back(c1);
        line_msg->colors.push_back(c2);

        line_pub->publish(*line_msg);
    }
}