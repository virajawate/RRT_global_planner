#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H
#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <navfn/MakeNavPlan.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <vector>
#include <string>
#include "utils.h"

using std::string;

namespace rrt_planner
{
    class RRT_PLANNER : public nav_core::BaseGlobalPlanner
    {
    public:
        RRT_PLANNER();
        RRT_PLANNER(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        ~RRT_PLANNER();
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        void initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id);

        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);

        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan,
                      double tolerance);

        void publishPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

        bool makePlanService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

    protected:
        void _outlineMap(unsigned char *costarr);

        void _publishExpand(std::vector<Node> &expand);

        bool _getPlanFromPath(std::vector<Node> path, std::vector<geometry_msgs::PoseStamped> &plan);

        void _mapToWorld(double mx, double my, double &wx, double &wy);

        bool _worldToMap(double wx, double wy, double &mx, double &my);

        void _pubLine(visualization_msgs::Marker *line_msg, ros::Publisher *line_pub, int id, int pid);

    private:
        double step_size, iterationlimit, resolution_;
        costmap_2d::Costmap2D *costmap_;
        bool initialized_;
        global_planner::GlobalPlanner *g_planner_ {nullptr};
        unsigned int nx_, ny_;
        std::string frame_id_;
        double origin_x_, origin_y_;

        boost::mutex mutex_;
        double convert_offset_;
        double tolerance_;
        double factor_;
        double opt_r_;
        double radius;
        int sample_points;
        bool is_expand_;
        bool is_outline_;

        ros::Publisher plan_pub;
        ros::Publisher expand_pub;
        ros::ServiceServer make_plan_srv;
    };
}
#endif