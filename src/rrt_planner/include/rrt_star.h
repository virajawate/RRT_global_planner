#ifndef RRT_STAR_H
#define RRT_STAR_H
#include <ros/ros.h>
#include "rrt.h"
#include "utils.h"

namespace rrt_planner
{
    class RRTStar : public RRT
    {
    public:
        RRTStar(int nx, int ny, double resolution, int sample_num, double max_dist, double r);

        bool plan(const unsigned char *global_costmap, const Node &start, const Node &goal, std::vector<Node> &path, std::vector<Node> &expand);

    protected:
        Node _findNearestPoint(std::unordered_set<Node, NodeIdAsHash, compare_coordinates> list, Node &node);

        double r_;
    };
}
#endif
