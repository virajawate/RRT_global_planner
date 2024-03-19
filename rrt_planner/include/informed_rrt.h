#ifndef INFORMED_RRT_H
#define INFORMED_RRT_H
#include <ros/ros.h>
#include "rrt_star.h"
#include "utils.h"

namespace rrt_planner
{
    class InformedRRT : public RRTStar
    {
    public:
        InformedRRT(int nx, int ny, double resolution, int sample_num, double max_dist, double r);

        bool plan(const unsigned char *gloal_costmap, const Node &start, const Node &goal, std::vector<Node> &path, std::vector<Node> &expand);

    protected:
        Node _transform(double x, double y);

        Node _generateRandomNode();

        double c_best_, c_min_;
    };
}
#endif