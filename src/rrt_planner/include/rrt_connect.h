#ifndef RRT_CONNECT_H
#define RRT_CONNECT_H

#include "rrt.h"
#include "utils.h"

namespace rrt_planner
{
    class RRTConnect : public RRT
    {
    public:
        RRTConnect(int nx, int ny, double resolution, int sample_num, double max_dist);

        bool plan(const unsigned char *gloal_costmap, const Node &start, const Node &goal, std::vector<Node> &path, std::vector<Node> &expand);

    protected:
        std::vector<Node> _convertClosedListToPath(const Node &boundary);

        std::unordered_set<Node, NodeIdAsHash, compare_coordinates> sample_list_f_;

        std::unordered_set<Node, NodeIdAsHash, compare_coordinates> sample_list_b_;
    };
}

#endif