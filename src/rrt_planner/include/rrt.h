#ifndef RRT_H
#define RRT_H

#include <tuple>
#include <unordered_map>

#include "utils.h"

namespace rrt_planner
{
    class RRT : public global_planner::GlobalPlanner
    {
    public:
        RRT(int nx, int ny, double resolution, int sample_num, double max_dist);

        bool plan(const unsigned char *global_costmap, const Node &start, const Node &goal, std::vector<Node> &path, std::vector<Node> &expand);

    protected:
        Node _findNearestPoint(std::unordered_set<Node, NodeIdAsHash, compare_coordinates> list, const Node &node);

        bool _isAnyObstacleInPath(const Node &n1, const Node &n2);

        Node _generateRandomNode();

        bool _checkGoal(const Node &new_node);

        double _dist(const Node &node1, const Node &node2);

        double _angle(const Node &node1, const Node &node2);

    protected:
        const unsigned char *costs_;
        Node start_, goal_;
        int sample_num_;
        double max_dist_;
        std::unordered_set<Node, NodeIdAsHash, compare_coordinates> sample_list_;
    };
}

#endif