#include <cmath>
#include <random>

#include "rrt_star.h"

namespace rrt_planner
{

    RRTStar::RRTStar(int nx, int ny, double resolution, int sample_num, double max_dist, double r) : RRT(nx, ny, resolution, sample_num, max_dist), r_(r)
    {
    }

    bool RRTStar::plan(const unsigned char *gloal_costmap, const Node &start, const Node &goal, std::vector<Node> &path, std::vector<Node> &expand)
    {
        sample_list_.clear();
        ROS_INFO("INITIALIZING RRT star...");

        start_ = start, goal_ = goal;
        costs_ = gloal_costmap;

        sample_list_.insert(start);
        expand.push_back(start);

        int iteration = 0;
        while (iteration < sample_num_)
        {
            Node sample_node = _generateRandomNode();

            if (gloal_costmap[sample_node.id_] >= lethal_cost_ * factor_)
                continue;

            if (sample_list_.find(sample_node) != sample_list_.end())
                continue;

            Node new_node = _findNearestPoint(sample_list_, sample_node);
            if (new_node.id_ == -1)
                continue;
            else
            {
                sample_list_.insert(new_node);
                expand.push_back(new_node);
            }

            if (_checkGoal(new_node))
            {
                path = _convertClosedListToPath(sample_list_, start, goal);
                return true;
            }

            iteration++;
        }
        return false;
    }

    Node RRTStar::_findNearestPoint(std::unordered_set<Node, NodeIdAsHash, compare_coordinates> list, Node &node)
    {
        Node nearest_node, new_node(node);
        double min_dist = std::numeric_limits<double>::max();

        for (const auto node_ : list)
        {
            double new_dist = _dist(node_, new_node);

            if (new_dist < min_dist)
            {
                nearest_node = node_;
                new_node.pid_ = nearest_node.id_;
                new_node.g_ = new_dist + node_.g_;
                min_dist = new_dist;
            }
        }

        if (min_dist > max_dist_)
        {
            double theta = _angle(nearest_node, new_node);
            new_node.x_ = nearest_node.x_ + (int)(max_dist_ * cos(theta));
            new_node.y_ = nearest_node.y_ + (int)(max_dist_ * sin(theta));
            new_node.id_ = grid2Index(new_node.x_, new_node.y_);
            new_node.g_ = max_dist_ + nearest_node.g_;
        }

        if (!_isAnyObstacleInPath(new_node, nearest_node))
        {
            for (auto node_ : sample_list_)
            {
                double new_dist = _dist(node_, new_node);
                if (new_dist < r_)
                {
                    double cost = node_.g_ + new_dist;
                    if (new_node.g_ > cost)
                    {
                        if (!_isAnyObstacleInPath(new_node, node_))
                        {
                            new_node.pid_ = node_.id_;
                            new_node.g_ = cost;
                        }
                    }
                    else
                    {
                        cost = new_node.g_ + new_dist;
                        if (cost < node_.g_)
                        {
                            if (!_isAnyObstacleInPath(new_node, node_))
                            {
                                node_.pid_ = new_node.id_;
                                node_.g_ = cost;
                            }
                        }
                    }
                }
                else
                    continue;
            }
        }
        else
            new_node.id_ = -1;
        return new_node;
    }
}