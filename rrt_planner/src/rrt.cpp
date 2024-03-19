#include <cmath>
#include <random>

#include "rrt.h"

namespace rrt_planner
{

    RRT::RRT(int nx, int ny, double resolution, int sample_num, double max_dist) : GlobalPlanner(nx, ny, resolution), sample_num_(sample_num), max_dist_(max_dist)
    {
    }

    bool RRT::plan(const unsigned char *gloal_costmap, const Node &start, const Node &goal, std::vector<Node> &path, std::vector<Node> &expand)
    {
        path.clear();
        expand.clear();

        sample_list_.clear();
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

    Node RRT::_generateRandomNode()
    {
        std::random_device rd;
        std::mt19937 eng(rd());
        std::uniform_real_distribution<float> p(0, 1);
        std::uniform_int_distribution<int> distr(0, ns_ - 1);
        if (p(eng) > 0.05)
        {
            const int id = distr(eng);
            int x, y;
            index2Grid(id, x, y);
            return Node(x, y, 0, 0, id, 0);
        }
        else
            return Node(goal_.x_, goal_.y_, 0, 0, goal_.id_, 0);
    }

    Node RRT::_findNearestPoint(std::unordered_set<Node, NodeIdAsHash, compare_coordinates> list, const Node &node)
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

        if (_isAnyObstacleInPath(new_node, nearest_node))
            new_node.id_ = -1;

        return new_node;
    }

    bool RRT::_isAnyObstacleInPath(const Node &n1, const Node &n2)
    {
        double theta = _angle(n1, n2);
        double dist = _dist(n1, n2);

        if (dist > max_dist_)
            return true;

        int n_step = (int)(dist / resolution_);
        for (int i = 0; i < n_step; i++)
        {
            float line_x = (float)n1.x_ + (float)(i * resolution_ * cos(theta));
            float line_y = (float)n1.y_ + (float)(i * resolution_ * sin(theta));
            if (costs_[grid2Index((int)line_x, (int)line_y)] >= lethal_cost_ * factor_)
                return true;
        }
        return false;
    }

    bool RRT::_checkGoal(const Node &new_node)
    {
        auto dist = _dist(new_node, goal_);
        if (dist > max_dist_)
            return false;

        if (!_isAnyObstacleInPath(new_node, goal_))
        {
            Node goal(goal_.x_, goal_.y_, dist + new_node.g_, 0, grid2Index(goal_.x_, goal_.y_), new_node.id_);
            sample_list_.insert(goal);
            return true;
        }
        return false;
    }

    double RRT::_dist(const Node &node1, const Node &node2)
    {
        return std::sqrt(std::pow(node1.x_ - node2.x_, 2) + std::pow(node1.y_ - node2.y_, 2));
    }

    double RRT::_angle(const Node &node1, const Node &node2)
    {
        return atan2(node2.y_ - node1.y_, node2.x_ - node1.x_);
    }
}
