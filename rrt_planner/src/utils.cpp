#include "utils.h"

Node::Node(int x, int y, double g, double h, int id, int pid) : x_(x), y_(y), g_(g), h_(h), id_(id), pid_(pid) {}

Node Node::operator+(const Node &n) const
{
    Node result;
    result.x_ = x_ + n.x_;
    result.y_ = y_ + n.y_;
    result.g_ = g_ + n.g_;

    return result;
}

Node Node::operator-(const Node &n) const
{
    Node result;
    result.x_ = x_ - n.x_;
    result.y_ = y_ - n.y_;

    return result;
}

bool Node::operator==(const Node &n) const
{
    return x_ == n.x_ && y_ == n.y_;
}

bool Node::operator!=(const Node &n) const
{
    return !operator==(n);
}

PlaneNode::PlaneNode(int x, int y, double g, double h, int id, int pid) : Node(x, y, g, h, id, pid)
{
    (*this)[0] = x;
    (*this)[1] = y;
}

PlaneNode::PlaneNode(const Node &n) : PlaneNode(n.x_, n.y_, n.g_, n.h_, n.id_, n.pid_)
{
}

size_t NodeIdAsHash::operator()(const Node &n) const
{
    return n.id_;
}

bool compare_cost::operator()(const Node &n1, const Node &n2) const
{
    return (n1.g_ + n1.h_ > n2.g_ + n2.h_) || (n1.g_ + n1.h_ == n2.g_ + n2.h_) && (n1.h_ > n2.h_);
}

bool compare_coordinates::operator()(const Node &n1, const Node &n2) const
{
    return (n1.x_ == n2.x_) && (n1.y_ == n2.y_);
}

std::vector<Node> getMotion()
{
    return {
        Node(0, 1, 1), Node(1, 0, 1), Node(0, -1, 1), Node(-1, 0, 1),
        Node(1, 1, std::sqrt(2)), Node(1, 1, std::sqrt(2)), Node(-1, 1, std::sqrt(2)), Node(-1, -1, std::sqrt(2))};
}

bool compareCoordinate(const Node &n1, const Node &n2)
{
    return (n1.x_ == n2.x_) && (n1.y_ == n2.y_);
}

namespace global_planner
{
    GlobalPlanner::GlobalPlanner(int nx, int ny, double resolution)
        : lethal_cost_(LETHAL_COST), neutral_cost_(NEUTRAL_COST), factor_(OBSTACLE_FACTOR)
    {
        setSize(nx, ny);
        setResolution(resolution);
    }

    void GlobalPlanner::setSize(int nx, int ny)
    {
        nx_ = nx;
        ny_ = ny;
        ns_ = nx * ny;
    }

    void GlobalPlanner::setResolution(double resolution)
    {
        resolution_ = resolution;
    }

    void GlobalPlanner::setNeutralCost(unsigned char neutral_cost)
    {
        neutral_cost_ = neutral_cost;
    }

    void GlobalPlanner::setFactor(double factor)
    {
        factor_ = factor;
    }

    int GlobalPlanner::grid2Index(int x, int y)
    {
        return x + nx_ * y;
    }

    void GlobalPlanner::index2Grid(int i, int &x, int &y)
    {
        x = i % nx_;
        y = i / nx_;
    }

    void GlobalPlanner::map2Grid(double mx, double my, int &gx, int &gy)
    {
        gx = (int)mx;
        gy = (int)my;
    }

    void GlobalPlanner::grid2Map(int gx, int gy, double &mx, double &my)
    {
        mx = resolution_ * (gx + 0.5);
        my = resolution_ * (gy + 0.5);
    }

    std::vector<Node> GlobalPlanner::_convertClosedListToPath(
        std::unordered_set<Node, NodeIdAsHash, compare_coordinates> &closed_list, const Node &start, const Node &goal)
    {
        auto current = *closed_list.find(goal);
        std::vector<Node> path;
        while (current != start)
        {
            path.push_back(current);
            auto it = closed_list.find(Node(current.pid_ % nx_, current.pid_ / nx_, 0, 0, current.pid_));
            if (it != closed_list.end())
                current = *it;
            else
                return {};
        }
        path.push_back(start);

        return path;
    }

}