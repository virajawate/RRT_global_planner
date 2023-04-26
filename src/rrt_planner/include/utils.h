#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <array>
#include <vector>
#include <unordered_set>

#define LETHAL_COST 253
#define NEUTRAL_COST 50
#define OBSTACLE_FACTOR 0.5

constexpr int spacing_for_grid = 10;

class Node
{
public:
    Node(int x = 0, int y = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = 0);

    Node operator+(const Node &n) const;

    Node operator-(const Node &n) const;

    bool operator==(const Node &n) const;

    bool operator!=(const Node &n) const;

    int x_, y_, id_, pid_;
    double g_, h_;
};

class PlaneNode : public Node, public std::array<int, 2>
{
public:
    PlaneNode(int x = 0, int y = 0, double g = 0, double h = 0, int id = 0, int pid = 0);

    PlaneNode(const Node &n);

    static const int dim = 2;
};

class NodeIdAsHash
{
public:
    size_t operator()(const Node &n) const;
};

struct compare_cost
{
    bool operator()(const Node &n1, const Node &n2) const;
};

struct compare_coordinates
{
    bool operator()(const Node &n1, const Node &n2) const;
};

bool compareCoordinates(const Node &n1, const Node &n2);

namespace global_planner
{
    class GlobalPlanner
    {
    public:
        GlobalPlanner(int nx, int ny, double resolution);

        virtual ~GlobalPlanner() = default;

        virtual bool plan(const unsigned char *global_costmap, const Node &start, const Node &goal, std::vector<Node> &path, std::vector<Node> &expand) = 0;

        void setSize(int nx, int ny);

        void setResolution(double resolution);

        void setLethalCost(unsigned char lethal_cost);

        void setNeutralCost(unsigned char neutral_cost);

        void setFactor(double factor);

        int grid2Index(int x, int y);

        void index2Grid(int i, int &x, int &y);

        void map2Grid(double mx, double my, int &gx, int &gy);

        void grid2Map(int gx, int gy, double &mx, double &my);

    protected:
        std::vector<Node> _convertClosedListToPath(std::unordered_set<Node, NodeIdAsHash, compare_coordinates> &closed_list, const Node &start, const Node &goal);

        unsigned char lethal_cost_, neutral_cost_;

        int nx_, ny_, ns_;

        double resolution_, factor_;
    };
}
#endif