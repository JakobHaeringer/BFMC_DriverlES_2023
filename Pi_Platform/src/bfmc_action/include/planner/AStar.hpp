/**
 * @file AStar.hpp
 * @author Seid Jadadic
 * @brief The header file for the AStar algorithm used in the BFMC 2023.
 * @version 1.0
 * @date 2023-07-17
 */

#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>

#include "planner/AStarReadFile.hpp"
#include "planner/Environment.hpp"

using namespace std;

const float toleranz = 0.03f;
struct Node {
    int   id;
    float fScore;
    float gScore;
    bool  operator<(const Node& other) const { return fScore > other.fScore; }
};

class AStarAlgorithm : ReadFile {
public:
    AStarAlgorithm();
    void aStarAlgorithm(int startId, int goalId, ITMOVES::Environment* env);
    // void shortenStraightRoad();

    vector<uint16_t> getPath();
    ReadFile&        getReadFile();

private:
    ReadFile         m_readFile;
    vector<uint16_t> m_path;

    float heuristic(NodeData& nodeA, NodeData& nodeB);
    bool  tolerancX(float currX, float prevX);
    bool  tolerancY(float currY, float prevY);
};
#endif
