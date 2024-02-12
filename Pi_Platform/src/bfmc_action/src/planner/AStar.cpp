/**
 * @file AStar.cpp
 * @author Seid Jadadic
 * @brief The file implements the AStar algorithm used in the BFMC 2023.
 * @version 1.0
 * @date 2023-07-17
 */

#include "planner/AStar.hpp"

AStarAlgorithm::AStarAlgorithm() {
}

vector<uint16_t> AStarAlgorithm::getPath() {
    return this->m_path;
}

ReadFile& AStarAlgorithm::getReadFile() {
    return this->m_readFile;
}

// euklidische Entfernung
float AStarAlgorithm::heuristic(NodeData& nodeA, NodeData& nodeB) {
    return sqrt(pow(nodeA.m_x - nodeB.m_x, 2) + pow(nodeA.m_y - nodeB.m_y, 2));
}

bool AStarAlgorithm::tolerancX(float currX, float prevX) {
    return (((currX - toleranz) <= prevX) && (prevX <= (currX + toleranz)));
}

bool AStarAlgorithm::tolerancY(float currY, float prevY) {
    return (((currY - toleranz) <= prevY) && (prevY <= (currY + toleranz)));
}

void AStarAlgorithm::aStarAlgorithm(int startId, int goalId, ITMOVES::Environment* env) {
    // Create two hash maps to store the gScore and fScore for each node
    unordered_map<uint16_t, float>    gScores;
    unordered_map<uint16_t, uint16_t> cameFrom;
    priority_queue<Node>              openSet;

    gScores[startId] = 0.0f;
    openSet.push({startId, heuristic(m_readFile.getNodes()[startId], m_readFile.getNodes()[goalId]), 0.0f});

    while (!openSet.empty()) {
        Node current = openSet.top();
        if (current.id == goalId) {
            // Reconstruct path
            int   node           = current.id;
            float node_x         = m_readFile.getNodes()[node].m_x;
            float node_y         = m_readFile.getNodes()[node].m_y;
            bool  node_crossLine = m_readFile.getNodes()[node].m_crossLine;

            // m_path.push_back(node);
            env->plannedRoute.push_back({node, node_x, node_y, node_crossLine});
            while (cameFrom.find(node) != cameFrom.end()) {
                node           = cameFrom[node];
                node_x         = m_readFile.getNodes()[node].m_x;
                node_y         = m_readFile.getNodes()[node].m_y;
                node_crossLine = m_readFile.getNodes()[node].m_crossLine;
                // m_path.push_back(node);
                env->plannedRoute.push_back({node, node_x, node_y, node_crossLine});
            }
            // reverse(m_path.begin(), m_path.end());
            reverse(env->plannedRoute.begin(), env->plannedRoute.end());
            // Path found
            break;
        }

        openSet.pop();
        vector<int> currentIdConnectedNodes = m_readFile.getNodes()[current.id].connectedNodes;
        for (const auto& neighborId : currentIdConnectedNodes) {
            float tentativeGScore = gScores[current.id] + heuristic(m_readFile.getNodes()[current.id], m_readFile.getNodes()[neighborId]);
            if (gScores.find(neighborId) == gScores.end() || tentativeGScore < gScores[neighborId]) {
                gScores[neighborId] = tentativeGScore;
                float fScore        = tentativeGScore + heuristic(m_readFile.getNodes()[neighborId], m_readFile.getNodes()[goalId]);
                openSet.push({neighborId, fScore, tentativeGScore});
                cameFrom[neighborId] = current.id;
            }
        }
    }
}

// void AStarAlgorithm::shortenStraightRoad()
// {
//     for(uint16_t i{1U}; i < (m_path.size() - 1); ++i)
//     {
//         if(tolerancX(m_readFile.getNodes()[m_path.at(i)].m_x, m_readFile.getNodes()[m_path.at(i + 1)].m_x)
//             || tolerancY(m_readFile.getNodes()[m_path.at(i)].m_y, m_readFile.getNodes()[m_path.at(i + 1)].m_y))
//         {
//             m_path.erase(m_path.begin() + i);
//         }
//     }
// }
