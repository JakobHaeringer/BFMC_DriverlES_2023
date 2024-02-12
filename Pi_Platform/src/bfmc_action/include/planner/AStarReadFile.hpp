/**
 * @file AStarReadFile.hpp
 * @author Seid Jadadic
 * @brief The header file for accessing the map data to perform the AStar algorithm used in the BFMC 2023.
 * @version 1.0
 * @date 2023-07-17
 */
#ifndef ASTAR_READFILE_HPP
#define ASTAR_READFILE_HPP

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std;

constexpr char c_filename[] = "test.graphml";

struct NodeData {
    NodeData() :
        m_id{0U}, m_x{0.0}, m_y{0.0} {
    }

    NodeData(uint16_t id, float x, float y, bool crossLine, vector<int> connectedNodes) {
        this->m_id           = id;
        this->m_x            = x;
        this->m_y            = y;
        this->m_crossLine    = crossLine;
        this->connectedNodes = connectedNodes;
    }

    uint16_t    m_id;
    float       m_x;
    float       m_y;
    bool        m_crossLine;
    vector<int> connectedNodes;

    bool operator<(const NodeData& other) const {
        return (this->m_id < other.m_id);
    }
};

class ReadFile {
public:
    ReadFile();
    ReadFile(string filename);

    void readFile();

    void                              setFilename(string filename);
    unordered_map<uint16_t, NodeData> getNodes();
    void                              exportMapToInl();

protected:
    NodeData                          m_currentNode;
    unordered_map<uint16_t, NodeData> m_nodes;

    fstream graphMlFile;
    string  m_filename;
    string  m_currentLine;

    void readNodeDataFromFile();
    void readEdgeDataFromFile();
};

#endif
