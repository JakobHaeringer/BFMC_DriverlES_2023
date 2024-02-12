/**
 * @file AStarReadFile.cpp
 * @author Seid Jadadic
 * @brief The file implements the access to the map data to perform the AStar algorithm used in the BFMC 2023.
 * @version 1.0
 * @date 2023-07-17
 */
#include "planner/AStarReadFile.hpp"

ReadFile::ReadFile() {
    this->m_filename = c_filename;
}

ReadFile::ReadFile(string filename) {
    this->m_filename = filename;
}

void ReadFile::readFile() {

    // Datei muss unter build liegen!
    graphMlFile.open(m_filename);

    // Überprüfe, ob das Öffnen erfolgreich war
    if (!graphMlFile) {
        cerr << "Fehler beim Öffnen von " << m_filename << "!" << endl;
        throw;
    }

    while (getline(graphMlFile, m_currentLine)) {
        if (m_currentLine.find("<node") != string::npos) {
            readNodeDataFromFile();
            m_nodes[m_currentNode.m_id] = m_currentNode;
        }
        else if (m_currentLine.find("<edge", 0) != string::npos) {
            readEdgeDataFromFile();
        }
        else {
            continue;
        }
    }
}

void ReadFile::readNodeDataFromFile() {
    do {
        if (m_currentLine.find("<node", 0) != string::npos) {
            string searchString = "id=\"";
            string endString    = "\">";

            size_t startPos = m_currentLine.find(searchString) + searchString.length();
            size_t endPos   = m_currentLine.find(endString);
            string value    = m_currentLine.substr(startPos, endPos - startPos);

            m_currentNode.m_id = stoi(value);
        }
        else if (m_currentLine.find("<data key", 0) != string::npos) {
            // get x value
            string searchString = "key=\"d0\">";
            string endString    = "</data>";

            size_t startPos = m_currentLine.find(searchString) + searchString.length();
            size_t endPos   = m_currentLine.find(endString);

            string value      = m_currentLine.substr(startPos, endPos - startPos);
            m_currentNode.m_x = stof(value);

            // get y value
            getline(graphMlFile, m_currentLine);

            searchString = "key=\"d1\">";

            startPos = m_currentLine.find(searchString) + searchString.length();
            endPos   = m_currentLine.find(endString);

            value             = m_currentLine.substr(startPos, endPos - startPos);
            m_currentNode.m_y = stof(value);
        }
        else if (m_currentLine.find("</node>", 0) != string::npos) {
            break;
        }
    }
    while (getline(graphMlFile, m_currentLine));
}

void ReadFile::readEdgeDataFromFile() {
    do {
        if (m_currentLine.find("<edge", 0) != string::npos) {
            string searchString = "source=\"";
            string endString    = "\" target=\"";

            size_t startPos = m_currentLine.find(searchString) + searchString.length();
            size_t endPos   = m_currentLine.find(endString);
            string value    = m_currentLine.substr(startPos, endPos - startPos);

            int src = stoi(value);

            searchString = "target=\"";
            endString    = "\">";

            startPos = m_currentLine.find(searchString) + searchString.length();
            endPos   = m_currentLine.find(endString);
            value    = m_currentLine.substr(startPos, endPos - startPos);

            // m_nodes.at(src - 1).connectedNodes.push_back(std::stoi(value));

            bool crossLine = false;

            m_nodes[src].connectedNodes.push_back(std::stoi(value));

            getline(graphMlFile, m_currentLine);

            if (m_currentLine.find("<data key", 0) != string::npos) {
                searchString = "key=\"d2\">";
                endString    = "</data>";

                startPos = m_currentLine.find(searchString) + searchString.length();
                endPos   = m_currentLine.find(endString);

                value = m_currentLine.substr(startPos, endPos - startPos);

                if (value == "True") crossLine = true;
                else if (value == "False") crossLine = false;
            }
            m_nodes[src].m_crossLine = crossLine;
        }
        else if (m_currentLine.find("</edge>", 0) != string::npos) {
            break;
        }
    }
    while (getline(graphMlFile, m_currentLine));
}

void ReadFile::setFilename(string filename) {
    this->m_filename = filename;
}

unordered_map<uint16_t, NodeData> ReadFile::getNodes() {
    return this->m_nodes;
}

void ReadFile::exportMapToInl() {
}
