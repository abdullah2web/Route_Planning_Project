#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    m_Model=model;

    // Used the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// Implementation the CalculateHValue method.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node){
    std::vector<RouteModel::Node> path_found;
    distance = 0.0f;
    RouteModel::Node parent;

    while (current_node->parent !=nullptr)
    {
        path_found.emplace_back(*current_node);
        parent = *(current_node->parent);
        distance +=current_node->distance(parent);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale();
    return path_found;
}


// Used the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AStarSearch(){
    start_node->visited = true;
    open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;

    while (open_list.size() > 0)
    {
        current_node = NextNode();
        if (current_node->distance(*end_node) == 0) {
            m_Model.path = ConstructFinalPath(current_node);

            return;
        }
    }
}

// Used the NextNode method to sort the open list and return the next node.
float RoutePlanner::CalculateHValue(const RouteModel::Node *node) {
    return node->distance(*end_node);
}

// Used the ConstructFinalPath method to return the final path found from A* search.
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](const auto &value_1, const auto &value_2){
        return value_1->h_value + value_1->g_value < value_2->h_value + value_2->g_value;
    });
    
    RouteModel::Node *lowest_node = open_list.front();
    open_list.erase(open_list.begin());
    return lowest_node;
}

// Used the AddNeighbors method to add all of the neighbors of the current node to the open_list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (auto neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->h_value = CalculateHValue(neighbor);

        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}
