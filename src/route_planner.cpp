#include "route_planner.h"
#include "model.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto node: current_node->neighbors) {
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->visited = true;
        open_list.push_back(node);
    }
}


bool Compare(RouteModel::Node* a, RouteModel::Node* b) {
  return a->h_value + a->g_value >= b->h_value + b->g_value; 
}
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node* next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node* current = current_node;
    while(current != start_node) {
        path_found.push_back(*current);
        distance += current->distance(*(current->parent));
        current = current->parent;
    }
    path_found.push_back(*current);
    
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    start_node->visited = true;
    current_node = start_node;
    do {
        AddNeighbors(current_node);
        current_node = NextNode();
    } while (current_node != end_node);
    if (current_node != end_node) {
        std::cout << "Warning! The A* start search did not terminate at the end node" << std::endl;
    }
    m_Model.path = ConstructFinalPath(current_node);
}
