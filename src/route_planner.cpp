#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return (node -> distance(*end_node));
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(auto *neigbour: current_node->neighbors){
        neigbour->parent = current_node;
        neigbour->h_value = CalculateHValue(neigbour);
        neigbour->g_value = current_node->g_value + neigbour->distance((*current_node));;
        neigbour->visited = true;
        open_list.push_back(neigbour);
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(),open_list.end(),[](auto node1, auto node2){
        return (node1->g_value + node1->h_value) > (node2->g_value + node2->h_value);
    });
    RouteModel::Node* lowest_sum = open_list.back();
    open_list.pop_back();
    return lowest_sum;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node* node = current_node;
    while(node != nullptr){
        path_found.emplace_back(*node);
        if(node->parent != nullptr){
            distance += node->distance(*(node->parent));
        }
        node = node->parent;
    }
    std::reverse(path_found.begin(),path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

	open_list.push_back(start_node);
    current_node = open_list.back();
    current_node->visited = true;
    while (current_node != end_node){
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    m_Model.path = ConstructFinalPath(current_node);
    
    
}