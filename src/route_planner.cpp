#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        open_list.emplace_back(neighbor);
		neighbor->visited = true;
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](const auto& node1, const auto& node2) {
        int sum1 = node1->h_value + node1->g_value;
        int sum2 = node2->h_value + node2->g_value;
        return sum1 < sum2;
    });

    RouteModel::Node* lowest_node = open_list.front();
	open_list.erase(open_list.begin());
	return lowest_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    bool shouldContinue = true;
    while(shouldContinue == true)
	{
		path_found.emplace_back(*current_node);
		distance += current_node->distance(*(current_node->parent));
		current_node = current_node->parent;

        if(!current_node->parent) {
            shouldContinue = false;
        }
	}

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    // Switch order of vector to have the start values as first element
    std::reverse(path_found.begin(), path_found.end());
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

	start_node->visited = true;
	open_list.emplace_back(start_node);

	while(!open_list.empty())
	{
		RouteModel::Node* current_node = NextNode();

		if(current_node->distance(*end_node) == 0)
		{
			m_Model.path = ConstructFinalPath(end_node);
			return;
		}

		AddNeighbors(current_node);
	}

}