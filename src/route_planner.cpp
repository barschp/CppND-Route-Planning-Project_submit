#include "route_planner.h"
#include <algorithm>

bool Compare(const RouteModel::Node* first_node, const RouteModel::Node* second_node){
    float f_first = first_node->g_value + first_node->h_value;
    float f_second = second_node->g_value + second_node->h_value;
    return f_first > f_second;
}

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &model.FindClosestNode(start_x, start_y);
    // std::cout << "This is start x: " << start_node->x << " and this is start y: " << start_node->y << "\n";
    end_node = &model.FindClosestNode(end_x, end_y);
    // std::cout << "This is end x: " << end_node->x << " and this is end y: " << end_node->y << "\n";
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Populate all neighbors of current_node
    current_node->FindNeighbors();
    for (RouteModel::Node *current_neighbor : current_node->neighbors) {
        // storing the g_value to not having to calculate it multiple time
        float possible_g_value = current_node->g_value + current_node->distance(*current_neighbor);
        // in case the neighbor is not yet in the open_list
        if (!current_neighbor->visited){
            current_neighbor->parent = current_node;
            current_neighbor->h_value = this->CalculateHValue(current_neighbor);
            current_neighbor->g_value = possible_g_value;
            current_neighbor->visited = true;
            open_list.push_back(current_neighbor);        
        } 
        // the node is already in the open list --> see if we found a faster way to reach it
        else if (current_neighbor->g_value > possible_g_value) {
            // no need to recalculate h_value and to set visited
            // OPEN QUESTION: What if the node has already been removed from the open list, because it was processed?
            current_neighbor->parent = current_node;
            current_neighbor->g_value = possible_g_value;
        }
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);

    // TODO_self: I should add some handling in case the open list is empty and there is no next node
    RouteModel::Node *next_Node = open_list.back();
    open_list.pop_back();
    return next_Node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.

    // adding the end node
    path_found.push_back(*current_node);

    // initially calculate distance to parent
    distance += current_node->distance(*current_node->parent);

    // initially calculating the previous node
    RouteModel::Node *previous_node = current_node->parent;

    // iterating through all nodes between end and start node
    while (previous_node != start_node){
        path_found.push_back(*previous_node);
        distance += previous_node->distance(*previous_node->parent);
        previous_node = previous_node->parent;
    }

    // adding the start node
    path_found.push_back(*previous_node);
    
    // reversing the vector
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
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

    // TODO: Implement your solution here.

    // initialize with start_node
    current_node = start_node;

    current_node->h_value = this->CalculateHValue(current_node);
    current_node->g_value = 0;
    current_node->visited = true;

    // iterate until we found the end node
    while (current_node != end_node)
    {
        AddNeighbors(current_node);

        // TODO_self: I should enter handling in case the open list gets empty / there is no next node
        current_node = NextNode();
    }
    
    // construct final path
    m_Model.path = ConstructFinalPath(current_node);
}