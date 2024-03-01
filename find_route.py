#Name: Richard Olu Jordan
#Class: CSE 5360
#Homework: Artificial Intelligence - Search Algorithms

import sys 
import heapq

'''
The Node class represents a city and contains a list of outgoing edges (self.edges).
The Edge class represents a connection between two cities and stores information about the source, destination, and weight of the edge.
The add_edge method in the Node class allows you to add an outgoing edge to the node, creating a relationship between the source and destination cities.
'''

#Node class to represent each vertex in the graph
class Node:
  def __init__(self, name):
    self.name = name
    self.heuristic = None # none initially
    self.edges = []

#addEdge function will represent adding edges into a list called edges which is an attribute of Node class
  def addEdge(self, destination, weight):
     self.edges.append(Edge(self, destination, weight))
    
  def addHeuristic(self, value):
      self.heuristic = value
       
#Edge class to represent each edge in the graph will contain the source Node, destination Node and weight of each edge.  
class Edge:
  def __init__(self, source, destination, weight):
        self.source = source
        self.destination = destination
        self.weight = weight

#Heuristic class will represents the heuristic value needed to build the heuristic graph.
class Heuristic:
    def __init__(self, h_value):
        self.h_value = h_value


#parse_input function will parse our input1.txt which is an edge list with the format "Bremen Dortmund 234"
def parse_input(filename):
    graph = {} #dictionary that will represent graph representation of our input file. #format of graph is {node_name: Node object}
    try:
        with open(filename, 'r') as f:
            for line in f:
                 if line.strip() == "END OF INPUT":
                    break  # exit the loop if we encounter the string END OF INPUT
                 words = line.strip().split(" ") # parse into words... using a space as the delimiter
                 if len(words) == 3: 
                     source_name, dest_name, weight = words
                     weight = float (weight) #convert to float so we can handle input files with decimals. 

                     if source_name not in graph: # check if the name is in the graph if not add name to the graph and create the NODE object associated with it.     
                         graph[source_name] = Node(source_name)
                        
                     if dest_name not in graph: 
                        graph[dest_name] = Node(dest_name) #repeat above steps for the destination name
                  
                     graph[source_name].addEdge(graph[dest_name], weight) #create the edges in the graph using the addEdge function. 
                     graph[dest_name].addEdge(graph[source_name], weight) #create edges going in both directions.                           
    except IOError:
        print(f"Could not read file: {filename}")
    return graph


#this function is used to add the heuristic to the already created graph and save it into another graph called the heuristic_graph
def addHeuristic(filename, graph):
    heuristic_graph = graph
    try:
        with open(filename, 'r') as f:
            for line in f:
                 
                 if line.strip() == "END OF INPUT":
                    break  # exit the loop if we encounter the string END OF INPUT
                 words = line.strip().split(" ") # parse into words... using a space as the delimiter
                 if len(words) == 2: 
                     node_name, heuristic_value = words
                     if node_name in heuristic_graph:
                        h_value = float(heuristic_value)
                        heuristic_graph[node_name].addHeuristic(h_value)

    except IOError:
        print(f"Could not read file: {filename}")
    return heuristic_graph   


def print_fn(graph, nodes_popped,nodes_expanded_counter, nodes_generated, cost, path):
    if cost == -1:
        cost = 'infinity'
    print(f"Nodes Popped: {nodes_popped}")
    print(f"Nodes Expanded: {nodes_expanded_counter}")
    print(f"Nodes Generated: {nodes_generated}")
    print(f"Distance: {cost} km")
    print("Route:")
    if not path:
            print("None")
    else: 
        for i in range(len(path) - 1):
            weight = calculate_cost(graph, path[i], path[i + 1] )
            print(path[i], "to", path[i + 1], ",", weight, "km")

#this function calculates the cost between two nodes   
def calculate_cost(graph, source_node_name, dest_node_name):
    if source_node_name in graph:
        for edge in graph[source_node_name].edges:
            if edge.destination.name == dest_node_name:
                return edge.weight
    return None

def aStar(heuristic_graph, graph, start_node, goal_node):
    g_value = 0 
    f_value = 0
    closed_set = set() 
    
    priority_queue = [(f_value, g_value, start_node)]

    cum_g_value = {start_node: 0}
   

    parents_map = {start_node: None} # currently has no parents meaning root node
    nodes_popped_counter = 0
    nodes_expanded_counter = 0
    node_generated_counter = 1 #we start at one because start node already counts as being generated

    while priority_queue: #means while priority_queue is not empty
        f_value, current_g, current_node = heapq.heappop(priority_queue)
        nodes_popped_counter += 1

        if current_node == goal_node:
            return reconstruct_path(parents_map, start_node, goal_node), nodes_popped_counter,nodes_expanded_counter, node_generated_counter, current_g
        
        if current_node not in closed_set:
            closed_set.add(current_node)
            nodes_expanded_counter += 1 
        
        for edge in graph[current_node].edges: 
            node_generated_counter += 1      
            next_node = edge.destination.name
            next_g = edge.weight + current_g
            next_f = next_g + heuristic_graph[next_node].heuristic
                                
            if next_node not in cum_g_value or next_g < cum_g_value[next_node]:            
                cum_g_value[next_node] = next_g 
                parents_map[next_node] = current_node
                heapq.heappush(priority_queue, (next_f, next_g, next_node))

    return [], nodes_popped_counter,nodes_expanded_counter, node_generated_counter, -1
        

def ucs(graph, start_node, goal_node):
    priority_queue = [(0, start_node)]
    
    ''' Optimized List: This list ensures efficiency by only adding nodes that meet one of two criteria: (a) the node is not already in the list,
      or (b) the node offers a lower cumulative cost than previously recorded versions. This approach minimizes the number of nodes added to the fringe,
        reducing the overall count of nodes we need to process, thereby lowering the nodes_popped counter.
        Non-Optimized List (Dictionary with Lists as Values): Contrary to the optimized approach, this version records 
        every node added to the fringe, regardless of whether it's already present, and irrespective of its cumulative cost being
        lower or higher. As a result, this method increases the nodes_popped count because it leads to processing multiple 
        instances of the same node, each with varying cumulative costs, including those already evaluated and placed in the closed set. ''' 
    cum_cost_optimized_list = {start_node: 0}  # current cost and start node
    cum_cost_non_optimized_list = {start_node: [0]}
    closed_set = set()  # Tracks nodes that have been expanded

    parents_map = {start_node: None} #will be used to reconstruct path
    nodes_popped_counter = 0
    nodes_expanded_counter = 0
    node_generated_counter = 1
        
    while priority_queue:
        current_cost, current_node = heapq.heappop(priority_queue)
        nodes_popped_counter += 1  # Increment because a node is popped

        if current_node == goal_node:
            return reconstruct_path(parents_map, start_node, goal_node), nodes_popped_counter, nodes_expanded_counter, node_generated_counter, current_cost

        # Increment nodes expanded only if the node is being expanded for the first time
        if current_node not in closed_set:
            closed_set.add(current_node)    
            nodes_expanded_counter += 1  
           
            for edge in graph[current_node].edges:
                node_generated_counter += 1
                next_node = edge.destination.name
                cum_next_cost = current_cost + edge.weight

                if next_node not in cum_cost_optimized_list or cum_next_cost < cum_cost_optimized_list[next_node]:
                      # Increment here for each consideration for optimized paths
                    cum_cost_optimized_list[next_node] = cum_next_cost
                    parents_map[next_node] = current_node

                # in this optimization, we keep track of all the nodes pushed onto the fringe by using a dictionary with node names
                # as its keys and a lists of cum_costs as its values.
                if next_node not in cum_cost_non_optimized_list:
                    cum_cost_non_optimized_list[next_node] = [cum_next_cost]
                else:
                    cum_cost_non_optimized_list[next_node].append(cum_next_cost)  
                
                heapq.heappush(priority_queue, (cum_next_cost, next_node))


    return [], nodes_popped_counter, nodes_expanded_counter, node_generated_counter, -1  # If goal not reached


def reconstruct_path(parent_dict, start_node, goal_node):
    path = []
    current = goal_node
    while current != start_node:
        path.append(current)
        current = parent_dict[current]
    path.append(start_node)
    path.reverse()
    return path

def main():
    if len(sys.argv) == 4:
        # Only filename, startNode, and endNode provided: run UCS
        input_filename = sys.argv[1]
        startNode = sys.argv[2]
        endNode = sys.argv[3]
        graph = parse_input(input_filename)
        path, nodes_popped, nodes_expanded_counter, nodes_generated, cost = ucs(graph, startNode, endNode)
        print_fn(graph, nodes_popped, nodes_expanded_counter, nodes_generated, cost, path)
   
    elif len(sys.argv) == 5:
        # Filename, startNode, endNode, and heuristic_filename provided: run A* search
        input_filename = sys.argv[1]
        startNode = sys.argv[2]
        endNode = sys.argv[3]
        heuristic_filename = sys.argv[4]
        graph = parse_input(input_filename)
        heuristic_graph = addHeuristic(heuristic_filename, graph)
        path, nodes_popped, nodes_expanded_counter, nodes_generated, cost = aStar(heuristic_graph, graph, startNode, endNode)
        print_fn(graph, nodes_popped, nodes_expanded_counter, nodes_generated, cost, path)
    else:
        print("Usage: python script.py input_filename.txt startNode endNode [heuristic_filename]")
        sys.exit(1)        

if __name__ == '__main__':
    main()

            
