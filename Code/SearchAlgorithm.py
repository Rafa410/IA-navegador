# This file contains all the required routines to make an A* search algorithm.
#
__authors__ = '1533434'
__group__ = 'DL.10'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2016- 2017
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from utils import *
import os
import math
import copy


def expand(path, map):
    """
     It expands a SINGLE station and returns the list of class Path.
     Format of the parameter is:
        Args:
            path (object of Path class): Specific path to be expanded
            map (object of Map class):: All the information needed to expand the node
        Returns:
            path_list (list): List of paths that are connected to the given path.
    """
    path_list = []
    for connection in map.connections[path.last]:
        # new_path = Path(path.route.copy())
        # new_path.add_route(connection)
        # new_path.update_g(path.g)
        new_path = copy.deepcopy(path)
        new_path.add_route(connection)
        path_list.append(new_path)

    return path_list


def remove_cycles(path_list):
    """
     It removes from path_list the set of paths that include some cycles in their path.
     Format of the parameter is:
        Args:
            path_list (LIST of Path Class): Expanded paths
        Returns:
            path_list (list): Expanded paths without cycles.
    """ 
    path_list = [ 
        path 
        for path in path_list 
        if all(station != path.last 
            for station in path.route[:-1]) 
    ]

    return path_list


def insert_depth_first_search(expand_paths, list_of_path):
    """
     expand_paths is inserted to the list_of_path according to DEPTH FIRST SEARCH algorithm
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            list_of_path (LIST of Path Class): The paths to be visited
        Returns:
            list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    return expand_paths + list_of_path


def depth_first_search(origin_id, destination_id, map):
    """
     Depth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): the route that goes from origin_id to destination_id
    """
    list_of_path = [Path(origin_id)]
    
    while list_of_path and list_of_path[0].last != destination_id:
        head = list_of_path[0]
        new_path = expand(head, map)
        new_path = remove_cycles(new_path)
        list_of_path = insert_depth_first_search(new_path, list_of_path[1:])

    if list_of_path:
        return list_of_path[0]
    else:
        return 'No solution found'


def insert_breadth_first_search(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to BREADTH FIRST SEARCH algorithm
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    return list_of_path + expand_paths


def breadth_first_search(origin_id, destination_id, map):
    """
     Breadth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    list_of_path = [Path(origin_id)]
    
    while list_of_path and list_of_path[0].last != destination_id:
        head = list_of_path[0]
        new_path = expand(head, map)
        new_path = remove_cycles(new_path)
        list_of_path = insert_breadth_first_search(new_path, list_of_path[1:])

    if list_of_path:
        return list_of_path[0]
    else:
        return 'No solution found'


def calculate_cost(expand_paths, map, type_preference=0):
    """
         Calculate the cost according to type preference
         Format of the parameter is:
            Args:
                expand_paths (LIST of Paths Class): Expanded paths
                map (object of Map class): All the map information
                type_preference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
            Returns:
                expand_paths (LIST of Paths): Expanded path with updated cost
    """
    if type_preference == 0: # Adjacency
        for path in expand_paths:
            path.update_g(1)

    elif type_preference == 1: # Minimum time
        for path in expand_paths:
            time = map.connections[path.penultimate][path.last]
            path.update_g(time)

    elif type_preference == 2: # Minimum distance
        for path in expand_paths:
            if map.stations[path.penultimate]['line'] == map.stations[path.last]['line']:
                velocity = map.stations[path.penultimate]['velocity']
                time = map.connections[path.penultimate][path.last]
                dist = velocity * time
                path.update_g(dist)

    elif type_preference == 3: # Minimum transfers
        for path in expand_paths:
            if map.stations[path.penultimate]['line'] != map.stations[path.last]['line']:
                path.update_g(1)

    return expand_paths
    

def insert_cost(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to COST VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to cost
    """
    sorted_list = sorted(list_of_path + expand_paths, key = lambda path: path.g)
    return sorted_list
    

def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    """
     Uniform Cost Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    list_of_path = [Path(origin_id)]
    
    while list_of_path and list_of_path[0].last != destination_id:
        head = list_of_path[0]
        new_path = expand(head, map)
        new_path = remove_cycles(new_path)
        new_path = calculate_cost(new_path, map, type_preference)
        list_of_path = insert_cost(new_path, list_of_path[1:])

    if list_of_path:
        return list_of_path[0]
    else:
        return 'No solution found'


def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    """
     Calculate and UPDATE the heuristics of a path according to type preference
     WARNING: In calculate_cost, we didn't update the cost of the path inside the function
              for the reasons which will be clear when you code Astar (HINT: check remove_redundant_paths() function).
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            expand_paths (LIST of Path Class): Expanded paths with updated heuristics
    """
    if type_preference == 0: # Adjacency
        for path in expand_paths:
            if path.last != destination_id:
                path.update_h(1)
            else:
                path.update_h(0)

    elif type_preference == 1: # Minimum time
        velocity = max(map.velocity.values())
        for path in expand_paths:
            dist = euclidean_dist([map.stations[path.last]['x'], map.stations[path.last]['y']], 
                                       [map.stations[destination_id]['x'], map.stations[destination_id]['y']])
            time = dist / velocity
            path.update_h(time)

    elif type_preference == 2: # Minimum distance
        for path in expand_paths:
            dist = euclidean_dist([map.stations[path.last]['x'], map.stations[path.last]['y']], 
                                  [map.stations[destination_id]['x'], map.stations[destination_id]['y']])
            path.update_h(dist)

    elif type_preference == 3: # Minimum transfers
        for path in expand_paths:
            if map.stations[path.last]['line'] != map.stations[destination_id]['line']:
                path.update_h(1)
            else:
                path.update_h(0)

    return expand_paths


def update_f(expand_paths):
    """
      Update the f of a path
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
         Returns:
             expand_paths (LIST of Path Class): Expanded paths with updated costs
    """
    for path in expand_paths:
        path.update_f()

    return expand_paths


def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
      It removes the Redundant Paths. They are not optimal solution!
      If a station is visited and have a lower g in this moment, we should remove this path.
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
             list_of_path (LIST of Path Class): All the paths to be expanded
             visited_stations_cost (dict): All visited stations cost
         Returns:
             new_paths (LIST of Path Class): Expanded paths without redundant paths
             list_of_path (LIST of Path Class): list_of_path without redundant paths
    """
    new_paths = []
    for path_exp in expand_paths:
        if path_exp.last in visited_stations_cost:
            if visited_stations_cost[path_exp.last] > path_exp.g:
                list_of_path = [
                    path
                    for path in list_of_path
                    if path.last != path_exp.last
                ]
                new_paths.append(path_exp)
                visited_stations_cost[path_exp.last] = path_exp.g
        else:
            new_paths.append(path_exp)
            visited_stations_cost[path_exp.last] = path_exp.g

    return new_paths, list_of_path, visited_stations_cost


def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """
    sorted_list = sorted(list_of_path + expand_paths, key = lambda path: path.f)
    return sorted_list


def coord2station(coord, map):
    """
        From coordinates, it searches the closest station.
        Format of the parameter is:
        Args:
            coord (list):  Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            possible_origins (list): List of the Indexes of stations, which corresponds to the closest station
    """
    list_stations = []
    dist_min = INF

    for station_id in map.stations:
        dist = euclidean_dist(coord, [map.stations[station_id]['x'], map.stations[station_id]['y']])
        if dist < dist_min:
            dist_min = dist
            list_stations.clear()
            list_stations.append(station_id)
        elif dist == dist_min:
            list_stations.append(station_id)

    return list_stations


def Astar(origin_coor, dest_coor, map, type_preference = 0):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_coor (list): Starting coordinates
            dest_coor (int): Final coordinates
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    origin_id = coord2station(origin_coor, map)[0]
    destination_id = coord2station(dest_coor, map)[0]

    list_of_path = [Path(origin_id)]
    visited_stations_cost = {}

    while list_of_path and list_of_path[0].last != destination_id:
        head = list_of_path[0]
        new_path = expand(head, map)
        new_path = remove_cycles(new_path)
        new_path = calculate_cost(new_path, map, type_preference)
        new_path = calculate_heuristics(new_path, map, destination_id, type_preference)
        new_path = update_f(new_path)
        new_path, list_of_path, visited_stations_cost = remove_redundant_paths(new_path, list_of_path, visited_stations_cost)
        list_of_path = insert_cost_f(new_path, list_of_path[1:])

    if list_of_path:
        return list_of_path[0]
    else:
        return 'No solution found'


def coord2station_improved(coord, destination_id, map, type_preference = 0):
    """
        From coordinates, it searches the most optimal station.
        Format of the parameter is:
        Args:
            coord (list):  Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            possible_origins (list): List of the Indexes of stations, which corresponds to the closest station
    """
    closest_stations = [[0, INF], [0, INF], [0, INF]] # List of 3 closest stations [station_id, cost]

    for station_id in map.stations: # Find the 3 closest stations to the origin coordinates
        dist = euclidean_dist(coord, [map.stations[station_id]['x'], map.stations[station_id]['y']]) # Distance from current coordinates to station
        if dist < closest_stations[0][1]: # Check if the distance of the current station is lower than the first closest station
            closest_stations[2] = closest_stations[1]
            closest_stations[1] = closest_stations[0]
            closest_stations[0] = [station_id, dist]
        elif dist < closest_stations[1][1]: # Check if is lower than the second closest station
            closest_stations[2] = closest_stations[1]
            closest_stations[1] = [station_id, dist]
        elif dist < closest_stations[2][1]: # Check if is lower than the third closest station
            closest_stations[2] = [station_id, dist]


    walking_speed = 5

    for station in closest_stations: # Calculate cost and heuristics for the 3 closest stations
        station[1] = calculate_heuristics([Path(station[0])], map, destination_id, type_preference)[0].h
        walking_dist = euclidean_dist(coord, [map.stations[station[0]]['x'], map.stations[station[0]]['y']]) # Distance between current position and station
        
        if type_preference == 1: # Minimum time
            time = walking_dist / walking_speed # Time to arrive at the station walking
            station[1] += time # station[1] = heuristic + cost

        elif type_preference == 2: # Minimum distance
            station[1] += walking_dist #station[1] = heuristic + cost
    

    optimal_station = min(closest_stations, key = lambda station: station[1])[0] # Find the most optimal station among the closest 3, taking into account the cost and heuristics (station[1])
    
    return optimal_station


def Astar_improved(origin_coor, dest_coor, map, type_preference = 0):
    """
     A* improved search algorithm
     Format of the parameter is:
        Args:
            origin_coor (list): Starting coordinates
            dest_coor (int): Final station coordinates
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    destination_id = coord2station(dest_coor, map)[0]
    origin_id = coord2station_improved(origin_coor, destination_id, map, type_preference) # Finds the most optimal station

    list_of_path = [Path(origin_id)]
    visited_stations_cost = {}

    while list_of_path and list_of_path[0].last != destination_id:
        head = list_of_path[0]
        new_path = expand(head, map)
        new_path = remove_cycles(new_path)
        new_path = calculate_cost(new_path, map, type_preference)
        new_path = calculate_heuristics(new_path, map, destination_id, type_preference)
        new_path = update_f(new_path)
        new_path, list_of_path, visited_stations_cost = remove_redundant_paths(new_path, list_of_path, visited_stations_cost)
        list_of_path = insert_cost_f(new_path, list_of_path[1:])

    if list_of_path:
        return list_of_path[0]
    else:
        return 'No solution found'
        
