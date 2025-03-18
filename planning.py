import pygame
import heapq
import localization as Local
import numpy as np
import robot_api as Robot
import math
import queue
import cv2


def grid_based_sampling(surface, cell_size=20):
   
    world_width = surface.get_width()
    world_height = surface.get_height()
    sampled_nodes = []

    for x in range(0, world_width, cell_size):
        for y in range(0, world_height, cell_size):
            # Check if there's free space in this cell
            
                nx, ny = x + cell_size // 2, y + cell_size // 2
                if nx < world_width and ny < world_height and surface.get_at((nx, ny))[:3] == (255, 255, 255):
                    sampled_nodes.append((nx,ny))
                   

    return (sampled_nodes)

def is_corner(coordinates, surface): #checking if a pixel is a corner
    neighbors=[]
    directions=[(1,0),(0,1),(-1,0),(0,-1)]
    for direction in directions:
        if surface.get_at((coordinates[0]+direction[0],coordinates[1]+direction[1])) == (255, 255, 255):
            neighbors.append((coordinates[0]+direction[0],coordinates[1]+direction[1]))
            
    if len(neighbors)==2:
        dx1=neighbors[0][0]-coordinates[0]
        dy1=neighbors[0][1]-coordinates[1]
        dx2=neighbors[1][0]-coordinates[0]
        dy2=neighbors[1][1]-coordinates[1]

        if dx1 * dx2 + dy1 * dy2 == 0:
            return True
    
    return False

def is_intersection(coordinates, surface): #checking if a pixel is an intersection
    neighbors=[]
    directions=[(1,0),(0,1),(-1,0),(0,-1)]

    for direction in directions:
        
        if surface.get_at((coordinates[0]+direction[0],coordinates[1]+direction[1])) == (255, 255, 255):
            neighbors.append((coordinates[0]+direction[0],coordinates[1]+direction[1]))
            
    if len(neighbors)>2:
        return True
    
    return False

def get_intersections(surface, world_height, world_width):  #finding all the intersections in the map
    intersections = []
    for  sampled_node in grid_based_sampling(surface):
                if is_intersection(sampled_node, surface):
                    intersections.append(sampled_node)
            
            
    return intersections

def get_nodes(surface, world_height, world_width): #finding all the nodes in the map
    intersection=get_intersections(surface, world_height, world_width)
    
    nodes=intersection
    return nodes   


def check_edge(node1, node2, surface): #VS code literally wrote this for me
    x1, y1 = node1
    x2, y2 = node2
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    if x1 < x2:
        sx = 1
    else:
        sx = -1
    if y1 < y2:
        sy = 1
    else:
        sy = -1
    err = dx - dy
    while True:
        if surface.get_at((x1, y1)) != (255, 255, 255):
            return False
        if x1 == x2 and y1 == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err = err - dy
            x1 = x1 + sx
        if e2 < dx:
            err = err + dx
            y1 = y1 + sy
    return True

def build_graph(nodes,surface, radius=50): #building a graph of the map
    
    graph = {}
    for node in nodes:
        graph[node] = []
    for node1 in nodes:
        for node2 in nodes:
            if math.dist(node1, node2) < radius:
                if node1 != node2 and check_edge(node1, node2, surface):
                    graph[node1].append(node2)
    return graph
def find_nearest_node(agent_pos, graph_nodes):
    return min(graph_nodes, key=lambda node: distance(node, agent_pos))

def distance(node1, node2): #doubles as heuristic cost and neighbor cost
    return (math.dist(node1, node2))



    
class Planner:
    def __init__(self,surface):
        self.surface=surface
        self.world_height = surface.get_height()
        self.world_width = surface.get_width()
        self.localization=Local.Localization(self.world_width, self.world_height)
        self.occupancy_grid=self.localization.occupancy_grid
        self.frontiers=[]
        self.point=None
    def grid_based_sampling(self, cell_size=20):
   
        
        sampled_nodes = []

        for x in range(0, self.world_width, cell_size):
            for y in range(0, self.world_height, cell_size):
                # Check if there's free space in this cell
            
                nx, ny = x + cell_size // 2, y + cell_size // 2
                if nx < self.world_width and ny < self.world_height and self.localization.occupancy_grid[nx,ny] < 1:
                    sampled_nodes.append((nx,ny))
                   
    def get_path(self, surface, world_height, world_width,start, goal):
        nodes=get_nodes(surface, world_height, world_width)
        
        graph=build_graph(nodes, surface) 
        start_node = find_nearest_node(start, nodes)
        goal_node = find_nearest_node(goal, nodes)
        open_list = [] 
        closed_list = []
        came_from = {}
        g_score = {node: float('inf') for node in graph}
        g_score[start_node] = 0
        g_score[goal_node] = float('inf')
        f_score = {node: float('inf') for node in graph}
        f_score[start_node] = distance(start_node, goal_node)
        f_score[goal_node] = float('inf')
        heapq.heappush(open_list, (f_score[start_node], start_node))
        while open_list:
            current = heapq.heappop(open_list)[1]
            if current == goal_node:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_node)
                return path[::-1]
            closed_list.append(current)
            for neighbor in graph[current]:
                if neighbor in closed_list:
                    continue
                tentative_g_score = g_score[current] + distance(current, neighbor)
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + distance(neighbor, goal_node)
                    if neighbor not in open_list:
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))

        
        return []
    

   