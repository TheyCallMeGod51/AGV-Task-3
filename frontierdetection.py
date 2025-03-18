from collections import deque
import pygame
import numpy as np
import cv2
from downsizingarray import DownsizeArray

class ExpandingWavefrontFrontierDetection:
    def __init__(self, grid):
        """
        Initialize the frontier detection algorithm.
        :param grid: 2D occupancy grid (0 = free, 1 = occupied, 0.5 = unknown)
        """
        self.grid = np.array(grid,dtype=np.float32)
        grid=DownsizeArray(grid)

        self.visited = set()  # Track visited cells
        self.frontiers = set()  # Current frontiers
        self.clusters = []  # List of frontier clusters
        scale_factor = 4
        self.small_grid=grid.downsize(scale_factor)
        
        
    def is_free_space(self, cell):
        """Check if a cell is free space."""
        x, y = cell
        return self.small_grid[int(x)][int(y)] == 0

    def is_frontier(self, cell):
        """Check if a cell is a frontier."""
        x, y = cell
        neighbors = self.get_adjacent_cells(cell)
        for nx, ny in neighbors:
            if self.small_grid[int(nx)][int(ny)] == 0.5:  # Unknown space adjacent to free space
                return True
        return False

    def get_adjacent_cells(self, cell):
        x, y = cell
        return [(x+dx, y+dy) for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)] 
            if 0 <= x+dx < self.small_grid.shape[0] 
            and 0 <= y+dy < self.small_grid.shape[1]]

    def kosaraju_dfs(self):
        """Perform Kosaraju's DFS for clustering connected frontiers."""
        visited_clusters = set()
        
        def dfs(cell):
            cluster = []
            stack = [cell]
            while stack:
                current = stack.pop()
                if current not in visited_clusters:
                    visited_clusters.add(current)
                    cluster.append(current)
                    for neighbor in self.get_adjacent_cells(current):
                        if neighbor in self.frontiers and neighbor not in visited_clusters:
                            stack.append(neighbor)
            return cluster

        # Detect clusters using DFS
        self.clusters = []
        for cell in self.frontiers:
            if cell not in visited_clusters:
                new_cluster = dfs(cell)
                self.clusters.append(new_cluster)

    def detect_frontiers(self, robot_starting_cell):
        """
        Detect frontiers using the expanding wavefront algorithm.
        
        :param robot_starting_cell: Tuple (x, y) representing the robot's starting position.
        :return: A list of frontier clusters.
        """
        
        queue = deque([(robot_starting_cell[0]//4, robot_starting_cell[1]//4)])
        
        # Initialize queue based on whether there are existing frontiers
        if not self.frontiers:  # First iteration
           queue = deque([(robot_starting_cell[0]//4, robot_starting_cell[1]//4)]) 
        else:  # Subsequent iterations
            queue=deque(self.frontiers.intersection(self.visited))

        while queue:
            c = queue.popleft()
            if c in self.visited:
                continue

            self.visited.add(c)

            if self.is_free_space(c):
                if self.is_frontier(c):  # Check if it's a new frontier
                    self.frontiers.add(c)
                elif c in self.frontiers:
                    self.frontiers.remove(c)

                # Add unvisited adjacent cells to the queue
                adjacent_cells = set(self.get_adjacent_cells(c)) - self.visited
                queue.extend(adjacent_cells)

        # Perform Kosaraju's DFS to label connected components of frontiers
        self.kosaraju_dfs()

       
        return self.clusters
        """# Convert grid to numpy arrays
        free_mask = (self.grid == 0).astype(np.uint8)
        unknown_mask = (self.grid == 0.5).astype(np.uint8)
    
        # Find frontier regions using morphological operations
        kernel = np.ones((3,3), np.uint8)
        dilated_unknown = cv2.dilate(unknown_mask, kernel)
        frontiers = cv2.bitwise_and(free_mask, dilated_unknown)
    
        # Cluster using connected components
        _, labels = cv2.connectedComponents(frontiers)
        #self.clusters = [np.argwhere(labels == i) 
                  # for i in range(1, np.max(labels)+1)]
        # Replace with:
        unique_labels = np.unique(labels)[1:]  # Exclude background
        sparse_labels = csr_matrix(labels)
        self.clusters = [np.column_stack(sparse_labels.getrow(i).nonzero()) 
                for i in range(1, sparse_labels.shape[0])]"""

        return self.clusters
    """def calculate_medians(self):
        medians = [tuple(np.median(cluster, axis=0).astype(int)) for cluster in frontiers]
        return self._cached_medians"""


    def findclosestmedian(self,pose):
        #Find the closest frontier cluster median to the robot's pose.
        closest_median = None
        closest_distance = float('inf')
        for median in self.calculate_medians():
            distance = np.linalg.norm(np.array(pose[:2]) - np.array(median))
            if distance < closest_distance:
                closest_distance = distance
                closest_median = median
        return closest_median

# Example Usage:

# Example grid (5x5):
# 0 -> Free space
# 1 -> Occupied space
# 0.5 -> Unknown space
'''
grid=np.array([[0,1,0,0],
              [0,1,0,0],
              [0,1,0.5,0],
              [0,0,0,0.5]])



frontierdetect1=ExpandingWavefrontFrontierDetection(grid)
robot_starting_cell=(0,0)

print(frontierdetect1.visited)
frontiers=frontierdetect1.detect_frontiers(robot_starting_cell)

print(frontiers)
grid=np.array([[0,1,0,0],
              [0,1,0,0],
              [0,1,0.5,0],
              [0,0,0,0.5]])
grid_=DownsizeArray(grid)
print(grid_.downsize(4))
frontierdetect2=ExpandingWavefrontFrontierDetection(grid)
frontierdetect2.visited=frontierdetect1.visited
frontiers=frontierdetect2.detect_frontiers(robot_starting_cell)
print(frontiers)'''