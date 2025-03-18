import numpy as np
import cv2
class DownsizeArray:
    def __init__(self, grid):
        """
        Initialize the downsizing array algorithm.
        :param grid: 2D occupancy grid (0 = free, 1 = occupied, 0.5 = unknown)
        """
        self.grid = np.array(grid,dtype=np.float32)
        
    
    def downsize(self,scale_factor):
        """Downsize the array by a scale factor."""
        
        return cv2.resize(self.grid, (0,0), fx=1/scale_factor, fy=1/scale_factor, interpolation=cv2.INTER_NEAREST)
    

#test
'''grid = np.array([
    [0, 0, 0.5, 0.5, 0, 0, 0.5, 0],
    [0, 0, 0.5, 0.5, 0, 0, 0.5, 0],
    [0, 0, 0, 0, 0, 0, 0.5, 0],
    [0, 0, 0, 0, 0, 0, 0.5, 0],
    [0, 0, 0.5, 0.5, 0, 0, 0.5, 0],
    [0, 0, 0.5, 0.5, 0, 0, 0.5, 0],
    [0, 0, 0.5, 0.5, 0, 0, 0.5, 0],
    [0, 0, 0.5, 0.5, 0, 0, 0.5, 0]
],dtype=np.float32)
print(grid)
downsize_array = DownsizeArray(grid)
print(downsize_array.downsize(4))'''
