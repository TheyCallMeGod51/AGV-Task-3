import pygame
from robot_api import RobotAPI
import numpy as np
import math
from frontierdetection import ExpandingWavefrontFrontierDetection
WHITE = (255, 255, 255)

class Localization:
    def __init__(self, world_width, world_height):
        pygame.init()
        self.screen=pygame.display.set_mode((world_width, world_height))
        self.map = pygame.Surface((world_width, world_height)) 
        self.map.fill((0,0,0))
        self.world_height = world_height
        self.world_width = world_width
        self.initial_pos=None
        self.pose=[None,None, 0]
        self.occupancy_grid=np.full((world_width, world_height), 0.5, dtype=float)
        self.screen.blit(self.map, (0, 0))
       

    def initialize_pose(self,agent):
        self.initial_pos = (self.world_height,self.world_width)
        self.pose = [self.initial_pos[0], self.initial_pos[1], 0]
        
        return True



    def update(self, agent):
        lidar_data=agent.scan()
        current_angle=agent.get_imu_data()
        
        for i in range(len(lidar_data)):
            self._update_ray(agent.get_pos()[0],agent.get_pos()[1], current_angle-180+i*2, lidar_data[i])

        self.screen.blit(self.map, (0, 0))

        return
    
    def _update_ray(self, x, y, angle, distance):
        angle_rad=math.radians(angle)
        max_range=min(distance, self.world_width/2)
        
        step = 1.0
        distance = 0.0
        while distance < max_range:
            test_x = x + distance * math.cos(angle_rad)
            test_y = y + distance * math.sin(angle_rad)
            ix, iy = int(test_x), int(test_y)
            
            if ix < 0 or iy < 0 or ix >= self.world_width or iy >= self.world_height:
                return
            else:
                self.occupancy_grid[ix,iy]=0
                self.map.set_at((ix,iy),WHITE)
            distance += step
        if max_range!=self.world_width/2:
            self.occupancy_grid[int(x + max_range * math.cos(angle_rad)),int(y + max_range * math.sin(angle_rad))]=1
        return
    
    '''def frontier_detection(self,grid):
        frontiers=ExpandingWavefrontFrontierDetection.detect_frontiers(self.occupancy_grid)
        return frontiers'''
        
    def at(self, x, y):
        return self.map.get_at((int(x), int(y)))
    
#debugging
'''run=True
localization=Localization(800, 800)
world_width = 800
world_height = 800
shared_world = pygame.Surface((world_width, world_height))
agent1 = RobotAPI(world_width, world_height, shared_world, start_pos=[world_width * 0.6, world_height * 0.6])
while run: 
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
    
    localization.update(agent1)
    pygame.display.flip()

occupancy_grid=localization.occupancy_grid
count=0
for i in occupancy_grid:
    for j in i:
        if j==0:
            count+=1
print(count)
frontiers=localization.frontier_detection(occupancy_grid)
print(frontiers)
pygame.quit()
'''