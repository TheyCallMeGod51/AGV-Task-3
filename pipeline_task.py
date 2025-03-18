"""
API for agent: (Read robot_api.py for more detailed documentation)
agent.move(dist)     : moves the agent in forward direction for dist units. Avoid using values more than 5 the value of distance for one 
					   iteration of work. It will still work but you won't see it being animated (was too much effort implementing that)
agent.scan()         : returns a list of size 180 which gives distance of nearest obstacle/wall for every 2 degrees 
						with 0th element being the distance at degree the agent is facing - 180
agent.rotate(deg)    : rotates the agent by deg degrees
agent.get_imu_data() : gives the direction the agent is heading towards.

(For Subtasks)
agent.get_world() :  gives pygame.Surface of the world, if this.get_at((x, y)) != WHITE then its a wall
agent.get_pos()   : returns coordinates of the agent. Usable only once. (x, y) = (pos[0], pos[1])

Task : This class is supposed to make the two agents meet somewhere on the map. You can create the map yourself in the pygame GUI, you can
save and load maps from your system. Your code will be tested on different maps which I will make randomly lol. You are 
supposed to document your progress. Your approach and effort to solve this problem matters more than the final solution. 
One approach to solve this is to implement localization and mapping to get the map for both agents and then use some kind of map merging algorithm 
to get a transform between the maps and use that to get relative displacement between the two agents and use that to make them meet. Subtasks 
given below are supposed to reduce the number of things you need to do make the agents meet. You can use any resource (pre-written code, LLMs, 
books etc) but you are supposed to mention what you used in your documentation and have a basic understanding of what you did. 

Subtask 1: You can use initial coordinates of both agents and the world surface map
Subtask 2: In conditions of subtask 1, ensure a smooth path. Read about Dubin Curves or Reeds-Shepp
Subtask 3: You can use initial coordinates of both agents but not the map
Subtask 4: You can use the map but not coordinates of either of agents
Subtask 5: You can use only scan and imu data.

Subtask 6 (Bonus): Do this optimally without scanning the full map if possible. (Some image processing techniques may be used)

If you do Subtask x, you get points for all Subtask y such that y <= x

GUI:
You can use WASD and arrow keys to control the agents. It will be helpful in debugging (or you can just play around lol)
Add Walls : duh, it adds walls. You can use a brush to add walls to the map.
Remove Walls : You can use a brush to remove walls from the map.
Start Pipeline : It basically runs the work function in this class
View : There are two views 
		Full -> in this mode the normal map with brown walls and white background is shown
		Explored -> this shows the area that the scan has explored till now in white color rest is in black
Upload Map : This can be used to upload a .png file as map. Note: stuff might get buggy if you upload something that you didn't save from the GUI but essentially anything with brown rgb(181, 101, 29) walls and white backgruond should work. 
Save Map : This saves the current map as a .png file. 

Note: If you find any bugs, try to fix them and write about them in your documentation and DM me (aryanr_) on discord
"""

from localization import Localization
from planning import Planner
import math
from robot_api import RobotAPI
import numpy as np
from frontierdetection import ExpandingWavefrontFrontierDetection
from downsizingarray import DownsizeArray
from map_merger import MapMerger
import random
def rotate_to_target(agent, target_angle):
		current_angle = agent.get_imu_data()
		anglediff=(target_angle-current_angle+180)%360-180
		agent.rotate(anglediff)
		return None

def calculate_angle_to_target(agent_pos, target_pos):
    dx = target_pos[0] - agent_pos[0]
    dy = target_pos[1] - agent_pos[1]
    angle_to_target = math.degrees(math.atan2(dy, dx))
    return angle_to_target

def move_to_target(agent, target_pos, step_size=5):
    agent_pos = agent.get_pos()
    distance = math.sqrt((target_pos[0] - agent_pos[0])**2 + (target_pos[1] - agent_pos[1])**2)
    move_distance = min(distance, step_size)
    agent.move(move_distance)
    return move_distance

def navigate_to_node(agent, target_node):
    agent_pos = agent.get_pos()
    angle_to_target = calculate_angle_to_target(agent_pos, target_node)
    rotate_to_target(agent, angle_to_target)
    distance=move_to_target(agent, target_node)
    return distance

'''def random_walk(agent, steps=5, step_size=5):
    for _ in range(steps):
        rotation = random.uniform(-90, 90)
        agent.rotate(rotation)
        agent.move(step_size)
        if agent.scan()[0] < 5:  # If close to a wall, turn around
            agent.rotate(180)
            agent.move(step_size)

def is_stuck(agent, threshold=5, iterations=3):
    positions = []
    for _ in range(iterations):
        positions.append(agent.get_pos())
        agent.move(1)  # Small movement to check if stuck
    
    max_distance = max(math.dist(positions[0], pos) for pos in positions)
    return max_distance < threshold'''

class Pipeline:
	def __init__(self, world_width, world_height, map):
		# initial position will have to be assumed to be (world_width, world_height)
		self.map = map
		self.explored_map1 = Localization(2 * world_width, 2 * world_height) # map explored by agent 1
		self.detector_agent1 = ExpandingWavefrontFrontierDetection(self.explored_map1.occupancy_grid)
		self.previousfrontiers = set()
		self.visitedfrontiers = set()
		self.explored_map2 = Localization(2 * world_width, 2 * world_height) # map explored by agent 2
		self.planner1 = Planner(self.explored_map1.map) # planner for agent 1
		self.planner2 = Planner(self.explored_map2.map) # planner for agent 2
		self.map_merger = MapMerger()
		self.path_agent1 = None
		self.path_agent2 = None
		self.visited = set()
		self.ind1 = 0
		self.ind2 = 0
		self.count = 0
		self.world_height = world_height
		self.world_width = world_width
		return

	def reset(self):
		return

	def work(self, agent1, agent2):
		if self.explored_map1.pose==[None,None,0]:
			self.explored_map1.initialize_pose(agent1)
		if self.explored_map2.pose==[None,None,0]:
			self.explored_map2.initialize_pose(agent2)
	
		self.explored_map1.update(agent1)
		self.explored_map2.update(agent2)
		
		occupancygrid1 = self.explored_map1.occupancy_grid
		occupancygrid2 = self.explored_map2.occupancy_grid
		print(len(occupancygrid1))
		count1 = 0
		count2 = 0
		for j in occupancygrid1:
			for i in j:
				if i == 0:
					count1 += 1
		for j in occupancygrid2:
			for i in j:
				if i == 0:
					count2 += 1
		print(count1, count2)
		self.detector_agent1 = ExpandingWavefrontFrontierDetection(occupancygrid1)
		self.detector_agent2 = ExpandingWavefrontFrontierDetection(occupancygrid2)
		
		print(len(self.detector_agent1.small_grid))
		print(len(self.detector_agent2.small_grid))
		
		frontiers1 = self.detector_agent1.detect_frontiers(agent1.get_pos())
		frontiers2 = self.detector_agent2.detect_frontiers(agent2.get_pos())
		
		if not frontiers1:
			agent1.rotate(45)
			agent1.move(5)
			agent1.move(5)
			agent1.move(5)
			agent1.move(5)
			agent1.move(5)
			agent1.move(5)
			agent1.move(5)
			agent1.move(5)
			agent1.move(5)
			agent1.move(5)
			# self.explored_map1.pose=[self.explored_map1.pose[0]+(50*math.cos(agent1.get_imu_data())),self.explored_map1.pose[1]+(50*math.sin(agent1.get_imu_data())),agent1.get_imu_data()]
			print("No frontiers for agent1")
			return
		
		if not frontiers2:
			agent2.rotate(45)
			agent2.move(5)
			agent2.move(5)
			agent2.move(5)
			agent2.move(5)
			agent2.move(5)
			agent2.move(5)
			agent2.move(5)
			agent2.move(5)
			agent2.move(5)
			agent2.move(5)
			# self.explored_map1.pose=[self.explored_map2.pose[0]+(50*math.cos(agent1.get_imu_data())),self.explored_map2.pose[1]+(50*math.sin(agent1.get_imu_data())),agent1.get_imu_data()]
			print("No frontiers for agent2")
			return
		
		medians1 = []
		medians2 = []
		distances1 = []
		distances2 = []
		
		for cluster in frontiers1:
			medians1.append(tuple(4 * np.median(cluster, axis=0).astype(int)))
		for cluster in frontiers2:
			medians2.append(tuple(4 * np.median(cluster, axis=0).astype(int)))
		
		print(medians1)
		print(medians2)
		
		closest_median1 = None
		closest_median2 = None
		closest_distance1 = float('inf')
		closest_distance2 = float('inf')
		
		for median in medians1:
			distance = math.dist(agent1.get_pos(), median)
			if tuple(median) not in self.visited:
				distances1.append(distance)
			else:
				distances1.append(float('inf'))
			if distance < closest_distance1 and tuple(median) not in self.visited:
				closest_distance1 = distance
				closest_median1 = median
		
		for median in medians2:
			distance = math.dist(agent2.get_pos(), median)
			if tuple(median) not in self.visited:
				distances2.append(distance)
			else:
				distances2.append(float('inf'))
			if distance < closest_distance2 and tuple(median) not in self.visited:
				closest_distance2 = distance
				closest_median2 = median
		
		print(len(distances1))
		print(len(distances2))
		print(closest_median1, agent1.get_pos())
		print(closest_median2, agent2.get_pos())
		
		if closest_median1 is None:
			agent1.rotate(45)
			agent1.move(5)
			print("Dead medians for agent1")
			return
		
		if closest_median2 is None:
			agent2.rotate(45)
			agent2.move(5)
			print("Dead medians for agent2")
			return
		
		self.path_agent1 = self.planner1.get_path(self.explored_map1.map, 800, 800, agent1.get_pos(), closest_median1)
		self.path_agent2 = self.planner2.get_path(self.explored_map2.map, 800, 800, agent2.get_pos(), closest_median2)
		print(self.ind1, self.path_agent1)
		print(self.ind2, self.path_agent2)
		
		if self.ind1 >= len(self.path_agent1):
			self.ind1 = 0
		
		if self.ind2 >= len(self.path_agent2):
			self.ind2 = 0
		
		if self.path_agent1==[]:
			print("No path for agent 1")
			return
		
		if self.path_agent2==[]:
			print("No path for agent 2")
			return
		if agent1.scan()[0] < 5:
			self.visited.add(closest_median1)
			
			agent1.rotate(45)
			return
		
		if agent2.scan()[0] < 5:
			self.visited.add(closest_median2)
			
			agent2.rotate(45)
			return
	
		dist1=navigate_to_node(agent1, self.path_agent1[self.ind1])
		dist2=navigate_to_node(agent2, self.path_agent2[self.ind2])
		self.explored_map1.pose=[self.explored_map1.pose[0]+(dist1*math.cos(math.radians(agent1.get_imu_data()))),self.explored_map1.pose[1]+(dist1*math.sin(math.radians(agent1.get_imu_data()))),agent1.get_imu_data()]
		self.explored_map2.pose=[self.explored_map2.pose[0]+(dist2*math.cos(math.radians(agent2.get_imu_data()))),self.explored_map2.pose[1]+(dist2*math.sin(math.radians(agent2.get_imu_data()))),agent2.get_imu_data()]
		print(agent1.get_pos(),self.explored_map1.pose)
		if math.dist(agent1.get_pos(), self.path_agent1[self.ind1]) < 5:
			print("True for agent1")
			self.ind1 += 1
		
		if math.dist(agent2.get_pos(), self.path_agent2[self.ind2]) < 5:
			print("True for agent2")
			self.ind2 += 1
		
		if self.ind1 >= len(self.path_agent1):
			print("True 2 for agent1")
			self.ind1 = 0
			self.visited.add(closest_median1)
			self.path_agent1 = None
			return
		
		if self.ind2 >= len(self.path_agent2):
			print("True 2 for agent2")
			self.ind2 = 0
			self.visited.add(closest_median2)
			self.path_agent2 = None
			return
		
		
		merged_map,affine_matrix=self.map_merger.merge_maps(occupancygrid1, occupancygrid2)
		print(merged_map,affine_matrix)
		flag=True
		for i in range(1):
			for j in range(1):
				if i==j:
					print(affine_matrix[i][j])
					if not math.isclose(affine_matrix[i][j],1,abs_tol=1e-2):
						
						flag=False

				else:
					if not math.isclose(affine_matrix[i][j],0,abs_tol=1e-2):
						flag=False


		if flag==True:		 
			ty=affine_matrix[0][1]
			tx=affine_matrix[1][1]
			self.planner1.get_path

			print("Maps merged")
		return

