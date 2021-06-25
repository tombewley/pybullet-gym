from ..assets.car import Car
from ..assets.object import Object

import gym
import numpy as np
import math
import pybullet as p
import matplotlib.pyplot as plt

class SimpleDrivingEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(self, 
                 gui=False,
                 obstacles=[(-3,-3),(-3,3),(3,-3),(3,3)]
                 ):
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        self.action_space = gym.spaces.box.Box(
            low=np.array([-0.25, -.6], dtype=np.float32),
            high=np.array([1, .6], dtype=np.float32))
        self.observation_space = gym.spaces.box.Box(
            low=np.array([-10, -10, -1, -1, -5, 5, 0, -math.pi], dtype=np.float32),
            high=np.array([10, 10, 1, 1, -5, 5, 20, math.pi], dtype=np.float32))
        self.obstacles = obstacles
        self.dt = 1/30
        p.setTimeStep(self.dt, self.client)
        self.rendered_img = None
        self.seed()

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -10)
        # Pick a random goal location
        ang = 2*np.pi*np.random.rand()
        rad = 5.5 + 4*np.random.rand()
        x = rad * np.cos(ang)
        y = rad * np.sin(ang)
        self.goal = [x, y, 0]
        # Pick a random orientation for the car.
        ori = 2*np.pi*np.random.rand()
        # Load car and passive objects.
        self.car = Car(self.client, basePosition=[0,0,0.1], baseRotation=[0,0,ori])
        Object(self.client, "simpleplane.urdf")
        Object(self.client, "simplegoal.urdf", basePosition=self.goal)
        Object(self.client, "wall.urdf", basePosition=[0,-10,0])
        Object(self.client, "wall.urdf", basePosition=[0,10,0])
        Object(self.client, "wall.urdf", basePosition=[-10,0,0], baseRotation=[0,0,np.pi/2])
        Object(self.client, "wall.urdf", basePosition=[10,0,0], baseRotation=[0,0,np.pi/2])
        for x, y in self.obstacles:
            Object(self.client, "obstacle.urdf", basePosition=[x,y,0])
        # Get observation to return
        obs = self.get_observation()
        self.prev_dist_to_goal = obs[-2]
        return obs

    def step(self, action):
        # Feed action to the car and get observation of car's state
        assert action in self.action_space
        self.car.apply_action(action, self.dt)
        for _ in range(3): 
            p.stepSimulation() # <<< NOTE: Multiple sim steps.
        obs = self.get_observation()
        # Compute reward as L2 change in distance to goal
        dist_to_goal = obs[-2]
        reward = self.prev_dist_to_goal - dist_to_goal
        self.prev_dist_to_goal = dist_to_goal
        done = False
        # Done by running off boundaries
        if (obs[0] >= 10 or obs[0] <= -10 or obs[1] >= 10 or obs[1] <= -10):
            done = True
        # Done by reaching goal
        elif dist_to_goal < 1:
            done = True
            reward = 50
        return obs, reward, done, dict()

    def get_observation(self):
        pos, ang, vel = self.car.get_pos_ang_vel()
        ori = (math.cos(ang), math.sin(ang))
        # speed = math.sqrt(vel[0]**2 + vel[1]**2)
        dist_to_goal = math.sqrt((pos[0] - self.goal[0])**2 + (pos[1] - self.goal[1])**2)
        ang_to_goal = math.atan2(pos[1] - self.goal[1], pos[0] - self.goal[0])
        rel_ang_to_goal = ((ang_to_goal - ang) % (2*math.pi)) - math.pi
        return np.array(pos + ori + vel + (dist_to_goal, rel_ang_to_goal))

    def render(self, mode="human", pause=1e-5):        
        proj_matrix = p.computeProjectionMatrixFOV(fov=80, aspect=1, nearVal=0.01, farVal=100)
        resolution = 200
        if False:
            # Base information
            pos, ori = [list(l) for l in p.getBasePositionAndOrientation(self.car.id, self.client)]
            pos[2] = 0.2
            # Rotate camera direction
            rot_mat = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
            camera_vec = np.matmul(rot_mat, [1, 0, 0])
            up_vec = np.matmul(rot_mat, np.array([0, 0, 1]))
        if True:
            pos = np.array([0, 0, 12])
            camera_vec = np.array([0, 0, -1])
            up_vec = np.array([0, 1, 0])
        view_matrix = p.computeViewMatrix(pos, pos + camera_vec, up_vec)
        # Construct image
        frame = p.getCameraImage(resolution, resolution, view_matrix, proj_matrix)[2]
        frame = np.reshape(frame, (resolution, resolution, 4))
        if mode == "human":
            if self.rendered_img is None:
                self.rendered_img = plt.imshow(np.zeros((100, 100, 4)))
            self.rendered_img.set_data(frame)
            plt.draw()
            plt.pause(pause)
        elif mode == "rgb_array": return frame

    def close(self): pass
        # p.disconnect(self.client)