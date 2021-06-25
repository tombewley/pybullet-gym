import pybullet as p
import os


class Object:
    def __init__(
        self, 
        client, 
        path:str, 
        basePosition:list=[0,0,0], 
        baseOrientation:list=[0,0,0,1],
        ):
        self.client = client
        self.id = p.loadURDF(
            fileName=os.path.join(os.path.dirname(__file__), path),
            basePosition=basePosition,
            baseOrientation=baseOrientation,
            physicsClientId=self.client
            )