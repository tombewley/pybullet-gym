from .object import Object

import pybullet as p
import os


class Car(Object):
    def __init__(self, client, basePosition, baseOrientation):
        super(Car, self).__init__(client, "simplecar.urdf", basePosition, baseOrientation)
        # Joint indices as found by p.getJointInfo()
        self.steering_joints = [0,2]
        self.drive_joints = [1,3,4,5]
        # Joint speed
        self.joint_speed = 0
        # Drag constants
        self.c_rolling = 0.2
        self.c_drag = 0.01
        # Throttle constant increases speed of the car
        self.c_throttle = 20

    def apply_action(self, action, dt):
        # Expects action to be two dimensional
        throttle, steering_angle = action

        # Clip throttle and steering angle to reasonable values
        throttle = min(max(throttle, -0.25), 1)
        steering_angle = max(min(steering_angle, 0.6), -0.6)

        # Set the steering joint positions
        p.setJointMotorControlArray(self.id, self.steering_joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=[steering_angle] * 2,
                                    physicsClientId=self.client)

        # Calculate drag / mechanical resistance ourselves
        # Using velocity control, as torque control requires precise models 
        friction = -self.joint_speed * (self.joint_speed * self.c_drag + self.c_rolling)
        acceleration = self.c_throttle * throttle + friction
        self.joint_speed = self.joint_speed + acceleration * dt

        # Set the velocity of the wheel joints directly
        p.setJointMotorControlArray(
            bodyUniqueId=self.id,
            jointIndices=self.drive_joints,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=[self.joint_speed] * 4,
            forces=[1.2] * 4,
            physicsClientId=self.client)

    def get_pos_ang_vel(self):
        pos, ang = p.getBasePositionAndOrientation(self.id, self.client)
        ang = p.getEulerFromQuaternion(ang)        
        vel = p.getBaseVelocity(self.id, self.client)[0]
        return pos[:2], ang[2], vel[:2] # Only care about x, y plane