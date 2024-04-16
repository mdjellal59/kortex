#! /usr/bin/env python3

import sys, os, time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 10

# Import the utilities helper module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import utilities

# Create connection to the device and get the router
router = utilities.DeviceConnection.createTcpConnection(utilities.parseConnectionArguments()).__enter__()

# Create required services
base = BaseClient(router)
base_cyclic = BaseCyclicClient(router)
 
# Make sure the arm is in Single Level Servoing mode
base_servo_mode = Base_pb2.ServoingModeInformation()
base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
base.SetServoingMode(base_servo_mode)

# Move arm to ready position
print("Moving the arm to a safe position")
action_type = Base_pb2.RequestedActionType()
action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
action_list = base.ReadAllActions(action_type)
action_handle = None
for action in action_list.action_list:
    if action.name == "Home":
        action_handle = action.handle

base.ExecuteActionFromReference(action_handle)
time.sleep(TIMEOUT_DURATION)


print("Starting Cartesian action movement ...")
action = Base_pb2.Action()
action.name = "Example Cartesian action movement"
action.application_data = ""

feedback = base_cyclic.RefreshFeedback()

cartesian_pose = action.reach_pose.target_pose
cartesian_pose.x = feedback.base.tool_pose_x          # (meters)
cartesian_pose.y = feedback.base.tool_pose_y - 0.1    # (meters)
cartesian_pose.z = feedback.base.tool_pose_z - 0.2    # (meters)
cartesian_pose.theta_x = feedback.base.tool_pose_theta_x # (degrees)
cartesian_pose.theta_y = feedback.base.tool_pose_theta_y # (degrees)
cartesian_pose.theta_z = feedback.base.tool_pose_theta_z # (degrees)

print("Executing action")
base.ExecuteAction(action)
time.sleep(TIMEOUT_DURATION)

print("Starting angular action movement ...")
action = Base_pb2.Action()
action.name = "Example angular action movement"
action.application_data = ""

actuator_count = base.GetActuatorCount()

# Place arm straight up
for joint_id in range(actuator_count.count):
    joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
    joint_angle.joint_identifier = joint_id
    joint_angle.value = 0

print("Executing action")
base.ExecuteAction(action)
time.sleep(TIMEOUT_DURATION)