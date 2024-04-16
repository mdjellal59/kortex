#! /usr/bin/env python3

# les lignes suivantes (jusqu'en bas) sont des commandes spécifiques à l'initialisation du robot (services de l'API kortex)
#↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
import sys, os, time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2


# Import the utilities helper module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))             # a supprimer si le fochier utilities.py dans le meme dossier 102
import utilities # copy et coller le contenue du fichier utilitiers.py 

# Create connection to the device and get the router
router = utilities.DeviceConnection.createTcpConnection(utilities.parseConnectionArguments()).__enter__()     # creer un routeur pour communiquer avec le robot     

# Create required services
base = BaseClient(router)
base_cyclic = BaseCyclicClient(router)
 
# Make sure the arm is in Single Level Servoing mode
base_servo_mode = Base_pb2.ServoingModeInformation()
base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
base.SetServoingMode(base_servo_mode)
#↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 10         # durée pemettant au rebot de terminer l'action demandée 


# Move arm to ready position
print("Moving the arm to a safe position")
action_type = Base_pb2.RequestedActionType()             # créaation d'une structures de données 
action_type.action_type = Base_pb2.REACH_JOINT_ANGLES    # affrctation du type REACH_JOINT_ANGLES (ex=7)
action_list = base.ReadAllActions(action_type)           # Établir une listes des action de type Joint   
vec_action = action_list.action_list
action_handle = None                                     # Création de la variable action_handle = reference
i = 0
while i < len(vec_action) and action_handle == None:
    if vec_action[i].name == "Home":
        action_handle = vec_action[i].handle
    i += 1

base.ExecuteActionFromReference(action_handle)           # On demande a API d'exécuter l'action          
time.sleep(TIMEOUT_DURATION)


print("Starting Cartesian action movement ...")

#action.name = "Example Cartesian action movement"
#action.application_data = ""

feedback = base_cyclic.RefreshFeedback()                 # lire les coordonnées actuelles du robot

action = Base_pb2.Action()                               # creer une structure d'une action    
action.reach_pose.target_pose.x = feedback.base.tool_pose_x          # (meters)
action.reach_pose.target_pose.y = feedback.base.tool_pose_y - 0.1    # (meters)
action.reach_pose.target_pose.z = feedback.base.tool_pose_z - 0.2    # (meters)
action.reach_pose.target_pose.theta_x = feedback.base.tool_pose_theta_x # (degrees)
action.reach_pose.target_pose.theta_y = feedback.base.tool_pose_theta_y # (degrees)
action.reach_pose.target_pose.theta_z = feedback.base.tool_pose_theta_z # (degrees)

print("Executing action")
base.ExecuteAction(action)                            # execution de l'action 
time.sleep(TIMEOUT_DURATION)

print("Starting angular action movement ...")
action = Base_pb2.Action()
action.name = "Example angular action movement"
action.application_data = ""

actuator_count = base.GetActuatorCount()               # chercher le nombre d'actionneurs             

# Place arm straight up
#for joint_id in range(actuator_count.count):
actuator = action.reach_joint_angles.joint_angles.joint_angles
for joint_id in range(6):
    actuator.add()                                    # creer un nouveau joint                 
    actuator[joint_id].joint_identifier = joint_id    # attribuer au joint créer le no joint_id   
    actuator[joint_id].value = joint_id*25            # attribuer au joint créer l'angle 0 

print("Executing action")
base.ExecuteAction(action)                            # execution de l'action   
time.sleep(TIMEOUT_DURATION)