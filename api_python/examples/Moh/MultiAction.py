#! /usr/bin/env python3

# les lignes suivantes (jusqu'en bas) sont des commandes spécifiques à l'initialisation du robot (services de l'API kortex)
#↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
import sys, os, time, threading, atexit, signal

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
state = base_cyclic.RefreshFeedback()
if state.base.active_state == Base_pb2.ARMSTATE_IN_FAULT:
    print("Detecter etat de faute. Reset...")
    base.ClearFaults() # Enlever letats de fault si actif
    time.sleep(1)
#↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 15         # durée pemettant au rebot de terminer l'action demandée 
ACTUATOR_LIMIT = {
    0 : [-154.1, 154.1],
    1 : [-150.1, 150.1],
    2 : [-150.1, 150.1],
    3 : [-148.98, 148.98],
    4 : [-144.97, 145.0],
    5 : [-148.98, 148.98],
}
def _check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        if notification.action_event == Base_pb2.ACTION_END:
            e.set()
        elif notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
            print("EVENT : " + \
                Base_pb2.ActionEvent.Name(notification.action_event))
            print("ERROR DETAILS : " + \
                Base_pb2.SubErrorCodes.Name(notification.abort_details))
    return check

def _handleNotification(f):
    # Si on bouge, on va forcer un arret
    base.Stop()
    # Attent 0.1 sec pour etre que que on est arreter
    time.sleep(0.1)

    # Surveiller lexecution pour arreter d'attendre quand elle finis
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        _check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    f()
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)
    return finished

# Cherche une position preprogrammer a partir de son nom (ie : "Home")
def allerPositionPreprogrammee(name):
    print(f"Execution de l'action : {name}")
    # Move arm to ready position
    action_type = Base_pb2.RequestedActionType()             # créaation d'une structures de données 
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES    # affrctation du type REACH_JOINT_ANGLES (ex=7)
    action_list = base.ReadAllActions(action_type)           # Établir une listes des action de type Joint   
    vec_action = action_list.action_list
    action_handle = None                                     # Création de la variable action_handle = reference
    i = 0
    while i < len(vec_action) and action_handle == None:
        if vec_action[i].name == name:
            action_handle = vec_action[i].handle
            break
        i += 1
    if action_handle == None:
        print(f"Action '{name}' non trouvee !")
    else:
        _handleNotification(lambda : base.ExecuteActionFromReference(action_handle))

# coord en metres
def _allerPositionCartesien(x, y, z, tx, ty, tz):
    action = Base_pb2.Action()                               # creer une structure d'une action    
    action.reach_pose.target_pose.x = x          # (meters)
    action.reach_pose.target_pose.y = y    # (meters)
    action.reach_pose.target_pose.z = z    # (meters)
    action.reach_pose.target_pose.theta_x = tx # (degrees)
    action.reach_pose.target_pose.theta_y = ty # (degrees)
    action.reach_pose.target_pose.theta_z = tz # (degrees)

    print(f"    Deplacement cartesien reel vers : x = {x:.2f}m, y = {y:.2f}m, z= {z:.2f}m, θx = {tx:.2f}º, θy = {ty:.2f}º, θz = {tz:.2f}º")
    _handleNotification(lambda : base.ExecuteAction(action))


# coord en metres
def allerPositionCartesienAbsolue(x = None, y = None, z = None, tx = None, ty = None, tz = None):
    feedback = base_cyclic.RefreshFeedback()                 # lire les coordonnées actuelles du robot

    messagePosition = []
    if x == None:
        x = feedback.base.tool_pose_x
    else:
        messagePosition.append(f"x = {x:.2f}m")
    if y == None:
        y = feedback.base.tool_pose_y
    else:
        messagePosition.append(f"x = {x:.2f}m")
    if z == None:
        z = feedback.base.tool_pose_z
    else:
        messagePosition.append(f"x = {x:.2f}m")
    if tx == None:
        tx = feedback.base.tool_pose_theta_x
    else:
        messagePosition.append(f"θx = {tx:.2f}º")
    if ty == None:
        ty = feedback.base.tool_pose_theta_y
    else:
        messagePosition.append(f"θy = {ty:.2f}º")
    if tz == None:
        tz = feedback.base.tool_pose_theta_z
    else:
        messagePosition.append(f"θz = {tz:.2f}º")
        
    print(f"Deplacement cartesien absolue vers : {', '.join(messagePosition)}")
    _allerPositionCartesien(x, y, z, tx, ty, tz)

# coord en metres
def allerPositionCartesienRelative(x = 0, y = 0, z = 0, tx = 0, ty = 0, tz = 0):
    feedback = base_cyclic.RefreshFeedback()                 # lire les coordonnées actuelles du robot
    x += feedback.base.tool_pose_x
    y += feedback.base.tool_pose_y
    z += feedback.base.tool_pose_z
    tx += feedback.base.tool_pose_theta_x
    ty += feedback.base.tool_pose_theta_y
    tz += feedback.base.tool_pose_theta_z

    messagePosition = []
    if x != 0:
        messagePosition.append(f"x = {x:.2f}m")
    if y != 0:
        messagePosition.append(f"x = {x:.2f}m")
    if z != 0:
        messagePosition.append(f"x = {x:.2f}m")
    if tx != 0:
        messagePosition.append(f"θx = {tx:.2f}º")
    if ty != 0:
        messagePosition.append(f"θy = {ty:.2f}º")
    if tz != 0:
        messagePosition.append(f"θz = {tz:.2f}º")
    print(f"Deplacement cartesien relatif vers : {', '.join(messagePosition)}")
    _allerPositionCartesien(x, y, z, tx, ty, tz)

#pct est entre 0 et 1
def _commandGripper(pct):
    print(f"Ouverture du gripper a {int(1-pct) * 100}%")
    gripper_command = Base_pb2.GripperCommand()
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    finger = gripper_command.gripper.finger.add()
    finger.finger_identifier = 1
    finger.value = max(0, min(pct, 1))
    base.SendGripperCommand(gripper_command)
    timeout = time.time() + TIMEOUT_DURATION
    time.sleep(.5)
    while abs(base_cyclic.RefreshFeedback().interconnect.gripper_feedback.motor[0].velocity) > 0.01:
        time.sleep(0.1)
        if time.time() > timeout:
            break

def _allerAngulaire(tableauAngles):
    action = Base_pb2.Action()
    actuator = action.reach_joint_angles.joint_angles.joint_angles
    for joint_id in range(6):
        # Transform langle entre -180 et 180
        angle = (tableauAngles[joint_id] + 180.0) % 360.0 - 180.0
        # Impose les limite du join a langle
        angle = max(ACTUATOR_LIMIT[joint_id][0], min(ACTUATOR_LIMIT[joint_id][1], angle))
        tableauAngles[joint_id] = angle
        actuator.add()                                    # creer un nouveau joint                 
        actuator[joint_id].joint_identifier = joint_id    # attribuer au joint créer le no joint_id   
        actuator[joint_id].value = angle                  # attribuer au joint créer l'angle 0 
    print(f"    Deplacement angulaire reel des actuateurs : {"º, ".join([f'{v:.2f}' for v in tableauAngles])}º")
    _handleNotification(lambda : base.ExecuteAction(action))

# angle en degree
def allerAngulaireAbsolue(t1 = None, t2 = None, t3 = None, t4 = None, t5 = None, t6 = None):
    feedback = base_cyclic.RefreshFeedback()                 # lire les coordonnées actuelles du robot
    messagePosition = []
    if t1 == None:
        t1 = feedback.actuators[0].position
    else:
        messagePosition.append(f"#1 = {t1:.2f}º")
    if t2 == None:
        t2 = feedback.actuators[1].position
    else:
        messagePosition.append(f"#2 = {t2:.2f}º")
    if t3 == None:
        t3 = feedback.actuators[2].position
    else:
        messagePosition.append(f"#3 = {t3:.2f}º")
    if t4 == None:
        t4 = feedback.actuators[3].position
    else:
        messagePosition.append(f"#4 = {t4:.2f}º")
    if t5 == None:
        t5 = feedback.actuators[4].position
    else:
        messagePosition.append(f"#5 = {t5:.2f}º")
    if t6 == None:
        t6 = feedback.actuators[5].position
    else:
        messagePosition.append(f"#6 = {t5:.2f}º")
    print(f"Deplacement angulaire absolue des actuateurs : {', '.join(messagePosition)}")
    _allerAngulaire([t1, t2, t3, t4, t5, t6])

def allerAngulaireRelative(t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0):
    feedback = base_cyclic.RefreshFeedback()                 # lire les coordonnées actuelles du robot
    t1 += feedback.actuators[0].position
    t2 += feedback.actuators[1].position
    t3 += feedback.actuators[2].position
    t4 += feedback.actuators[3].position
    t5 += feedback.actuators[4].position
    t6 += feedback.actuators[5].position
    messagePosition = []
    if t1 != 0:
        messagePosition.append(f"#1 = {t1:.2f}º")
    if t2 != 0:
        messagePosition.append(f"#2 = {t2:.2f}º")
    if t3 != 0:
        messagePosition.append(f"#3 = {t3:.2f}º")
    if t4 != 0:
        messagePosition.append(f"#4 = {t4:.2f}º")
    if t5 != 0:
        messagePosition.append(f"#5 = {t5:.2f}º")
    if t6 != 0:
        messagePosition.append(f"#6 = {t6:.2f}º")
    print(f"Deplacement angulaire relatif des actuateurs : {', '.join(messagePosition)}")
    _allerAngulaire([t1, t2, t3, t4, t5, t6])

def ouvrirGripper():
    _commandGripper(0)

def fermerGripper():
    _commandGripper(1)

def allerHome():
    allerPositionPreprogrammee("Home")

# Etape securitaire pour aller en mode etteint.
def allerDormir():
    allerHome()
    fermerGripper()
    allerPositionPreprogrammee("PositionCoude")
    allerAngulaireRelative(t1 = -90)
    allerAngulaireAbsolue(t5 = -140)

# Etapes securitaire pour aller depuis le mode etteint vers le mode Home
def allerReveiller():
    allerAngulaireAbsolue(t5 = 0)
    allerAngulaireRelative(t1 = 90)
    allerPositionPreprogrammee("PositionCoude")
    ouvrirGripper()
    allerHome()


allerReveiller()

fermerGripper()
ouvrirGripper()
allerPositionPreprogrammee("Retract")
allerHome()
allerPositionCartesienAbsolue(x=0.3, y = 0.3, z = 0.3)
allerPositionCartesienRelative(x=0.2)
fermerGripper()
allerPositionCartesienRelative(z=0.2)
allerAngulaireAbsolue(0, 0, 0, 0, 0, 0)
allerHome()
ouvrirGripper()

allerDormir()
