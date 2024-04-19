#! /usr/bin/env python3

# les lignes suivantes (jusqu'en bas) sont des commandes spécifiques à l'initialisation du robot (services de l'API kortex)
#↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
import sys, os, time, threading

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

# Angle Min et Max de chaque join selon la documentation Kinova
ACTUATOR_LIMIT = {
    0 : [-154.1, 154.1],
    1 : [-150.1, 150.1],
    2 : [-150.1, 150.1],
    3 : [-148.98, 148.98],
    4 : [-144.97, 145.0],
    5 : [-148.98, 148.98],
}

# Utilitaire pour executer chaque action (deplacement de moteur) et attendre qu'elle aie finis.
# Ex : Quand on arrive a Home, automatiquement arreter au lieu d'attendre
# un montant fixe de secondes.
def _ExecuteEtAttend(ordreAExecuter):
    def _check_for_end_or_abort(e):
        def _check(notification, e = e):
            if notification.action_event == Base_pb2.ACTION_END:
                e.set()
            elif notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
                print("EVENT : " + \
                    Base_pb2.ActionEvent.Name(notification.action_event))
                print("ERROR DETAILS : " + \
                    Base_pb2.SubErrorCodes.Name(notification.abort_details))
        return _check

    # Si on etais entrain d'executer une action, on va forcer un arret
    base.Stop()
    # Attent 0.1 sec pour etre que que on est arreter
    time.sleep(0.1)

    # Surveiller lexecution pour arreter d'attendre quand elle finis
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        _check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    ordreAExecuter()

    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)
    return finished

# Cherche une position preprogrammer a partir de son nom (ie : "Home") et y aller
def allerPositionPreprogrammee(name):
    print(f"Execution de l'action : {name}")
    # Move arm to ready position
    action_type = Base_pb2.RequestedActionType()             # créaation d'une structures de données 
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES    # affrctation du type REACH_JOINT_ANGLES (ex=7)
    listeActions = base.ReadAllActions(action_type)           # Établir une listes des action de type Joint   
    action_handle = None                                     # Création de la variable action_handle = reference
    
    # Recherche de l'action correspondante dans la liste d'action
    for action in listeActions.action_list:
        # Si le nom de l'action est le meme que le nom fournis a notre fonction
        if action.name == name:
            # On sauvegarde le handle (address interne) de l'action
            action_handle = action.handle
            # on brise la boucle
            break

    if action_handle == None:
        print(f"Action '{name}' non trouvee !")
    else:
        _ExecuteEtAttend(lambda : base.ExecuteActionFromReference(action_handle))

# Aller a une position cartesienne. C'est une fonction interne qui a besoin de x, y et z (en m) ainsi que tx, ty et tz en degree
def _allerPositionCartesien(x, y, z, tx, ty, tz):
    action = Base_pb2.Action()                      # creer une structure d'une action    
    action.reach_pose.target_pose.x = x             # (meters)
    action.reach_pose.target_pose.y = y             # (meters)
    action.reach_pose.target_pose.z = z             # (meters)
    action.reach_pose.target_pose.theta_x = tx      # (degrees)
    action.reach_pose.target_pose.theta_y = ty      # (degrees)
    action.reach_pose.target_pose.theta_z = tz      # (degrees)

    # On imprime un message d'information qui contiens les coordonnes reel vers les quel on se deplace
    print(f"    Deplacement cartesien reel vers : x = {x:.2f}m, y = {y:.2f}m, z= {z:.2f}m, θx = {tx:.2f}º, θy = {ty:.2f}º, θz = {tz:.2f}º")
    _ExecuteEtAttend(lambda : base.ExecuteAction(action))


# Aller a une position absolue sue le plan cartesien. Ne fournis que les valeur requise
# ex : si on veux deplacer a la positon y = 20cm en z = 30cm, mais pas en x ou rotation, simplement appeler
# allerPositionCartesienAbsolue(y = 0.2, z = 0.3)
def allerPositionCartesienAbsolue(x = None, y = None, z = None, tx = None, ty = None, tz = None):
    feedback = base_cyclic.RefreshFeedback()                # lire les coordonnées actuelles du robot

    messagePosition = []                                    # On prepare le message a afficher a la console
    # Si la valeur n'est pas fourni, on va reutiliser la valeur actuelle pour ne pas se deplacer
    if x == None:
        x = feedback.base.tool_pose_x
    else:
    # Parcontre, si la valeur est fourni, on va la rajouer a notre message pour que l'on affiche a la fin 
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

    # On cree un message qui detaille les parametre passee a la fonction de deplacement
    print(f"Deplacement cartesien absolue vers : {', '.join(messagePosition)}")
    _allerPositionCartesien(x, y, z, tx, ty, tz)

# Aller a une position relative sue le plan cartesien. Ne fournis que les valeur requise
# ex : si on veux monter 10cm en z, mais sans rien bouger dautre
# allerPositionCartesienRelative(z = 0.1)
def allerPositionCartesienRelative(x = 0, y = 0, z = 0, tx = 0, ty = 0, tz = 0):
    feedback = base_cyclic.RefreshFeedback()                 # lire les coordonnées actuelles du robot
    
    # On lis les valeur actuelle et on les rajoute aux valeur fournis. Si aucune valeur n'est fourni, on assume 0
    # et quand on rajoute la position actuelle a 0, elle reste la meme, effectivement ne pas bouger.
    x += feedback.base.tool_pose_x
    y += feedback.base.tool_pose_y
    z += feedback.base.tool_pose_z
    tx += feedback.base.tool_pose_theta_x
    ty += feedback.base.tool_pose_theta_y
    tz += feedback.base.tool_pose_theta_z

    # Parcontre, si la valeur est fourni, on va la rajouer a notre message pour que l'on affiche a la fin
    # On verifie si la valeur est differente de 0, sela signifie que il y a un mouvement, alors on rajoute
    # ca a notre message
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
    
    # On cree un message qui detaille les parametre passee a la fonction de deplacement
    print(f"Deplacement cartesien relatif vers : {', '.join(messagePosition)}")
    _allerPositionCartesien(x, y, z, tx, ty, tz)

# Permet de commander le gripper
# 0 signifie completement ouvert, 1 signifie completement fermer, et entre les deux signifie le % d'ouverture
def _commandGripper(pct):
    # On affiche un message pour donner la valeur d'ouverture (0% ouvert signifie fermee)
    print(f"Ouverture du gripper a {int(1-pct) * 100}%")
    gripper_command = Base_pb2.GripperCommand()
    gripper_command.mode = Base_pb2.GRIPPER_POSITION    # On se met on mode gripper
    finger = gripper_command.gripper.finger.add()       # On cree un doigt (nous avons qu'un seul gripper sur ce ropot)
    finger.finger_identifier = 1
    finger.value = max(0, min(pct, 1))                  # On fix la valeur d'ouverture entre 0 et 1
    base.SendGripperCommand(gripper_command)            # On execute la command
    timeout = time.time() + TIMEOUT_DURATION            # On calcul un temps maximal pour attendre (maintenant + 15sec)
    time.sleep(.5)                                      # On attend 0.5sec le temps que le gripper commence a bouger
    # Tant que la vitesse du griper (en absolue, vu que quand il s'ouvre il a une vitesse negative) est supperieur a 0.01
    # (quand le gripper ne peux plus bouger, sa vitesse est generalement ~0.002) on va attendre jusquau ce que le gripper
    # finissent la maneuvre ou que le tems s'est ecouler (15 sec)
    while abs(base_cyclic.RefreshFeedback().interconnect.gripper_feedback.motor[0].velocity) > 0.01:
        # Entre chaque verification, on attend 0.1sec pour ne pas surcharger le system
        time.sleep(0.1)
        # Si l'heure actuelle est plus grande que le temps maximal allouer, on estime que le gripper a quand meme fini et on sort
        if time.time() > timeout:
            break

# Fonction de deplacement angulaire interne. Prend un tableau de 6 angles (ie : [0, 0, 0, 0, 0, 0])
# et assigne les valeurs dans l'ordre a chaque moteur (actuateur)
def _allerAngulaire(tableauAngles):
    action = Base_pb2.Action()
    actuator = action.reach_joint_angles.joint_angles.joint_angles  # On cree notre liste d'actuateurs
    for joint_id in range(6):   # Sachaqnt qu'il y a 6 actuateurs
        angle = tableauAngles[joint_id]
        # Transform l'angle entre -180 et 180
        angle = (tableauAngles[joint_id] + 180.0) % 360.0 - 180.0
        # Impose les limite du join a langle
        # ACTUATOR_LIMIT contient l'infromation des limites pour chaque join.
        # ACTUATOR_LIMIT[3][0] signifie "L'angle minimum que l'actuateur #4 peut atteindre"
        # ACTUATOR_LIMIT[5][1] signifie "L'angle maximal que l'actuateur #6 peut atteindre"
        # C'est valeurs sont prise selon la notice technique.
        # On limite les actuateurs a ces plages de valeurs pour eviter les problemes
        angle = max(ACTUATOR_LIMIT[joint_id][0], min(ACTUATOR_LIMIT[joint_id][1], angle))
        
        # On met a jour l'angle dans le tableau originale, utile pour generer notre message plus tard
        tableauAngles[joint_id] = angle
        
        # On rajoute l'actuateur a la liste et on lui assigne son id et son angle
        actuator.add()                                    # creer un nouveau joint                 
        actuator[joint_id].joint_identifier = joint_id    # attribuer au joint créer le no joint_id   
        actuator[joint_id].value = angle                  # attribuer au joint créer l'angle
    # On imprime un message detaillant chaque angle
    print(f"    Deplacement angulaire reel des actuateurs : {"º, ".join([f'{v:.2f}' for v in tableauAngles])}º")
    _ExecuteEtAttend(lambda : base.ExecuteAction(action))

# Aller a une position absolue sue le plan angulaire. Ne fournis que les valeur requise
# ex : si on veux monter bouger tout les moteurs a la position verticale 0, on a :
# allerPositionCartesienRelative(t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0)
# Note : Cet angle peut etre modifier plus tard par la fonction _allerAngulaire pour le limiter a la valeur maximale du robot
def allerAngulaireAbsolue(t1 = None, t2 = None, t3 = None, t4 = None, t5 = None, t6 = None):
    feedback = base_cyclic.RefreshFeedback()                # lire les coordonnées actuelles du robot
    messagePosition = []                                    # On cree notre message
    if t1 == None:
        t1 = feedback.actuators[0].position                 # Si une valeur n'est pas assignee, on assigne la valeur angulaire du moteur en cours
    else:
        messagePosition.append(f"#1 = {t1:.2f}º")           # Sinon, on rajoute l'angle a notre message.
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
    # On imprime le message
    print(f"Deplacement angulaire absolue des actuateurs : {', '.join(messagePosition)}")
    _allerAngulaire([t1, t2, t3, t4, t5, t6])

# Aller a une position relative sue le plan angulaire. Ne fournis que les valeur requise
# ex : si on veux bouger le moteur #1 de 90 degree, et le moteur #5 de -180 degree, on fait :
# allerAngulaireRelative(t1 = 90, t5 = -180)
# Note : Cet angle peut etre modifier plus tard par la fonction _allerAngulaire pour le limiter a la valeur maximale du robot
def allerAngulaireRelative(t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0):
    feedback = base_cyclic.RefreshFeedback()                 # lire les coordonnées actuelles du robot

    
    # On lis les valeur actuelle et on les rajoute aux valeur fournis. Si aucune valeur n'est fourni, on assume 0
    # et quand on rajoute l'angle actuel a 0, il reste le meme, effectivement ne bouge pas.
    t1 += feedback.actuators[0].position
    t2 += feedback.actuators[1].position
    t3 += feedback.actuators[2].position
    t4 += feedback.actuators[3].position
    t5 += feedback.actuators[4].position
    t6 += feedback.actuators[5].position

    
    # Parcontre, si la valeur est fourni, on va la rajouer a notre message pour que l'on affiche a la fin
    # On verifie si la valeur est differente de 0, sela signifie que il y a un mouvement, alors on rajoute
    # ca a notre message
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

# Ouvre completement le gripper
def ouvrirGripper():
    _commandGripper(0)

# Ferme completement le gripper
def fermerGripper():
    _commandGripper(1)

# Va a la position "Home"
def allerHome():
    allerPositionPreprogrammee("Home")

# Etape securitaire pour aller en mode etteint. (sans toucher l'ecran)
def allerDormir():
    allerHome()
    fermerGripper()
    allerPositionPreprogrammee("PositionCoude")
    allerAngulaireRelative(t1 = -90)
    allerAngulaireAbsolue(t5 = -140)

# Etapes securitaire pour aller depuis le mode etteint vers le mode Home (sans toucher l'ecran)
def allerReveiller():
    allerAngulaireAbsolue(t5 = 0)
    allerAngulaireRelative(t1 = 90)
    allerPositionPreprogrammee("PositionCoude")
    ouvrirGripper()
    allerHome()


# Example d'utilisation : 

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
