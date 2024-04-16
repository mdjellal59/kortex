#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed under the
# terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2

class GripperCommandExample:                                                       # créer la classe GripperCommandExample
    def __init__(self, router, proportional_gain = 2.0):                           # créer le constructeur de la classe GripperCommandExample  

        self.proportional_gain = proportional_gain
        self.router = router

        # Create base client using TCP router
        self.base = BaseClient(self.router)

    def ExampleSendGripperCommands(self):                                         # créer la fonction ExampleSendGripperCommands

        # Create the GripperCommand we will send                                   # créer la commande à envoyer 
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        print("Performing gripper test in position...")
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        position = 0.00
        finger.finger_identifier = 1
        while position < 1.0:                                                     # tant que la position inferieure à 1 (100% = pince completement fermée)  
            finger.value = position                                               # affecter la valeur de la position demandée au doigt   
            print("Going to position {:0.2f}...".format(finger.value))            # afficher la position demandée (avec 2 chiffres apres la virgule)  
            self.base.SendGripperCommand(gripper_command)                         # transmettre et exécuyter la position demandée  
            position += 0.1                                                       # augmenter la position demandée de 0.1 (10%)   
            time.sleep(1)

        # Set speed to open gripper
        print ("Opening gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = 0.1
        self.base.SendGripperCommand(gripper_command)
        gripper_request = Base_pb2.GripperRequest()

        # Wait for reported position to be opened
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        # exit(0)
        while True:
            time.sleep(0.1)
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len (gripper_measure.finger):
                print("Current position is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value < 0.01:
                    break
            else: # Else, no finger present in answer, end loop
                break

        # Set speed to close gripper
        print ("Closing gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = -0.1
        self.base.SendGripperCommand(gripper_command)

        # Wait for reported speed to be 0
        gripper_request.mode = Base_pb2.GRIPPER_SPEED
        while True:
            time.sleep(0.1)
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len (gripper_measure.finger):
                print("Current speed is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value == 0.0:
                    break
            else: # Else, no finger present in answer, end loop
                break

    def ExampleSendGripperCommands(self, position):            # ajouter avec Aymen pour imposer position = 0.50 (50%) de n'importe quelle position initiale
            # Create the GripperCommand we will send
            gripper_command = Base_pb2.GripperCommand()
            finger = gripper_command.gripper.finger.add()

            # Close the gripper with position increments
            print("Performing gripper test in position...")
            gripper_command.mode = Base_pb2.GRIPPER_POSITION             # mode de commande PPOSITION
            # position = 0.00
            finger.finger_identifier = 1                                 # choisir le doigt no 1
            finger.value = position                                      # donner la valeur de la position (recu du main ligne 116)
            self.base.SendGripperCommand(gripper_command)                # transmettre et exécuyter la position demandée   
            time.sleep(1)


def main():
    # Import the utilities helper module
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        example = GripperCommandExample(router)           # creer l'ogjet
        example.ExampleSendGripperCommands(0.50)              # executer la commande 

if __name__ == "__main__":
    main()