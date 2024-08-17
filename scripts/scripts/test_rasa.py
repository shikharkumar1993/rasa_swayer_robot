#! /usr/bin/env python
"""A program to move the robot through a sequence of poses using the gripper function"""

import rospy
import time
import tf
import numpy as np
from intera_interface import Limb
from intera_motion_msgs.msg import TrajectoryOptions
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
#from shelfs_movement import qr_code_recog
from geometry_msgs.msg import PoseStamped
from gripper import gripperChange  # we have the gripperChange function implemented in the gripper module
#from shelfs_movement import
import cv2
from shelfs_movement import robot_movement
from  qr_code_reader import barcode_read_qr_code 
import requests
import json
from PIL import Image
from getkey import getkey
from PIL import ImageDraw
from PIL import ImageFont
from intera_interface import HeadDisplay
import os
def rasa_request(message,sender_id):
    API_ENDPOINT = "http://0.0.0.0:5005/webhooks/rest/webhook"
    headers = { 'Content-Type': 'application/json'}
    payload = json.dumps({"sender": sender_id, "message": message})

    try:
        r = requests.post(API_ENDPOINT, data=payload, headers=headers)
        r.raise_for_status()  # Raises HTTPError for bad responses
        response = r.json()  # Using r.json() instead of json.loads(r.content)
        
        if not response:
            # If response list is empty, handle it
            print("Warning: Received empty response from Rasa server.")
            return "No response from Rasa server."
        
        return response[0]['text']
    
    except requests.exceptions.RequestException as e:
        # Handle any requests exceptions
        print(f"Error: {e}")
        return "Request failed."
    except (json.JSONDecodeError, KeyError, IndexError) as e:
        # Handle JSON decode errors and indexing errors
        print(f"Error: {e}")
        return "Invalid response format."
def main():
    abc=rasa_request("I am E2G2","E2G2")
    abc=rasa_request("why is error","E2G2")
    print(abc)

if __name__=='__main__':
    main()