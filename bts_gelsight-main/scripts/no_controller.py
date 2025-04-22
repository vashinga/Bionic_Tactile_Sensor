#!/usr/bin/env python3
"""
*******************************************************************************
Project: bts_gelsight
File: no_controller.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: Mar 13, 2025

Description:
Python script for reading and visualizing data from a GelSight Mini sensor
using the Phidget interface. The script processes images to calculate a 
mean absolute difference (absdiff) score. A threshold-based controller then 
checks if the score exceeds a preset value for a required duration and, if so, 
activates the Dynamixel motors (e.g., to close a gripper).

License:
This script is licensed under the MIT License.
    https://opensource.org/licenses/MIT
SPDX-License-Identifier: MIT

Disclaimer:
This software is provided "as is", without warranty of any kind.
*******************************************************************************
"""

from Phidget22.Phidget import *
from Phidget22.Devices.VoltageInput import *
import time
import cv2
import yaml
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread
from queue import Queue
from scipy.signal import find_peaks
from pylsl import StreamInfo, StreamOutlet
from gelsight import gsdevice
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Load configuration
with open("../config/config.yml", 'r') as file:
    config = yaml.safe_load(file)

# Phidget channels
POT_1_CHANNEL = config["PHIDGET"]["POT_1_CHANNEL"]
POT_2_CHANNEL = config["PHIDGET"]["POT_2_CHANNEL"]
BTS_CHANNEL   = config["PHIDGET"]["BTS_CHANNEL"]

# Dynamixel control table addresses and parameters
ADDR_TORQUE_ENABLE    = config["DXL"]["ADDR_TORQUE_ENABLE"]
ADDR_GOAL_POSITION    = config["DXL"]["ADDR_GOAL_POSITION"]
ADDR_MOVING_SPEED     = config["DXL"]["ADDR_MOVING_SPEED"]
ADDR_TORQUE_LIMIT     = config["DXL"]["ADDR_TORQUE_LIMIT"]
ADDR_PRESENT_POSITION = config["DXL"]["ADDR_PRESENT_POSITION"]

PROTOCOL_VERSION = config["DXL"]["PROTOCOL_VERSION"]
BAUDRATE         = config["DXL"]["BAUDRATE"]
DEVICENAME       = config["DXL"]["DEVICENAME"]

TORQUE_ENABLE    = config["DXL"]["TORQUE_ENABLE"]
TORQUE_DISABLE   = config["DXL"]["TORQUE_DISABLE"]
DXL_MOVING_STATUS_THRESHOLD = config["DXL"]["DXL_MOVING_STATUS_THRESHOLD"]

DXL1_ID = config["DXL"]["DXL1_ID"]
DXL2_ID = config["DXL"]["DXL2_ID"]

DXL1_MIN_VALUE = config["DXL"]["DXL1_MIN_VALUE"]
DXL1_MAX_VALUE = config["DXL"]["DXL1_MAX_VALUE"]
DXL2_MIN_VALUE = config["DXL"]["DXL2_MIN_VALUE"]
DXL2_MAX_VALUE = config["DXL"]["DXL2_MAX_VALUE"]

# Additional Dynamixel addresses for extra readings (if needed)
# ...

# Set up LSL outlet for GelSight score streaming
stream_name = "GELSIGHT_SCORE"
stream_type = "Data"
num_channels = 1
gelsight_sampling_rate = 1000  # Hz
data_type = "float32"
source_id = "gelsight_stream_id"
gelsight_info = StreamInfo(stream_name, stream_type, num_channels, gelsight_sampling_rate, data_type, source_id)
gelsight_outlet = StreamOutlet(gelsight_info)

# Set up LSL outlet for Dynamixel data (if needed)
stream_name = "DYNAMIXELs"
stream_type = "Data"
num_channels = 1
dxl_info = StreamInfo(stream_name, stream_type, num_channels, 1000, data_type, "dxl_stream_id")
dxl_outlet = StreamOutlet(dxl_info)

# Global variables for control and processing
stop_flag = False
voltage_buffer = []  # (Not used in this version, but available for additional processing)
BUFFER_SIZE = 50
MA_WINDOW = 5
PEAK_THRESHOLD = 2.79
PEAK_DISTANCE = 5
absdiff_threshold_v = 1.5  # TODO
gelsight_absdiff_score_g = 0

# Controller parameters for GelSight score
controller_bounding_threshold = 0.0045
controller_MA_window_size = 10
controller_MA_window_size_2 = 1  # Number of samples for recent average
vsa_increase_step = 10
required_slip_duration = 0.1  # in seconds
vsa_holding_empty_cup_min_contact_pos = 120

# For threshold control timing
slip_start_time = None

start_time = time.time()

# Initialize Dynamixel PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# ----------------- Dynamixel Control Functions ----------------- #

def control_dxl_RX_24F(dxl_id, pos, vel, torq):
    """
    Command a Dynamixel motor to move to a specified position.
    """
    if dxl_id == DXL1_ID:
        if not (DXL1_MIN_VALUE <= pos <= DXL1_MAX_VALUE):
            print("Not a valid value for motor 1 position!")
            return
    elif dxl_id == DXL2_ID:
        if not (DXL2_MIN_VALUE <= pos <= DXL2_MAX_VALUE):
            print("Not a valid value for motor 2 position!")
            return
    else:
        print("Not a valid motor ID!")
        return

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, pos)
    if dxl_comm_result != COMM_SUCCESS:
        print("Communication error:", packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("Dynamixel error:", packetHandler.getRxPacketError(dxl_error))
    else:
        print(f"Goal position set to: {pos}")

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MOVING_SPEED, vel)
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))
    else:
        print(f"Speed set to {vel}.")

def close_vsa_gripper(pos, vel):
    if 1 < pos <= 600:
        control_dxl_RX_24F(DXL1_ID, 1023, 1023, 1023)  # Homing for motor 1
        control_dxl_RX_24F(DXL2_ID, pos, vel, 1023)
    else:
        print("Invalid input pos for dynamixels!")

def open_vsa_gripper():
    control_dxl_RX_24F(DXL2_ID, 121, 1023, 1023)  # Homing for motor 2
    control_dxl_RX_24F(DXL1_ID, 1, 1023, 1023)

# ----------------- GelSight Processing Functions ----------------- #

def img_processing(img):
    """Convert to grayscale and apply thresholding/filters."""
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    thresh = cv2.adaptiveThreshold(gray, 255,
                                   cv2.ADAPTIVE_THRESH_MEAN_C,
                                   cv2.THRESH_BINARY_INV, 21, 4)
    thresh = cv2.medianBlur(thresh, 9)
    thresh = cv2.dilate(thresh, (3, 3), iterations=2)
    thresh = cv2.medianBlur(thresh, 3)
    thresh = cv2.dilate(thresh, (3, 3), iterations=2)
    return thresh

def get_gelsight_score():
    """
    Continuously reads images from the GelSight Mini sensor,
    processes them, computes the mean absolute difference (absdiff) score 
    compared to the previous frame, and pushes (timestamp, score) into LSL.
    """
    global gelsight_absdiff_score_g
    sensor = gsdevice.Camera("GelSight Mini")
    sensor.connect()

    init_img = sensor.get_image()
    init_img = img_processing(init_img)

    while not stop_flag:
        raw_img = sensor.get_image()
        current_img = img_processing(raw_img)
        diff = cv2.absdiff(init_img, current_img)
        gelsight_absdiff_score_g = np.mean(diff)
        init_img = current_img  # Update the reference
        
        gelsight_outlet.push_sample([gelsight_absdiff_score_g])
        dxl_outlet.push_sample([vsa_holding_empty_cup_min_contact_pos])
        # time.sleep(0.05)  # Adjust as needed for processing speed

# ----------------- Visualization and Control ----------------- #

def phidget_online_visualization():
    plt.ion()
    fig, ax = plt.subplots()
    
    # (Optional plotting of voltage if desired; here we focus on the GelSight score.)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 5)  # Adjust based on expected score range
    plt.xlabel("Time (s)")
    plt.ylabel("GelSight Score")
    plt.title("Real-Time GelSight Score & Slip Detection")

    vis_start_time = time.time()

    global slip_start_time, vsa_holding_empty_cup_min_contact_pos, gelsight_absdiff_score_g

    while True:
        # Here, we simply check the global gelsight score
        time_1 = time.time()
        current_time = time.time() - vis_start_time

        # Display the current score (you can also add plotting of the score)
        print(f"Time: {current_time:.2f} s, GelSight Score: {gelsight_absdiff_score_g:.3f}", end="\r")
        
        # --- Controller: Thresholding Control Algorithm ---
        # If the current score exceeds the absolute difference threshold, start timing
        # if gelsight_absdiff_score_g >= absdiff_threshold_v:
        #     if slip_start_time is None:
        #         slip_start_time = time.time()
        #     elif (time.time() - slip_start_time) >= required_slip_duration:
        #         print("\nBTS, slip detected!")
        #         vsa_holding_empty_cup_min_contact_pos += vsa_increase_step
        #         close_vsa_gripper(vsa_holding_empty_cup_min_contact_pos, 1023)
        #         slip_start_time = None  # Reset after action
        # else:
        #     slip_start_time = None

        print("controller delay: ", time.time() - time_1)
        fig.canvas.draw()
        fig.canvas.flush_events()
        # time.sleep(0.1)

# ----------------- Main Execution ----------------- #

if not portHandler.openPort():
    print("Failed to open Dynamixel port!")
    quit()
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set baudrate!")
    quit()

# Create and start the GelSight score acquisition thread
thread1 = Thread(target=get_gelsight_score, daemon=True)
thread1.start()

# Optionally, delay to let some score data accumulate
time.sleep(3)

# (Optional: you may test a direct command)
close_vsa_gripper(vsa_holding_empty_cup_min_contact_pos, 1023)

# Run the visualization/control in the main thread
try:
    phidget_online_visualization()

except KeyboardInterrupt:
    # Clean up resources if needed
    VoltageInput().close()
    print("Program stopped.")
