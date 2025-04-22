"""
*******************************************************************************

Project: graspio
File: controller_bts.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: Jan 16, 2025

Description:
Python script for reading and visualizing data from BTS, using Phidget interface.

License:
This script is licensed under the MIT License.
You may obtain a copy of the License at
    https://opensource.org/licenses/MIT

SPDX-License-Identifier: MIT

Disclaimer:
This software is provided "as is", without warranty of any kind, express or
implied, including but not limited to the warranties of merchantability,
fitness for a particular purpose, and noninfringement. In no event shall the
authors be liable for any claim, damages, or other liability, whether in an
action of contract, tort, or otherwise, arising from, out of, or in connection
with the software or the use or other dealings in the software.

*******************************************************************************
"""
#!/usr/bin/env python3

from Phidget22.Phidget import *
from Phidget22.Devices.VoltageInput import *
import time
import yaml
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageInput import *
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread
from queue import Queue
from scipy.signal import find_peaks
from pylsl import StreamInfo, StreamOutlet
from dynamixel_sdk import *  # Uses Dynamixel SDK library


with open("../config/config.yml", 'r') as file:
    config = yaml.safe_load(file)

POT_1_CHANNEL = config["PHIDGET"]["POT_1_CHANNEL"]  # Address for torque enable
POT_2_CHANNEL = config["PHIDGET"]["POT_2_CHANNEL"]  # Address for torque enable
BTS_CHANNEL = config["PHIDGET"]["BTS_CHANNEL"]  # Address for torque enable

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

POT_1_CHANNEL = config["PHIDGET"]["POT_1_CHANNEL"]
POT_2_CHANNEL = config["PHIDGET"]["POT_2_CHANNEL"]

# Additional Dynamixel addresses for extra readings
DXL_ADDR_PRESENT_POSITION   = config["DXL"]["ADDR_PRESENT_POSITION"]
DXL_ADDR_PRESENT_SPEED      = config["DXL"]["ADDR_PRESENT_SPEED"]
DXL_ADDR_PRESENT_LOAD       = config["DXL"]["ADDR_PRESENT_LOAD"]
DXL_ADDR_PRESENT_INPUT_VOLTAGE = config["DXL"]["ADDR_PRESENT_INPUT_VOLTAGE"]
DXL_ADDR_PRESENT_TEMP       = config["DXL"]["ADDR_PRESENT_TEMP"]

# Queue for thread-safe communication
data_queue = Queue()

stream_name = "PHIDGET_BTS"
stream_type = "Data"
num_channels = 1  # TODO
phidget_sampling_rate = 1000  # Hz (Target Rate)
data_type = "float32"
source_id = "phidget_BTS_stream_id"
phidget_bts_info = StreamInfo(stream_name, stream_type, num_channels, phidget_sampling_rate, data_type, source_id)
phidget_bts_outlet = StreamOutlet(phidget_bts_info)


stream_name = "DYNAMIXELs"
stream_type = "Data"
num_channels = 1  # TODO
phidget_sampling_rate = 1000  # Hz (Target Rate)
data_type = "float32"
source_id = "dxl_stream_id"
dxl_info = StreamInfo(stream_name, stream_type, num_channels, phidget_sampling_rate, data_type, source_id)
dxl_outlet = StreamOutlet(dxl_info)


stop_flag = False
voltage_buffer = []
BUFFER_SIZE = 50       # Number of recent samples to keep
MA_WINDOW = 5          # Window size for moving average filter
PEAK_THRESHOLD = 2.79   # Minimum height for a peak (adjust as needed)
PEAK_DISTANCE = 5      # Minimum samples between peaks
slip_start_time = time.time()

controller_bounding_threshold = 0.0115  # 0.0115
controller_MA_window_size = 10
controller_MA_window_size_2 = 1
vsa_increase_step = 10  # 150
required_slip_duration = 0.1
vsa_holding_empty_cup_min_contact_pos = 120  # 150

start_time = time.time()

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


def control_dxl_RX_24F(dxl_id, pos, vel, torq):
    """
    Command a Dynamixel motor to move to a specified position with a given speed and torque limit.
    Assumes that the Dynamixel port has already been opened and that a global lock (dxl_lock)
    is available for serializing access.
    """
    # Validate motor position ranges.
    if dxl_id == DXL1_ID:
        if not (DXL1_MIN_VALUE <= pos <= DXL1_MAX_VALUE):
            print("\n\nNot a valid value for motor 1 position!\n\n")
            return
    elif dxl_id == DXL2_ID:
        if not (DXL2_MIN_VALUE <= pos <= DXL2_MAX_VALUE):
            print("\n\nNot a valid value for motor 2 position!\n\n")
            return
    else:
        print("\n\nNot a valid motor ID!\n\n")
        return

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, pos)


def close_vsa_gripper(pos, vel):
    if 1 < pos <= 600:
        control_dxl_RX_24F(DXL1_ID, 1023, 300, 300)  # Homing for motor 1
        control_dxl_RX_24F(DXL2_ID, pos, vel, 1023)
    else:
        print("Invalid input pos for dynamixels!")

def open_vsa_gripper():
    control_dxl_RX_24F(DXL2_ID, 121, 500, 300)  # Homing for motor 2
    control_dxl_RX_24F(DXL1_ID, 1, 500, 1023)


# Callback function to handle voltage change
def onVoltageChange(self, voltage):
    global voltage_buffer

    data_queue.put((time.time(), voltage))  # Push timestamp and voltage into the queue
    print(f"Voltage: {voltage:.3f}", end=" \r")
    
    phidget_bts_outlet.push_sample([voltage])
    dxl_outlet.push_sample([vsa_holding_empty_cup_min_contact_pos])


    # Append the new voltage sample to the buffer
    voltage_buffer.append(voltage)
    # Keep only the most recent BUFFER_SIZE samples
    if len(voltage_buffer) > BUFFER_SIZE:
        voltage_buffer.pop(0)
    
    # Optional: Immediately try to detect peaks when the buffer is full
    # if len(voltage_buffer) == BUFFER_SIZE:
    #     # Apply a simple moving average to reduce noise
    #     ma_voltage = np.convolve(voltage_buffer, np.ones(MA_WINDOW)/MA_WINDOW, mode='valid')
    #     # Use SciPy's find_peaks to detect peaks in the smoothed signal
    #     peaks, properties = find_peaks(ma_voltage, height=PEAK_THRESHOLD, distance=PEAK_DISTANCE)
    #     if peaks.size > 0:
    #         pass
    #         print("Peaks detected at buffer indices:", peaks)
            # For control, you might trigger some action here when a peak is detected


# Target function for the thread to fetch voltage data
def get_voltage_data():
    voltageInput0 = VoltageInput()
    voltageInput0.setChannel(BTS_CHANNEL)  # TODO
    voltageInput0.setOnVoltageChangeHandler(onVoltageChange)
    voltageInput0.openWaitForAttachment(5000)
    voltageInput0.setDataInterval(10)  # Adjust data collection interval in milliseconds
    
    while True:
        time.sleep(0.0001)  # Ensure the thread runs continuously
        

# Function for real-time visualization with Moving Average
def phidget_online_visualization():
    plt.ion()  # Enable interactive mode
    fig, ax = plt.subplots()
    
    # Create two plot lines: one for the raw signal and one for the moving average.
    line_raw, = ax.plot([], [], lw=1, label="Raw Voltage", color='blue')
    line_ma, = ax.plot([], [], lw=2, label="Moving Average", color='red')
    
    # Set up initial plot limits and labels.
    ax.set_xlim(0, 10)  # 10-second window initially
    ax.set_ylim(-1.5, 1.5)
    plt.xlabel("Time (s)")
    plt.ylabel("Voltage (V)")
    plt.title("Real-Time Voltage with Moving Average")
    plt.legend()

    vis_start_time = time.time()
    x_data, y_data = [], []
    
    global slip_start_time, vsa_holding_empty_cup_min_contact_pos
    # Ensure slip_start_time is defined globally (initially None)
    # vsa_holding_empty_cup_min_contact_pos should also be defined globally.

    window_size = 10  # Window for computing moving average

    while True:
        # Collect all available data from the queue.
        while not data_queue.empty():
            current_time, voltage = data_queue.get()
            x_data.append(current_time - vis_start_time)
            y_data.append(voltage)

        if x_data:
            # Compute moving average if enough samples are available.
            if len(y_data) >= window_size:
                y_ma = np.convolve(y_data, np.ones(window_size) / window_size, mode='valid')
                # Align x-axis with the computed moving average.
                x_ma = x_data[len(x_data) - len(y_ma):]
            else:
                y_ma = y_data[:]  # Not enough data: use raw data.
                x_ma = x_data[:]
            
            # Update plot data.
            # line_raw.set_data(x_data, y_data)  # raw BTS signal
            line_ma.set_data(x_ma, y_ma)
            
            # Dynamically adjust the plot's x and y limits.
            ax.set_xlim(max(0, x_data[-1] - 10), x_data[-1] + 1)
            ax.set_ylim(min(y_data) - 0.1, max(y_data) + 0.1)
            
            # --- Controller: Thresholding Control Algorithm ---
            # Use the last controller_MA_window_size samples to compute a baseline mean.
            time_1 = time.time()
            if len(y_ma) >= controller_MA_window_size:
                baseline_mean = np.mean(y_ma[-controller_MA_window_size:])
                upper_bound = baseline_mean + controller_bounding_threshold
                lower_bound = baseline_mean - controller_bounding_threshold

                # Use a short window (or the last sample) for comparison.
                if len(y_ma) >= controller_MA_window_size_2:
                    recent_avg = np.mean(y_ma[-controller_MA_window_size_2:])
                else:
                    recent_avg = y_ma[-1]

                # If the recent average is outside the acceptable bounds, start timing.
                if recent_avg > upper_bound or recent_avg < lower_bound:
                    if slip_start_time is None:
                        slip_start_time = time.time()
                    elif (time.time() - slip_start_time) >= required_slip_duration:
                        print("\nBTS, slip detected!")
                        vsa_holding_empty_cup_min_contact_pos += vsa_increase_step
                        close_vsa_gripper(vsa_holding_empty_cup_min_contact_pos, 50)
                        slip_start_time = None  # Reset after action
                else:
                    slip_start_time = None  # Reset if condition not met
            # print("controller delay: ", time.time() - time_1)
        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.001)  # Adjust refresh rate as needed


if not portHandler.openPort():
    print("Failed to open Dynamixel port!")
    quit()
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set baudrate!")
    quit()

# Create and start the data acquisition thread
thread1 = Thread(target=get_voltage_data, daemon=True)
thread1.start()


time.sleep(10)
close_vsa_gripper(vsa_holding_empty_cup_min_contact_pos,1023)

# Run the visualization in the main thread
try:
    phidget_online_visualization()

except KeyboardInterrupt:
    VoltageInput().close()
    print("Program stopped.")


