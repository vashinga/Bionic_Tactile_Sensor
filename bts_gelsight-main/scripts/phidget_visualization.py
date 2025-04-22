"""
*******************************************************************************

Project: bts_gelsight
File: phidget_visualization.py
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
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread
from queue import Queue
from scipy.signal import find_peaks

with open("../config/config.yml", 'r') as file:
    config = yaml.safe_load(file)

POT_1_CHANNEL = config["PHIDGET"]["POT_1_CHANNEL"]  # Address for torque enable
POT_2_CHANNEL = config["PHIDGET"]["POT_2_CHANNEL"]  # Address for torque enable
BTS_CHANNEL = config["PHIDGET"]["BTS_CHANNEL"]  # Address for torque enable

# Queue for thread-safe communication
data_queue = Queue()

stop_flag = False
voltage_buffer = []
BUFFER_SIZE = 50       # Number of recent samples to keep
MA_WINDOW = 5          # Window size for moving average filter
PEAK_THRESHOLD = 2.79   # Minimum height for a peak (adjust as needed)
PEAK_DISTANCE = 5      # Minimum samples between peaks

# Callback function to handle voltage change
def onVoltageChange(self, voltage):
    global voltage_buffer

    data_queue.put((time.time(), voltage))  # Push timestamp and voltage into the queue
    print(f"Voltage: {voltage:.3f}", end=" \r")

    # Append the new voltage sample to the buffer
    voltage_buffer.append(voltage)
    # Keep only the most recent BUFFER_SIZE samples
    if len(voltage_buffer) > BUFFER_SIZE:
        voltage_buffer.pop(0)
    
    # Optional: Immediately try to detect peaks when the buffer is full
    if len(voltage_buffer) == BUFFER_SIZE:
        # Apply a simple moving average to reduce noise
        ma_voltage = np.convolve(voltage_buffer, np.ones(MA_WINDOW)/MA_WINDOW, mode='valid')
        # Use SciPy's find_peaks to detect peaks in the smoothed signal
        peaks, properties = find_peaks(ma_voltage, height=PEAK_THRESHOLD, distance=PEAK_DISTANCE)
        if peaks.size > 0:
            pass
            print("Peaks detected at buffer indices:", peaks)
            # For control, you might trigger some action here when a peak is detected

# Target function for the thread to fetch voltage data
def get_voltage_data():
    voltageInput0 = VoltageInput()
    voltageInput0.setChannel(BTS_CHANNEL)  # TODO
    voltageInput0.setOnVoltageChangeHandler(onVoltageChange)
    voltageInput0.openWaitForAttachment(5000)
    voltageInput0.setDataInterval(10)  # Adjust data collection interval in milliseconds
    
    while True:
        time.sleep(0.1)  # Ensure the thread runs continuously
        

# Function for real-time visualization with Moving Average
def phidget_online_visualization():
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()
    
    line_raw, = ax.plot([], [], lw=1, label="Raw Voltage", color='blue')
    line_ma, = ax.plot([], [], lw=2, label="Moving Average", color='red')  # Moving average line
    
    # Set up initial plot limits
    ax.set_xlim(0, 10)  # Initial x-axis range (10 seconds)
    ax.set_ylim(-1.5, 1.5)  # Initial y-axis range
    plt.xlabel("Time (s)")
    plt.ylabel("Voltage (V)")
    plt.title("Real-Time Voltage with Moving Average")
    plt.legend()

    start_time = time.time()
    x_data, y_data = [], []
    window_size = 10  # Define moving average window

    while True:
        while not data_queue.empty():
            current_time, voltage = data_queue.get()
            x_data.append(current_time - start_time)
            y_data.append(voltage)

        if x_data:
            # Compute moving average using NumPy
            if len(y_data) >= window_size:
                y_ma = np.convolve(y_data, np.ones(window_size) / window_size, mode='valid')
                x_ma = x_data[len(x_data) - len(y_ma):]  # Align timestamps
            else:
                y_ma = y_data  # If not enough data, just plot raw
                x_ma = x_data  # Ensure x_ma is also assigned

            # Update plot data
            # line_raw.set_data(x_data, y_data)  # actual BTS signal
            line_ma.set_data(x_ma, y_ma)  # Avoids referencing uninitialized x_ma

            # Dynamically adjust plot limits
            ax.set_xlim(max(0, x_data[-1] - 10), x_data[-1] + 1)  # Keep a 10-second window
            ax.set_ylim(min(y_data) - 0.1, max(y_data) + 0.1)

        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.1)  # Adjust refresh rate as needed

# Create and start the data acquisition thread
thread1 = Thread(target=get_voltage_data, daemon=True)
thread1.start()

# Run the visualization in the main thread
try:
    phidget_online_visualization()

except KeyboardInterrupt:
    VoltageInput().close()
    print("Program stopped.")


