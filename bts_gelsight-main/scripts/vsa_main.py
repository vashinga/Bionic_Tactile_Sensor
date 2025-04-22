"""
*******************************************************************************

Project: bts_gelsight
File: vsa_main.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: Jan 17, 2025

Description:
Script for driving dynamixel motors (RX-24F) on variable-stiffness actuator (VSA) gripper.

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

import time
import yaml
import argparse
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageInput import *
from dynamixel_sdk import *  # Uses Dynamixel SDK library
from threading import Thread, Lock
from queue import Queue

with open("../config/config.yml", 'r') as file:
    config = yaml.safe_load(file)

phidget_data_queue = Queue()
dynamixel_pos_data_queue = Queue()

# Control table addresses for RX-24F (Protocol 1.0)
DXL_ADDR_TORQUE_ENABLE = config["DXL"]["ADDR_TORQUE_ENABLE"]  # Address for torque enable
DXL_ADDR_GOAL_POSITION = config["DXL"]["ADDR_GOAL_POSITION"]  # Address for goal position
DXL_ADDR_MOVING_SPEED = config["DXL"]["ADDR_MOVING_SPEED"]        # Moving speed address
DXL_ADDR_TORQUE_LIMIT = config["DXL"]["ADDR_TORQUE_LIMIT"]          # Torque limit address
DXL_ADDR_PRESENT_POSITION   = config["DXL"]["ADDR_PRESENT_POSITION"]
DXL_ADDR_PRESENT_SPEED      = config["DXL"]["ADDR_PRESENT_SPEED"]
DXL_ADDR_PRESENT_LOAD       = config["DXL"]["ADDR_PRESENT_LOAD"]
DXL_ADDR_PRESENT_INPUT_VOLTAGE = config["DXL"]["ADDR_PRESENT_INPUT_VOLTAGE"]
DXL_ADDR_PRESENT_TEMP       = config["DXL"]["ADDR_PRESENT_TEMP"]

# Protocol version
PROTOCOL_VERSION = config["DXL"]["PROTOCOL_VERSION"]  # RX-24F uses Protocol 1.0
DXL_RESOLUTION = config["DXL"]["RESOLUTION"]
POT_RESOLUTION = config["LINEAR_POTENTIOMETER"]["RESOLUTION"]

# Default settings
BAUDRATE = config["DXL"]["BAUDRATE"]  # Dynamixel baud rate
DEVICENAME = config["DXL"]["DEVICENAME"]  # Port (e.g., /dev/ttyUSB0 or COM3)

TORQUE_ENABLE = config["DXL"]["TORQUE_ENABLE"]  # Value for enabling torque
TORQUE_DISABLE = config["DXL"]["TORQUE_DISABLE"]  # Value for disabling torque
DXL_MOVING_STATUS_THRESHOLD = config["DXL"]["DXL_MOVING_STATUS_THRESHOLD"]  # Dynamixel moving status threshold

DXL1_ID = config["DXL"]["DXL1_ID"]
DXL2_ID = config["DXL"]["DXL2_ID"]

DXL1_MIN_VALUE = config["DXL"]["DXL1_MIN_VALUE"]
DXL1_MAX_VALUE = config["DXL"]["DXL1_MAX_VALUE"]
DXL2_MIN_VALUE = config["DXL"]["DXL2_MIN_VALUE"]
DXL2_MAX_VALUE = config["DXL"]["DXL2_MAX_VALUE"]

POT_1_CHANNEL = config["PHIDGET"]["POT_1_CHANNEL"]
POT_2_CHANNEL = config["PHIDGET"]["POT_2_CHANNEL"]

POT_1_FREQ_HZ = config["PHIDGET"]["POT_1_FREQ_HZ"]
POT_2_FREQ_HZ = config["PHIDGET"]["POT_2_FREQ_HZ"]

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

dxl_lock = Lock()


def control_dxl_RX_24F(dxl_id, pos, vel, torq):
    """
    Command a Dynamixel motor to move to a specified position with a given speed and torque limit.
    Assumes that the Dynamixel port has already been opened and that a global lock (dxl_lock)
    is available for serializing access.
    """
    try:
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

        # Assume the port is already open.
        # Enable Dynamixel torque.
        with dxl_lock:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, DXL_ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Communication error: {packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"Dynamixel error: {packetHandler.getRxPacketError(dxl_error)}")
        else:
            print("Torque enabled!")

        # Set torque limit.
        with dxl_lock:
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, dxl_id, DXL_ADDR_TORQUE_LIMIT, torq)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"{packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"{packetHandler.getRxPacketError(dxl_error)}")
        else:
            print(f"Torque limit set to {torq}.")

        # Set goal position.
        with dxl_lock:
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, dxl_id, DXL_ADDR_GOAL_POSITION, pos)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Communication error: {packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"Dynamixel error: {packetHandler.getRxPacketError(dxl_error)}")
        else:
            print(f"Goal position set to: {pos}")

        # Set moving speed.
        with dxl_lock:
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, dxl_id, DXL_ADDR_MOVING_SPEED, vel)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"{packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"{packetHandler.getRxPacketError(dxl_error)}")
        else:
            print(f"Speed set to {vel}.")

        # Wait until the motor reaches the goal position.
        while True:
            with dxl_lock:
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, dxl_id, DXL_ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Communication error: {packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"Dynamixel error: {packetHandler.getRxPacketError(dxl_error)}")
            print(f"Present Position: {dxl_present_position}", end=" \r")
            if abs(pos - dxl_present_position) < DXL_MOVING_STATUS_THRESHOLD:
                break
            time.sleep(0.1)

        print("\nGoal position reached!")

    finally:
        # Disable Dynamixel torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, DXL_ADDR_PRESENT_POSITION, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Communication error: {packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"Dynamixel error: {packetHandler.getRxPacketError(dxl_error)}")
        else:
            print("Torque disabled!")



def read_dxl_current_pos(dxl_id):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, dxl_id, DXL_ADDR_PRESENT_POSITION)
    print("dxl current pos: ", dxl_present_position, end=" \r")

def open_vsa_gripper():
    """ Open gripper fingers to the max. """
    control_dxl_RX_24F(dxl_id=DXL2_ID, pos=0, vel=1023, torq=300)  # homming
    control_dxl_RX_24F(dxl_id=DXL1_ID, pos=0, vel=1023, torq=1023)
    
    
def close_vsa_gripper(pos, vel):
    """ 
    Close gripper fingers to a degree:
    input: deg_of_close = (0 (home) - 600 (stiffest))
    """
    control_dxl_RX_24F(dxl_id=DXL1_ID, pos=1023, vel=1023, torq=1023)  # homming
    control_dxl_RX_24F(dxl_id=DXL2_ID, pos=pos, vel=vel, torq=1023)


def read_present_position(dxl_id, addr):
    with dxl_lock:
        position, result, error = packetHandler.read2ByteTxRx(portHandler, dxl_id, addr)
    if result != COMM_SUCCESS:
        print(f"Error reading position: {packetHandler.getTxRxResult(result)}")
    elif error != 0:
        print(f"Error code: {packetHandler.getRxPacketError(error)}")
    return position


def read_dxl():
    """ Continuously read data from both motors and stream via LSL. """

    pot_1_instance = VoltageInput()
    pot_1_instance.setChannel(POT_1_CHANNEL)
    pot_1_instance.openWaitForAttachment(5000)
    pot_1_instance.setDataInterval(int(1000 / POT_1_FREQ_HZ))

    try:
        while True:
            # with dxl_lock:
            
            dxl_1_pos_bit = read_present_position(DXL1_ID, DXL_ADDR_PRESENT_POSITION)
            dxl_2_pos_bit = read_present_position(DXL2_ID, DXL_ADDR_PRESENT_POSITION)
            pot_1_voltage = pot_1_instance.getVoltage()

            # print(pot1_voltage, 'motor 1: ', dxl_1_pos_bit, 'motor 2: ', dxl_2_pos_bit)

            AG1, AG2 = measure_vsa_air_gaps(pot_1_voltage, dxl_1_pos_bit, dxl_2_pos_bit)

            print("AG1: ", AG1, "AG2: ", AG2)

            time.sleep(0.01)

    except KeyboardInterrupt:
        pot_1_instance.close()
    
    while True:
        time.sleep(0.01)


def write_dxl():
    """ Continuously wait for user input to send commands. """
    while True:
        try:
            user_input = input("\nEnter a number (1:open gripper, 2: close gripper): ")

            if user_input == "1":
                with dxl_lock:
                    open_vsa_gripper()
                    # dxl_comm_result1, dxl1_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
                    # dxl_comm_result2, dxl2_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)


            elif user_input == "2":
                with dxl_lock:
                    close_vsa_gripper(pos=300, vel=100)

            else:
                print("Please enter a valid number.")

        except Exception as e:
            print(f"Error reading input: {e}")


def measure_vsa_air_gaps(pot_1_voltage, dxl_1_pos_bit, dxl_2_pos_bit):


    dxl_1_dist = 206.3 - 45.75 - (1024 - dxl_1_pos_bit) * DXL_RESOLUTION
    dxl_2_dist = 38.5 + dxl_2_pos_bit * DXL_RESOLUTION

    finger_1_dist = 47.4 + (5 - pot_1_voltage) * POT_RESOLUTION

    AG_1 = dxl_1_dist - finger_1_dist - 15./2.
    AG_2 = finger_1_dist - dxl_2_dist - 15./2.

    AG_1 = AG_1 / 1000
    AG_1 = AG_1 / 1000

    return AG_1, AG_2

    

def Pot_1_onVoltageChange(self, voltage):

    phidget_data_queue.put(voltage)
    # print(f"Phidget 1 Voltage: {voltage:.3f}")


def Pot_2_onVoltageChange(self, voltage):

    print(f"Phidget 2 Voltage: {voltage:.3f}", end="\r")


def get_phidget_data():

    pot_1_instance = VoltageInput()
    pot_1_instance.setChannel(POT_1_CHANNEL)
    pot_1_instance.setOnVoltageChangeHandler(Pot_1_onVoltageChange)
    pot_1_instance.openWaitForAttachment(5000)
    pot_1_instance.setDataInterval(int(1000 / POT_1_FREQ_HZ))

    # pot_2_instance = VoltageInput()
    # pot_2_instance.setChannel(POT_2_CHANNEL)
    # pot_2_instance.setOnVoltageChangeHandler(Pot_2_onVoltageChange)
    # pot_2_instance.openWaitForAttachment(5000)
    # pot_2_instance.setDataInterval(int(1000 / POT_2_FREQ_HZ))


    try:
        # Keep the thread alive
        while True:
            time.sleep(0.01)

    except KeyboardInterrupt:
        pot_1_instance.close()
        # pot_2_instance.close()
        print("\nStop reading sensor!")


if __name__ == "__main__":

    # Open Dynamixel port once
    if not portHandler.openPort():
        print("Failed to open Dynamixel port!")
        quit()

    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to set baudrate!")
        quit()


    parser = argparse.ArgumentParser(description="Script to call different functions based on arguments.")
    parser.add_argument("action", nargs='?', type=str, choices=["open", "close"],
                        default=None,
                        help="Specify the action to perform: 'open' or 'close'. If not provided, sensor threads are started.")
    args = parser.parse_args()

    if args.action == "open":
        open_vsa_gripper()

    elif args.action == "close":
        # time.sleep(20)
        close_vsa_gripper(pos=120, vel=1023)  # 400

    else:
        dxl_read_thread = Thread(target=read_dxl, daemon=True)
        dxl_read_thread.start()

        # Keep threads alive
        while True:
            time.sleep(0.01)