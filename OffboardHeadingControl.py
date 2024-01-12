import tkinter as tk
from tkinter import *
import pymavlink
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink
from  pymavlink import quaternion
import numpy as np
import struct
import array
import time
import os
import sys
import math


def calculate_roll(speed, turn_rate, current_heading, target_heading, time_step):
    # Convert turn rate from degrees per second to radians per second
    turn_rate_rad = np.radians(turn_rate)

    # Calculate the required bank angle using the turn rate formula
    # Bank angle (phi) = arctan((V * omega) / g)
    # where V is the speed, omega is the turn rate, and g is the gravitational acceleration
    bank_angle_rad = np.arctan((speed * turn_rate_rad) / 9.81)
    bank_angle_rad = np.clip(bank_angle_rad, np.deg2rad(-45), np.deg2rad(45))
    # Determine the heading change required to align with the target heading
    heading_change_required = target_heading - current_heading

    # Ensure the heading change required is in the range [-pi, pi] radians
    heading_change_required = (heading_change_required + np.pi) % (2 * np.pi) - np.pi

    # Determine the total heading change, including the effect of the bank angle
    total_heading_change = heading_change_required + (bank_angle_rad * time_step)

    # Apply the sign to the bank angle
    bank_angle_rad *= np.sign(total_heading_change)
    if abs(target_heading - current_heading) <= np.deg2rad(.5):
        bank_angle_rad = 0
    if abs(target_heading - current_heading) <= np.deg2rad(3):
        bank_angle_rad = bank_angle_rad/5
    if abs(target_heading - current_heading) <= np.deg2rad(5):
        bank_angle_rad = bank_angle_rad/10
    return bank_angle_rad

def calculate_pitch(master, current_altitude, target_altitude, time_step):
    # Calculate the altitude error
    altitude_error = target_altitude - current_altitude
    
    # Use a linear relation to determine pitch angle
   
    pitch_angle = altitude_error
    # print("PITCH: ", pitch_angle)

    # Convert the altitude to a degree value
    pitch_angle = np.clip(pitch_angle, -np.deg2rad(30), np.deg2rad(30))

    # Ensure the pitch angle is within the bounds
    # If the pitch angle is > 1, PX4 freaks out. If < - 0.5, PX4 tanks the plane
    pitch_angle = np.clip(pitch_angle, -0.5, 1.0)
    if np.abs(target_altitude-current_altitude)<=0.5:
        pitch_angle = 0
    if np.abs(target_altitude-current_altitude)<=1:
            pitch_angle *= 0.0001

    if target_altitude > current_altitude:
        if np.abs(target_altitude-current_altitude)<=10:
            pitch_angle*=0.001
        if np.abs(target_altitude-current_altitude)<=20:
            pitch_angle*=0.01

    if current_altitude > target_altitude:
        if np.abs(current_altitude - target_altitude)<=10:
            pitch_angle/=5
    # Return the normalized pitch angle
    return pitch_angle

boot_time = time.time()



euler = [0, 0, 0]
master = mavutil.mavlink_connection('udp::14550')
print(master.address)
master.wait_heartbeat()
print('heartbeat from system (system %u component %u)' %(master.target_system, master.target_component))

print('')
print('Enter all three fields in the user interface and press send')
print('Open the PX4 shell and type \'commander takeoff\' to launch the drone')
print('After commander has taken off type \'commander mode offboard\' in the PX4 shell')
master.mav.param_request_read_send(
    master.target_system, master.target_component,
    b'VFR_HUD',
    -1
)

follower_state = master.recv_match(type='VFR_HUD', blocking=True).to_dict()
alt_0 = follower_state['alt']
# Running bool is used to make sure the entry bar only takes data when the
# user tells it to
running = False

# Wait bool is used to stop the drone from sending a heading of 0 when the
# program starts
wait = True

global h
h = "0"
global alt
alt = "0"
global vel
vel = "0"
# initialize position parameters
global x
x = 0
global y
y = 0
turn_rate = 70

def Send():
    global running

    follower_state = master.recv_match(type='VFR_HUD', blocking=True).to_dict()

    if running:
        global h
        if heading.get() != "":
            h = heading.get()
        global alt
        if altitude.get() != "":
            alt = altitude.get()
        global vel
        if velocity.get() != "":
            vel = velocity.get()

        # Clear entry spaces
        heading.set("")
        altitude.set("")
        velocity.set("")

        running = False

    # if float(alt) > 100:
    #     alt = 100
    if float(alt) < 0:
        alt = 0
    # if float(vel) > 10:
    #     vel = 10
    if float(vel) < 0:
        h = float(h) + 180
        if float(vel) <-10:
            vel = -10
        vel = float(vel)*-1

    while float(h) < 0:
        h = float(h)+360
    while float(h) > 360:
        h = float(h)-360

    dir = float(h) #heading
    a = float(alt) #altitude
    v = float(vel) #velocity
    thrust = (v/207.5)
    # print(dir, a, v)
    roll = calculate_roll(3.6*follower_state['airspeed'], turn_rate, np.deg2rad(follower_state['heading']), np.deg2rad(dir), 0.001)
    pitch = calculate_pitch(master, follower_state['alt']-alt_0, a, 0.001)
    euler = [roll, pitch, 0]
    # print(euler)
    global wait
    if not wait:
        if dir >= 0:
            msg = master.mav.set_attitude_target_encode(
            int(1e3 * (time.time() - boot_time)),
            master.target_system,
            master.target_component,
            4,
            quaternion.Quaternion._euler_to_q(master, euler),
            0, 0, 0, thrust#target_speed/max_speed
            )
            
            master.mav.send(msg)
    else:
        euler = [0, 0, 0]
        msg = master.mav.set_attitude_target_encode(
            int(1e3 * (time.time() - boot_time)),
            master.target_system,
            master.target_component,
            4,
            quaternion.Quaternion._euler_to_q(master, euler),
            0, 0, 0, 0#target_speed/max_speed
            )
           
        master.mav.send(msg)
    root.after(1, Send)

#Set running to true and wait to false.
#This function runs when the send heading button is pressed
#Running is set to true so the main loop knows to repeatedly send the heading
#and  wait is set to false so the widget knows to grab whatever is
#in the entry bar
def start():
    global running
    running = True
    global wait
    wait = False
# Define the GUI
root = tk.Tk()
root.title("PX4 Heading Control")
root.geometry('300x85')

#Define the input
heading = tk.StringVar()
altitude = tk.StringVar()
velocity = tk.StringVar()

#Add widgets to the GUI
heading_label = tk.Label(root, text='Heading', font=('calibre',10,'bold'))
heading_entry = tk.Entry(root, textvariable=heading, font=('calibre',10,'normal'))
altitude_label = tk.Label(root, text='Altitude', font=('calibre',10,'bold'))
altitude_entry = tk.Entry(root, textvariable=altitude, font=('calibre',10,'normal'))
velocity_label = tk.Label(root, text='Velocity', font=('calibre',10,'bold'))
velocity_entry = tk.Entry(root, textvariable=velocity, font=('calibre',10,'normal'))
submit_btn = tk.Button(root, text='Send', command = start)

#Adjust the layout
heading_label.grid(row = 1, column = 0)
heading_entry.grid(row = 1, column = 1)
altitude_label.grid(row = 2, column = 0)
altitude_entry.grid(row = 2, column = 1)
velocity_label.grid(row = 3, column = 0)
velocity_entry.grid(row = 3, column = 1)
submit_btn.grid(row = 2, column = 2)

#Run the GUI
root.after(1, Send)
root.mainloop()
time.sleep(0.5)
