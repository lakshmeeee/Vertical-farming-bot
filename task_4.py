'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 4 of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''


# Team ID:			BM_2114
# Author List:		Aathan A, Hakash M P, Lakshmipriya R, Lakshmi Sruthi K
# Filename:			task_4.py
# Functions:
# Global variables:
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

import cv2
import numpy as np
import os
import sys
import traceback
import math
import time
from pyzbar.pyzbar import decode
from task_1b import *
from task_2a import *
import json

##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
    import sim

except Exception:
    print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
    print('\n[WARNING] Make sure to have following files in the directory:')
    print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
    sys.exit()


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

def send_identified_berry_data(client_id,berry_name,x_coor,y_coor,depth):
	"""
	Purpose:
	---
	Teams should call this function as soon as they identify a berry to pluck. This function should be called only when running via executable.
	
	NOTE: 
	1. 	Correct Pluck marks will only be awarded if the team plucks the last detected berry. 
		Hence before plucking, the correct berry should be identified and sent via this function.

	2.	Accuracy of detection should be +-0.025m.

	Input Arguments:
	---
	`client_id` 	:  [ integer ]
		the client_id generated from start connection remote API, it should be stored in a global variable

	'berry_name'		:	[ string ]
			name of the detected berry.

	'x_coor'			:	[ float ]
			x-coordinate of the centroid of the detected berry.

	'y_coor'			:	[ float ]
			y-coordinate of the centroid of the detected berry.

	'depth'			:	[ float ]
			z-coordinate of the centroid of the detected berry.

	Returns:
	---
	`return_code`		:	[ integer ]
			A remote API function return code
			https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes

	Example call:
	---
	return_code=send_identified_berry_data(berry_name,x_coor,y_coor)
	
	"""
	##################################################
	## You are NOT allowed to make any changes in the code below. ##
	emptybuff = bytearray()

	if(type(berry_name)!=str):
		berry_name=str(berry_name)

	if(type(x_coor)!=float):
		x_coor=float(x_coor)

	if(type(y_coor)!=float):
		y_coor=float(y_coor)	
	
	if(type(depth)!=float):
		depth=float(depth)
	
	data_to_send=[berry_name,str(x_coor),str(y_coor),str(depth)]					
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'eval_bm',sim.sim_scripttype_childscript,'detected_berry_by_team',[],[],data_to_send,emptybuff,sim.simx_opmode_blocking)
	return return_code
	
	##################################################


# def json_to_py():
# # {'CB1': {0: 3, 1: 0, 2: 0}, 'CB2': {0: 0, 1: 2, 2: 1}}
#     f = open('Theme_Config.json')
#     data = json.load(f)
#     final = {'CB1': {0: 0, 1: 0, 2: 0}, 'CB2': {0: 0, 1: 0, 2: 0}}
#     c=0
#     for i in data:
#         d=data[i].split('_')
#         final[d[1]][c] = int(d[0])
#         c+=1
#     return final

# def json_to_py():
#     f = open('Theme_Config.json')
#     data = json.load(f)
#     berry = []
#     for i in data:
#         d = data[i].split('_')
#         d[0] = int(d[0])
#         d.append(i)
#         berry.append(d)
#     # print(berry)
#     return berry


def json_to_py():
    f = open('Theme_Config.json')
    data = json.load(f)
    # print(data)
    # berry={}
    berry = {'CB1': {'B': 0, 'L': 0, 'S': 0}, 'CB2': {'B': 0, 'L': 0, 'S': 0}}

    for i in data:
        d = data[i].split('_')
        d[0] = int(d[0])
        berry[d[1]][i] = d[0]
    # print(berry)
    for i in berry:
        for j in berry[i]:
            if j == 'B':
                berry[i]['Blueberry'] = berry[i].pop(j)
            if j == 'L':
                berry[i]['Lemon'] = berry[i].pop(j)
            if j == 'S':
                berry[i]['Strawberry'] = berry[i].pop(j)
    # print(berry)
    return berry


def balsumcal(value):
    sum = 0
    for val in bal[value]:
        sum += bal[value][val]
    return sum


original = json_to_py()
bal = original.copy()


def task_4_primary(client_id):
    global bal
    sim.simxClearStringSignal(client_id, 'start', sim.simx_opmode_oneshot)
    f = 0
    B = ['Blueberry', 'Lemon', 'Strawberry']
    if f == 0:
        a = 'CB1'
    if balsumcal('CB1') == 0:
        a = 'CB2'
    if balsumcal('CB2') == 0:
        f = 1

    while True:
        #sim.simxSetStringSignal(client_id, 'start', '1',sim.simx_opmode_oneshot)
        #time.sleep(5)

        # detection of berry position
        return_code, vision_sensor_handle = sim.simxGetObjectHandle(
            client_id, 'vision_sensor_2', sim.simx_opmode_oneshot_wait)
        vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
            client_id, vision_sensor_handle)
        vision_sensor_depth_image, depth_image_resolution, return_code_2 = get_vision_sensor_depth_image(
            client_id, vision_sensor_handle)
        transformed_image = task_1b.transform_vision_sensor_image(
            vision_sensor_image, image_resolution)
        transformed_depth_image = transform_vision_sensor_depth_image(
            vision_sensor_depth_image, depth_image_resolution)
        berries_dictionary = detect_berries(
            transformed_image, transformed_depth_image)
        berry_positions_dictionary = detect_berry_positions(berries_dictionary)
        #berry_dict = { 0:'Blueberry', 1:'Lemon', 2:'Strawberry'}
        nearest_berries = []
        berries = [len(berry_positions_dictionary['Blueberry']), len(
            berry_positions_dictionary['Lemon']), len(berry_positions_dictionary['Strawberry'])]

        # if f==0:
        #     a = 'CB1'
        # if bal['CB1'][0]==0 and bal['CB1'][1]==0 and bal['CB1'][2]==0:
        #     f=1
        # if f==1:
        #     a = 'CB2'
        #     if bal['CB2'][0]==0 and bal['CB2'][1]==0 and bal['CB2'][2]==0:
        #         f=2
        # if f!=2:
        #     for i in range(3):
        #         n = original[a][i]
        #         if n!=0:
        #             for j in range(n):
        #                 if berries[i]!=0:
        #                     nearest_berries.append(berry_positions_dictionary[berry_dict[i]][j])
        #                     bal[a][i]-=1
        #                     berries[i]-=1

        if f != 1:
            #print('task_4')
            for index, berry in enumerate(B):
                if(berries[0] >= bal[a][berry]):
                    for i in range(bal[a][berry]):
                        nearest_berries.append(berry_positions_dictionary[berry][i])
                    bal[a][berry] = 0
                elif(berries[0] < bal[a][berry]):
                    newbal = bal[a][berry]
                    for i in range(berries[index]):
                        nearest_berries.append(berry_positions_dictionary[berry][i])
                        newbal -= 1
                    # print(newbal)
                    bal[a][berry] = newbal

        # st_coo = berry_positions_dictionary["Strawberry"]
        # le_coo = berry_positions_dictionary["Lemon"]
        # bb_coo = berry_positions_dictionary["Blueberry"]

        # send_identified_berry_data(client_id,"Blueberry", nearest_berries[0][0], nearest_berries[0][1], nearest_berries[0][2])
        # send_identified_berry_data(client_id,"Lemon", nearest_berries[1][0], nearest_berries[1][1], nearest_berries[1][2])
        break
        # nearest_berries = [bb_coo[3], le_coo[3], st_coo[3]]
        # send_identified_berry_data(client_id,"Blueberry", bb_coo[3][0], bb_coo[3][1], bb_coo[3][2])
        # send_identified_berry_data(client_id,"Lemon", le_coo[3][0], le_coo[3][1], le_coo[3][2])
        # send_identified_berry_data(client_id,"Strawberry", st_coo[3][0], st_coo[3][1], st_coo[3][2])

        # t = [(len(nearest_berries)*3+1)]
        # for berry in nearest_berries:
        #     (x, y, z) = berry
        #     t.append(x)
        #     t.append(y)
        #     t.append(z)
        # for i in range((len(nearest_berries)*3)+1):
        #     t[i] = str(t[i])
        # s = ','
        # s = s.join(t)
        # sim.simxSetStringSignal(client_id, 'sig', s, sim.simx_opmode_oneshot)

        # def call_open_close(client_id, command):
        #     command = [command]
        #     emptybuff = bytearray()
        #     return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(
        #         client_id, 'gripper', sim.sim_scripttype_childscript, 'open_close', [], [], command, emptybuff, sim.simx_opmode_blocking)

        # flag = 0
        # check = 0
        # bcheck = 0
        # scheck = 0
        # while(True):
        #     return_code, signal_value = sim.simxGetStringSignal(
        #         client_id, 'gripper', sim.simx_opmode_blocking)
        #     signal_value = signal_value.decode()
        #     if(signal_value == '1'):
        #         call_open_close(client_id, "open")
        #     elif(signal_value == '21'):
        #         #print('21')
        #         if(check<1):    
        #             send_identified_berry_data(client_id,"Blueberry", nearest_berries[0][0], nearest_berries[0][1], nearest_berries[0][2]) 
        #             check = check + 1
        #         call_open_close(client_id, "close")
        #     elif(signal_value == '22'):
        #         #print('22')
        #         if(bcheck<1):
        #             send_identified_berry_data(client_id,"Lemon", nearest_berries[1][0], nearest_berries[1][1], nearest_berries[1][2])
        #             bcheck = bcheck + 1
        #         call_open_close(client_id, "close")
            # elif(signal_value == '23'):
            #     #print('23')
            #     if(scheck<1):
            #         send_identified_berry_data(client_id,"Strawberry", nearest_berries[2][0], nearest_berries[2][1], nearest_berries[2][2])
            #         scheck = scheck + 1
            #     call_open_close(client_id, "close")
        #     return_code, signal_value_2 = sim.simxGetStringSignal(client_id, 'stop', sim.simx_opmode_blocking)
        #     signal_value_2 = signal_value_2.decode()
        #     if(signal_value_2 == '1'):
        #         flag = 1
        #         break
        # if(flag == 1):
        #     break
    sim.simxClearStringSignal(client_id, 'start', sim.simx_opmode_oneshot)
    return bal


if __name__ == "__main__":

    ##################################################
    ## You are NOT allowed to make any changes in the code below ##

    # Initiate the Remote API connection with CoppeliaSim server
    print('\nConnection to CoppeliaSim Remote API Server initiated.')
    print('Trying to connect to Remote API Server...')

    try:
        client_id = init_remote_api_server()
        if (client_id != -1):
            print('\nConnected successfully to Remote API Server in CoppeliaSim!')

            # Starting the Simulation
            try:
                return_code = start_simulation(client_id)

                if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
                    print('\nSimulation started correctly in CoppeliaSim.')

                else:
                    print(
                        '\n[ERROR] Failed starting the simulation in CoppeliaSim!')
                    print(
                        'start_simulation function is not configured correctly, check the code!')
                    print()
                    sys.exit()

            except Exception:
                print(
                    '\n[ERROR] Your start_simulation function throwed an Exception, kindly debug your code!')
                print('Stop the CoppeliaSim simulation manually.\n')
                traceback.print_exc(file=sys.stdout)
                print()
                sys.exit()

        else:
            print('\n[ERROR] Failed connecting to Remote API server!')
            print('[WARNING] Make sure the CoppeliaSim software is running and')
            print(
                '[WARNING] Make sure the Port number for Remote API Server is set to 19997.')
            print(
                '[ERROR] OR init_remote_api_server function is not configured correctly, check the code!')
            print()
            sys.exit()

    except Exception:
        print(
            '\n[ERROR] Your init_remote_api_server function throwed an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually if started.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()

    try:

        task_4_primary(client_id)
        time.sleep(1)

        try:
            return_code = stop_simulation(client_id)
            if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
                print('\nSimulation stopped correctly.')

                # Stop the Remote API connection with CoppeliaSim server
                try:
                    exit_remote_api_server(client_id)
                    if (start_simulation(client_id) == sim.simx_return_initialize_error_flag):
                        print(
                            '\nDisconnected successfully from Remote API Server in CoppeliaSim!')

                    else:
                        print(
                            '\n[ERROR] Failed disconnecting from Remote API server!')
                        print(
                            '[ERROR] exit_remote_api_server function is not configured correctly, check the code!')

                except Exception:
                    print(
                        '\n[ERROR] Your exit_remote_api_server function throwed an Exception, kindly debug your code!')
                    print('Stop the CoppeliaSim simulation manually.\n')
                    traceback.print_exc(file=sys.stdout)
                    print()
                    sys.exit()

            else:
                print(
                    '\n[ERROR] Failed stopping the simulation in CoppeliaSim server!')
                print(
                    '[ERROR] stop_simulation function is not configured correctly, check the code!')
                print('Stop the CoppeliaSim simulation manually.')

            print()
            sys.exit()

        except Exception:
            print(
                '\n[ERROR] Your stop_simulation function throwed an Exception, kindly debug your code!')
            print('Stop the CoppeliaSim simulation manually.\n')
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()

    except Exception:
        print(
            '\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually if started.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()
