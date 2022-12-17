##################################################################
# This is the latest file I'm working on
# The theme_implementation in File
##################################################################

import cv2
import numpy as np
import os
import sys
import traceback
import math
import time
import sys
import json
from pyzbar.pyzbar import decode
from task_4 import *
from i import *
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


def init_remote_api_server():
    client_id = -1
    client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    return client_id


def start_simulation(client_id):
    return_code = -2
    return_code = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)
    return return_code


def stop_simulation(client_id):
    return_code = -2
    return_code = sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot)
    return return_code


def exit_remote_api_server(client_id):
    sim.simxFinish(client_id)


def get_vision_sensor_image(client_id):
    return_code = 0
    return_code, cam_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_oneshot_wait)
    return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, cam_handle, 0, sim.simx_opmode_oneshot_wait)
    return vision_sensor_image, image_resolution, return_code


def transform_vision_sensor_image(vision_sensor_image, image_resolution):
    transformed_image = None
    im = np.array(vision_sensor_image, dtype=np.uint8)
    im.resize([image_resolution[0], image_resolution[1], 3])
    transformed_image = cv2.flip(im, 1)
    transformed_image = cv2.cvtColor(transformed_image, cv2.COLOR_RGB2BGR)
    return transformed_image


def detect_qr_codes(transformed_image):
    def decoder(image):
        gray_img = cv2.cvtColor(image, 0)
        barcode = decode(gray_img)
        flag = 0
        for obj in barcode:
            barcodeData = obj.data.decode("utf-8")
            flag = flag + 1
            c = barcodeData
            return c
    c = decoder(transformed_image)
    return c


def coordinates(client_id):
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
    transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    code = detect_qr_codes(transformed_image)
    result = (-1, -1)
    if(code != None):
        result = tuple(int(num) for num in code.replace('(', '').replace(')', '').replace('...', '').split(', '))
    return result


def init_setup(client_id):
    rcfl, fl_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_fl', sim.simx_opmode_blocking)
    rcfr, fr_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_fr', sim.simx_opmode_blocking)
    rcfl, rl_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_rl', sim.simx_opmode_blocking)
    rcfr, rr_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_rr', sim.simx_opmode_blocking)
    wheel_joints = [fl_handle, rl_handle, rr_handle, fr_handle]
    return wheel_joints


def encoders(client_id):
    return_code, signal_value = sim.simxGetStringSignal(client_id, 'combined_joint_position', sim.simx_opmode_blocking)
    signal_value = signal_value.decode()
    joints_position = signal_value.split("%")
    if joints_position[0] != '':
        for index, joint_val in enumerate(joints_position):
            joints_position[index] = float(joint_val)
    else:
        return [0, 0, 0, 0]
    for i in range(len(joints_position)):
        joints_position[i] = joints_position[i] * 0.1/2  # distance travelled by wheels

    return joints_position


def set_bot_movement(client_id, w1, w2, w3, w4):
    wheel = init_setup(client_id)
    rcrfl1 = sim.simxSetJointTargetVelocity(client_id, wheel[0], w1, sim.simx_opmode_streaming)
    rcrrl1 = sim.simxSetJointTargetVelocity(client_id, wheel[1], w2, sim.simx_opmode_streaming)
    rcrrr1 = sim.simxSetJointTargetVelocity(client_id, wheel[2], w3, sim.simx_opmode_streaming)
    rcrfr1 = sim.simxSetJointTargetVelocity(client_id, wheel[3], w4, sim.simx_opmode_streaming)


def shortest_path(x1, y1, x2, y2):
    V = 5
    x = x2-x1
    y = y2-y1
    r = math.sqrt(x*x + y*y)
    s = x/r
    c = y/r
    Vx = V * s
    Vy = V * c

    # correction after rotation
    vx = Vx*math.cos(theta)-Vy*math.sin(theta)
    vy = Vx*math.sin(theta)+Vy*math.cos(theta)
    if abs(vx) < 0.0000001:
        vx = 0
    if abs(vy) < 0.0000001:
        vy = 0

    return vx, vy, r  # fb_vel, rl_vel, distance


def cb(num):
    # cb1
    if num == 1:
        l = [(4, 10), (1, 10), (4, 10), (4, 4)]
    # cb2
    elif num == 2:
        l = [(4, 10), (7, 10)]

    return l


def room1(x2, y2):
    pos = 'g'
    # (2, 5)
    if (x2, y2) == (2, 5):
        pos = 'l'
        l = [(x2, 4), (x2, 6)]
    # (3, 6)
    elif x2 == 3:
        pos = 'u'
        l = [(4, y2), (2, y2)]
    # (0, 5)
    else:
        l = [(x2, 4), (x2, 6)]

    return l+[(1, 7)]+l[::-1]+[(4, 4)], pos


def room2(x2, y2):
    pos = 'g'
    # (6, 5)
    if y2 == 5:
        pos = 'u'
        l = [(x2, 4), (x2, 6)]
    # (5, 8)
    elif (x2, y2) == (5, 8):
        pos = 'g'  # should be r
        l = [(4, y2), (6, y2)]
    # (5, 6)
    else:
        l = [(4, y2), (6, y2)]

    return l+[(7, 7)]+l[::-1]+[(4, 4)], pos


def room3(x2, y2):
    pos = 'g'
    # (6, 3), (8, 3)
    if y2 == 3:
        l = [(x2, 4), (x2, 2)]
    # (5, 2)
    elif x2 == 5:
        pos = 'u'
        l = [(4, y2), (6, y2)]

    return l+[(7, 1)]+l[::-1]+[(4, 4)], pos


def room4(x2, y2):
    pos = 'g'
    # (2, 3)
    if y2 == 3:
        pos = 'u'
        l = [(x2, 4), (x2, 2)]
    # (3, 2)
    elif (x2, y2) == (3, 2):
        pos = 'l'
        l = [(4, y2), (2, y2)]
    # (3, 0)
    else:
        l = [(4, y2), (2, y2)]

    return l+[(1, 1)]+l[::-1]+[(4, 4)], pos


bal = {}  # this has the berries not yet collected

def balsumcal(value):
    sum = 0
    for val in bal[value]:
        sum += bal[value][val]
    return sum


# function for dropping berries in collection box
def dropberries(client_id):
    sim.simxSetStringSignal(client_id, 'drop', '1', sim.simx_opmode_oneshot)
    rc = sim.simxClearStringSignal(client_id, 'dropcom', sim.simx_opmode_oneshot)
    while True:
        return_code, signal_value = sim.simxGetStringSignal(client_id, 'dropcom', sim.simx_opmode_blocking)
        signal_value = signal_value.decode()
        if(signal_value == '1'):
            time.sleep(3)
            rc = sim.simxSetStringSignal(client_id, 'drop', '2', sim.simx_opmode_oneshot_wait)
        elif(signal_value == '2'):
            time.sleep(3)
            sim.simxSetStringSignal(client_id, 'drop', '3', sim.simx_opmode_oneshot)
        elif(signal_value == '3'):
            time.sleep(3)
            sim.simxSetStringSignal(client_id, 'drop', '4', sim.simx_opmode_oneshot)
        elif(signal_value == '4'):
            rc = sim.simxClearStringSignal(client_id, 'dropcom', sim.simx_opmode_oneshot_wait)
            rc = sim.simxClearStringSignal(client_id, 'drop', sim.simx_opmode_oneshot_wait)
            break


def get_angle(qrcode):
    poly = qrcode.polygon
    angle = math.atan2(poly[1].y - poly[0].y, poly[1].x - poly[0].x)
    return angle - math.pi/2


theta = 0

def rotation(client_id, dir):
    global theta
    time.sleep(2)
    # this calculates the value of theta to be achieved and the velocity to be given
    if dir == 'c':
        theta += math.pi/2
        w = x = 1
        z = y = -1
    elif dir == 'ac':
        theta -= math.pi/2
        w = x = -1
        z = y = 1

    print('rotation\n')
    set_bot_movement(client_id, w, x, y, z)
    flag = 0
    while True:
        flag+=1
        vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
        transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
        gray_img = cv2.cvtColor(transformed_image, 0)
        barcode = decode(gray_img)
        for obj in barcode:
            a = get_angle(obj)
            # print(math.degrees(abs(a)))
            if flag>5:
                if(dir == 'c' and math.degrees(abs(a)) > 87):
                    # stopping the bot after rotation
                    print('clockwise rotation stopped')
                    r = set_velocity(client_id, 0, 90, 0, 0, 0, 'r', 0, 0)
                    # set_bot_movement(client_id, 0, 0, 0, 0)
                    time.sleep(1)
                    return None
                if(dir == 'ac' and math.degrees(abs(a)) < 4):
                    print('anticlockwise rotation stopped')
                    # stopping the bot after rotation
                    r = set_velocity(client_id, 0, 90, 0, 0, 0, 'r', 0, 0)
                    # set_bot_movement(client_id, 0, 0, 0, 0)
                    time.sleep(1)
                    return None


# this function stops the bot at the required position in the vision sensor
def newStop(client_id, X, Y, r='g'):
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
    transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    gray_img = cv2.cvtColor(transformed_image, 0)
    barcode = decode(gray_img)
    for obj in barcode:
        (x, y, w, h) = obj.rect
        a = (x + w//2)
        b = (y + h//2)
        code = coordinates(client_id)
        # cv2.line(transformed_image, (a, b),(a+1, b), (255, 0, 0), thickness=2)

        if code == (X, Y):
            #print(r)
            if(r == 'c' and a > 226 and a < 284 and b > 226 and b < 284):
                set_bot_movement(client_id, 0, 0, 0, 0)
                print('centre')
                return 1
            elif(r == 'u' and b > 284):
                set_bot_movement(client_id, 0, 0, 0, 0)
                print('up')
                return 1
            elif(r == 'd' and b < 226-50):
                set_bot_movement(client_id, 0, 0, 0, 0)
                print('down')
                return 1
            elif(r == 'l' and a > 284+50 ):
                set_bot_movement(client_id, 0, 0, 0, 0)
                print('left')
                return 1
            elif(r == 'r' and a < 226-50 ):
                set_bot_movement(client_id, 0, 0, 0, 0)
                print('right')
                return 1
            elif(r == 'g'):
                set_bot_movement(client_id, 0, 0, 0, 0)
                print('general')
                return 1
    # cv2.imshow('img', transformed_image)
    # cv2.waitKey(3)
    return 0


u = 0
prev_vel = [0, 0, 0, 0]
prev_encoder = [0, 0]
prev_stop_enc = [0, 0]

def set_velocity(client_id, flag, dist, forw_back_vel, left_right_vel, rot_vel, pos, x2, y2):
    global prev_vel
    global prev_encoder
    global prev_stop_enc
    global u
    i = 1
    
    # initialising when starting from a coordinate
    if flag == 0:
        #print('d', dist)
        prev_encoder[0] = 0
        prev_encoder[1] = 0
        u = 0

    # getting encoder values for front two wheels
    joint_pos = encoders(client_id)
    fl = joint_pos[0]
    fr = joint_pos[3]

    abs_fl = abs(fl)
    abs_fr = abs(fr)

    # when the encoder values goes from positive to zero to negative or vice versa,
    # the prev value needs to be added to account for the distance travelled
    # u = 0 is for adding it for the first time
    if ((prev_encoder[0] > 0 and fl < 0) or (prev_encoder[0] < 0 and fl > 0)) and u == 0:
        abs_fl += prev_stop_enc[0]
        u = 1
    if ((prev_encoder[1] > 0 and fr < 0) or (prev_encoder[1] < 0 and fr > 0)) and u == 0:
        abs_fr += prev_stop_enc[1]
        u = 2

    if u == 1:
        abs_fl += prev_stop_enc[0]
    if u == 2:
        abs_fr += prev_stop_enc[1]

    # print(abs_fl, abs_fr, '\nline', joint_pos[0], joint_pos[3])

    # the main part
    # after 70% of distance, we slow down by 1.5 times each time and call newStop to stop at the required pos
    # after stopping we update the prev values to use it next time and we return 1 to nav_logic function
    if dist>2:
        cover = 0.8
    else:
        cover = 0.65

    if u == 0:
        condition = abs(abs_fl-prev_stop_enc[0]) > dist*cover or abs(abs_fr-prev_stop_enc[1]) > dist*cover
    else:
        condition = abs_fl > dist*cover or abs_fr > dist*cover

    if condition:
        #print('here', prev_vel[0], prev_vel[3])
    ##############################
        i = 1.5             # this needs to be adjusted to an optimal value
    ##############################
        stop = newStop(client_id, x2, y2, pos)
        if stop == 1:
            print('stopped\n')
            # these are the encoder values at the last stop
            prev_stop_enc[0] = abs_fl
            prev_stop_enc[1] = abs_fr
            return 1
    ####################

    # this is for setting vel for the first time when starting from any coordinate
    if flag == 0:
        if pos == 'r':  # this part is to update prev values after rotation
            prev_stop_enc[0] = abs_fl
            prev_stop_enc[1] = abs_fr
        w1 = (left_right_vel + forw_back_vel + rot_vel)
        w2 = (left_right_vel - forw_back_vel + rot_vel)
        w3 = (left_right_vel + forw_back_vel - rot_vel)
        w4 = (left_right_vel - forw_back_vel - rot_vel)

    # this is the part where we slow down,
    # if i = 1 it doesnt slow down
    else:
        if abs(prev_vel[0]) < 1 and abs(prev_vel[3]) < 1:  # min velocity
            i = 1
        w1 = prev_vel[0]/i
        w2 = prev_vel[1]/i
        w3 = prev_vel[2]/i
        w4 = prev_vel[3]/i

    # these are the variables which has the previous encoder values each time we call set_velocity
    prev_encoder[0] = fl
    prev_encoder[1] = fr

    set_bot_movement(client_id, w1, w2, w3, w4)

    # these are the previous velocity values
    prev_vel[0] = w1
    prev_vel[1] = w2
    prev_vel[2] = w3
    prev_vel[3] = w4
    # print(w1,w2,w3,w4)
    return 0


def nav_logic(client_id, target_points, p, pos):
    global bal

    # let this be dont remove
    (x1, y1) = (4, 4)
    if p > 0:
        (x1, y1) = target_points[p-1]
    (x2, y2) = target_points[p]
    ##

    fb_vel, lr_vel, dist = shortest_path(x1, y1, x2, y2)
    flag = 0
    dist *= 0.53
    stop = 0

    while True:
        # when we call set_velocity for the first time flag = 0 then flag = 1
        stop = set_velocity(client_id, flag, dist, fb_vel, lr_vel, 0, pos, x2, y2)
        flag = 1

        if stop == 1:  # if stop==1 the bot has already stopped
            # at the plant
            if (x2, y2) == (1, 7) or (x2, y2) == (7, 7) or (x2, y2) == (7, 1) or (x2, y2) == (1, 1):
                # centre(client_id)
                # bal is the number of berries we havent collected yet
                bal = task_4_primary(client_id)
            # at the collection box
            if (x2, y2) == (1, 10) or (x2, y2) == (7, 10):
                # this drops the berries in the collection box
                centre(client_id)
                dropberries(client_id)
            if (x2, y2) == (4, 4):
                centre(client_id)
            break


f = 0

def theme_implementation_primary(client_id, rooms_entry):
    global f
    points = []

    # for traversing to all 4 rooms
    for i in range(len(rooms_entry)):
        (x2, y2) = rooms_entry[i]

        if i == 0:
            # rotation(client_id, 'ac')
            points, pos = room1(x2, y2)
        elif i == 1:
            while (theta) != math.pi/2:
                # rotating until required theta value is achieved
                rotation(client_id, 'c')
            points, pos = room2(x2, y2)
            # a = []
            # a[4] = 1500
        elif i == 2:
            while (theta) != math.pi:
                rotation(client_id, 'c')
            points, pos = room3(x2, y2)
        elif i == 3:
            print('\n\nroom4')
            while (theta) != 3*math.pi/2:
                rotation(client_id, 'c')
            points, pos = room4(x2, y2)

        for j in range(len(points)):
            if j != 0:
                pos = 'g'
            nav_logic(client_id, points, j, pos)

    # for going to collection box
        points.clear()  # this is so that we dont go back to any of the rooms in case we dont go to collection box
        # collection box 1
        if f == 0:
            if balsumcal('CB1') == 0:  # this calculates if there are any berries to be collected
                while (theta) != 0:
                    rotation(client_id, 'ac')
                points = cb(1)
                f = 1
        # collection box 2
        elif f == 1:
            if balsumcal('CB2') == 0:
                while (theta) != 0:
                    rotation(client_id, 'ac')
                points = cb(2)
                f = 2

        if len(points) != 0:
            pos = 'g'
            #print(points)
            for j in range(len(points)):
                nav_logic(client_id, points, j, pos)
            # the end
            if f == 2:
                print('end')
                set_bot_movement(client_id, 0, 0, 0, 0)
                time.sleep(3)
                break


if __name__ == "__main__":

    # Room entry co-ordinate
    rooms_entry = [(0, 5), (6, 5), (5, 2), (3, 0)]     # example list of tuples

    ###############################################################
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

        # Running student's logic
        theme_implementation_primary(client_id, rooms_entry)

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
        print('\n[ERROR] Your theme_implementation_primary function throwed an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually if started.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()

    except KeyboardInterrupt:
        print('\n[ERROR] Script interrupted by user!')
