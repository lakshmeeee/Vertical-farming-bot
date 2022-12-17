import cv2
import numpy as np
import time
import math
from pyzbar.pyzbar import decode
from task_1b import *


try:
    import sim

except Exception:
    print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
    print('\n[WARNING] Make sure to have following files in the directory:')
    print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
    sys.exit()


def init_setup(client_id):
    rcfl, fl_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_fl', sim.simx_opmode_blocking)
    rcfr, fr_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_fr', sim.simx_opmode_blocking)
    rcfl, rl_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_rl', sim.simx_opmode_blocking)
    rcfr, rr_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_rr', sim.simx_opmode_blocking)
    wheel_joints = [fl_handle, rl_handle, rr_handle, fr_handle]
    return wheel_joints


def stop(client_id):
    wheel_joints = init_setup(client_id)
    rcrfl1 = sim.simxSetJointTargetVelocity(client_id, wheel_joints[0], 0, sim.simx_opmode_streaming)
    rcrfr1 = sim.simxSetJointTargetVelocity(client_id, wheel_joints[1], 0, sim.simx_opmode_streaming)
    rcrrl1 = sim.simxSetJointTargetVelocity(client_id, wheel_joints[2], 0, sim.simx_opmode_streaming)
    rcrrr1 = sim.simxSetJointTargetVelocity(client_id, wheel_joints[3], 0, sim.simx_opmode_streaming)


def new_short(x1, y1):
    x2 = 255
    y2 = 255
    V = 0.5
    x = x2-x1
    y = y2-y1
    r = math.sqrt(x*x + y*y)
    s = x/r
    c = y/r
    Vx = V * s
    Vy = V * c
    
    return Vx, Vy, r


def set_bot_mov(client_id, w1, w2, w3, w4):
    wheel = init_setup(client_id)
    rcrfl1 = sim.simxSetJointTargetVelocity(client_id, wheel[0], w2, sim.simx_opmode_streaming)
    rcrrl1 = sim.simxSetJointTargetVelocity(client_id, wheel[1], w1, sim.simx_opmode_streaming)
    rcrrr1 = sim.simxSetJointTargetVelocity(client_id, wheel[2], w4, sim.simx_opmode_streaming)
    rcrfr1 = sim.simxSetJointTargetVelocity(client_id, wheel[3], w3, sim.simx_opmode_streaming)


def set_bot_movement(client_id, forw_back_vel, left_right_vel):
    w1 = (left_right_vel + forw_back_vel)
    w2 = (left_right_vel - forw_back_vel)
    w3 = (left_right_vel + forw_back_vel)
    w4 = (left_right_vel - forw_back_vel)
    
    set_bot_mov(client_id, w1, w2, w3, w4)


def isinside(a, b):
    x = (a-255)*(a-255) + (b-255)*(b-255) - 900
    if(x < 1):
        return True
    else:
        return False


def centre(client_id):
    time.sleep(1)
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
    transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    gray_img = cv2.cvtColor(transformed_image, 0)
    barcode = decode(gray_img)
    for obj in barcode:
        (x, y, w, h) = obj.rect
        a = (x + w//2)
        b = (y + h//2)
        print(a, b)
        fw_vel, rl_vel, dist = new_short(a, b)
        print(fw_vel, rl_vel)
        set_bot_movement(client_id, fw_vel, rl_vel)
    flag = 0
    while True:
        vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
        transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
        gray_img = cv2.cvtColor(transformed_image, 0)
        barcode = decode(gray_img)
        for obj in barcode:
            (x, y, w, h) = obj.rect
            a = (x + w//2)
            b = (y + h//2)

            if(isinside(a, b)):
                print('stop')
                stop(client_id)
                time.sleep(1)
                flag=1
                break
        if flag == 1:
            break


if __name__ == "__main__":

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
                    print('\n[ERROR] Failed starting the simulation in CoppeliaSim!')
                    print('start_simulation function is not configured correctly, check the code!')
                    print()
                    sys.exit()

            except Exception:
                print('\n[ERROR] Your start_simulation function throwed an Exception, kindly debug your code!')
                print('Stop the CoppeliaSim simulation manually.\n')
                traceback.print_exc(file=sys.stdout)
                print()
                sys.exit()

        else:
            print('\n[ERROR] Failed connecting to Remote API server!')
            print('[WARNING] Make sure the CoppeliaSim software is running and')
            print('[WARNING] Make sure the Port number for Remote API Server is set to 19997.')
            print('[ERROR] OR init_remote_api_server function is not configured correctly, check the code!')
            print()
            sys.exit()

    except Exception:
        print('\n[ERROR] Your init_remote_api_server function throwed an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually if started.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()

    try:
        centre(client_id)

        try:
            return_code = stop_simulation(client_id)
            if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
                print('\nSimulation stopped correctly.')

                # Stop the Remote API connection with CoppeliaSim server
                try:
                    exit_remote_api_server(client_id)
                    if (start_simulation(client_id) == sim.simx_return_initialize_error_flag):
                        print('\nDisconnected successfully from Remote API Server in CoppeliaSim!')

                    else:
                        print('\n[ERROR] Failed disconnecting from Remote API server!')
                        print('[ERROR] exit_remote_api_server function is not configured correctly, check the code!')

                except Exception:
                    print('\n[ERROR] Your exit_remote_api_server function throwed an Exception, kindly debug your code!')
                    print('Stop the CoppeliaSim simulation manually.\n')
                    traceback.print_exc(file=sys.stdout)
                    print()
                    sys.exit()

            else:
                print('\n[ERROR] Failed stopping the simulation in CoppeliaSim server!')
                print('[ERROR] stop_simulation function is not configured correctly, check the code!')
                print('Stop the CoppeliaSim simulation manually.')

            print()
            sys.exit()

        except Exception:
            print('\n[ERROR] Your stop_simulation function throwed an Exception, kindly debug your code!')
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
