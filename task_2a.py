'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 2A of Berryminator(BM) Theme (eYRC 2021-22).
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
# Filename:			task_2a.py
# Functions:
# Global variables:
# 					[ List of global variables defined in this file ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv, os)                ##
##############################################################
import cv2
import numpy as np
import os
import sys
import traceback
##############################################################

try:
    import sim

except Exception:
    print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
    print('\n[WARNING] Make sure to have following files in the directory:')
    print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
    sys.exit()

try:
    import task_1b

except ImportError:
    print('\n[ERROR] task_1b.py file is not present in the current directory.')
    print('Your current directory is: ', os.getcwd())
    print('Make sure task_1b.py is present in this current directory.\n')
    sys.exit()


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################


##############################################################

def get_vision_sensor_image(client_id, vision_sensor_handle):
    """
    Purpose:
    ---
    This function takes the client id and handle of the vision sensor scene object as input
    arguments and returns the vision sensor's image array from the CoppeliaSim scene.

    Input Arguments:
    ---
    `client_id`    :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()
    `vision_sensor_handle`    :   [ integer ]
            the handle of the vision sensor scene object

    Returns:
    ---
    `vision_sensor_image` 	:  [ list ]
            the image array returned from the get vision sensor image remote API
    `image_resolution` 		:  [ list ]
            the image resolution returned from the get vision sensor image remote API
    `return_code` 			:  [ integer ]
            the return code generated from the remote API

    Example call:
    ---
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image()
    """

    vision_sensor_image = []
    image_resolution = []
    return_code = 0

    ##############	ADD YOUR CODE HERE	##############

    return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(
        client_id, vision_sensor_handle, 0, sim.simx_opmode_oneshot_wait)

    ##################################################

    return vision_sensor_image, image_resolution, return_code


def get_vision_sensor_depth_image(client_id, vision_sensor_handle):
    """
    Purpose:
    ---
    This function takes the client id and handle of the vision sensor scene object as input
    arguments and returns the vision sensor's depth buffer array from the CoppeliaSim scene.
    Input Arguments:
    ---
    `client_id`               :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()
    `vision_sensor_handle`    :   [ integer ]
            the handle of the vision sensor scene object

    Returns:
    ---
    `vision_sensor_depth_image` 	:  [ list ]
            the depth buffer array returned from the get vision sensor image remote API
    `image_resolution` 		:  [ list ]
            the image resolution returned from the get vision sensor image remote API
    `return_code` 			:  [ integer ]
            the return code generated from the remote API

    Example call:
    ---
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image()
    """

    vision_sensor_depth_image = []
    image_resolution = []
    return_code = 0

    ##############	ADD YOUR CODE HERE	##############

    return_code, image_resolution, vision_sensor_depth_image = sim.simxGetVisionSensorDepthBuffer(
        client_id, vision_sensor_handle, sim.simx_opmode_oneshot_wait)

    ##################################################

    return vision_sensor_depth_image, image_resolution, return_code


def transform_vision_sensor_depth_image(vision_sensor_depth_image, image_resolution):
    """
    Purpose:
    ---
    This function converts the depth buffer array received from vision sensor and converts it into
    a numpy array that can be processed by OpenCV
    This function should:
    1. First convert the vision_sensor_depth_image list to a NumPy array with data-type as float32.
    2. Since the depth image returned from Vision Sensor is in the form of a 1-D (one dimensional) array,
    the new NumPy array should then be resized to a 2-D (two dimensional) NumPy array.
    3. Flip the resultant image array about the appropriate axis. The resultant image NumPy array should be returned.

    Input Arguments:
    ---
    `vision_sensor_depth_image` 	:  [ list ]
            the image array returned from the get vision sensor image remote API
    `image_resolution` 		:  [ list ]
            the image resolution returned from the get_vision_sensor_depth_image() function

    Returns:
    ---
    `transformed_depth_image` 	:  [ numpy array ]
            the resultant transformed image array after performing above 3 steps

    Example call:
    ---
    transformed_image = transform_vision_sensor_image(
        vision_sensor_image, image_resolution)

    """

    transformed_depth_image = None

    ##############	ADD YOUR CODE HERE	##############

    image = np.array(
        vision_sensor_depth_image, dtype=np.float32)
    image.resize([image_resolution[0], image_resolution[1]])
    transformed_depth_image = cv2.flip(image, 1)
    # cv2.imshow('1', transformed_depth_image)

    ##################################################

    return transformed_depth_image


def detect_berries(transformed_image, transformed_depth_image):
    """
    Purpose:
    ---
    This function takes the transformed image and transformed depth image as input arguments and returns
    the pixel coordinates and depth values in form of a dictionary.

    Input Arguments:
    ---
    `transformed_image` 	:  [ numpy array ]
            the transformed image array
    `transformed_depth_image` 	:  [ numpy array ]
            the transformed depth image array

    Returns:
    ---
    `berries_dictionary` 	:  [ dictionary ]
            the resultant dictionary with details of all the berries

    Example call:
    ---
    berries_dictionary = detect_berries(
        transformed_image, transformed_depth_image)

    """
    berries_dictionary = {}
    berries = ["Strawberry", "Blueberry", "Lemon"]

    ##############	ADD YOUR CODE HERE	##############

    berries_dictionary = {
        berries[0]: [],
        berries[1]: [],
        berries[2]: []
    }
    gray = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2GRAY)
    return_code, thresh = cv2.threshold(gray, 20, 175, cv2.THRESH_BINARY)
    contours, hier = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    blank = transformed_image
    for contour in contours:
        appox = cv2.approxPolyDP(
            contour, 0.03*cv2.arcLength(contour, True), True)
        if(len(appox > 10)):
            x, y, w, h = cv2.boundingRect(appox)
            if(blank[y+h//2][x+w//2][2] >= 0 and blank[y+h//2][x+w//2][2] <= 25 and blank[y+h//2][x+w//2][1] <= 95 and blank[y+h//2][x+w//2][1] >= 50 and blank[y+h//2][x+w//2][0] >= 210 and blank[y+h//2][x+w//2][0] <= 255):
                a = transformed_depth_image[y+h//2][x+w//2]
                berries_dictionary[berries[1]].append((x+w//2, y+h//2, a))
            elif(blank[y+h//2][x+w//2][2] >= 210 and blank[y+h//2][x+w//2][2] <= 255 and blank[y+h//2][x+w//2][1] <= 255 and blank[y+h//2][x+w//2][1] >= 210 and blank[y+h//2][x+w//2][0] >= 0 and blank[y+h//2][x+w//2][0] <= 25):
                a = transformed_depth_image[y+h//2][x+w//2]
                berries_dictionary[berries[2]].append((x+w//2, y+h//2, a))
            elif(blank[y+h//2][x+w//2][2] >= 190 and blank[y+h//2][x+w//2][2] <= 255 and blank[y+h//2][x+w//2][1] <= 45 and blank[y+h//2][x+w//2][1] >= 0 and blank[y+h//2][x+w//2][0] >= 20 and blank[y+h//2][x+w//2][0] <= 120):
            # else:
                a = transformed_depth_image[y+h//2][x+w//2]
                berries_dictionary[berries[0]].append((x+w//2, y+h//2, a))

    ##################################################
    return berries_dictionary


def detect_berry_positions(berries_dictionary):
    """
    Purpose:
    ---
    This function takes the berries_dictionary as input arguments and calculates the 3D positions of the
    berries with respect to vision sensor. The final output is returned in another dictionary.

    Input Arguments:
    ---
    `berries_dictionary` 	:  [ dictionary ]
            the dictionary returned by detect_berries() function

    Returns:
    ---
    `berry_positions_dictionary` 	:  [ dictionary ]
            the resultant dictionary with details of 3D positions of all the berries

    Example call:
    ---
    berry_positions_dictionary = detect_berry_positions(berries_dictionary)

    """
    berry_positions_dictionary = {}
    berries = ["Strawberry", "Blueberry", "Lemon"]

    ##############	ADD YOUR CODE HERE	##############

    berry_positions_dictionary = {
        berries[0]: [],
        berries[2]: [],
        berries[1]: []
    }
    for berry in berries:
        for co in berries_dictionary[berry]:
            x, y, z = co
            d = 2*z
            cx, cy = x - 256, y - 256
            cx, cy = cx/512, cy/512
            berry_positions_dictionary[berry].append(
                (round(cx, 2)*1.65, round(cy, 2)*1.65, round(d, 2)+0.025))

    ##################################################

    return berry_positions_dictionary


def get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary):
    ######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########
    """
    Purpose:
    ---
    This function takes the transformed_image and the dictionaries returned by detect_berries()
    and  detect_berry_positions() functions. This function is already completed for your reference
    and will be helpful for debugging purposes.

    Input Arguments:
    ---
    `transformed_image` :	[ numpy array ]
                    numpy array of image returned by cv2 library

    `berries_dictionary` 	:  [ dictionary ]
            the resultant dictionary with details of all the berries

    `berry_positions_dictionary` 	:  [ dictionary ]
            the resultant dictionary with details of 3D positions of all the berries

    Returns:
    ---
    `labelled_image` :	[ numpy array ]
                    labelled image

    Example call:
    ---
    transformed_image = get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary)
    """
    labelled_image = np.array(transformed_image)
    ######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########

    for berry_type in berries_dictionary.keys():
        berry_details_list = berries_dictionary[berry_type]
        berry_positions_list = berry_positions_dictionary[berry_type]
        for index in range(len(berry_details_list)):
            pixel_x, pixel_y, depth_val = berry_details_list[index]
            coordinates = (pixel_x, pixel_y)
            horizontal_displacement, vertical_displacement, distance_from_sensor = berry_positions_list[
                index]
            horizontal_displacement, vertical_displacement, distance_from_sensor = round(
                horizontal_displacement, 2), round(vertical_displacement, 2), round(distance_from_sensor, 2)
            cv2.putText(labelled_image, str((horizontal_displacement, vertical_displacement,
                        distance_from_sensor)), coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    return labelled_image


if __name__ == "__main__":

    berries_dictionary = {}
    berry_positions_dictionary = {}

    print('\nConnection to CoppeliaSim Remote API Server initiated.')
    print('Trying to connect to Remote API Server...')
    cv2.namedWindow('transformed image', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('transformed depth image', cv2.WINDOW_AUTOSIZE)

    try:
        # Initiate Remote API connection
        client_id = task_1b.init_remote_api_server()

        if (client_id != -1):
            print('\nConnected successfully to Remote API Server in CoppeliaSim!')
            return_code, vision_sensor_handle = sim.simxGetObjectHandle(
                client_id, 'vision_sensor', sim.simx_opmode_blocking)

            # Starting the Simulation
            try:
                return_code = task_1b.start_simulation(client_id)

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

    while True:

        # Get image array and depth buffer from vision sensor in CoppeliaSim scene
        try:
            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id, vision_sensor_handle)
            vision_sensor_depth_image, depth_image_resolution, return_code_2 = get_vision_sensor_depth_image(
                client_id, vision_sensor_handle)

            if ((return_code == sim.simx_return_ok) and (return_code_2 == sim.simx_return_ok) and (len(image_resolution) == 2) and (len(depth_image_resolution) == 2) and (len(vision_sensor_image) > 0) and (len(vision_sensor_depth_image) > 0)):
                print('\nImage captured from Vision Sensor in CoppeliaSim successfully!')

                # Get the transformed vision sensor image captured in correct format
                try:
                    transformed_image = task_1b.transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    transformed_depth_image = transform_vision_sensor_depth_image(
                        vision_sensor_depth_image, depth_image_resolution)

                    if (type(transformed_image) is np.ndarray) and (type(transformed_depth_image) is np.ndarray):

                        berries_dictionary = detect_berries(
                            transformed_image, transformed_depth_image)
                        print("Berries Dictionary = ", berries_dictionary)
                        berry_positions_dictionary = detect_berry_positions(
                            berries_dictionary)
                        print("Berry Positions Dictionary = ",
                              berry_positions_dictionary)

                        labelled_image = get_labeled_image(
                            transformed_image, berries_dictionary, berry_positions_dictionary)

                        cv2.imshow('transformed image', transformed_image)
                        cv2.imshow('transformed depth image',
                                   transformed_depth_image)
                        cv2.imshow('labelled image', labelled_image)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break

                    else:
                        print(
                            '\n[ERROR] transform_vision_sensor_image function is not configured correctly, check the code.')
                        print('Stop the CoppeliaSim simulation manually.')
                        print()
                        sys.exit()

                except Exception:
                    print(
                        '\n[ERROR] Your transform_vision_sensor_image function throwed an Exception, kindly debug your code!')
                    print('Stop the CoppeliaSim simulation manually.\n')
                    traceback.print_exc(file=sys.stdout)
                    print()
                    sys.exit()

            else:
                print(
                    '\n[ERROR] get_vision_sensor function is not configured correctly, check the code.')
                print('Stop the CoppeliaSim simulation manually.')
                print()
                sys.exit()

        except Exception:
            print(
                '\n[ERROR] Your get_vision_sensor_image function throwed an Exception, kindly debug your code!')
            print('Stop the CoppeliaSim simulation manually.\n')
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()

    cv2.destroyAllWindows()

    # Ending the Simulation
    try:
        return_code = task_1b.stop_simulation(client_id)

        if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
            print('\nSimulation stopped correctly.')

            # Stop the Remote API connection with CoppeliaSim server
            try:
                task_1b.exit_remote_api_server(client_id)

                if (task_1b.start_simulation(client_id) == sim.simx_return_initialize_error_flag):
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
            print('\n[ERROR] Failed stopping the simulation in CoppeliaSim server!')
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
