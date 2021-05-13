import sim as vrep
import time
import cv2
import numpy as np
import imutils

# set all used variables
red_lower = np.array([-10, 50, 50])
red_upper = np.array([10, 255, 255])
fire_detected = False
speed_2 = -0.6
speed_3 = 0.6
slow_down = False
move_forward = False

stair = False
obstacle = False

speed = -4
speed1 = -5
speed3 = -50
speed2 = -1

speed_33 = -3

speed_stairs_1 = -10
speed_stairs_2 = -50

cx = -2000
area = -2000

object_detected = False

rightClear = False
leftClear = False

detection_1 = False
detection_2 = False
detection_3 = False

Stop = False
test = False

turn_left = False
turn_right = False

turn_back_left = False
turn_back_right = False
turn_time = 0

distance_u = -2000

stop_moving = False
move_forward_2 = False


def form_link():
    # start link with CoppelaiSim
    vrep.simxFinish(-1)
    global clientID
    clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

    if clientID != -1:
        print('Connected to remote API server')
        print('Vision Sensor object handling')
    else:
        print("Failed to connect to remote API Server")
        vrep.simxFinish(clientID)


def initialize_sensors():
    global camera_sensor, distance_sensor_1, distance_sensor_2, distance_sensor_3, u1, u2, u3

    # vision sensor (camera)
    res, camera_sensor = vrep.simxGetObjectHandle(clientID, 'vs1', vrep.simx_opmode_oneshot_wait)

    # distance sensors
    err, distance_sensor_1 = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor', vrep.simx_opmode_oneshot_wait)
    err, distance_sensor_2 = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor0', vrep.simx_opmode_oneshot_wait)
    err, distance_sensor_3 = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor4', vrep.simx_opmode_oneshot_wait)

    # ultrasonic sensors
    err, u1 = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor1', vrep.simx_opmode_oneshot_wait)
    err, u2 = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor2', vrep.simx_opmode_oneshot_wait)
    err, u3 = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor3', vrep.simx_opmode_oneshot_wait)


def initialize_motors():
    # initialize all motors
    global L_wheel1, L_wheel2, L_wheel3, L_wheel4, L_wheel5, L_wheel6, L_wheel7, L_wheel8, L_wheel9, L_wheel10, L_wheel11, L_wheel12, L_wheel13
    global R_wheel1, R_wheel2, R_wheel3, R_wheel4, R_wheel5, R_wheel6, R_wheel7, R_wheel8, R_wheel9, R_wheel10, R_wheel11, R_wheel12, R_wheel13

    errorCode, L_wheel1 = vrep.simxGetObjectHandle(clientID, 'L_motor1', vrep.simx_opmode_blocking)
    errorCode, L_wheel2 = vrep.simxGetObjectHandle(clientID, 'L_motor2', vrep.simx_opmode_blocking)
    errorCode, L_wheel3 = vrep.simxGetObjectHandle(clientID, 'L_motor3', vrep.simx_opmode_blocking)
    errorCode, L_wheel4 = vrep.simxGetObjectHandle(clientID, 'L_motor4', vrep.simx_opmode_blocking)
    errorCode, L_wheel5 = vrep.simxGetObjectHandle(clientID, 'L_motor5', vrep.simx_opmode_blocking)
    errorCode, L_wheel6 = vrep.simxGetObjectHandle(clientID, 'L_motor6', vrep.simx_opmode_blocking)
    errorCode, L_wheel7 = vrep.simxGetObjectHandle(clientID, 'L_motor7', vrep.simx_opmode_blocking)
    errorCode, L_wheel8 = vrep.simxGetObjectHandle(clientID, 'L_motor8', vrep.simx_opmode_blocking)
    errorCode, L_wheel9 = vrep.simxGetObjectHandle(clientID, 'L_motor9', vrep.simx_opmode_blocking)
    errorCode, L_wheel10 = vrep.simxGetObjectHandle(clientID, 'L_motor10', vrep.simx_opmode_blocking)
    errorCode, L_wheel11 = vrep.simxGetObjectHandle(clientID, 'L_motor11', vrep.simx_opmode_blocking)
    errorCode, L_wheel12 = vrep.simxGetObjectHandle(clientID, 'L_motor12', vrep.simx_opmode_blocking)
    errorCode, L_wheel13 = vrep.simxGetObjectHandle(clientID, 'L_motor13', vrep.simx_opmode_blocking)

    errorCode, R_wheel1 = vrep.simxGetObjectHandle(clientID, 'R_motor1', vrep.simx_opmode_blocking)
    errorCode, R_wheel2 = vrep.simxGetObjectHandle(clientID, 'R_motor2', vrep.simx_opmode_blocking)
    errorCode, R_wheel3 = vrep.simxGetObjectHandle(clientID, 'R_motor3', vrep.simx_opmode_blocking)
    errorCode, R_wheel4 = vrep.simxGetObjectHandle(clientID, 'R_motor4', vrep.simx_opmode_blocking)
    errorCode, R_wheel5 = vrep.simxGetObjectHandle(clientID, 'R_motor5', vrep.simx_opmode_blocking)
    errorCode, R_wheel6 = vrep.simxGetObjectHandle(clientID, 'R_motor6', vrep.simx_opmode_blocking)
    errorCode, R_wheel7 = vrep.simxGetObjectHandle(clientID, 'R_motor7', vrep.simx_opmode_blocking)
    errorCode, R_wheel8 = vrep.simxGetObjectHandle(clientID, 'R_motor8', vrep.simx_opmode_blocking)
    errorCode, R_wheel9 = vrep.simxGetObjectHandle(clientID, 'R_motor9', vrep.simx_opmode_blocking)
    errorCode, R_wheel10 = vrep.simxGetObjectHandle(clientID, 'R_motor10', vrep.simx_opmode_blocking)
    errorCode, R_wheel11 = vrep.simxGetObjectHandle(clientID, 'R_motor11', vrep.simx_opmode_blocking)
    errorCode, R_wheel12 = vrep.simxGetObjectHandle(clientID, 'R_motor12', vrep.simx_opmode_blocking)
    errorCode, R_wheel13 = vrep.simxGetObjectHandle(clientID, 'R_motor13', vrep.simx_opmode_blocking)


def set_motor_left(speed_l):
    # set left motors speed
    vrep.simxSetJointTargetVelocity(clientID, L_wheel1, speed_l, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, L_wheel2, speed_l, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, L_wheel3, speed_l, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, L_wheel4, speed_l, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, L_wheel5, speed_l, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, L_wheel6, speed_l, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, L_wheel7, speed_l, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, L_wheel8, speed_l, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, L_wheel9, speed_l, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, L_wheel10, speed_l, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, L_wheel11, speed_l, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, L_wheel12, speed_l, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, L_wheel13, speed_l, vrep.simx_opmode_streaming)


def set_motor_right(speed_r):
    # set right motor speed
    vrep.simxSetJointTargetVelocity(clientID, R_wheel1, speed_r, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, R_wheel2, speed_r, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, R_wheel3, speed_r, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, R_wheel4, speed_r, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, R_wheel5, speed_r, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, R_wheel6, speed_r, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, R_wheel7, speed_r, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, R_wheel8, speed_r, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, R_wheel9, speed_r, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, R_wheel10, speed_r, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, R_wheel11, speed_r, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, R_wheel12, speed_r, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, R_wheel13, speed_r, vrep.simx_opmode_streaming)


def detect_fire():
    # get image, resolution
    global err, fire_detected
    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, camera_sensor, 0, vrep.simx_opmode_streaming)
    if err == vrep.simx_return_ok:

        # transform into array that can be used with openCv
        img = np.array(image, dtype=np.uint8)

        # resize image to exact resolution used in sensor
        img.resize([resolution[0], resolution[1], 3])
        # rotate the image since when displayed its flipped
        img = imutils.rotate_bound(img, 180)

        # opencv works with BGR colors, the image from the vision sensor is RGB, therefore it is converted first
        img_2 = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        # that image is then converted to HSV in order to create a mask for the red color
        hsvFrame = cv2.cvtColor(img_2, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
        mask_image = cv2.bitwise_and(img_2, img_2, mask=red_mask)
        mask_image = imutils.rotate_bound(mask_image, 180)

        # create contour around the red color
        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            global fire_detected
            global area
            #get area of contour
            area = cv2.contourArea(contour)

            M = cv2.moments(contour)

            # find center of contour
            if M['m00'] > 0 and M['m00']:
                global cx, cy
                cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])

            # draw contour
            if area > 100:
                x, y, w, h = cv2.boundingRect(contour)
                imageFrame = cv2.rectangle(img_2, (x, y),
                                           (x + w, y + h),
                                           (0, 0, 255), 2)
                fire_detected = True

        # show final image
        cv2.imshow("Multiple Color Detection in Real-TIme", img_2)


def move_to_fire():
    global test
    global move_forward
    global fire_detected

    detect_fire()

    # move straight if center of countour is on the center of image
    if cx == 128 or cx == 127 or cx == 129 or cx == 126 or cx == 130:
        test = True
        move_forward = True
        print(move_forward)
    # turn right if center is to the left of image
    elif 0 < cx < 126 and test == False:
        print('turn right')
        set_motor_left(speed_2)
    # turn left if center is to left of the image
    elif cx > 130 and test == False:
        print('turn left')
        set_motor_right(speed_2)

    # using area of contour to move robot forward and stop it, if robot overshoots it will go back or if fire
    # increases it reverses
    if (move_forward == True) and 0 < area < 6700:
        set_motor_left(speed)
        set_motor_right(speed)
    elif (move_forward == True) and (6700 < area < 35000):
        set_motor_left(0)
        set_motor_right(0)
        fire_detected = False
        print('stop')

    elif (move_forward == True) and area > 35000:
        set_motor_left(speed_3)
        set_motor_right(speed_3)
        print('go back')


def detect_stairs():
    # get values from sensor
    err, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
        clientID, distance_sensor_1, vrep.simx_opmode_streaming)
    err, detectionState1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = vrep.simxReadProximitySensor(
        clientID, distance_sensor_2, vrep.simx_opmode_streaming)

    err, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
        clientID, distance_sensor_1, vrep.simx_opmode_buffer)
    err, detectionState1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = vrep.simxReadProximitySensor(
        clientID, distance_sensor_2, vrep.simx_opmode_buffer)

    # measure the difference in ditance between the two detected points
    distance = detectedPoint1[2] - detectedPoint[2]

    global stair, obstacle

    if err == vrep.simx_return_ok:

        # if distance is between certain values then the obstacle will be considered as a stairs
        if 0.1 < distance < 0.5:
            print('a stair')
            stair = True
        elif 0 < distance < 0.1:
            print('a obstacle')
            obstacle = True
        else:
            stair = False


def move_up_stairs():
    global stair
    err, detectionState3, detectedPoint3, detectedObjectHandle3, detectedSurfaceNormalVector3 = vrep.simxReadProximitySensor(
        clientID, distance_sensor_3, vrep.simx_opmode_streaming)

    err, detectionState3, detectedPoint3, detectedObjectHandle3, detectedSurfaceNormalVector3 = vrep.simxReadProximitySensor(
        clientID, distance_sensor_3, vrep.simx_opmode_buffer)

    if stair == True:
        vrep.simxSetJointTargetVelocity(clientID, L_wheel1, speed_stairs_1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, L_wheel2, speed_stairs_1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, L_wheel3, speed_stairs_1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, L_wheel4, speed_stairs_1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, L_wheel5, speed_stairs_2, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, L_wheel6, speed_stairs_2, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, L_wheel7, speed_stairs_2, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, L_wheel8, speed_stairs_2, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, L_wheel9, speed_stairs_2, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, L_wheel10, speed_stairs_1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, L_wheel11, speed_stairs_1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, L_wheel12, speed_stairs_1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, L_wheel13, speed_stairs_1, vrep.simx_opmode_streaming)

        vrep.simxSetJointTargetVelocity(clientID, R_wheel1, speed_stairs_1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, R_wheel2, speed_stairs_1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, R_wheel3, speed_stairs_1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, R_wheel4, speed_stairs_1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, R_wheel5, speed_stairs_2, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, R_wheel6, speed_stairs_2, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, R_wheel7, speed_stairs_2, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, R_wheel8, speed_stairs_2, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, R_wheel9, speed_stairs_2, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, R_wheel10, speed_stairs_1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, R_wheel11, speed_stairs_1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, R_wheel12, speed_stairs_1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, R_wheel13, speed_stairs_1, vrep.simx_opmode_streaming)
        time.sleep(1)

    elif obstacle == True:
        set_motor_right(0)
        set_motor_left(0)

    print(detectionState3)

    if stair == True and detectionState3 == True:
        print('stop')
        set_motor_right(0)
        set_motor_left(0)
        stair = False


def detect_obstacle():
    global Stop
    err, detectionState1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = vrep.simxReadProximitySensor(
        clientID, u1, vrep.simx_opmode_streaming)

    err, detectionState1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = vrep.simxReadProximitySensor(
        clientID, u1, vrep.simx_opmode_buffer)

    if detectionState1 == True:
        print('obstacle ahead')
        square = np.square(detectedPoint1)
        total = np.sum(square)
        distance_u = np.sqrt(total)
        print(object_detected)
        if 0 < distance_u < 1.2:
            set_motor_left(0)
            set_motor_right(0)
            print('stop')
            Stop = True
            set_motor_left(0)
            set_motor_right(0)


def cut_range():
    global object_detected, turn_back_left, turn_back_right, distance_u, turn_time, fire_detected, test
    global rightClear, leftClear, move_forward, detection_1, detection_2, detection_3
    global Stop

    err, detectionState1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = vrep.simxReadProximitySensor(
        clientID, u1, vrep.simx_opmode_streaming)
    err, detectionState2, detectedPoint2, detectedObjectHandle2, detectedSurfaceNormalVector2 = vrep.simxReadProximitySensor(
        clientID, u2, vrep.simx_opmode_streaming)
    err, detectionState3, detectedPoint3, detectedObjectHandle3, detectedSurfaceNormalVector3 = vrep.simxReadProximitySensor(
        clientID, u3, vrep.simx_opmode_streaming)

    time.sleep(0.2)

    # sensor in the middle
    err, detectionState1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = vrep.simxReadProximitySensor(
        clientID, u1, vrep.simx_opmode_buffer)
    # sensor on the right
    err, detectionState2, detectedPoint2, detectedObjectHandle2, detectedSurfaceNormalVector2 = vrep.simxReadProximitySensor(
        clientID, u2, vrep.simx_opmode_buffer)
    # sensor on the left
    err, detectionState3, detectedPoint3, detectedObjectHandle3, detectedSurfaceNormalVector3 = vrep.simxReadProximitySensor(
        clientID, u3, vrep.simx_opmode_buffer)

    square = np.square(detectedPoint1)
    total = np.sum(square)
    distance_u = np.sqrt(total)
    # check if obstacle detected infront of robot
    if detectionState1 == True and distance_u < 1:
        detection_1 = True
    else:
        detection_1 = False

    # check if right is clear
    square_1 = np.square(detectedPoint2)
    total_1 = np.sum(square_1)
    distance_u_1 = np.sqrt(total_1)
    if detectionState2 == True and distance_u_1 < 1:
        detection_2 = True
    else:
        detection_2 = False

    # check if left is clear
    square_2 = np.square(detectedPoint3)
    total_2 = np.sum(square_2)
    distance_u_2 = np.sqrt(total_2)
    if detectionState3 == True and distance_u_2 < 1:
        detection_3 = True
    else:
        detection_3 = False


# detect objects using ultrasonic sensor
def object_detection():
    global object_detected, turn_back_left, turn_back_right, distance_u, turn_time, fire_detected, test
    global rightClear, leftClear, move_forward,distance_u
    global Stop

    err, detectionState1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = vrep.simxReadProximitySensor(
        clientID, u1, vrep.simx_opmode_streaming)

    err, detectionState1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = vrep.simxReadProximitySensor(
        clientID, u1, vrep.simx_opmode_buffer)

    cut_range()

    #check front
    if detectionState1 == True:
        print('obstacle ahead')
        square = np.square(detectedPoint1)
        total = np.sum(square)
        distance_u = np.sqrt(total)
        object_detected = True
        print(object_detected)

    #check right
    if detection_2 == False:
        rightClear = True
        print('right clear')
    #check left
    if detection_3 == False:
        leftClear = True
        print('left clear')

    # measure distance to obstacle ahead
    if 0 < distance_u < 1 and Stop == False:
        set_motor_left(0)
        set_motor_right(0)
        print('stop')
        Stop = True

    # if right clear  turn right, even if left is also clear go right
    if rightClear == True and Stop == True:
        start = time.time()
        cut_range()
        while (detection_1 == True or detection_3 == True):
            cut_range()
            set_motor_left(speed2)
            set_motor_right(0)
            print('turn right')

        turn_time = time.time() - start
        object_detected = False
        Stop = False
        turn_back_left = True
    # if left clear turn left
    elif leftClear == True and Stop == True:
        start = time.time()
        cut_range()
        while (detection_1 == True or detection_2 == True):
            cut_range()
            set_motor_right(speed2)
            set_motor_left(0)
            print('turn left')
        # time taken to turn
        turn_time = time.time() - start
        print(turn_time)

        object_detected = False
        Stop = False
        turn_back_right = True

    # move forward for period of time
    if object_detected == False:
        start_3 = time.time()
        while time.time() - start_3 < turn_time:
            set_motor_right(speed_33)
            set_motor_left(speed_33)

    # turn back robot to its original path
    if turn_back_left == True:
        start_1 = time.time()
        while time.time() - start_1 < (turn_time / 2):
            set_motor_left(speed2)
            print("turn right 2")
        turn_back_left = False
        object_detected = False
        Stop = False
        test = False
        rightClear = False
        leftClear = False
        move_forward = False
        set_motor_left(0)
        set_motor_right(0)
    elif turn_back_right:
        start_2 = time.time()
        while time.time() - start_2 < (turn_time / 2.5):
            set_motor_right(speed2)
            print("turn left 2")

        turn_back_right = False
        object_detected = False
        Stop = False
        test = False
        rightClear = False
        leftClear = False
        move_forward = False
        set_motor_left(0)
        set_motor_right(0)

    distance_u = 2000



def main():
    form_link()
    initialize_sensors()
    initialize_motors()
    set_motor_left(0)
    set_motor_right(0)
    while vrep.simxGetConnectionId(clientID) != -1:
        time.sleep(0.2)
        detect_fire()
        while fire_detected == True and Stop == False:
            detect_stairs()
            move_to_fire()
            detect_obstacle()
            while Stop == True and stair == False:
                object_detection()
                print('moving')

            while Stop == False and stair == True:
                move_up_stairs()


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        elif err == vrep.simx_return_novalue_flag:
            print("no image yet")
            pass
        else:
            print(err)


if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()
