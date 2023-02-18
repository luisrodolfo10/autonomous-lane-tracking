import sim
import sys
import time
import cv2
import numpy as np
import math
import time

#funciones:
def convert_to_HSV(frame):
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  #cv2.imshow("HSV",hsv)
  return hsv

def detect_edges(hsv):
    #hsv = convert_to_HSV(frame)
    lower_black = np.array([0, 0, 0], dtype = "uint8") # lower limit of black color
    upper_black = np.array([40, 40, 40], dtype="uint8") # upper limit of black color
    mask = cv2.inRange(hsv,lower_black,upper_black) # this mask will filter out everything but black
    # detect edges
    edges = cv2.Canny(mask, 50, 100) 
    #cv2.imshow("edges",edges)
    return edges

def region_of_interest(edges):
    height, width = edges.shape 
    mask = np.zeros_like(edges) 

    # coordinates of 4 points (lower left, upper left, upper right, lower right)
    polygon = np.array([[
        (0, height), 
        (0,  height-200),
        (width , height-200),
        (width , height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask) 
    #cv2.imshow("roi",cropped_edges)
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1  
    theta = np.pi / 180  
    min_threshold = 10 
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=5, maxLineGap=0)

    #print(line_segments)
    return line_segments

def average_slope_intercept(frame, line_segments):
    lane_lines = []


    if line_segments is None:
        print("no line segment detected")
        return lane_lines

    height, width,_ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1/3

    left_region_boundary = width * (1 - boundary) 
    right_region_boundary = width * boundary 

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print("Vertical Lines!! (slope = infinity)")
                continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)

            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  
    y2 = int(y1 / 2)  

    if slope == 0: 
        slope = 0.1    

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6):
    line_image = np.zeros_like(frame)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)  
    return line_image

def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape

    if len(lane_lines) == 2: # if two lane lines are detected
        _, _, left_x2, _ = lane_lines[0][0] 
        _, _, right_x2, _ = lane_lines[1][0] 
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)  

    elif len(lane_lines) == 1: # if only one line is detected
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)

    elif len(lane_lines) == 0: # if no line is detected
        x_offset = 0
        y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
    steering_angle = angle_to_mid_deg + 90 

    return steering_angle

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5 ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)

    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


##### Robot functions
def stering_angle(steeringLeft, steeringRight, steeringAngle):

    # 2w = 0.3688
    d= 0.1844 # 2*d=distance between left and right wheels   #= w   
    l= 0.2595 # l=distance between front and read wheels
    if steeringAngle != 0:
        steeringAngleLeft=math.atan(l/(-d+l/math.tan(steeringAngle)))
        steeringAngleRight=math.atan(l/(d+l/math.tan(steeringAngle)))
    else: 
        steeringAngleLeft = 0
        steeringAngleRight = 0
    sim.simxSetJointTargetPosition(clientID, steeringLeft,steeringAngleLeft, sim.simx_opmode_streaming)
    sim.simxSetJointTargetPosition(clientID, steeringRight,steeringAngleRight, sim.simx_opmode_streaming)

def motors_speed(motorLeft, motorRight, wheelRotSpeed):
    sim.simxSetJointTargetVelocity(clientID, motorLeft, -wheelRotSpeed, sim.simx_opmode_streaming)   #Negative speed due the motors
    sim.simxSetJointTargetVelocity(clientID, motorRight, -wheelRotSpeed, sim.simx_opmode_streaming)





sim.simxFinish(-1)
 

clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)


if clientID!=-1:

    print ('Established connection')
else:

    sys.exit("Error: Couldn't establish connection ")

#### Handle objects
#Steering wheels
error_s_left, steeringLeft = sim.simxGetObjectHandle(clientID, 'steering_joint_fl', sim.simx_opmode_oneshot_wait)
error_s_right, steeringRight = sim.simxGetObjectHandle(clientID, 'steering_joint_fr', sim.simx_opmode_oneshot_wait) 
#Motors 
error_left, motorLeft = sim.simxGetObjectHandle(clientID, 'driving_joint_rear_left', sim.simx_opmode_oneshot_wait)
error_right, motorRight = sim.simxGetObjectHandle(clientID, 'driving_joint_rear_right', sim.simx_opmode_oneshot_wait)

if error_left or error_right:
    sys.exit("Error: Couldn't connect with the motors")

#Modo sincrono
sim.simxSynchronous(clientID, False)
     

error_cam, camara = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_oneshot_wait)
 
if error_cam:

    sys.exit("Error: couldn't connect to the camera")

_, resolution, image = sim.simxGetVisionSensorImage(clientID, camara, 0, sim.simx_opmode_streaming)
time.sleep(1)


print("Press ESC on the windows to exit...")

### Variables:
SteeringAngle = 0
desiredWheelRotSpeed = 0

wheelRotSpeed = 350*math.pi/180  ## Linear velocity, static   #350, 400, 500

lastTime = 0
lastError = 0

#Direction
Kp = 0.6      
Kd = 0.12       
Ki = 0

#Speed
Kp_s = 2
Kd_s = 0.3

proportional = 0
integral = 0
derivative = 0

start_time = time.time()

last_desired_steering_angle = 90

#wheelRotSpeed =   ## Linear velocity, static
while(1):
    #Capturamos un frame de la cámara del robot. Guardamos la imagen y su resolución:
  
    #_, resolution, image = sim.simxGetVisionSensorImage(clientID, camara, 0, sim.simx_opmode_buffer)

    _, resolution, image = sim.simxGetVisionSensorImage(clientID, camara, 0, sim.simx_opmode_buffer)

    #Modificaremos esta imagen para que OpenCV pueda tratarla:
    img = np.array(image, dtype = np.uint8) #La convertimos a un array de numpy
    img.resize([resolution[1], resolution[0], 3]) #Cambiamos sus dimensiones   Se cambio el codigo de img.resize([resolution[0], resolution[1], 3])
    img = np.rot90(img,2) #La rotamos 90 grados para enderezarla
    img = np.fliplr(img) #La invertimos
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) #Cambiamos su espacio de color de RGB a BGR

    ##Functions
    hsv = convert_to_HSV(img)
    edges = detect_edges(hsv)
    roi = region_of_interest(edges)
    line_segments = detect_line_segments(roi)
    lane_lines = average_slope_intercept(img,line_segments)
    lane_lines_image = display_lines(img,lane_lines)
    desired_steering_angle = get_steering_angle(img, lane_lines)
    heading_image = display_heading_line(lane_lines_image,desired_steering_angle)

    


    #if flag_infinity:
    #    desired_steering_angle = last_desired_steering_angle

    ### 
    ### PID 
    now = time.time()
    dt = now - lastTime
    error = desired_steering_angle - 90
    if error < -1 or error > 1:   #Margin of error
        #PID direction
        derivative = (error - lastError)/dt
        proportional = error
        integral += error*dt
        PID = int(Kp*proportional + Kd* derivative + Ki*integral)
        steering_angle = PID
        #PD speed
        wheelRotSpeed = (400-int(Kp_s*proportional + Kd_s*derivative))*math.pi/180
    else:
        steering_angle = 0

    if steering_angle < -25:
        steering_angle = -25

    elif steering_angle > 25:
        steering_angle = 25

    # if steering_angle > 15:
    #     wheelRotSpeed = 200*math.pi/180
    # else:
    #     wheelRotSpeed = 500*math.pi/180


    ### Robot
    stering_angle(steeringLeft, steeringRight, -steering_angle*math.pi/180)
    motors_speed(motorLeft, motorRight, wheelRotSpeed)
    
    print("-----------------------------------")
    print("time: ", time.time() - start_time)
    print("Proportional: ", proportional)
    print("Derivative: ", derivative)
    print("Integral: ", integral)
    print("Desired Steering Angle: ", desired_steering_angle)
    print("Steering Angle: ", steering_angle)
    print("Wheel Rotation Speed: ",  wheelRotSpeed)

    lastError = error
    lastTime = time.time()

    last_desired_steering_angle = desired_steering_angle


    #Mostramos la imagen:
    #cv2.imshow('lanes', lane_lines_image)
    cv2.imshow('lanes_final', heading_image)
    #cv2.imshow('Image', img)|
    
    #Se sale con ESC.
    tecla = cv2.waitKey(5) & 0xFF
    if tecla == 27:
        break
         
time.sleep(1)
 
#Cerramos la conexión:
sim.simxFinish(-1)
