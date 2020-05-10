# SECTION 1: SETUP

# Import required libraries
import cv2
import numpy as np
from picamera import PiCamera
import time
import serial
import RPi.GPIO as GPIO
from math import degrees, atan2, cos, sin
from csv import reader

# Setup for the GPIO pins (LED flash ring), Pi Camera module and OpenCV image window
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17,GPIO.OUT)
GPIO.output(17,GPIO.LOW)
camera = PiCamera()
camera.resolution = (2592,1944)
camera.framerate = 15
cv2.namedWindow('image',cv2.WINDOW_NORMAL)
cv2.resizeWindow('image',(1000,800))

# Open serial connection to printer
# NB: com port name must be determined after connecting to printer
# NB: code untested
ser = serial.Serial(com_port_name,115200)
# Wait for Serial connection to initalize (needs tested)
time.sleep(3)


# Missing: connection to end effector servo motor and vacuum pump.






# FUNCTIONS

def take_picture(img_save_loc):
    camera.start_preview()
    GPIO.output(17,GPIO.HIGH)
    time.sleep(4)
    camera.capture(img_save_loc)
    camera.stop_preview()
    GPIO.output(17,GPIO.LOW)
    time.sleep(1)
    
def send_gcode(gcode_string):
    ser.write(b'{gcode_string}\r\n')
    
def pick_component(comx,comy):
    # move to 5cm above component
    send_gcode(f'G1 X{comx} Y{comy} Z50')
    # move to pick height
    send_gcode(f'G1 Z{zpick}')
    # pump on
    set_pump(1)
    # raise head again
    send_gcode(f'G1 Z50')

def rotate_component(com_ang,pcb_ang):
    # calculate angle to turn
    turn_ang = pcb_ang - com_ang
    # Function incomplete: send instruction to servo to turn through angle

def place_component(pcbx,pcby):
    # move to 5cm above placement zone
    send_gcode(f'G1 X{pcbx+pcb_origx} Y{pcby+pcb_origy} Z50')
    # move to place height
    send_gcode(f'G1 Z{zplace}')
    # pump to negative
    set_pump(-1)
    # raise head again
    send_gcode(f'G1 Z50')
    # pump off
    set_pump(0)

def set_pump(pump_val):
    if pump_val == 1:
        pass
        # Turn pump on (positive)
    elif pump_val == 0:
        # Turn pump off
        pass
    elif pump_val == -1:
        # Turn pump on (negative)
        pass
    # Function incomplete: need to send information to vacuum pump

# CONSTANTS
# zpick is the z-position the machine must be moved to in order to pick up a component, in mm
zpick = 20
# zplace is the z-position the machine to place a component, in mm
# (accounting for PCB thickness plus component thickness)
zplace = 21
# location of PCB origin in mm
pcb_origx = 50
pcb_origy = 50








# Read centroid file (convert from csv format to list of required_components)
# required_components of form [type,pcbx,pcby,pcbang]


with open("centroid1.csv") as file:
    csv_reader = reader(file)
    required_components = list(csv_reader)
    del required_components[0]
    for row in required_components:
        print(f"Place {row[0]} resistor at ({row[1]},{row[2]}) on the PCB, with angle {row[3]}")







        
        



# SECTION 2: MOVE TO COMPONENT AREA & CAPTURE IMAGE

time.sleep(1)

# Send G-Code to move to above component area
send_gcode('G1 X100 Y100 Z50\r\n')

# Image capture
take_picture('/home/pi/Documents/imagetest.jpg')











# SECTION 3: IMAGE PROCESSING

# Read image from file
img = cv2.imread('/home/pi/Documents/imagetest.jpg')
# Convert to grayscale
img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
# Gaussian blur
gauss_img = cv2.GaussianBlur(img_gray,(11,11),10)
# Binary inverse threshold
ret,thresh_img = cv2.threshold(gauss_img,70,255,cv2.THRESH_BINARY_INV)










# SECTION 4: CONTOURS AND IMAGE MOMENTS TO IDENTIFY COMPONENTS

# If using OpenCV version 2 (modifies source image)
contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# If using OpenCV version 3 (does not modify source image, outputs new image)
# thresh_img, contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


# Set up empty array and initialise loop variable
found_components = []
comp_count = 0

# For every entry in contours
for i in range(len(contours)):
    # Grab the contour of the current loop
    cnt = contours[i]
    # Factor that affects the level of approximation to a rectangle
    epsilon = 0.12*cv2.arcLength(cnt,True)
    # Calculate approximated form of contour
    approx_cnt = cv2.approxPolyDP(cnt,epsilon,True)
    # Calculate image moments
    M = cv2.moments(approx_cnt)
    
    # Filter to remove erroneously detected contours that are too small to be a resistor
    if M['m00'] > 1000:
        
        # Calculate centroid coordinates using raw moments
        cx = M['m10']/M['m00']
        cy = M['m01']/M['m00']
        
        # Calculate angle of resistor using second order central moments
        theta = -0.5*atan2(2*M['mu11']/M['m00'],M['mu20']/M['m00']-M['mu02']/M['m00'])
        thetadeg = degrees(theta)
        
        # Define resistor type based on area of detected contour
        if M['m00'] < 2000:
            restype = "0603"
        else:
            restype = "0805"
            
        # Add the found component with resistor type, X coordinate (mm), Y coordinate (mm) and angle (deg)
        found_components.append([restype,int(600*cx/2592),(450*(1-cy/1944)),thetadeg])
        
        # Draw detected contour in green
        rect = cv2.minAreaRect(approx_cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img,[box],0,(0,255,0),3)
        
        # Draw determined angle as red line
        pt1 = (int(cx-100*cos(theta)),int(cy+100*sin(theta)))
        pt2 = (int(cx+100*cos(theta)),int(cy-100*sin(theta)))
        cv2.line(img,pt1,pt2,(255,0,0),6)
        
        # Draw determined centroid as blue circle
        cv2.circle(img,(int(cx),int(cy)),8,(0,0,255),-1)
        
        # Add text to figure stating resistor type, (X,Y) coordinates (mm) and angle (deg)
        cv2.putText(img,f"{found_components[comp_count][0]} resistor, ({int(found_components[comp_count][1])},{int(found_components[comp_count][2])})mm, {int(found_components[comp_count][3])}deg",(int(cx+50),int(cy+50)),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),3,cv2.LINE_AA)
        
        # Increment counter so that loop repeats for next contour
        comp_count += 1

# If needed:
# Print list of found components       
#print(found_components)
# Save image in desired location
#cv2.imwrite('/home/pi/Documents/filename',img)
#cv2.imshow("image",img)













# SECTION 5: COMPARE found_components TO required_components TO GET components_to_pick

components_to_pick = []

for required_index, required_component in enumerate(required_components):
    for found_index, found_component in enumerate(found_components):
        # Check if found component is of the required type
        if required_component[0] == found_component[0]:
            # If yes, add the required PCB placement coordinates and angle to the component 
            found_component.extend([required_component[1],required_component[2],required_component[3]])
            
            # Add this component to the components_to_pick array
            components_to_pick.append(found_component.copy())
            
            # Next few lines needed to remove selected components from the lists without affecting continuation of loop
            required_components[required_index][0] = None
            found_components[found_index][0] = None
            
            # Break out of inner loop checking found components if a resistor of correct type has been found
            break

# Remove components that have been found from required_components, leaving any that can not be fulfilled from the available resistors
required_components = [required_component for required_component in required_components if required_component[0] != None]
# Remove components that will be picked up from found_components, leaving components that will be leftover
found_components = [found_component for found_component in found_components if found_component[0] != None]












    
# SECTION 6: PICK AND PLACE FROM components_to_pick
# NB: code here untested and unfinished

# Structure of each sublist of components to pick:
[restype,comx,comy,comang,pcbx,pcby,pcbang]

for component in components_to_pick:
    pick_component(component[1],component[2])
    rotate_component(component[3],component[6])
    place_component(component[4],component[5])
    
    
    
    

    
    
    
    
    
# END

# Close serial connection
ser.close()

# Close OpenCV image windows
cv2.waitKey(0)
cv2.destroyAllWindows()
