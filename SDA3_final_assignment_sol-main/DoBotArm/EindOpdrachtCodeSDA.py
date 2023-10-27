import threading
import DoBotArm as Dbt
import time
from serial.tools import list_ports

import cv2
import time
import argparse
import numpy as np

# Get values from the HSV trackbar
HMin = 88 #H is for the color, in this case its blue
SMin = 97
VMin = 71
HMax = 130
SMax = 255
VMax = 255
minHSV = np.array([HMin, SMin, VMin])
maxHSV = np.array([HMax, SMax, VMax])


def port_selection():
    # Choosing port
    available_ports = list_ports.comports()
    print('Available COM-ports:')
    for i, port in enumerate(available_ports):
        print(f"  {i}: {port.description}")

    choice = int(input('Choose port by typing a number followed by [Enter]: '))
    return available_ports[choice].device

#--Main Program--
def main():
    #List selected ports for selection
    port = port_selection()
        
    # Preprogrammed sequence
    homeX, homeY, homeZ = 135, 0, 50
    print("Connecting")
    print("Homing")
    ctrlBot = Dbt.DoBotArm(port, homeX, homeY, homeZ, home = True) #Create DoBot Class Object with home position x,y,z
    
    # Create a video capture object
    vid_capture = cv2.VideoCapture(1, cv2.CAP_DSHOW)

    if not vid_capture.isOpened():
        print("Error opening video stream")

    # Get height and width of the frame
    frame_width = int(vid_capture.get(3))
    frame_height = int(vid_capture.get(4))
    frame_size = (frame_width, frame_height)
    fps = 20

    # Eerst een keer een frame lezen en laten zien
    ret, frame = vid_capture.read()
    cv2.imshow('Centroids and Shapes', frame)


    while True:

        # Twee keer hetzelfde omdat de dobot het eerste coordinaat overslaat
        ctrlBot.moveArmXYZ (135, 0, 100)
        ctrlBot.moveArmXYZ (135, 0, 100)
        

        #funcite coordinatien krijgen
        ret, frame = vid_capture.read()
        #cv2.waitKey(1)
        

        if ret:
            ret, frame = vid_capture.read()
            original = frame.copy()

            #print(original.shape) # Print image dimensions
            original = original[150:370, 130:350] # frame croppen

            # Convert the BGR image to other color spaces
            imageHSV = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)

            # Create the mask using the min and max values obtained from the trackbar and apply bitwise and operation to get the results
            maskHSV = cv2.inRange(imageHSV, minHSV, maxHSV)
            resultHSV = cv2.bitwise_and(original, original, mask=maskHSV)

            resultBGR = cv2.cvtColor(resultHSV, cv2.COLOR_HSV2BGR)
            img_gray = cv2.cvtColor(resultBGR, cv2.COLOR_BGR2GRAY)

            ret, thresh = cv2.threshold(img_gray, 0, 200, cv2.THRESH_BINARY)

            contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)

            #image_copy = resultHSV.copy() # camera live feed all black except the blue
            image_copy = original.copy() # camera live feed with surroundings

            # Detect and label shapes and display centroids
            for contour in contours:
                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                num_vertices = len(approx)

                # Calculate centroid
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                # Label and draw the shape
                
                if num_vertices == 3:
                    shape_label = "Triangle"
                elif num_vertices == 4:
                    shape_label = "Rectangle"
                elif num_vertices == 5:
                    shape_label = "Pentagon"
                elif num_vertices == 6:
                    shape_label = "Hexagon"
                else:
                    shape_label = "Circle"
                
                # Wanneer die niks ziet move voor oppakken uit zeten
                if num_vertices == 0:
                    Move = False
                else:
                    Move = True

                # Adjust the position of the text
                text_x = cX - 30
                text_y = cY - 30

                # Berekeningen voor de locatie van het object dat die gaat oppakken
                X = 245 - (cX*0.72)
                Y = -(258 - (cY*0.72)) 

                cv2.drawContours(image=image_copy, contours=[contour], contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
                cv2.putText(image_copy, (cX,cY) (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.circle(image_copy, (cX, cY), 5, (255, 255, 255), -1)

            cv2.imshow('Centroids and Shapes', image_copy)
            cv2.waitKey(1)

        

        if Move == True:
            # op pakken
            ctrlBot.moveArmXYZ (125, -160, 100)
            ctrlBot.moveArmXYZ (X, Y, -43)
            ctrlBot.toggleSuction(True)
            ctrlBot.moveArmXYZ (125, -160, 100)

            #droppen
            ctrlBot.moveArmXYZ (135, 0, 50)
            ctrlBot.moveArmXYZ (135, 0, 8)
            ctrlBot.toggleSuction(False)
            ctrlBot.moveArmXYZ (135, 0, 50)

            ctrlBot.SetConveyor(True)
            time.sleep(1)
            #wachten

            ctrlBot.SetConveyor(False)

        #weer opnieuw beginnen

    print("Disconnecting")

if __name__ == "__main__":
    main()
