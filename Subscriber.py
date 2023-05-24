
import cv2
import rospy
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Point 
import serial
import time

def qr_callback(msg):
    global qr_data
    print(msg.data)
    if msg.data == -1:
        return
        # print("QR code not detected")
    else:
        qr_data = msg.data
        # print("QR code detected: ", qr_data)

def marker1_callback(msg):
    global marker1
    marker1 = msg
    # print("Marker 1 detected: ", marker1)

def marker2_callback(msg):
    global marker2
    marker2 = msg
    # print("Marker 2 detected: ", marker2)

def marker3_callback(msg):
    global marker3
    marker3 = msg
    # print("Marker 3 detected: ", marker3)

def main():
    global qr_data, marker1, marker2, marker3
    ser = serial.Serial('/dev/ttyUSB0', 9600)  # Replace '/dev/ttyACM0' with your Arduino's serial port
    time.sleep(2)  # Wait for the serial connection to be established
    #connection between arduino and pi here, w 3shan a access 7aga goa el marker hnktb marker1.x msln
    qr_data = -1
    marker1 = None
    marker2 = None
    marker3 = None
    prev_marker1 = None
    prev_marker2 = None
    prev_marker3 = None
    while not rospy.is_shutdown():
        if qr_data == 1:
            if marker1==None:
                print("searching for marker 1") #kol print ht3ml 7aga, mmkn nktb ba2y el code hna
                ser.write(b'A')
            else:
                if marker1 != prev_marker1:
                    print("marker 1 detected", marker1)
                    prev_marker1 = marker1
                    #send command to move to marker 1
                    if marker1.z <0.43:
                        ser.write(b'6')
                        time.sleep(10)
                        ser.write(b'7')
                        time.sleep(3)
                    else:
                        if marker1.x > 0.05:
                            ser.write(b'1')
                        elif marker1.x < -0.05:
                            ser.write(b'2')
                        elif -0.05 <= marker1.x <= 0.05:
                            ser.write(b'3')
                    
                else:
                    print("marker 1 detected but not moving, reseting marker 1, return to search")
                    marker1 = None
                    prev_marker1 = None
                
        elif qr_data == 2:
            if marker2==None:
                print("searching for marker 2")
                ser.write(b'A')
            else:
                if marker2 != prev_marker2:
                    print("marker 2 detected", marker2)
                    prev_marker2 = marker2
                    #send command to move to marker 2
                    if marker2.z <0.43:
                        ser.write(b'6')
                        time.sleep(10)
                        ser.write(b'7')
                        time.sleep(3)
                    else:
                        if marker2.x > 0.05:
                            ser.write(b'1')
                        elif marker2.x < -0.05:
                            ser.write(b'2')
                        elif -0.05 <= marker2.x <= 0.05:
                            ser.write(b'3')
                else:
                    print("marker 2 detected but not moving, reseting marker 2, return to search")
                    if marker2.x > 0.1:
                        ser.write(b'4')
                    elif marker2.x < -0.1:
                        ser.write(b'5')
                    marker2 = None
                    prev_marker2 = None
                
        elif qr_data == 3:
            if marker3==None:
                print("searching for marker 3")
                ser.write(b'A')
            else:
                if marker3 != prev_marker3:
                    print("marker 3 detected", marker3)
                    prev_marker3 = marker3
                    #send command to move to marker 3
                    if marker3.z <0.43:
                        ser.write(b'6')
                        time.sleep(10)
                        ser.write(b'7')
                        time.sleep(3)
                    else:
                        if marker3.x > 0.05:
                            ser.write(b'1')
                        elif marker3.x < -0.05:
                            ser.write(b'2')
                        elif -0.05 <= marker3.x <= 0.05:
                            ser.write(b'3')
                else:
                    print("marker 3 detected but not moving, reseting marker 3, return to search")
                    if marker3.x > 0.1:
                        ser.write(b'4')
                    elif marker3.x < -0.1:
                        ser.write(b'5')
                    marker3 = None
                    prev_marker3 = None
                
        else:
            print("QR code not detected")
        rospy.sleep(1)

rospy.init_node('subscriber_code')
rospy.Subscriber('/qr_data', Int32, qr_callback)
rospy.Subscriber('/marker1', Point, marker1_callback)
rospy.Subscriber('/marker2', Point, marker2_callback)
rospy.Subscriber('/marker3', Point, marker3_callback)
main()