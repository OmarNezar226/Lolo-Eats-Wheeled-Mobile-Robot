
import cv2
import rospy
import numpy as np
from std_msgs.msg import Int32
import cv2.aruco as aruco 
from geometry_msgs.msg import Point 
import socket

class QR_code_ros:
    def _init_(self):
        self.init_Aruco()
        self.cap = cv2.VideoCapture(0)
        self.qr_code_detector = cv2.QRCodeDetector()
        rospy.init_node('QR_Code_Detector')
        self.qr_data_pub = rospy.Publisher('/qr_data', Int32, queue_size= 10)
        self.marker1_pub = rospy.Publisher('/marker1', Point, queue_size= 10) 
        self.marker2_pub = rospy.Publisher('/marker2', Point, queue_size= 10)
        self.marker3_pub = rospy.Publisher('/marker3', Point, queue_size= 10)
        
    def init_Aruco(self):
        # Define the camera matrix
        self.camera_matrix = np.array([[616.04440248, 0, 326.03721253],
                          [0, 616.78329332, 244.04541953],
                          [0, 0, 1]])
        # Define the distortion coefficients
        self.dist_coeffs = np.array([[-0.53562068, 0.38170241, -0.00103521, 0.00109988, -0.08813086]])

        # Create an ArUco dictionary
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)

        # Create the detector parameters
        self.parameters = aruco.DetectorParameters_create()

        # Define the marker size in meters
        self.marker_size = 0.120
        

    def run(self):
        while True:
            ret, frame = self.cap.read()
            qr_code_detected, qr_data, bbox, _ = self.qr_code_detector.detectAndDecodeMulti(frame)
            if len(qr_data) > 0:
                msg = Int32()
                print(qr_data)
                if qr_data[0] == '' :
                    msg.data = -1 # If it detects unusual Qr code it will display -1
                else:
                    msg.data = int(qr_data[0])
                #print(qr_data[0])
                self.qr_data_pub.publish(msg)
            
            corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)
            if ids is not None:
               
                for marker_index,id in enumerate(ids):
                # Estimate the marker pose
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        corners[marker_index], self.marker_size, self.camera_matrix, self.dist_coeffs
                    )

                    # Extract the x, y, z coordinates
                    x = tvecs[0][0][0]
                    y = tvecs[0][0][1]
                    z = tvecs[0][0][2]

                    if id == 1:
                        self.marker1_pub.publish(Point(x,y,z))
                    elif id == 2:
                        self.marker2_pub.publish(Point(x,y,z))
                    elif id == 3:
                        self.marker3_pub.publish(Point(x,y,z))
            
            cv2.imshow('img',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

def start_server():
    host = "192.168.43.162"  # Listen on all available network interfaces
    port = 5555  # Choose a port number matching the one used in the QT application

    try:
        # Create a socket object
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # Bind the socket to the host and port
            s.bind((host, port))
            # Listen for incoming connections
            s.listen()

            print("Server started. Waiting for connections...")

            while True:
                # Accept a client connection
                client_socket, address = s.accept()
                print("Connection established from:", address)

                # Receive data from the client
                data = client_socket.recv(1024).decode()
                print("Received data:", data)
                
                # Process the received data as needed

                # Send a response back to the client if required
                response = "Server received the data successfully"
                client_socket.sendall(response.encode())

                # Close the client connection
                client_socket.close()
                if data == "Order received and ready to deliver":
                    print('22')
                    break

    except socket.error as e:
        print("Error:", e)




if __name__ == '__main__':
    # Start the server
    start_server()
    qr_node = QR_code_ros()
    qr_node.run()