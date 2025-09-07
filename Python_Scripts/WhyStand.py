import socket
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
import numpy as np
import threading
import struct
from pyqtgraph.Qt import QtGui, QtCore

# These must match the Pico W's settings
WHYSTAND_IP = "192.168.4.1"  # The AP's sacred address
PORT = 80                # The port of the Wi-Fi server

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True)
plot = win.addPlot()
plot.showGrid(x=True, y=True, alpha=0.5)

# Define key press event
def keyPressEvent(event):
    if event.key() == QtCore.Qt.Key_C:  # Check if 'C' key is pressed
        plot.clear()  # Clear the plot
        # global motor_1 = []
        # motor_2 = []
        # motor_3 = []
        # motor_4 = []
        # print("Window cleared!")

# Assign the custom keyPressEvent to the window
win.keyPressEvent = keyPressEvent

angle_curve = plot.plot(pen='r')
angle_setpoint_curve = plot.plot(pen='g')

leftMotor_speed_curve = plot.plot(pen='b')
leftMotor_speed_setpoint_curve = plot.plot(pen='c')

rightMotor_speed_curve = plot.plot(pen='y', label="X Unfiltered")
rightMotor_speed_setpoint_curve = plot.plot(pen='w')

# motor_1_curve = plot.plot(pen=pg.mkPen(color=(255, 165, 0)))  # Orange
# motor_2_curve = plot.plot(pen=pg.mkPen(color=(0, 128, 128)))  # Teal
# motor_3_curve = plot.plot(pen=pg.mkPen(color=(148, 0, 211)))  # Violet
# motor_4_curve = plot.plot(pen=pg.mkPen(color=(50, 205, 50)))  # Lime Green

angle_dataList = []
angle_setpoint_dataList = []

leftMotor_speed_dataList = []
leftMotor_speed_setpoint_dataList = []

rightMotor_speed_dataList = [0]
rightMotor_speed_setpoint_dataList = [0]

motor_1 = []
motor_2 = []
motor_3 = []
motor_4 = []

maxDataPoints = 2000 * 10
limit_graph_length = True

data_list_size = []


def fetch_data():
    global dataList
    while True:

        try:
            # Create a TCP socket
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                
                s.connect((WHYSTAND_IP, PORT))
                s.sendall(b"Hej Elizabot!")
                print(s)
                
                while True:
                    raw_data = s.recv(2048)
                    # print(raw_data.decode())
                    if raw_data:
                        
                        data_cunks = raw_data.split(b'D')
                        for cunk in data_cunks:
                            
                            if len(cunk) == 4*4:
                                data = struct.unpack('<4f', cunk)
                                
                                angle_dataList.append(data[0])
                                angle_setpoint_dataList.append(data[1])

                                leftMotor_speed_setpoint_dataList.append(data[2])
                                leftMotor_speed_dataList.append(data[3])

                            # if len(cunk) == 24:
                            #     data = struct.unpack('<4i4h', cunk)
                                
                            #     scaled_data = [v / (2 ** 24) for v in data]

                            #     # pidx, pidy, filteredx, filteredy (32 bit), motor1, motor2, motor3, motor4 (16bit)

                            #     angle_dataList.append(scaled_data[0])
                            #     angle_setpoint_dataList.append(scaled_data[1])
                            #     # leftMotor_speed_dataList.append(scaled_data[2])

                            #     leftMotor_speed_setpoint_dataList.append(scaled_data[2])
                            #     rightMotor_speed_dataList.append(scaled_data[3])
                            #     # rightMotor_speed_setpoint_dataList.append(scaled_data[4])

                            #     # leftMotor_speed_setpoint_dataList.append(leftMotor_speed_setpoint_dataList[-1] * alpha + angle_dataList[-1] * (1-alpha))
                            #     # rightMotor_speed_dataList.append(rightMotor_speed_dataList[-1] * alpha + angle_setpoint_dataList[-1] * (1-alpha))
                            #     # rightMotor_speed_setpoint_dataList.append(rightMotor_speed_setpoint_dataList[-1] * alpha + leftMotor_speed_dataList[-1] * (1-alpha))
                                
                            #     motor_1.append(data[4])
                            #     motor_2.append(data[5])
                            #     motor_3.append(data[6])
                            #     motor_4.append(data[7])


                        while len(angle_dataList) > maxDataPoints and limit_graph_length:
                            try:
                                angle_dataList.pop(0)
                                angle_setpoint_dataList.pop(0)
                                # leftMotor_speed_dataList.pop(0)

                                leftMotor_speed_setpoint_dataList.pop(0)
                                rightMotor_speed_dataList.pop(0)
                                # rightMotor_speed_setpoint_dataList.pop(0)

                                motor_1.pop(0)
                                motor_2.pop(0)
                                motor_3.pop(0)
                                motor_4.pop(0)
                            except:
                                pass
        
        except Exception as e:
            print("Alas! An error hath occurred:", e)

def update_plot():
    """Update the plot with new data"""

    

    # rightMotor_speed_curve.setData(leftMotor_speed_setpoint_dataList)
    # rightMotor_speed_setpoint_curve.setData(rightMotor_speed_dataList)
    # leftMotor_speed_setpoint_curve.setData(rightMotor_speed_setpoint_dataList)

    # motor_1_curve.setData(motor_1)
    # motor_2_curve.setData(motor_2)
    # motor_3_curve.setData(motor_3)
    # motor_4_curve.setData(motor_4)
    # angle_curve.setData(angle_dataList)
    # angle_setpoint_curve.setData(angle_setpoint_dataList)
    leftMotor_speed_curve.setData(leftMotor_speed_dataList)
    leftMotor_speed_setpoint_curve.setData(leftMotor_speed_setpoint_dataList)


if __name__ == "__main__": 

    threading.Thread(target=fetch_data, daemon=True).start()

    timer = pg.QtCore.QTimer()
    timer.timeout.connect(update_plot)
    timer.start(1)  # Update every 1ms (FAST)
    
    app.exec_()
    
    



