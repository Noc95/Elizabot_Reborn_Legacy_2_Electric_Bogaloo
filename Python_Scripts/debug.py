import socket
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
import numpy as np
import threading

# These must match the Pico W's settings
WHYFLY_IP = "192.168.42.1"  # The AP's sacred address
PORT = 80                # The port of the Wi-Fi server

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True)
plot = win.addPlot()
curve = plot.plot(pen='y')

dataList = []
maxDataPoints = 2000 * 10

data_list_size = []

# def connect():
#     """Connecteth to the Pico W and retrieveth its message."""
#     try:
#         # Create a TCP socket
#         with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#             print("Seeking the wisdom of Pico W...")
#             s.connect((WHYFLY_IP, PORT))  # Connect to the server
            
#             # Receive the divine message
#             data = s.recv(1024)  # Read up to 1024 bytes
#             print("Lo! The Pico W hath spoken:")
#             print(data.decode())  # Decode and display the message
    
#     except Exception as e:
#         print("Alas! An error hath occurred:", e)

def fetch_data():
    global dataList
    while True:

        try:
            # Create a TCP socket
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                
                s.connect((WHYFLY_IP, PORT))
                print(s)
                
                while True:
                    raw_data = s.recv(2048)
                    # print(raw_data.decode())
                    if raw_data.decode() == 'Now we are bound as one':
                        continue
                    elif raw_data:
                        data_cunks = raw_data.split(b'D')
                        for cunk in data_cunks:
                            dataList.append(int.from_bytes(cunk, byteorder="big"))
                            # data_list_size.append(len(dataList))

                    # data = raw_data.
                    # if data:
                    #     # print(data.decode())
                    #     dataList.append(data)
                        while len(dataList) > maxDataPoints:
                            dataList.pop(0)
        
        except Exception as e:
            print("Alas! An error hath occurred:", e)

def update_plot():
    """Update the plot with new data"""
    curve.setData(dataList)


if __name__ == "__main__": 

    print("Started!")
    # fetch_data()
    threading.Thread(target=fetch_data, daemon=True).start()

    timer = pg.QtCore.QTimer()
    timer.timeout.connect(update_plot)
    timer.start(1)  # Update every 1ms (FAST)
    
    app.exec_()
    
    



