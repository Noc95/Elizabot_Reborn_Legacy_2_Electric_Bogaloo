import bluetooth
from bleak import BleakScanner
from bleak import BleakClient
import keyboard
from time import sleep


forward_signal = 1000
backward_signal = 1000
turning_signal = 1000


def connect_via_bluetooth():
    # Look for devices and connect to Elizabot
    while True:
        print("Searching for devices...")
        devices = bluetooth.discover_devices(lookup_names=True)
        
        for addr, name in devices:
            # print(f"{addr} - {name}")
            if name == "elizabot":
                print("Found Elizabot!")
                sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                sock.connect((addr, 1))  # 1 is the default channel for HC-05/HC-06
                return sock
        sleep(5)

def send_data_via_bluetooth(sock, data_string : str):
    sock.send(data_string)

def connect_via_ble():
    devices = BleakScanner.discover()
    for d in devices:
        print(d)


if __name__ == '__main__':
    sock = connect_via_bluetooth()
    # connect_via_ble()

    # Read inputs from the keyboard and send data to Elizabot
    while True:         # fxxx

        message = ""

        try:

            if keyboard.is_pressed('up'):
                # print('Forward')
                message += "f" + format(forward_signal, 'x')
            elif keyboard.is_pressed('down'):
                # print('Backward')
                message += "b" + hex(backward_signal)[2:]
            else:
                message = "xxxx"

            if keyboard.is_pressed('left'):
                # print('left')
                message += "l" + hex(turning_signal)[2:]
            elif keyboard.is_pressed('right'):
                # print('right')
                message += "r" + hex(turning_signal)[2:]
            else:
                message += "xxxx"
            
            print(message)
            send_data_via_bluetooth(sock, message)
            sleep(0.1)
        except:
            break