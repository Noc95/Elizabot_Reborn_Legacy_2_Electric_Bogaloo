import socket
import keyboard
from time import sleep

# ESP8266 details
host = "192.168.4.1"  # Default IP of the ESP8266 in AP mode
port = 80             # Port must match the ESP8266 server port



forward_signal = 1000
backward_signal = 1000
turning_signal = 1000

def connect_WiFi(client_socket):
    client_socket.connect((host, port))
    print(f"Connected to {host}:{port}")

def disconnect_WiFi(client_socket):
    client_socket.close()

def send_message(client_socket, message):
    try:
        client_socket.sendall((message + "\n").encode('utf-8'))
        
    except Exception as e:
        print("Error:", e)

def recieve_message(client_socket):
    response = client_socket.recv(1024).decode('utf-8')
    print("Response from ESP8266:", response)

    return response



if __name__ == '__main__':
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock = connect_WiFi(client_socket)

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
            send_message(client_socket, message)
            sleep(0.1)
        except:
            break