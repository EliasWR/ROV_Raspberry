# Server should be run on the application that is constantly
# running/ always on. Should be run on Raspberry Pi

import socket


# Value is found by using command <ifconfig> in terminal
HOST = "169.254.226.72"  # The IP address of the RASPBERRY Pi assigns to this communication


PORT = 65432  # Port to listen on (non-privileged ports are > 1023)


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            print(data)
            if not data:
                break
            conn.sendall(data)
