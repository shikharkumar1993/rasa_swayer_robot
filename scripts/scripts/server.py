#!/usr/bin/env python
import tkinter as tk
import socket
import struct
import time

def communication():
    TCP_IP='132.27.96.201'
    TCP_PORT=5006
    BUFFER_SIZE=1024

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    #
    #print ('Connected by', addr)
    while 1:
       conn, addr = s.accept()
       conn.send("abc".encode())
       while True:
         print('123')
         data = conn.recv(BUFFER_SIZE)
       
         print(data.decode())
         if not data:          
            break
         print ("received data:", data.decode())
         # echo
       conn.close()

def main():
   communication()
if __name__=="__main__":
   main()


# import socket

# def communication():
#     TCP_IP = '132.72.96.201'  # Replace with the actual IP address
#     TCP_PORT = 5006
#     BUFFER_SIZE = 1024

#     s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     s.bind((TCP_IP, TCP_PORT))
#     s.listen(1)
#     print("Server is listening on {}:{}".format(TCP_IP, TCP_PORT))

#     conn, addr = s.accept()
#     conn.send("Acknowledged".encode())
#     print('Connected by', addr)
#     while True:
#         data = conn.recv(BUFFER_SIZE)
#         if not data:
#             break
#         print("Received data:", data.decode())
        
#     conn.close()

# def main():
#     communication()

# if __name__ == "__main__":
#     main()
