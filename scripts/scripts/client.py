#!/usr/bin/env python

import tkinter as tk
import socket
import struct
import time
from tkinter import ttk
from PIL import Image, ImageTk

class TCP_IP():
    def __init__(self):
        self.TCP_IP = '132.27.96.201'
        self.TCP_PORT = 5006
        self.BUFFER_SIZE = 1024
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def recieve_communication(self):

        try:
            self.s.connect((self.TCP_IP, self.TCP_PORT))
            self.data = self.s.recv(self.BUFFER_SIZE)
            print("received data:", self.data.decode())
            #self.s.sendall('fuckyou'.encode())
            if(self.data):
                abc=GUI_sawyer()
                message=abc.main_gui(data_server=self.data)
                self.s.sendall(message.encode())
            #MESSAGE = input("enter a message")



        except Exception as e:
            print("connection failed", e)




class GUI_sawyer():
    def __init__(self):
        self.message_from_gui=""
        pass

    def destroy_window(self, app):
        self.message_from_gui = entry.get()
        print(self.message_from_gui)
        app.destroy()

    def main_gui(self, data_server):
        global label_message, The_message, label_question, entry, button_continue
        app = tk.Tk()
        app.title("Sawyer GUI")
        app.configure(bg="#f0f0f0")

        # Title
        title = tk.Label(app, text="Sawyer GUI", font=("Helvetica", 40, "bold"), bg="#f0f0f0", fg="#003366")
        title.pack(pady=20)

        # Message
        label_message = tk.Label(app, text="Message:", font=("Helvetica", 25), bg="#f0f0f0", fg="#003366")
        label_message.pack(pady=5)

        The_message = tk.Label(app, text=data_server, fg="red", font=("Helvetica", 25, "italic"), bg="#f0f0f0")
        The_message.pack(pady=10)

        # Instructions
        label_question = tk.Label(app, text="Press the continue button or ask me any question:", font=("Helvetica", 25), bg="#f0f0f0", fg="#003366")
        label_question.pack(pady=5)

        # Entry
        entry = tk.Entry(app, width=80, font=("Helvetica", 18), fg="#666666", bd=2, relief="solid")
        entry.pack(pady=10)
        entry.insert(0, "Example: What/Why is the error?")

        # Continue button
        button_continue = ttk.Button(app, text="Continue", command=lambda: self.destroy_window(app), style="C.TButton")
        button_continue.pack(pady=10)

      #   # Load and display image
      #   image = Image.open("/mnt/data/image.png")
      #   image = image.resize((150, 150), Image.ANTIALIAS)
      #   image_tk = ImageTk.PhotoImage(image)
      #   label_image = tk.Label(app, image=image_tk, bg="#f0f0f0")
      #   label_image.image = image_tk  # Keep a reference to avoid garbage collection
      #   label_image.pack(pady=20)

        # Style configurations
        style = ttk.Style()
        style.configure('C.TButton', font=('Helvetica', 25), padding=10)

        app.mainloop()
        print(self.message_from_gui)
        return self.message_from_gui

    # Initial blank screen
    def blank_screen(self,data):
        app = tk.Tk()
        app.title("HRI Experiment")
        blank_screen = tk.Tk()
        blank_screen.title("Connecting...")
        label_connecting = tk.Label(blank_screen, text="Connecting to server...", font=("Helvetica", 50))
        label_connecting.pack(pady=20)
        message=self.main_gui(app,data)
        app.mainloop()
        return message




def main():
    while True:
        abc = TCP_IP()
        abc.recieve_communication()



if __name__ == "__main__":
    main()
