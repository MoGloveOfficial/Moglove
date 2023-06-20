import tkinter as tk
from PIL import Image, ImageTk
import os
import csv
import matplotlib.pyplot as plt
import random
import cv2
import serial

f = open("Data.csv", "a", newline='')
writer = csv.writer(f)

directory = "/home/uwu/Desktop/opticalMocap/MoGlove Trainer/SampleImages"
files = os.listdir(directory)
image_files = [file for file in files if file.endswith(".png")]
photo_images = []

arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=5)

def receiveSerial():
    raw = []
    data = []
    while len(data) != 5:
        try:
            raw = arduino.readline().decode('utf-8').rstrip().split(',')
            data = [float(num) for num in raw]
        except:
            pass
    print(data)
    return data


def button_clicked(i, data):
    data.append(i)
    writer.writerow(data)
    print("Button clicked:", i)


def main():
    root = tk.Tk()
    data = receiveSerial()

    for filename in image_files:
        image_path = os.path.join(directory, filename)
        image = Image.open(image_path)
        image = image.resize((200, 200))
        photo = ImageTk.PhotoImage(image)
        photo_images.append(photo)

    rows = int(len(photo_images) / 4) 
    cols = 5

    buttons = []
    for i, photo in enumerate(photo_images):
        button = tk.Button(root, image=photo, command=lambda i=i: button_clicked(i, data))
        button.grid(row=i // cols, column=i % cols, padx=5, pady=5)
        buttons.append(button)
    
    def update_data():
        nonlocal data
        data = receiveSerial()  # Update data values
        root.after(1, update_data)  # Schedule the next update after 10 milliseconds

    update_data()  # Start the initial data update 

    def exit_program(event):
        if event.char == 'q':
            root.destroy()
            return None

    root.bind('<Key>', exit_program)
    root.mainloop()


if __name__ == "__main__":
    main()
    f.close()
    plt.ioff()
    cv2.destroyAllWindows()
