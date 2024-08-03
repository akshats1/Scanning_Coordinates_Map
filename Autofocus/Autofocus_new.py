import serial
import time
import cv2
from time import sleep
from picamera import PiCamera
import numpy as np
import io
import os
from datetime import datetime
from PIL import Image

# declare x_axis, y_axis, z_axis globally
x_axis = 0
y_axis = 0
z_axis = 0

class Autofocus:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.camera = PiCamera()
        self.x = 0
        self.y = 0
        self.z = 0
        sleep(1)

    def movexclock(self, distance):
        command = "xclk,{}".format(distance)
        self.ser.write(command.encode("utf-8"))
        self.x += distance

    def movexanticlock(self, distance):
        command = "xcclk,{}".format(distance)
        self.ser.write(command.encode("utf-8"))
        self.x -= distance

    def movey(self, steps):
        command = "ycclk,{}".format(steps)
        self.ser.write(command.encode("utf-8"))
        self.y += steps
        sleep(1)

    def moveyc(self, steps):
        command = "yclk,{}".format(steps)
        self.ser.write(command.encode("utf-8"))
        self.y -= steps
        sleep(1)

    def movezclock(self, distance):
        command = "zclk,{}".format(distance)
        self.ser.write(command.encode("utf-8"))
        self.z -= distance

    def movezanticlock(self, distance):
        command = "zcclk,{}".format(distance)
        self.ser.write(command.encode("utf-8"))
        self.z += distance

    def variance(self, image):
        bg = cv2.GaussianBlur(image, (11, 11), 0)
        v = cv2.Laplacian(bg, cv2.CV_64F).var()
        return v

    def home(self, x_axis, y_axis, z_axis):
        print("Home Position for x, y, z in Process")
        self.movexclock(x_axis)
        sleep(4)
        self.movey(y_axis)
        sleep(5)
        self.movezanticlock(z_axis)
        sleep(8)

    def roixy_4x(self, x, y):
        self.movexclock(x)
        sleep(8)
        self.moveyc(y)
        global x_axis, y_axis, z_axis
        x_axis += x
        y_axis += y

        print("Movement of Steps")
        print("x_axis ", x_axis)
        print("y_axis ", y_axis)
        print("z_axis", z_axis)

        print("Movement of Steps in mm")
        print("x_axis ", str(x_axis * 0.075) + " mm")
        print("y_axis ", str(y_axis * 0.075) + " mm")
        print("z_axis", z_axis)

        sleep(3)

    def roixy_10x(self, x, y):
        self.movexclock(x)
        global x_axis, y_axis, z_axis
        x_axis += x

        print("Movement of Steps")
        print("x_axis ", x_axis)
        print("y_axis ", y_axis)
        print("z_axis", z_axis)

        print("Movement of Steps in mm")
        print("x_axis ", str(x_axis * 0.075) + " mm")
        print("y_axis ", str(y_axis * 0.075) + " mm")
        print("z_axis", z_axis)

        sleep(8)

    def init(self):
        self.ser.write("init".encode())

    def auto(self, obj_value):
        global x_axis, y_axis, z_axis

        z_positions = []
        variances = []
        max_variance = 0
        max_z = self.z

        self.camera.start_preview()
        self.ser.reset_input_buffer()
        sleep(1)

        self.camera.resolution = (320, 240)
        self.camera.framerate = 24

        if obj_value == 4:
            step_size = 50
        elif obj_value == 10:
            step_size = 25
        elif obj_value == 40:
            step_size = 5

        def capture_image():
            stream = io.BytesIO()
            self.camera.capture(stream, format='jpeg')
            stream.seek(0)
            image = np.frombuffer(stream.getvalue(), dtype=np.uint8)
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)
            return image

        def update_variance(z_move_function, direction):
            nonlocal max_variance, max_z
            z_move_function(step_size)
            sleep(1)
            image = capture_image()
            current_variance = self.variance(image)
            variances.append(current_variance)
            z_positions.append(self.z)

            if current_variance > max_variance:
                max_variance = current_variance
                max_z = self.z
                print(f"New max variance {max_variance} at z {self.z} in direction {direction}")
                return True
            return False

        # Initial upward movement
        for _ in range(2):
            self.movezanticlock(step_size)
            z_axis += step_size
            sleep(1)

        # Check variance in initial directions
        improving = True
        while improving:
            improving = update_variance(self.movezanticlock, "up")
        
        if not improving:
            while update_variance(self.movezclock, "down"):
                pass

        adjust_steps = self.z - max_z
        if adjust_steps > 0:
            self.movezclock(adjust_steps)
            z_axis -= adjust_steps
        else:
            self.movezanticlock(abs(adjust_steps))
            z_axis += abs(adjust_steps)

        self.camera.resolution = (4056, 3040)
        sleep(2)
        stream = io.BytesIO()
        self.camera.capture(stream, format='jpeg')
        stream.seek(0)
        high_res_image = np.frombuffer(stream.getvalue(), dtype=np.uint8)
        high_res_image = cv2.imdecode(high_res_image, cv2.IMREAD_COLOR)

        base_dir = "/home/pi/Downloads/autoscan"
        objective_dir = os.path.join(base_dir, f"{obj_value}X")
        os.makedirs(objective_dir, exist_ok=True)
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_path = os.path.join(objective_dir, f"focusedimage_{current_time}.tiff")

        high_res_image_pil = Image.fromarray(cv2.cvtColor(high_res_image, cv2.COLOR_BGR2RGB))
        high_res_image_pil.save(image_path, format='TIFF')

        sleep(5)
        print(variances)
        print(z_positions)
        print(max_z)
        print(max_variance)

if __name__ == "__main__":
    af = Autofocus()

    print("Initialization Starting Please Wait......")
    xi = 350
    yi = 200
    af.init()
    sleep(21)

    print("Initialization Stops for x, y, z in Process")

    x_axis = 0
    y_axis = 0
    z_axis = 0
    print("x_axis", x_axis)
    print("y_axis", y_axis)
    print("z_axis", z_axis)

    x_axis = 30
    y_axis = 100
    z_axis = 4000
    af.home(x_axis, y_axis, z_axis)
    print("Movement of Steps After Home Position")
    print("x_axis ", x_axis)
    print("y_axis ", y_axis)
    print("z_axis", z_axis)
    print("Movement of Steps in mm")
    print("x_axis ", str(x_axis * 0.075) + " mm")
    print("y_axis ", str(y_axis * 0.075) + " mm")
    print("z_axis", z_axis)

    print("Please enter the Objective value 4, 10 or 40")
    in_obj = int(input())

    if in_obj == 4:
        if z_axis > 3500:
            ans_z = z_axis - 3500
            z_axis = z_axis - ans_z
            af.movezclock(ans_z)
        else:
            ans_z = 3500 - z_axis
            z_axis = z_axis + ans_z
            af.movezanticlock(ans_z)

        x = 65
        y = 0
        af.roixy_4x(x, y)
        sleep(2)

    elif in_obj == 10:
        if z_axis > 13550:
            ans_z = z_axis - 13550
            z_axis = z_axis - ans_z
            af.movezclock(ans_z)
        else:
            ans_z = 13550 - z_axis
            z_axis = z_axis + ans_z
            af.movezanticlock(ans_z)

        sleep(6)

        print("Movement of Steps")
        print("x_axis ", x_axis)
        print("y_axis ", y_axis)
        print("z_axis", z_axis)
        print("Movement of Steps in mm")
        print("x_axis ", str(x_axis * 0.075) + " mm")
        print("y_axis ", str(y_axis * 0.075) + " mm")
        print("z_axis", z_axis)

        x = 98
        y = 0
        af.roixy_10x(x, y) ## going to one random region
        sleep(4)

    elif in_obj == 40:
        af.movezanticlock(16650)
        sleep(22)

    af.auto(in_obj)
    print("Movement of Steps After Autofocus")
    print("x_axis ", x_axis)
    print("y_axis ", y_axis)
    print("z_axis", z_axis)
    print("Movement of Steps After Autofocus in mm")
    print("x_axis ", str(x_axis * 0.075) + " mm")
    print("y_axis ", str(y_axis * 0.075) + " mm")
    print("z_axis", z_axis)

