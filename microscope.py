import serial
import cv2
from time import sleep
from datetime import datetime
import subprocess
import logging
from picamera import PiCamera
from PIL import Image
import io
import numpy as np
import os
import time

############## declaring x,y,z axis globally##########
x_axis=0
y_axis=0
z_axis=0

class Microscope:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=9600, timeout=1):
        self.board = serial.Serial(port, baud_rate, timeout=timeout)
        self.camera = PiCamera()
        self.x = 0
        self.y = 0
        self.z = 0
        self.scan_count = 0

    def movexclock(self, distance):
        self.board.write("xclk,{}".format(distance).encode())
        self._wait_for_completion()
        self.x += distance

    def movexanticlock(self, distance):
        self.board.write("xcclk,{}".format(distance).encode())
        self._wait_for_completion()
        self.x -= distance

    def movey(self, steps):
        self.board.write("ycclk,{}".format(steps).encode())
        self._wait_for_completion()
        self.y += steps

    def moveyc(self, steps):
        command = "yclk,{}".format(steps)
        self.board.write(command.encode("utf-8"))
        self.y -= steps
        sleep(1)

    def movezclock(self, distance):
        self.board.write("zclk,{}".format(distance).encode())
        self._wait_for_completion()
        self.z -= distance

    def movezanticlock(self, distance):
        self.board.write("zcclk,{}".format(distance).encode())
        self._wait_for_completion()
        self.z += distance

    def _wait_for_completion(self):
        while True:
            data = self.board.readline()
            if data == b'Done\r\n':
                break
        self.board.reset_input_buffer()

    def calculate_variance(self, image_bytes):
        image = Image.open(image_bytes)
        image = image.convert("L")
        image_array = np.array(image)
        variance = np.var(image_array)
        return variance

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
        global x_axis, y_axis, z_axis
        x_axis += x
        print("Movement of Steps")
        print("x_axis", x_axis)
        print("y_axis", y_axis)
        print("z_axis", z_axis)
        print("Movement of Steps in mm")
        print("x_axis", str(x_axis * 0.075) + " mm")
        print("y_axis", str(y_axis * 0.075) + " mm")
        print("z_axis", z_axis)
        sleep(3)
    
    def roixy_10x(self, x, y):
        self.movexclock(x)
        sleep(8)
        self.moveyc(y)
        global x_axis, y_axis, z_axis
        x_axis += x
        y_axis += y
        print("Movement of Steps")
        print("x_axis", x_axis)
        print("y_axis", y_axis)
        print("z_axis", z_axis)
        print("Movement of Steps in mm")
        print("x_axis", str(x_axis * 0.075) + " mm")
        print("y_axis", str(y_axis * 0.075) + " mm")
        print("z_axis", z_axis)
    
    def init(self):
        self.board.write("init".encode())
    """"
    def autofocus(self):
        logging.info("[Autofocus] Starting AutoFocus")
        var = []
        ze = []

        self.board.write("init".encode())
        self._wait_for_completion()

        self.board.write("zcclk,{}".format(13500).encode())
        self.z += 14500
        self._wait_for_completion()

        for i in range(25):
            image = self.capture_image()
            var.append(self.variance(image))

            self.board.write("zcclk,{}".format(50).encode())
            self.z += 50
            self._wait_for_completion()
            ze.append((i + 1) * 50)

        var = np.array(var)
        l = np.argmax(var)

        self.board.write("zclk,{}".format(1290 - l * 50).encode())
        self.z -= 1290 - l * 50
        self._wait_for_completion()
    """"
    def auto(self):
        obj_value = 10
        z_positions = []
        variances = []
        max_variance = 0
        max_z = self.z

       #self.camera.start_preview()
        self.board.reset_input_buffer()
        sleep(1)

        self.camera.resolution = (320, 240)
        self.camera.framerate = 24

        if obj_value == 4:
            step_size = 50
        else:
            step_size = 25

        direction = 1  # 1 for upward, -1 for downward
        for i in range(41):
            stream = io.BytesIO()
            self.camera.capture(stream, format='jpeg')
            stream.seek(0)
            image = np.frombuffer(stream.getvalue(), dtype=np.uint8)
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)

            current_variance = self.variance(image)
            variances.append(current_variance)
            z_positions.append(self.z)

            print(f"Iteration {i+1}: x_axis={self.x}, y_axis={self.y}, z_axis={self.z}")

            if current_variance > max_variance:
                max_variance = current_variance
                max_z = self.z

            if i < 20:
                if current_variance >= max_variance:
                    if direction == 1:
                        self.movezclock(step_size)
                    else:
                        self.movezanticlock(step_size)
                else:
                    if direction == 1:
                        direction = -1
                        self.movezanticlock(step_size)
                    else:
                        break

        # Adjust to the position with the maximum variance
        adjust_steps = self.z - max_z
        if adjust_steps > 0:
            self.movezanticlock(adjust_steps)
        else:
            self.movezclock(abs(adjust_steps))

        # Capture the final focused image at high resolution
        self.camera.resolution = (4056, 3040)
        sleep(2)  # Allow time for the camera to adjust
        stream = io.BytesIO()
        self.camera.capture(stream, format='jpeg')
        stream.seek(0)
        high_res_image = np.frombuffer(stream.getvalue(), dtype=np.uint8)
        high_res_image = cv2.imdecode(high_res_image, cv2.IMREAD_COLOR)

        # Create directories for different objectives and save image with date and time
        base_dir = "/home/pi/Downloads/autoscan"
        objective_dir = os.path.join(base_dir, f"{obj_value}X")
        os.makedirs(objective_dir, exist_ok=True)
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_path = os.path.join(objective_dir, f"focusedimage_{current_time}.tiff")
        #cv2.imwrite(image_path, high_res_image)

        #sleep(2)
        high_res_image_pil = Image.fromarray(cv2.cvtColor(high_res_image, cv2.COLOR_BGR2RGB))
        high_res_image_pil.save(image_path, format='TIFF')

        sleep(5)
        print(variances)
        print(z_positions)
        print(max_z)
        print(max_variance)

    def capture_image(self):
        """
        image = np.empty((240 * 320 * 3), dtype=np.uint8)
        self.camera.capture(image, "bgr")
        return image.reshape((240, 320, 3))
        """
        stream = io.BytesIO()
        self.camera.capture(stream, format='jpeg')
        stream.seek(0)
        image = np.frombuffer(stream.getvalue(), dtype=np.uint8)
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)
        return image

    def scan(self):
        cur_time = datetime.now()
        dir_path = "/home/pi/Downloads/Images_Scan/scan_{}/".format(cur_time.strftime("%Y%m%d_%H%M"))
        subprocess.run(["mkdir", dir_path])
        logging.info("[Scan] First Autofocus")
        self.auto()

        for i in range(25):
            for j in range(28):
                if j % 2 == 0:
                    self.auto()

                if i % 2 == 0:
                    self.capture_and_save_image(dir_path, i, j)
                    self.movexanticlock(8)
                else:
                    self.capture_and_save_image(dir_path, i, 27 - j)
                    self.movexclock(8)
                
                #x_axis+={self.x}, y_axis={self.y}, z_axis={self.z}")    

                print(f"Iteration {i * 28 + j + 1}: x_axis={self.x}, y_axis={self.y}, z_axis={self.z}")

            self.movey(8)
            logging.info("[Scan] Changing y Pos")

        save_dir = "/home/pi/Pictures/Saved_videos/"
        # do not save the stitched Images required computation 
        #self.stitch_images(dir_path, save_dir, 25, 28)

    def capture_and_save_image(self, dir_path, i, j):
        """
        image_path = "{}/imagerow{},{}.tiff".format(dir_path, i, j)
        stream = io.BytesIO()
        self.camera.capture(stream, format='jpeg')
        stream.seek(0)
        high_res_image = np.frombuffer(stream.getvalue(), dtype=np.uint8)
        high_res_image = cv2.imdecode(high_res_image, cv2.IMREAD_COLOR)
        cv2.imwrite(image_path, high_res_image)
        self.scan_count += 1
        """
        image = self.capture_image()
        image_path = "{}/imagerow{},{}.tiff".format(dir_path, i, j)
        image_pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        image_pil.save(image_path, format='TIFF')
        sleep(4)
        self.scan_count += 1

    def stitch_images(self, image_paths, save_dir, rows, cols):
        images = []
        max_width = 0
        max_height = 0

        for i in range(rows):
            for j in range(cols):
                image = Image.open(image_paths + "imagerow{},{}.tiff".format(i, j))
                images.append(image)
                max_width = max(max_width, image.width)
                max_height = max(max_height, image.height)

        stitched_width = max_width * cols
        stitched_height = max_height * rows
        stitched_image = Image.new('RGB', (stitched_width, stitched_height), (255, 255, 255))

        for i, image in enumerate(images):
            row = i // cols
            col = i % cols
            x = col * max_width
            y = row * max_height
            stitched_image.paste(image, (x, y))

        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = os.path.join(save_dir, f"complete_slide_{current_time}.tiff")
        stitched_image.save(output_path)

if __name__ == "__main__":
    
    ### Adding Duration Process ###
    start_time=time.perf_counter()
    
    
    microscope = Microscope()
    # Example usage:
    
    print("Initialization Starting Please Wait......")
    xi = 350
    yi = 200
    sleep(3)
    
    microscope.init()
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
    microscope.home(x_axis, y_axis, z_axis)
    print("Movement of Steps After Home Position")
    print("x_axis", x_axis)
    print("y_axis", y_axis)
    print("z_axis", z_axis)
    print("Movement of Steps in mm")
    print("x_axis", str(x_axis * 0.075) + " mm")
    print("y_axis", str(y_axis * 0.075) + " mm")
    print("z_axis", z_axis)
    
    print("10X Objective value 10 Selected")
    in_obj = 10
    
    if in_obj == 4:
        if z_axis > 3500:
            ans_z = z_axis - 3500
            z_axis = z_axis - ans_z
            microscope.movezclock(ans_z)
        else:
            ans_z = 3500 - z_axis
            z_axis = z_axis + ans_z
            microscope.movezanticlock(ans_z)
        
        x = 80
        y = 0
        microscope.roixy_4x(x, y)
        sleep(2)
    elif in_obj == 10:
        if z_axis > 13350:
            ans_z = z_axis - 13350
            z_axis = z_axis - ans_z
            microscope.movezclock(ans_z)
        else:
            ans_z = 13350 - z_axis
            z_axis = z_axis + ans_z
            microscope.movezanticlock(ans_z)
        sleep(6)
        
        print("Movement of Steps for Starting Position")
        print("x_axis", x_axis)
        print("y_axis", y_axis)
        print("z_axis", z_axis)
        print("Movement of Steps in mm")
        print("x_axis", str(x_axis * 0.075) + " mm")
        print("y_axis", str(y_axis * 0.075) + " mm")
        print("z_axis", z_axis)
        
        x = 175
        y = 125
        microscope.roixy_10x(x, y)## moving to Starting Position 
        sleep(4)
    elif in_obj == 40:
        microscope.movezanticlock(16650)
        sleep(22)
    
    microscope.scan()
    print("Movement of Steps After Autofocus")
    print("x_axis", x_axis)
    print("y_axis", y_axis)
    print("z_axis", z_axis)
    print("Movement of Steps After Autofocus in mm")
    print("x_axis", str(x_axis * 0.075) + " mm")
    print("y_axis", str(y_axis * 0.075) + " mm")
    print("z_axis", z_axis)
    end_time=time.perf_counter()

    print(end_time-start_time,"Duration")
